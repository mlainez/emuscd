// emuscd - headless Roland Sound Canvas daemon
// Listens for MIDI input and synthesizes audio to ALSA output

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <cerrno>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <alsa/asoundlib.h>
#include <csignal>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include "common.h"
#include "midi_port_match.h"
#include "../libemusc/src/synth.h"
#include "../libemusc/src/control_rom.h"
#include "../libemusc/src/wave_rom.h"
#include "../libemusc/tools/cpu_list.h"
#include "../libemusc/tools/mxcsr_ftz.h"

using namespace emuscd;

namespace {

// Enumerates ALSA PCM devices the way `aplay -L` does, so --pcm has a source
// of values to try without guessing at ALSA's own naming.
void list_pcm_devices() {
  void **hints;
  if (snd_device_name_hint(-1, "pcm", &hints) < 0) {
    std::cerr << "emuscd: could not enumerate PCM devices" << std::endl;
    return;
  }
  for (void **n = hints; *n != nullptr; n++) {
    char *name = snd_device_name_get_hint(*n, "NAME");
    char *desc = snd_device_name_get_hint(*n, "DESC");
    if (name) {
      std::cout << name;
      if (desc) {
        std::string d(desc);
        std::replace(d.begin(), d.end(), '\n', ' ');
        std::cout << "  -  " << d;
      }
      std::cout << std::endl;
    }
    free(name);
    free(desc);
  }
  snd_device_name_free_hint(hints);
}

// A sequencer port another client lets this one read from, as `aconnect -i`
// lists them. Named "<client>: <port>" so either half matches --midi-in.
struct SeqSource {
  int client, port;
  std::string name;
};

std::vector<SeqSource> list_seq_sources(snd_seq_t *seq) {
  std::vector<SeqSource> out;
  const int self = snd_seq_client_id(seq);
  const unsigned caps = SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ;
  snd_seq_client_info_t *ci;
  snd_seq_port_info_t *pi;
  snd_seq_client_info_alloca(&ci);
  snd_seq_port_info_alloca(&pi);
  snd_seq_client_info_set_client(ci, -1);
  while (snd_seq_query_next_client(seq, ci) >= 0) {
    const int client = snd_seq_client_info_get_client(ci);
    // The system client's ports are the timer and announcements, not MIDI.
    if (client == SND_SEQ_CLIENT_SYSTEM || client == self)
      continue;
    snd_seq_port_info_set_client(pi, client);
    snd_seq_port_info_set_port(pi, -1);
    while (snd_seq_query_next_port(seq, pi) >= 0) {
      const unsigned c = snd_seq_port_info_get_capability(pi);
      if ((c & caps) != caps || (c & SND_SEQ_PORT_CAP_NO_EXPORT))
        continue;
      out.push_back({client, snd_seq_port_info_get_port(pi),
                     std::string(snd_seq_client_info_get_name(ci)) + ": " +
                       snd_seq_port_info_get_name(pi)});
    }
  }
  return out;
}

void list_midi_in_devices() {
  snd_seq_t *seq;
  if (snd_seq_open(&seq, "default", SND_SEQ_OPEN_INPUT, 0) < 0) {
    std::cerr << "emuscd: could not open the ALSA sequencer" << std::endl;
    return;
  }
  std::vector<SeqSource> src = list_seq_sources(seq);
  if (src.empty())
    std::cout << "(no MIDI input ports found)" << std::endl;
  for (size_t i = 0; i < src.size(); i++)
    std::cout << i << ": " << src[i].name << " (" << src[i].client << ":"
              << src[i].port << ")" << std::endl;
  snd_seq_close(seq);
}

}  // namespace

// Render time per block against the time the block takes to play, reported
// every five seconds: the headroom the engine actually has. CPU figures read
// from /proc overstate it, since the render pool's idle threads spin briefly
// before they sleep.
class RenderStats {
  std::vector<double> loads;   // render time / block period
  std::chrono::steady_clock::time_point window =
    std::chrono::steady_clock::now();

public:
  void add(double load) { loads.push_back(load); }

  void report_if_due(double period_ms) {
    auto now = std::chrono::steady_clock::now();
    if (now - window < std::chrono::seconds(5) || loads.empty()) return;
    std::sort(loads.begin(), loads.end());
    double sum = 0.0;
    size_t over = 0;
    for (double l : loads) {
      sum += l;
      if (l > 1.0) over++;
    }
    size_t p99 = (loads.size() * 99) / 100;
    if (p99 >= loads.size()) p99 = loads.size() - 1;
    std::fprintf(stderr,
                 "emuscd: %zu blocks of %.1f ms: render avg %.0f%%, p99 "
                 "%.0f%%, max %.0f%% of the block; %zu over 100%%\n",
                 loads.size(), period_ms, 100.0 * sum / loads.size(),
                 100.0 * loads[p99], 100.0 * loads.back(), over);
    loads.clear();
    window = now;
  }
};

// Set by SIGUSR1; the render loop takes it, since a signal handler may do
// nothing but store to a lock-free atomic.
std::atomic<bool> panic_signalled{false};

void on_sigusr1(int) { panic_signalled = true; }

class EmuscdDaemon {
  snd_seq_t* seq = nullptr;
  int seq_port = -1;
  snd_pcm_t* pcm_out = nullptr;
  unsigned int sample_rate;
  unsigned int block_frames;
  unsigned int render_threads;
  std::vector<unsigned> worker_cpus;
  bool show_stats;
  float gain_lin;

  std::unique_ptr<EmuSC::ControlRom> ctrl_rom;
  std::unique_ptr<EmuSC::WaveRom> wave_rom;
  std::unique_ptr<EmuSC::Synth> synth;
  std::string device_name;
  std::string rom_dir;

  std::atomic<bool> running{true};
  std::atomic<bool> device_change_requested{false};
  std::atomic<bool> panic_requested{false};
  std::mutex request_mutex;
  std::string requested_device;

public:
  EmuscdDaemon(const std::string& dev, const std::string& port_name,
               const std::string& pcm_device, const std::string& rom_directory,
               unsigned int requested_rate, unsigned int latency_ms,
               unsigned int block, unsigned int threads,
               const std::vector<unsigned>& workers, bool stats,
               double gain_db, const std::string& midi_in)
      : sample_rate(requested_rate), block_frames(block),
        render_threads(threads), worker_cpus(workers), show_stats(stats),
        gain_lin(gain_db_to_linear(gain_db)), rom_dir(rom_directory) {
    init_alsa_midi(port_name);
    if (!midi_in.empty())
      connect_midi_in(midi_in);
    init_alsa_audio(pcm_device, latency_ms);
    if (!load_device(dev)) {
      std::cerr << "emuscd: failed to load initial device '" << dev << "'"
                << std::endl;
      exit(2);
    }
  }

  ~EmuscdDaemon() {
    running = false;
    if (seq) snd_seq_close(seq);
    if (pcm_out) snd_pcm_close(pcm_out);
  }

  // A real ALSA sequencer client, not a rawmidi "virtual:" port: the latter's
  // string after the colon is the boolean `merge` argument of alsa.conf's
  // rawmidi.virtual type, not a name, so it never made the port findable by
  // --name (aconnect/aplaymidi always saw the generic "Client-N" instead).
  // snd_seq_set_client_name() is what actually makes --name resolve, the same
  // way fluidsynth/timidity's own ALSA MIDI input ports do.
  void init_alsa_midi(const std::string& port_name) {
    int err = snd_seq_open(&seq, "default", SND_SEQ_OPEN_INPUT, 0);
    if (err < 0) {
      std::cerr << "ALSA sequencer open error: " << snd_strerror(err) << std::endl;
      exit(1);
    }
    snd_seq_set_client_name(seq, port_name.c_str());
    seq_port = snd_seq_create_simple_port(
        seq, "input", SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
        SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_SYNTHESIZER);
    if (seq_port < 0) {
      std::cerr << "ALSA sequencer port error: " << snd_strerror(seq_port) << std::endl;
      exit(1);
    }
    snd_seq_nonblock(seq, 1);
    std::cout << "ALSA MIDI input opened: " << port_name << std::endl;
  }

  // Subscribes this client's port to an existing source, as `aconnect` would,
  // for a source such as a USB keyboard that is not going to connect to it.
  // Programs can still connect to the port by --name as well.
  void connect_midi_in(const std::string& want) {
    std::vector<SeqSource> src = list_seq_sources(seq);
    if (src.empty()) {
      std::cerr << "emuscd: --midi-in: no MIDI input ports available"
                << std::endl;
      exit(1);
    }
    std::vector<std::string> names;
    for (const SeqSource& s : src) names.push_back(s.name);
    MidiPortMatch m = match_midi_port(want, names);
    if (!m.candidates.empty()) {
      std::cerr << "emuscd: --midi-in '" << want << "' matches more than one "
                << "MIDI input:" << std::endl;
      for (int i : m.candidates)
        std::cerr << "  " << i << ": " << names[i] << std::endl;
      exit(1);
    }
    if (m.isIndex && (m.index < 0 || (size_t) m.index >= src.size())) {
      std::cerr << "emuscd: --midi-in " << want << " out of range (0.."
                << src.size() - 1 << "); see --list-midi-in" << std::endl;
      exit(1);
    }
    if (m.index < 0) {
      std::cerr << "emuscd: --midi-in '" << want << "' matches no MIDI input; "
                << "see --list-midi-in" << std::endl;
      exit(1);
    }
    const SeqSource& s = src[m.index];
    int err = snd_seq_connect_from(seq, seq_port, s.client, s.port);
    if (err < 0) {
      std::cerr << "emuscd: could not connect from '" << s.name << "': "
                << snd_strerror(err) << std::endl;
      exit(1);
    }
    std::cout << "MIDI input connected: " << s.name << " (" << s.client << ":"
              << s.port << ")" << std::endl;
  }

  void init_alsa_audio(const std::string& pcm_device, unsigned int latency_ms) {
    snd_pcm_hw_params_t* hw_params;
    int err;

    err = snd_pcm_open(&pcm_out, pcm_device.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
      std::cerr << "PCM open error (" << pcm_device << "): "
                << snd_strerror(err) << std::endl;
      exit(1);
    }

    snd_pcm_hw_params_alloca(&hw_params);
    snd_pcm_hw_params_any(pcm_out, hw_params);
    snd_pcm_hw_params_set_access(pcm_out, hw_params,
                                 SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_out, hw_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(pcm_out, hw_params, 2);

    unsigned int rate = sample_rate;
    snd_pcm_hw_params_set_rate_near(pcm_out, hw_params, &rate, 0);

    unsigned int buffer_time_us = latency_ms * 1000;
    snd_pcm_hw_params_set_buffer_time_near(pcm_out, hw_params, &buffer_time_us, 0);

    snd_pcm_hw_params(pcm_out, hw_params);
    sample_rate = rate;

    std::cout << "ALSA audio output '" << pcm_device << "' initialized at "
              << rate << " Hz, ~" << latency_ms << " ms buffer" << std::endl;
  }

  bool load_device(const std::string& dev) {
    if (!device_supported(dev)) {
      std::cerr << "emuscd: unsupported device '" << dev << "' (supported: "
                << "sc55, sc55mkii, sc88, jv880, jv1080)" << std::endl;
      return false;
    }

    DeviceRoms roms = resolve_device_roms(dev, rom_dir);

    std::unique_ptr<EmuSC::ControlRom> new_ctrl;
    std::unique_ptr<EmuSC::WaveRom> new_wave;
    try {
      new_ctrl.reset(new EmuSC::ControlRom(roms.control_rom, roms.cpu_rom));
      new_wave.reset(new EmuSC::WaveRom(roms.wave_roms, *new_ctrl));
    } catch (const std::string &e) {
      std::cerr << "emuscd: failed to load '" << dev << "' ROMs: " << e
                << std::endl;
      return false;
    } catch (const std::exception &e) {
      std::cerr << "emuscd: failed to load '" << dev << "' ROMs: "
                << e.what() << std::endl;
      return false;
    }

    std::unique_ptr<EmuSC::Synth> new_synth(
      new EmuSC::Synth(*new_ctrl, *new_wave, SOUND_MAP));
    new_synth->set_audio_format(sample_rate, 2);
    new_synth->set_render_threads(render_threads, worker_cpus);

    // Power-on reset, which is what the hardware does when it is switched on
    // and what a GS reset in the stream asks for later. It must follow
    // set_audio_format(), which is where the 16 parts are instantiated: called
    // before that, the resetParts loop has nothing to walk.
    //
    // On a Sound Canvas the reset is redundant at startup - Settings::reset()
    // runs exactly the four _initialize_*/_apply_device_performance calls the
    // Settings constructor runs, so a freshly built Synth already holds the
    // same defaults, bank 0 program 0 on every part and Drum1 on part 10. It
    // is here because a real device resets at power-on regardless, and because
    // the same call has to be correct when Synth::_apply_midi_sysex reaches it
    // from a GS reset mid-song, where the parts are no longer fresh.
    //
    // On a device whose profile names a reset Performance (DeviceProfile's
    // PerformanceLayout::resetSelector) it is not redundant: it is what takes
    // the machine out of its layered boot Performance and onto the channels a
    // multitimbral stream expects.
    new_synth->reset(SOUND_MAP, true);

    // Destroy the old Synth before its ControlRom/WaveRom, which it holds by
    // reference, are replaced out from under it.
    synth.reset();
    ctrl_rom = std::move(new_ctrl);
    wave_rom = std::move(new_wave);
    synth = std::move(new_synth);
    device_name = dev;

    std::cout << "Loaded device: " << dev << " (" << ctrl_rom->model()
              << " v" << ctrl_rom->version() << ")" << std::endl;
    return true;
  }

  void run() {
    std::thread stdin_thread([this]() { read_stdin_commands(); });

    std::vector<int16_t> out_buf(block_frames * 2);
    std::vector<float> rendered(block_frames * 2);
    RenderStats stats;
    const double period_s = (double) block_frames / sample_rate;
    std::cout << "emuscd daemon running (device: " << device_name << "). "
              << "Type a device name to switch, 'panic' to silence it "
              << "(or send SIGUSR1), or 'quit' to exit."
              << std::endl;

    while (running) {
      // Between blocks, on this thread: the one panic every device obeys,
      // whatever MIDI it recognises.
      if ((panic_requested.exchange(false) | panic_signalled.exchange(false)) &&
          synth) {
        synth->panic();
        std::cout << "emuscd: panic" << std::endl;
      }

      if (device_change_requested.exchange(false)) {
        std::string dev;
        {
          std::lock_guard<std::mutex> lock(request_mutex);
          dev = requested_device;
        }
        load_device(dev);
      }

      snd_seq_event_t *ev;
      while (snd_seq_event_input(seq, &ev) >= 0) {
        if (synth) handle_seq_event(ev);
      }

      // A whole block per pass: the MIDI that arrived while the previous one
      // was written is applied at its start.
      if (synth && pcm_out) {
        auto start = std::chrono::steady_clock::now();
        synth->get_next_frames(rendered.data(), block_frames);
        if (show_stats) {
          std::chrono::duration<double> took =
            std::chrono::steady_clock::now() - start;
          stats.add(took.count() / period_s);
          stats.report_if_due(1000.0 * period_s);
        }
        for (unsigned int i = 0; i < block_frames; i++) {
          float left = rendered[i * 2], right = rendered[i * 2 + 1];
          apply_gain(left, right, gain_lin);
          out_buf[i * 2] = to_i16(left);
          out_buf[i * 2 + 1] = to_i16(right);
        }
        write_block(out_buf);
      } else {
        std::this_thread::yield();
      }
    }

    stdin_thread.join();
  }

private:
  // The sequencer has already resolved running status and reassembled SysEx
  // into a single event by the time it reaches here, unlike ALSA's raw
  // rawmidi byte stream - there is no byte-level parsing left to do.
  void handle_seq_event(const snd_seq_event_t *ev) {
    const auto &note = ev->data.note;
    const auto &ctrl = ev->data.control;
    switch (ev->type) {
    case SND_SEQ_EVENT_NOTEON:
      synth->midi_input(0x90 | (note.channel & 0x0f), note.note, note.velocity);
      break;
    case SND_SEQ_EVENT_NOTEOFF:
      synth->midi_input(0x80 | (note.channel & 0x0f), note.note, note.velocity);
      break;
    case SND_SEQ_EVENT_KEYPRESS:
      synth->midi_input(0xa0 | (note.channel & 0x0f), note.note, note.velocity);
      break;
    case SND_SEQ_EVENT_CONTROLLER:
      synth->midi_input(0xb0 | (ctrl.channel & 0x0f), ctrl.param & 0x7f, ctrl.value & 0x7f);
      break;
    case SND_SEQ_EVENT_PGMCHANGE:
      synth->midi_input(0xc0 | (ctrl.channel & 0x0f), ctrl.value & 0x7f, 0);
      break;
    case SND_SEQ_EVENT_CHANPRESS:
      synth->midi_input(0xd0 | (ctrl.channel & 0x0f), ctrl.value & 0x7f, 0);
      break;
    case SND_SEQ_EVENT_PITCHBEND: {
      // ALSA's seq value is signed -8192..8191; MIDI's 14-bit pitch bend is
      // unsigned 0..16383 split into two 7-bit data bytes, LSB first.
      int v = ctrl.value + 8192;
      synth->midi_input(0xe0 | (ctrl.channel & 0x0f), v & 0x7f, (v >> 7) & 0x7f);
      break;
    }
    case SND_SEQ_EVENT_SYSEX:
      if (ev->data.ext.len > 0 && ev->data.ext.len <= 0xffff)
        synth->midi_input_sysex(static_cast<uint8_t *>(ev->data.ext.ptr),
                                static_cast<uint16_t>(ev->data.ext.len));
      break;
    default:
      break;  // clock, active-sense and other event types the engine doesn't need
    }
  }

  // A batched write can legitimately return fewer frames than asked for, one
  // single-frame writei never could; retry the remainder, and recover once
  // on an xrun/suspend rather than dropping the rest of the block.
  void write_block(const std::vector<int16_t>& out_buf) {
    snd_pcm_uframes_t total = out_buf.size() / 2;
    snd_pcm_uframes_t offset = 0;
    while (offset < total) {
      snd_pcm_sframes_t written =
        snd_pcm_writei(pcm_out, out_buf.data() + offset * 2, total - offset);
      if (written < 0) {
        if (snd_pcm_recover(pcm_out, (int) written, 1) < 0) {
          std::cerr << "emuscd: PCM write error: "
                    << snd_strerror((int) written) << std::endl;
          return;
        }
        continue;
      }
      offset += written;
    }
  }

  void read_stdin_commands() {
    std::string line;
    while (running) {
      if (std::getline(std::cin, line)) {
        if (line == "quit" || line == "exit") {
          running = false;
        } else if (line == "panic") {
          panic_requested = true;
        } else if (!line.empty()) {
          std::lock_guard<std::mutex> lock(request_mutex);
          requested_device = line;
          device_change_requested = true;
        }
      } else {
        // stdin closed (e.g. running under a service manager): stop polling
        // it, but keep serving MIDI/audio.
        break;
      }
    }
  }
};

int main(int argc, char* argv[]) {
  set_flush_denormals_to_zero();

  std::string device = "sc88";
  std::string port_name = "emuscd";
  std::string midi_in;
  std::string pcm_device = "default";
  std::string rom_dir;
  unsigned int rate = 48000;
  unsigned int latency_ms = 20;
  unsigned int block = 256;
  unsigned int threads = 0;
  std::vector<unsigned> cpus;
  bool stats = false;
  int rt_priority = 0;
  double gain_db = 0.0;

  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    auto need = [&](const char *name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "emuscd: " << name << " requires an argument" << std::endl;
        exit(1);
      }
      return argv[++i];
    };
    if (arg == "--device") {
      device = need("--device");
    } else if (arg == "--name") {
      port_name = need("--name");
    } else if (arg == "--pcm") {
      pcm_device = need("--pcm");
    } else if (arg == "--rom-dir") {
      rom_dir = need("--rom-dir");
    } else if (arg == "--list-pcm") {
      list_pcm_devices();
      return 0;
    } else if (arg == "--midi-in") {
      midi_in = need("--midi-in");
    } else if (arg == "--list-midi-in") {
      list_midi_in_devices();
      return 0;
    } else if (arg == "--rate") {
      rate = static_cast<unsigned int>(std::stoul(need("--rate")));
    } else if (arg == "--latency") {
      latency_ms = static_cast<unsigned int>(std::stoul(need("--latency")));
    } else if (arg == "--block") {
      block = static_cast<unsigned int>(std::stoul(need("--block")));
      if (block < 1) {
        std::cerr << "emuscd: --block must be >= 1" << std::endl;
        return 1;
      }
    } else if (arg == "--threads") {
      threads = static_cast<unsigned int>(std::stoul(need("--threads")));
    } else if (arg == "--cpus") {
      if (!emusc_tools::parse_cpu_list(need("--cpus"), cpus)) {
        std::cerr << "emuscd: --cpus takes a list such as 0-3 or 0,2,3"
                  << std::endl;
        return 1;
      }
    } else if (arg == "--stats") {
      stats = true;
    } else if (arg == "--rt") {
      rt_priority = std::stoi(need("--rt"));
      if (rt_priority < 1 || rt_priority > 99) {
        std::cerr << "emuscd: --rt takes a priority from 1 to 99" << std::endl;
        return 1;
      }
    } else if (arg == "--gain-db") {
      gain_db = std::stod(need("--gain-db"));
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "emuscd - Roland Sound Canvas daemon\n"
          << "Usage: emuscd [options]\n"
          << "Options:\n"
          << "  --device NAME       Device to emulate (default: sc88)\n"
          << "                       Supported: sc55, sc55mkii, sc88, jv880, jv1080\n"
          << "  --name NAME         ALSA MIDI port name (default: emuscd)\n"
          << "  --midi-in PORT      Also connect from an existing MIDI input, such\n"
          << "                       as a USB keyboard: an index, or a name or any\n"
          << "                       part of one, any case. See --list-midi-in\n"
          << "  --list-midi-in      List MIDI input ports and exit\n"
          << "  --pcm DEVICE        ALSA PCM output device (default: default)\n"
          << "  --list-pcm          List ALSA PCM devices and exit\n"
          << "  --rom-dir DIR       Directory holding device ROM files (default:\n"
          << "                       $EMUSCD_ROM_DIR, else ./roms if it exists,\n"
          << "                       else /usr/share/emuscd/roms)\n"
          << "  --rate HZ           Requested audio sample rate (default: 48000)\n"
          << "  --latency MS        Requested output buffer size (default: 20)\n"
          << "  --block N           Audio frames per ALSA write (default: 256)\n"
          << "  --threads N         Threads the SC-88 and JV-1080 render their voices\n"
          << "                       on, this one included; 0 is one per CPU this\n"
          << "                       process may use (default: 0). The audio is the\n"
          << "                       same at every count\n"
          << "  --cpus LIST         Pin the render threads, one per CPU in LIST\n"
          << "                       (e.g. 0-3): this thread on the first, the\n"
          << "                       pool's from the last down, so a core kept\n"
          << "                       out of scheduling (isolcpus) still gets one.\n"
          << "                       Without --threads, one thread per CPU listed\n"
          << "  --stats             Every 5 s on stderr: render time per block as\n"
          << "                       a share of its playing time (avg, p99, max)\n"
          << "  --rt PRIO           Run the render threads SCHED_FIFO at PRIO (1-99)\n"
          << "                       with memory locked, so a block's render is not\n"
          << "                       preempted by ordinary processes or page faults.\n"
          << "                       Needs CAP_SYS_NICE (root)\n"
          << "  --gain-db DB        Output gain in dB, linear multiplier 10^(DB/20)\n"
          << "                       applied to the final samples (default: 0, no\n"
          << "                       change - a listening-convenience knob only)\n"
          << "  --help              Show this help\n"
          << "\n"
          << "On stdin: a device name switches device, 'panic' (or SIGUSR1)\n"
          << "silences every part of any device, 'quit' exits.\n"
          << "\n"
          << "ROM files are named <device>_control.bin, <device>_cpu.bin\n"
          << "(SC-55/SC-55mkII only) and <device>_waverom<N>.bin. See\n"
          << "README.md for exact ROM hashes.\n";
      return 0;
    } else {
      std::cerr << "emuscd: unknown option '" << arg << "'" << std::endl;
      return 1;
    }
  }

  if (!device_supported(device)) {
    std::cerr << "emuscd: unsupported device '" << device << "' (supported: "
              << "sc55, sc55mkii, sc88, jv880, jv1080)" << std::endl;
    return 1;
  }

  if (rom_dir.empty()) rom_dir = default_rom_dir();

  std::signal(SIGUSR1, on_sigusr1);

  // The first CPU listed takes this thread, which renders and also drives
  // ALSA; the pool's threads take the rest from the last down.
  std::vector<unsigned> worker_cpus;
  if (!cpus.empty()) {
    if (threads == 0) threads = (unsigned) cpus.size();
    worker_cpus = emusc_tools::worker_cpus(cpus);
    if (!emusc_tools::pin_this_thread(cpus[0]))
      std::cerr << "emuscd: cannot pin to CPU " << cpus[0] << std::endl;
  }

  // Before the synth exists, so that the render pool's threads, created
  // with it, inherit the policy: glibc's threads take their creator's.
  if (rt_priority) {
    sched_param param{};
    param.sched_priority = rt_priority;
    int err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (err != 0)
      std::cerr << "emuscd: cannot set SCHED_FIFO " << rt_priority << ": "
                << std::strerror(err) << std::endl;
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
      std::cerr << "emuscd: cannot lock memory: " << std::strerror(errno)
                << std::endl;
  }

  try {
    EmuscdDaemon daemon(device, port_name, pcm_device, rom_dir, rate,
                         latency_ms, block, threads, worker_cpus, stats,
                         gain_db, midi_in);
    daemon.run();
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
