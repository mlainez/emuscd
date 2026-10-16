// emusc-winmidi - Windows real-time Roland Sound Canvas MIDI device
//
// The WinMM equivalent of emuscd (main.cc, ALSA/Linux). It does not create a
// MIDI port of its own - unlike ALSA's "virtual:" ports, WinMM's midiIn API
// only opens an EXISTING one - so a virtual MIDI cable (loopMIDI on Windows
// 7+, Maple Virtual MIDI Cable on 95/98/ME) is what a sequencer or game
// connects to; this process then opens that port by device index.

#include <windows.h>
#include <mmsystem.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <algorithm>
#include <vector>

#include "audio_out_dsound.h"
#include "common.h"
#include "../libemusc/src/synth.h"
#include "../libemusc/src/control_rom.h"
#include "../libemusc/src/wave_rom.h"
#include "../libemusc/tools/mxcsr_ftz.h"
#include "../libemusc/tools/win_realtime.h"
#include "../libemusc/src/simple_mutex.h"

using namespace emuscd;

namespace {

const char *USAGE =
"emusc-winmidi - Roland Sound Canvas MIDI device for Windows\n"
"Usage: emusc-winmidi [options]\n"
"Options:\n"
"  --device NAME       Device to emulate (default: sc88)\n"
"                       Supported: sc55, sc55mkii, sc88, jv880, jv1080\n"
"  --midi-in N          MIDI input device index (default: 0)\n"
"  --audio-api API      Audio output API: auto, winmm or dsound (default:\n"
"                        auto - DirectSound, falling back to WinMM if it\n"
"                        cannot be opened; winmm/dsound force one, and a\n"
"                        forced dsound that fails exits instead)\n"
"  --wave-out N         WinMM wave output device index (default: system\n"
"                        default)\n"
"  --dsound-out N       DirectSound output device index (default: system\n"
"                        default)\n"
"  --list-midi-in       List MIDI input devices and exit\n"
"  --list-wave-out      List WinMM wave output devices and exit\n"
"  --list-dsound-out    List DirectSound output devices and exit\n"
"  --rom-dir DIR        Directory holding device ROM files (default:\n"
"                        %EMUSCD_ROM_DIR%, or .\\roms if unset)\n"
"  --rate HZ            Audio sample rate (default: 48000)\n"
"  --block N            Audio frames per wave buffer (default: 256)\n"
"  --latency MS         Requested output buffer size (default: 20). A larger\n"
"                        value trades response time for robustness against\n"
"                        transient OS contention (window dragging, disk I/O):\n"
"                        raise it if you hear dropouts, but every MIDI event\n"
"                        then sounds that much later. The right value depends\n"
"                        on the machine - CPU, sound driver, what else runs -\n"
"                        so find the lowest that plays cleanly on yours.\n"
"  --gain-db DB         Output gain in dB, linear multiplier 10^(DB/20)\n"
"                        applied to the final samples (default: 0, no\n"
"                        change - a listening-convenience knob only)\n"
"  --help               Show this help\n"
"\n"
"ROM files are named <device>_control.bin, <device>_cpu.bin\n"
"(SC-55/SC-55mkII only) and <device>_waverom<N>.bin. See README.md\n"
"for exact ROM hashes.\n"
"\n"
"emusc-winmidi does not create a MIDI port itself. Route MIDI into it with\n"
"a virtual MIDI cable (loopMIDI, or Maple Virtual MIDI Cable on Windows\n"
"95/98/ME), then pick that port with --midi-in.\n";

// Bytes 0-2 of MIM_DATA's packed dwParam1 are the status byte and up to two
// data bytes, already driver-normalised: WinMM has resolved running status
// and framed the message before this ever arrives, same as ALSA's sequencer
// API does on the Linux side (main.cc's handle_seq_event) - neither backend
// does its own byte-level MIDI parsing.
struct MidiEvt { uint8_t status, d1, d2; };

enum class AudioApi { Auto, WinMM, DSound };

inline MidiEvt unpack_midi_message(DWORD_PTR dwParam1) {
  DWORD packed = static_cast<DWORD>(dwParam1);
  return { static_cast<uint8_t>(packed & 0xff),
           static_cast<uint8_t>((packed >> 8) & 0xff),
           static_cast<uint8_t>((packed >> 16) & 0xff) };
}

void list_midi_in_devices() {
  UINT n = midiInGetNumDevs();
  if (n == 0) { std::printf("(no MIDI input devices found)\n"); return; }
  for (UINT i = 0; i < n; i++) {
    MIDIINCAPSA caps;
    if (midiInGetDevCapsA(i, &caps, sizeof(caps)) == MMSYSERR_NOERROR)
      std::printf("%u: %s\n", i, caps.szPname);
  }
}

void list_wave_out_devices() {
  UINT n = waveOutGetNumDevs();
  if (n == 0) { std::printf("(no wave output devices found)\n"); return; }
  for (UINT i = 0; i < n; i++) {
    WAVEOUTCAPSA caps;
    if (waveOutGetDevCapsA(i, &caps, sizeof(caps)) == MMSYSERR_NOERROR)
      std::printf("%u: %s\n", i, caps.szPname);
  }
}

}  // namespace

class WinMidiDaemon {
public:
  WinMidiDaemon(const std::string &dev, const std::string &romDir,
                int midiInId, int waveOutId, unsigned rate, unsigned block,
                unsigned latencyMs, double gainDb, AudioApi api,
                int dsoundOutId)
      : _sampleRate(rate), _blockFrames(block), _romDir(romDir),
        _gainLin(gain_db_to_linear(gainDb)) {
    InitializeCriticalSection(&_midiLock);
    InitializeCriticalSection(&_sysexLock);
    if (!load_device(dev)) {
      std::fprintf(stderr, "emusc-winmidi: failed to load initial device '%s'\n",
                   dev.c_str());
      std::exit(2);
    }
    open_midi_in(midiInId);
    if (api == AudioApi::WinMM || !open_dsound(dsoundOutId, latencyMs)) {
      if (api == AudioApi::DSound) std::exit(1);
      if (api == AudioApi::Auto)
        std::fprintf(stderr, "emusc-winmidi: DirectSound unavailable (reason "
                     "above); falling back to WinMM\n");
      open_wave_out(waveOutId, latencyMs);
    }
  }

  ~WinMidiDaemon() {
    _dsound.reset();
    if (_hMidiIn) { midiInStop(_hMidiIn); midiInReset(_hMidiIn); midiInClose(_hMidiIn); }
    if (_hWaveOut) { waveOutReset(_hWaveOut); waveOutClose(_hWaveOut); }
    DeleteCriticalSection(&_midiLock);
    DeleteCriticalSection(&_sysexLock);
  }

  // Mirrors emuscd's own device_supported()/rom-resolution/reset sequence
  // (main.cc, EmuscdDaemon::load_device) so the two behave identically aside
  // from the audio/MIDI backend underneath.
  bool load_device(const std::string &dev) {
    if (!device_supported(dev)) {
      std::fprintf(stderr, "emusc-winmidi: unsupported device '%s' (supported: "
                   "sc55, sc55mkii, sc88, jv880, jv1080)\n", dev.c_str());
      return false;
    }

    DeviceRoms roms = resolve_device_roms(dev, _romDir);

    std::unique_ptr<EmuSC::ControlRom> new_ctrl;
    std::unique_ptr<EmuSC::WaveRom> new_wave;
    try {
      new_ctrl.reset(new EmuSC::ControlRom(roms.control_rom, roms.cpu_rom));
      new_wave.reset(new EmuSC::WaveRom(roms.wave_roms, *new_ctrl));
    } catch (const std::string &e) {
      std::fprintf(stderr, "emusc-winmidi: failed to load '%s' ROMs: %s\n",
                   dev.c_str(), e.c_str());
      return false;
    } catch (const std::exception &e) {
      std::fprintf(stderr, "emusc-winmidi: failed to load '%s' ROMs: %s\n",
                   dev.c_str(), e.what());
      return false;
    }

    std::unique_ptr<EmuSC::Synth> new_synth(
      new EmuSC::Synth(*new_ctrl, *new_wave, SOUND_MAP));
    new_synth->set_audio_format(_sampleRate, 2);
    new_synth->reset(SOUND_MAP, true);

    // Only the main loop (run()) ever touches _synth after construction, on
    // both the fill/pump and the device-switch path below, so there's no
    // concurrent access to guard here - the MIDI callback only ever writes
    // to _midiQueue/_sysexPending under their own critical sections.
    _synth.reset();
    _ctrlRom = std::move(new_ctrl);
    _waveRom = std::move(new_wave);
    _synth = std::move(new_synth);

    std::fprintf(stderr, "emusc-winmidi: loaded device %s (%s v%s)\n",
                 dev.c_str(), _ctrlRom->model().c_str(), _ctrlRom->version().c_str());
    return true;
  }

  void open_midi_in(int id) {
    UINT n = midiInGetNumDevs();
    if (n == 0) {
      std::fprintf(stderr, "emusc-winmidi: no MIDI input devices available - "
                   "install a virtual MIDI cable (loopMIDI/Maple) first\n");
      std::exit(1);
    }
    if (id < 0 || static_cast<UINT>(id) >= n) {
      std::fprintf(stderr, "emusc-winmidi: --midi-in %d out of range (0..%u); "
                   "see --list-midi-in\n", id, n - 1);
      std::exit(1);
    }
    MIDIINCAPSA caps{};
    midiInGetDevCapsA(id, &caps, sizeof(caps));

    MMRESULT r = midiInOpen(&_hMidiIn, static_cast<UINT>(id),
                             reinterpret_cast<DWORD_PTR>(&midi_in_proc),
                             reinterpret_cast<DWORD_PTR>(this), CALLBACK_FUNCTION);
    if (r != MMSYSERR_NOERROR) {
      std::fprintf(stderr, "emusc-winmidi: midiInOpen failed (error %u)\n",
                   (unsigned) r);
      std::exit(1);
    }
    prepare_sysex_buffer();
    midiInStart(_hMidiIn);
    std::fprintf(stderr, "emusc-winmidi: MIDI input '%s' (device %d)\n",
                 caps.szPname, id);
  }

  void open_wave_out(int id, unsigned latencyMs) {
    WAVEFORMATEX wfx{};
    wfx.wFormatTag = WAVE_FORMAT_PCM;
    wfx.nChannels = 2;
    wfx.nSamplesPerSec = _sampleRate;
    wfx.wBitsPerSample = 16;
    wfx.nBlockAlign = static_cast<WORD>(wfx.nChannels * wfx.wBitsPerSample / 8);
    wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;

    UINT_PTR devId = (id < 0) ? WAVE_MAPPER : static_cast<UINT_PTR>(id);
    // CALLBACK_NULL: buffer completion is polled (WHDR_DONE) from the main
    // loop instead. WinMM's own documented list of functions safe to call
    // from inside a wave/MIDI callback does not include waveOutWrite or
    // midiInAddBuffer, so nothing here re-arms a buffer from a callback.
    MMRESULT r = waveOutOpen(&_hWaveOut, devId, &wfx, 0, 0, CALLBACK_NULL);
    if (r != MMSYSERR_NOERROR) {
      std::fprintf(stderr, "emusc-winmidi: waveOutOpen failed (error %u)\n",
                   (unsigned) r);
      std::exit(1);
    }

    unsigned numBuffers = (latencyMs * _sampleRate) / 1000 / _blockFrames;
    if (numBuffers < 2) numBuffers = 2;
    _buffers.resize(numBuffers);
    for (auto &b : _buffers) {
      b.data.assign(static_cast<size_t>(_blockFrames) * 2, 0);
      std::memset(&b.hdr, 0, sizeof(WAVEHDR));
      b.hdr.lpData = reinterpret_cast<LPSTR>(b.data.data());
      b.hdr.dwBufferLength = static_cast<DWORD>(b.data.size() * sizeof(int16_t));
      waveOutPrepareHeader(_hWaveOut, &b.hdr, sizeof(WAVEHDR));
      fill_buffer(b);
      waveOutWrite(_hWaveOut, &b.hdr, sizeof(WAVEHDR));
    }
    std::fprintf(stderr, "emusc-winmidi: audio output at %u Hz, %u x %u-frame buffers\n",
                 _sampleRate, numBuffers, _blockFrames);
  }

  // Returns false, with the reason already printed, if DirectSound could not
  // be opened.
  bool open_dsound(int id, unsigned latencyMs) {
    _dsound = DSoundOut::open(id, _sampleRate, _blockFrames, latencyMs,
                              &dsound_fill, this);
    return _dsound != nullptr;
  }

  void run() {
    // _running must already be true before the stdin thread starts: its own
    // loop checks the same flag first thing, and if it started running
    // before this thread set it, it would see it still false, return
    // immediately without ever reading a line, and leave "quit" unable to
    // stop the daemon at all.
    _running = true;
    HANDLE stdinThread = CreateThread(nullptr, 0, &stdin_thread_proc, this, 0, nullptr);

    // This thread alone refills the wave-out queue.
    emusc_tools::SystemTimerResolution timerRes(1);
    emusc_tools::raise_audio_thread_priority();

    std::fprintf(stderr, "emusc-winmidi running. Type a device name to switch, "
                 "or 'quit' to exit.\n");
    reset_utilization_window();
    while (_running) {
      if (_deviceChangeRequested.exchange(false)) {
        std::string dev;
        {
          std::lock_guard<EmuSC::SimpleMutex> lock(_requestMutex);
          dev = _requestedDevice;
        }
        load_device(dev);
        reset_utilization_window();
      }

      drain_midi_queue();
      drain_sysex();
      pump_wave_buffers();
      if (_dsound) _dsound->pump();
      report_utilization();
      Sleep(1);
    }

    if (stdinThread) {
      WaitForSingleObject(stdinThread, INFINITE);
      CloseHandle(stdinThread);
    }
  }

private:
  using Clock = std::chrono::steady_clock;

  struct WaveBuffer { WAVEHDR hdr; std::vector<int16_t> data; };

  void render_frames(int16_t *dst, unsigned frames) {
    const Clock::time_point start = Clock::now();
    if (_rendered.size() < frames * 2) _rendered.resize(frames * 2);
    if (_synth)
      _synth->get_next_frames(_rendered.data(), frames);
    else
      std::fill(_rendered.begin(), _rendered.begin() + frames * 2, 0.0f);
    for (unsigned i = 0; i < frames; i++) {
      float l = _rendered[i * 2], r = _rendered[i * 2 + 1];
      apply_gain(l, r, _gainLin);
      dst[i * 2]     = to_i16(l);
      dst[i * 2 + 1] = to_i16(r);
    }
    _renderBusy += Clock::now() - start;
  }

  void reset_utilization_window() {
    _renderBusy = Clock::duration::zero();
    _windowStart = Clock::now();
  }

  // Share of wall-clock time spent synthesizing, printed every ~2 s. Only
  // render_frames() counts as busy: both backends pace this loop by waiting
  // on the audio device, so a ratio of audio produced to time elapsed would
  // sit near 100% whatever the spare capacity, while this one falls as
  // headroom grows. Time the thread is preempted mid-render still counts as
  // busy, so under heavy contention the figure overstates emulator load.
  void report_utilization() {
    const Clock::time_point now = Clock::now();
    const Clock::duration window = now - _windowStart;
    if (window < std::chrono::seconds(2)) return;
    std::fprintf(stderr, "emusc-winmidi: CPU utilization %.1f%% (share of time "
                 "spent synthesizing; lower means more headroom)\n",
                 100.0 * std::chrono::duration<double>(_renderBusy).count() /
                   std::chrono::duration<double>(window).count());
    _renderBusy = Clock::duration::zero();
    _windowStart = now;
  }

  void fill_buffer(WaveBuffer &b) { render_frames(b.data.data(), _blockFrames); }

  static void dsound_fill(void *self, int16_t *dst, unsigned frames) {
    static_cast<WinMidiDaemon *>(self)->render_frames(dst, frames);
  }

  void pump_wave_buffers() {
    for (auto &b : _buffers) {
      if (b.hdr.dwFlags & WHDR_DONE) {
        fill_buffer(b);
        waveOutWrite(_hWaveOut, &b.hdr, sizeof(WAVEHDR));
      }
    }
  }

  void drain_midi_queue() {
    std::vector<MidiEvt> local;
    EnterCriticalSection(&_midiLock);
    local.swap(_midiQueue);
    LeaveCriticalSection(&_midiLock);
    if (_synth)
      for (const MidiEvt &e : local) _synth->midi_input(e.status, e.d1, e.d2);
  }

  void prepare_sysex_buffer() {
    _sysexHdr = {};
    _sysexBuf.assign(SYSEX_BUFFER_SIZE, 0);
    _sysexHdr.lpData = reinterpret_cast<LPSTR>(_sysexBuf.data());
    _sysexHdr.dwBufferLength = SYSEX_BUFFER_SIZE;
    midiInPrepareHeader(_hMidiIn, &_sysexHdr, sizeof(MIDIHDR));
    midiInAddBuffer(_hMidiIn, &_sysexHdr, sizeof(MIDIHDR));
  }

  // A SysEx longer than SYSEX_BUFFER_SIZE arrives as MIM_LONGDATA with
  // dwBytesRecorded == dwBufferLength and MIDIERR_STILLPLAYING-free but
  // incomplete; multi-buffer chaining for that case isn't implemented, so
  // such a message is forwarded truncated rather than dropped outright -
  // acceptable for the GS/GM resets and patch dumps this targets, which are
  // well under 4 kB.
  void drain_sysex() {
    bool pending;
    EnterCriticalSection(&_sysexLock);
    pending = _sysexPending;
    _sysexPending = false;
    LeaveCriticalSection(&_sysexLock);
    if (!pending) return;

    if (_synth && _sysexHdr.dwBytesRecorded > 0)
      _synth->midi_input_sysex(reinterpret_cast<uint8_t *>(_sysexHdr.lpData),
                                static_cast<uint16_t>(_sysexHdr.dwBytesRecorded));

    midiInUnprepareHeader(_hMidiIn, &_sysexHdr, sizeof(MIDIHDR));
    midiInPrepareHeader(_hMidiIn, &_sysexHdr, sizeof(MIDIHDR));
    midiInAddBuffer(_hMidiIn, &_sysexHdr, sizeof(MIDIHDR));
  }

  // Trampoline for CreateThread, whose LPTHREAD_START_ROUTINE signature takes
  // a plain void* rather than a bindable callable the way std::thread does.
  static DWORD WINAPI stdin_thread_proc(LPVOID param) {
    reinterpret_cast<WinMidiDaemon *>(param)->read_stdin_commands();
    return 0;
  }

  void read_stdin_commands() {
    char buf[4096];
    while (_running) {
      if (std::fgets(buf, sizeof(buf), stdin)) {
        size_t len = std::strlen(buf);
        while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r'))
          buf[--len] = '\0';
        std::string line(buf);
        if (line == "quit" || line == "exit") {
          _running = false;
        } else if (!line.empty()) {
          std::lock_guard<EmuSC::SimpleMutex> lock(_requestMutex);
          _requestedDevice = line;
          _deviceChangeRequested = true;
        }
      } else {
        break;
      }
    }
  }

  static void CALLBACK midi_in_proc(HMIDIIN, UINT wMsg, DWORD_PTR dwInstance,
                                     DWORD_PTR dwParam1, DWORD_PTR) {
    auto *self = reinterpret_cast<WinMidiDaemon *>(dwInstance);
    if (wMsg == MIM_DATA) {
      MidiEvt e = unpack_midi_message(dwParam1);
      EnterCriticalSection(&self->_midiLock);
      self->_midiQueue.push_back(e);
      LeaveCriticalSection(&self->_midiLock);
    } else if (wMsg == MIM_LONGDATA) {
      EnterCriticalSection(&self->_sysexLock);
      self->_sysexPending = true;
      LeaveCriticalSection(&self->_sysexLock);
    }
  }

  static const DWORD SYSEX_BUFFER_SIZE = 4096;

  unsigned _sampleRate, _blockFrames;
  std::string _romDir;
  float _gainLin = 1.0f;
  std::vector<float> _rendered;
  std::atomic<bool> _running{false};

  Clock::duration _renderBusy{};
  Clock::time_point _windowStart;

  std::unique_ptr<EmuSC::ControlRom> _ctrlRom;
  std::unique_ptr<EmuSC::WaveRom> _waveRom;
  std::unique_ptr<EmuSC::Synth> _synth;

  HMIDIIN _hMidiIn = nullptr;
  HWAVEOUT _hWaveOut = nullptr;
  std::vector<WaveBuffer> _buffers;
  std::unique_ptr<DSoundOut> _dsound;

  CRITICAL_SECTION _midiLock;
  std::vector<MidiEvt> _midiQueue;

  CRITICAL_SECTION _sysexLock;
  bool _sysexPending = false;
  MIDIHDR _sysexHdr{};
  std::vector<uint8_t> _sysexBuf;

  std::atomic<bool> _deviceChangeRequested{false};
  EmuSC::SimpleMutex _requestMutex;
  std::string _requestedDevice;
};

int main(int argc, char **argv) {
  set_flush_denormals_to_zero();

  std::string device = "sc88";
  std::string romDir;
  int midiInId = 0;
  int waveOutId = -1;
  int dsoundOutId = -1;
  AudioApi audioApi = AudioApi::Auto;
  unsigned rate = 48000;
  unsigned block = 256;
  unsigned latency = 20;
  double gainDb = 0.0;

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto need = [&](const char *name) -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "emusc-winmidi: %s requires an argument\n", name);
        std::exit(1);
      }
      return argv[++i];
    };
    if      (a == "--device")        device = need("--device");
    else if (a == "--rom-dir")       romDir = need("--rom-dir");
    else if (a == "--midi-in")       midiInId = std::stoi(need("--midi-in"));
    else if (a == "--wave-out")      waveOutId = std::stoi(need("--wave-out"));
    else if (a == "--dsound-out")    dsoundOutId = std::stoi(need("--dsound-out"));
    else if (a == "--audio-api") {
      std::string api = need("--audio-api");
      if (api == "auto") audioApi = AudioApi::Auto;
      else if (api == "winmm") audioApi = AudioApi::WinMM;
      else if (api == "dsound") audioApi = AudioApi::DSound;
      else {
        std::fprintf(stderr, "emusc-winmidi: --audio-api must be auto, winmm "
                     "or dsound\n");
        return 1;
      }
    }
    else if (a == "--rate")          rate = static_cast<unsigned>(std::stoul(need("--rate")));
    else if (a == "--block")         block = static_cast<unsigned>(std::stoul(need("--block")));
    else if (a == "--latency")       latency = static_cast<unsigned>(std::stoul(need("--latency")));
    else if (a == "--gain-db")       gainDb = std::stod(need("--gain-db"));
    else if (a == "--list-midi-in")  { list_midi_in_devices(); return 0; }
    else if (a == "--list-wave-out") { list_wave_out_devices(); return 0; }
    else if (a == "--list-dsound-out") return DSoundOut::list_devices() ? 0 : 1;
    else if (a == "--help" || a == "-h") { std::printf("%s", USAGE); return 0; }
    else {
      std::fprintf(stderr, "emusc-winmidi: unknown option '%s'\n", a.c_str());
      return 1;
    }
  }

  if (!device_supported(device)) {
    std::fprintf(stderr, "emusc-winmidi: unsupported device '%s' (supported: "
                 "sc55, sc55mkii, sc88, jv880, jv1080)\n", device.c_str());
    return 1;
  }
  if (block < 1) {
    std::fprintf(stderr, "emusc-winmidi: --block must be >= 1\n");
    return 1;
  }

  if (romDir.empty()) romDir = default_rom_dir();

  WinMidiDaemon daemon(device, romDir, midiInId, waveOutId, rate, block, latency, gainDb,
                       audioApi, dsoundOutId);
  daemon.run();
  return 0;
}
