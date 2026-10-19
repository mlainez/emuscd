/*  
 *  This file is part of libEmuSC, a Sound Canvas emulator library
 *  Copyright (C) 2022-2026  Håkon Skjelten
 *
 *  libEmuSC is free software: you can redistribute it and/or modify it
 *  under the terms of the GNU Lesser General Public License as published
 *  by the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  libEmuSC is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with libEmuSC. If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef __SYNTH_H__
#define __SYNTH_H__


#include "control_rom.h"
#include "engines/gp/analog_stage.h"
#include "engines/gp/resampler.h"
#include "engines/gp/system_effects.h"
#include "params.h"
#include "simple_mutex.h"
#include "wave_rom.h"

#include <array>
#include <atomic>
#include <deque>
#include <functional>
#include <string>
#include <vector>


/* Public API for libEmuSC
 *
 * This Synth class is responsible for receiving MIDI events from client,
 * process the event based on information in Control and Wave ROMS, and
 * finally responding to audio buffer requests.
 *
 * Synth class constructor depends on a valid Control Rom and Wave ROM.
 *
 * MIDI events is sent to the emulator via the midi_input() method using the
 * three bytes from raw MIDI events.
 * 
 * Audio samples are extracted by calling the get_next_frame() method. This
 * is typically done from a callback function triggered by the OS audio
 * driver when the audio buffer is running low.
 *
 * All settings are configured through the Settings class.
 */


// The SC-88's engine and its own device state. Forward-declared rather than
// included so that this header does not pull in the whole engines/xp/ tree.
namespace EmuSC { namespace Xp { struct Device; class WorkerPool; } }

namespace EmuSC {

namespace Gp { class Part; }
class Settings;

class Synth
{
public:

  enum class SoundMap {
    GS,                       // Default GS settings
    GS_GM,                    // GS settings in GM mode (available on SC55mk2+)
    MT32                      // MT32 arrangement
  };

  Synth(ControlRom &cRom, WaveRom &pRom, SoundMap map = SoundMap::GS);
  ~Synth();

  // Add start() and stop()? Won't start if sampleRate is not set?

  // MIDI in.  frameOffset says how far in the future the event happens, in
  // output frames counted from the frame get_next_frame() will return next.
  // A client that plays a file or a sequence knows this and can hand events
  // over early; one that is fed a live MIDI stream leaves it at 0 and gets
  // what it always got. The synth applies a note-on at the exact sample it
  // falls on when it can, because the SC-55mkII does (PROVENANCE.md P-0229).
  // `port` selects one of the device's MIDI ports, for a device that has more
  // than one. The SC-88 has two and addresses parts 1-16 from the first and
  // 17-32 from the second; every other device here has one, and ignores it.
  // Roland's own SC-88 demo files state the port only in their track names, so
  // a front-end that drops it puts two instruments on one channel.
  void midi_input(uint8_t status, uint8_t data1, uint8_t data2,
                  uint32_t frameOffset = 0, uint8_t port = 0);
  void midi_input_sysex(uint8_t *data, uint16_t length,
                        uint32_t frameOffset = 0, uint8_t port = 0);

  int get_next_frame(float &lOut, float &rOut);
  // `frames` stereo frames, interleaved, into `out`: the same samples that
  // many get_next_frame() calls would return, every queued MIDI event still
  // applied at the frame it was queued for. A block lets the SC-88 and
  // JV-1080 engines run each stage over many frames per call instead of
  // one, and gives set_render_threads() something to divide.
  int get_next_frames(float *out, size_t frames);
  uint32_t get_num_clipped_samples(bool reset = true);
  std::array<int, 16> get_parts_last_peak_sample(void);

  // Setting audio properties (default is 44100, 2)
  void set_audio_format(uint32_t sampleRate, uint8_t channels);

  // Caps simultaneous voices below whatever the loaded device's own real
  // hardware ceiling is (24 SC-55, 28 SC-55mkII/JV-880, 64 SC-88), for a
  // host too slow to sustain that device's worst case; never raises it.
  // Which engine is actually running underneath (the SC-88's own XP
  // engine, or the shared Part/Note engine every other device uses) is
  // not the caller's concern - this applies to whichever one is loaded.
  // Applies immediately if the SC-88 engine is already open, and also on
  // every future set_audio_format() (which reopens it at the new rate),
  // so call order relative to set_audio_format() doesn't matter; the
  // Part/Note engine re-reads this on every note, so it takes effect on
  // the very next one regardless of when it's called.
  void set_max_voices(unsigned maxVoices);

  // How many threads the SC-88 and JV-1080 engines render their voices on,
  // the calling one included. 1, the default, renders on the calling thread
  // alone; 0 asks for one per CPU this process may run on. The output is the
  // same bytes at every count. It takes effect for blocks rendered with
  // get_next_frames(); get_next_frame() renders one frame at a time, which
  // leaves nothing to divide. No effect on the other devices, or in a build
  // without thread support (32-bit Windows), where every count means 1. Not
  // to be called while another thread is rendering.
  // workerCpus, on Linux, pins the pool's own threads one per listed CPU in
  // order; the calling thread is the caller's to place.
  void set_render_threads(unsigned threads,
                          const std::vector<unsigned> &workerCpus = {});
  // The threads actually in use, which may be fewer than were asked for.
  unsigned render_threads(void) const;

  void reset(SoundMap sm, bool resetParts = false);

  // Whether the loaded device has a GS mode: whether it receives the GS
  // Reset and the GS parameter map.
  bool has_gs_mode(void) const;
  // Whether the loaded device has a GM mode that differs from its GS one,
  // i.e. whether reset(SoundMap::GS_GM) leaves it anywhere a GS reset does
  // not.
  // Both ask the engine, which for the SC-88 and JV-1080 exists only from
  // set_audio_format() on: call them after that.
  bool has_gm_mode(void) const;

  // A mode lock keeps the device in one mode whatever SysEx it receives, for
  // a host that offers a GS-only or GM-only mode. Locked in GS, a GM System
  // On resets the device into GS. Locked in GM, a GS Reset resets it into
  // GM, and a parameter write cannot turn GM's receive switches back on. It
  // does not itself enter the mode: reset() does that. None, the default,
  // leaves the mode to the stream. A lock on a mode the device does not have
  // is meaningless. The XP engine has no SysEx path from one of its modes to
  // the other, so only the GP engine needs and reads it.
  enum class ModeLock { None, GS, GM };
  void set_mode_lock(ModeLock lock);
  ModeLock mode_lock(void) const { return _modeLock; }

  // Silences the device whatever it is and whatever it was sent: queued MIDI
  // is dropped, every voice of every part is cut, pedals are released and
  // controllers reset. Programs, patches and effects are kept. Not a MIDI
  // message - a device that would ignore All Sound Off is silenced too -
  // so a front-end has one way to stop every device.
  void panic(void);

  // How many MIDI ports this device answers on: 2 for the SC-88, 1 otherwise.
  int midi_ports(void) const;

  // Returns libEmuSC version as a string
  static std::string version(void);

  // Seeds the process-wide sequence behind random pan, random pitch, Analog
  // Feel and the random LFO waveforms. The constructor seeds it from the wall
  // clock; call this after constructing to make a render reproducible. The
  // same seed gives the same sequence on every platform.
  static void seed_random(uint32_t seed);

  void set_part_instrument(uint8_t partId, uint8_t index, uint8_t bank);

  void add_part_midi_mod_callback(std::function<void(const int)> callback);
  void clear_part_midi_mod_callback(void);

  void add_part_change_callback(std::function<void(const int)> callback);
  void clear_part_change_callback(void);

  void set_part_envelope_callback(int partId,
                                  std::function<void(const float, const float,
                                                     const float, const float,
                                                     const float, const float)> callback);
  void clear_part_envelope_callback(int partId);
  void set_part_lfo_callback(int partId,
                             std::function<void(const int, const int, const int)
                             > callback);
  void clear_part_lfo_callback(int partId);

  // EmuSC clients methods for getting synth paramters
  uint8_t  get_param(enum SystemParam sp);
  uint16_t get_param_32nib(enum SystemParam sp);
  uint8_t  get_param(enum PatchParam pp, int8_t part = -1);
  uint16_t get_param_uint14(enum PatchParam pp, int8_t part = -1);
  uint8_t  get_param_nib16(enum PatchParam pp, int8_t part = -1);
  uint8_t  get_patch_param(uint16_t address, int8_t part = -1);
  uint8_t  get_param(enum DrumParam, uint8_t map, uint8_t key);
  std::string get_drum_map_name(int map);

  // EmuSC clients methods for setting synth paramters
  void set_param(enum SystemParam sp, uint8_t value);
  void set_param(enum SystemParam sp, uint32_t value);
  void set_param(enum SystemParam sp, uint8_t *data, uint8_t size = 1);
  void set_param_32nib(enum SystemParam sp, uint16_t value);
  void set_param(enum PatchParam pp, uint8_t value, int8_t part = -1);
  void set_param(enum PatchParam sp, uint8_t *data, uint8_t size = 1,
		 int8_t part = -1);
  void set_param_uint14(enum PatchParam pp, uint16_t value, int8_t part = -1);
  void set_param_nib16(enum PatchParam pp, uint8_t value, int8_t part = -1);
  void set_patch_param(uint16_t address, uint8_t value, int8_t part = 1);
  void set_param(enum DrumParam dp, uint8_t map, uint8_t key, uint8_t value);
  void set_param(enum DrumParam dp, uint8_t map, uint8_t *data, uint8_t length);

  /* End of public API. Below are internal data structures only */

private:
  // JV-880 control channel (D-46). The bank flag CC0 leaves for the next
  // Performance select; it persists, as the firmware's does.
  int _perfBankFlag = 0;
  bool _jv_control_channel(uint8_t channel) const;

private:
  Settings *_settings = nullptr;

  ModeLock _modeLock = ModeLock::None;

  // The SC-88 is not rendered by the Part and Note path: its engine reads the
  // control ROM itself and renders a stereo frame at a time off its own
  // scheduler clock. When the loaded ROM is an SC-88 this holds that engine and
  // every audio and MIDI call is forwarded to it. Null for every other device.
  Xp::Device *_xpDevice = nullptr;

  bool _xp_configure(uint32_t sampleRate);

  // set_max_voices()'s stored request, applied to whichever engine is
  // actually loaded: the XP engine's own free-slot list (_xp_configure,
  // on every (re)open) or _add_note()'s maxPolyphony clamp, whichever is
  // relevant. Not SC-88-specific despite _xp_configure being the one
  // place besides set_max_voices() itself that reads it.
  bool _maxVoicesSet = false;
  unsigned _maxVoices = 0;

  // set_render_threads()'s pool, kept across the XP engine's reopening by
  // set_audio_format(); null while one thread is asked for.
  Xp::WorkerPool *_renderPool = nullptr;
  
  uint32_t _sampleRate;
  uint8_t _channels;

  std::atomic<uint32_t> _numClippedSamples;

  SimpleMutex midiMutex;

  struct std::vector<Gp::Part> _parts;
  uint32_t _noteSerial = 0;   // Note on order, for voice allocation
  std::vector<std::function<void(const int)>> _partMidiModCallbacks;
  std::vector<std::function<void(const int)>> _partChangeCallbacks;

  ControlRom &_ctrlRom;
  WaveRom &_waveRom;

  float _phase;               // Fractional SC-55 sample position
  float _phaseIncrement;      // SC-55 samples per host sample
  int   _updateCounter;       // Counts SC-55 samples, fires at 256

  // Event scheduling, on the internal 32 kHz timeline.  _blockStart is the
  // first sample of the control period about to be generated and
  // _framesDelivered counts the output frames get_next_frame() has returned,
  // which is what a client's frameOffset is measured from.
  uint64_t _blockStart;
  uint64_t _framesDelivered;

  // The input is read at one MIDI byte per 100 us, so a message whose bytes
  // arrive while an earlier message is still being read waits for it. This is
  // the internal sample at which the input becomes free again.
  double _midiInputFree;

  struct PendingEvent {
    uint64_t applyAt;         // internal sample this event acts on
    bool     isSysEx;
    int      startDelay;      // sub-period offset, note-on only
    uint8_t  status, data1, data2;
    uint8_t  port;            // which MIDI port it arrived on
    std::vector<uint8_t> sysex;
  };
  std::deque<PendingEvent> _eventQueue;

  // MIDI bytes per second on the way in, and the fixed delay from the last
  // byte of a note-on to the first sample of the voice it starts, in internal
  // samples. Both measured on the SC-55mkII (PROVENANCE.md P-0229).
  static constexpr double midiByteSamples = 32000.0 * 0.0001;   // 100 us
  static constexpr double noteOnDelaySamples = 32000.0 * 0.0020; // 2.0 ms

  std::vector<float> _hostSampleBufL;
  std::vector<float> _hostSampleBufR;
  int _hostSampleBufRIndex;
  int _hostSampleBufWIndex;

  std::array<std::array<float, 256>, 2> _dryBus;

  // Mono: each part contributes its signal from before its own panner
  // (PROVENANCE.md P-0182), and each effect takes one input sample per frame.
  std::array<float, 256> _chorusBus;
  std::array<float, 256> _reverbBus;

  std::array<std::array<float, 256>, 2> _chorusOut;
  std::array<std::array<float, 256>, 2> _reverbOut;

  Gp::SystemEffects *_systemEffects = nullptr;
  Gp::Resampler *_resampler = nullptr;

  // The device's analog board, after the converter. See analog_stage.h: the
  // single place for what a module does to the finished mix that its chip did
  // not, and a no-op for every device whose board has not been measured.
  Gp::AnalogStage *_analogStage = nullptr;

  // MIDI message types
  static const uint8_t midi_NoteOff         = 0x80;
  static const uint8_t midi_NoteOn          = 0x90;
  static const uint8_t midi_PolyKeyPressure = 0xa0;
  static const uint8_t midi_CtrlChange      = 0xb0;
  static const uint8_t midi_PrgChange       = 0xc0;
  static const uint8_t midi_ChPressure      = 0xd0;
  static const uint8_t midi_PitchBend       = 0xe0;

  void _init_parts(void);
// int _export_sample_24(std::vector<int32_t> &sampleSet, std::string filename);
  void _add_note(uint8_t midiChannel, uint8_t key, uint8_t velocity,
                 int startDelay);

  // Put an event on the queue with the internal sample it acts on, and take
  // the queue's head events off again when the period they belong to is
  // generated.
  void _queue_event(uint8_t status, uint8_t data1, uint8_t data2,
                    const uint8_t *sysex, uint16_t sysexLength,
                    uint32_t frameOffset);
  void _dispatch_events(void);
  void _apply_midi(uint8_t status, uint8_t data1, uint8_t data2,
                   int startDelay);
  void _apply_midi_sysex(uint8_t *data, uint16_t length);
  int _partials_in_use(void);
  int _steal_partials(Gp::Part &requester);
  int _steal_partial_jv(Gp::Part &requester);

  void _midi_input_sysex_DT1(uint8_t model, uint8_t *data, uint16_t length);

  void _process_samples(void);

  Synth();
};

}

#endif  // __SYNTH_H__
