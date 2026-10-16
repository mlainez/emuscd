/* SPDX-License-Identifier: CC0-1.0 */
#ifndef EMUSC_XP_DEVICE_H
#define EMUSC_XP_DEVICE_H

#include "chorus.h"
#include "delay.h"
#include "eq.h"
#include "output.h"
#include "engine.h"
#include "reverb.h"
#include "devices/sc88.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* The controller destination matrix at `40 2x ss`, in the order
 * `04_protocol/sysex.md` prints: six source groups of eleven destinations.
 * The wire address is `group * 0x10 + destination`; the matrix's own part
 * structure holds the same six groups twelve bytes apart at +0x28, with a
 * byte at +3 this routine does not read (`05_data_model/part_state.md`). */
enum xp_matrix_source {
  XP_MATRIX_MODULATION = 0,
  XP_MATRIX_PITCH_BEND = 1,
  XP_MATRIX_CHANNEL_PRESSURE = 2,
  XP_MATRIX_POLY_PRESSURE = 3,
  XP_MATRIX_CC1 = 4,
  XP_MATRIX_CC2 = 5
};

enum xp_matrix_destination {
  XP_MATRIX_PITCH = 0,
  XP_MATRIX_CUTOFF = 1,
  XP_MATRIX_AMPLITUDE = 2,
  XP_MATRIX_LFO1_RATE = 3,
  XP_MATRIX_LFO1_PITCH_DEPTH = 4,
  XP_MATRIX_LFO1_TVF_DEPTH = 5,
  XP_MATRIX_LFO1_TVA_DEPTH = 6,
  XP_MATRIX_LFO2_RATE = 7,
  XP_MATRIX_LFO2_PITCH_DEPTH = 8,
  XP_MATRIX_LFO2_TVF_DEPTH = 9,
  XP_MATRIX_LFO2_TVA_DEPTH = 10
};

#ifdef __cplusplus
}

namespace EmuSC { namespace Xp {

// Top-level MIDI device (lifecycle, SysEx/MIDI protocol decoding, the
// controller-to-engine parameter-sync bridge, and effects-chain ownership)
// for the XP-generation-1 engine (see engines/xp/README.md).
//
// Unlike every xp_-prefixed struct elsewhere in engines/xp/ - those are
// SC-88 firmware-exact RAM and register layouts, reverse-engineered from
// SC-88's own disassembled CPU program, and stay plain C structs shared
// with not-yet-converted callers - ChannelState and Device below are this
// emulator's own bookkeeping: a received-MIDI-controller cache and a
// top-level orchestration shell, neither one a firmware RAM layout. That
// is what makes them safe to move into this namespace as real member
// state: synth.cc has only ever held a Device through an opaque pointer,
// and device_test.cc (the one caller that reads their fields directly) is
// C++ itself. Every other engines/xp/ struct (tva/tvf/renderer/engine
// internals) is untouched and keeps its extern "C" compatibility surface,
// because xp_*_test.c files besides device_test.cc still read those
// directly and are still plain C.
//
// Both remain plain aggregates with public fields rather than encapsulated
// classes - device_test.cc still reads them directly, the same reason
// every other struct in this tree stays a struct - so this move is a
// namespace and naming change, not an encapsulation one. A MidiDecoder/
// EffectsBus/Device split (protocol decoding is plausibly generic XP
// SysEx/MIDI shape, shared with a future JV-1080; the parameter-sync
// bridge, GS controller state -> engine registers, is plausibly
// SC-88-specific) stays the natural next step, visible in this file's own
// section comments, whenever that direct field access is retired too.

struct ChannelState {
  uint8_t variation;
  /* The two halves of the part's bank word at `d820 + 2*part`, which is
     what the tone map is: `tone_map_forced` is the byte CC32 and
     `40 4x 00` both write, 0 to defer to the part's own map and 1 or 2 to
     force the SC-55 or SC-88 one, and `tone_map_selected` is that own map,
     which only `40 4x 01` writes. `2d3e` and `2e7a` resolve the pair
     identically for melodic and rhythm parts: the forced byte when it is
     nonzero, the selected map otherwise.
     Their reset behaviour differs, which is why they are held apart: GS
     Reset clears the forced byte, while the selected map survives it, GM
     On, System Mode Set and power-on with the printed default of 02,
     SC-88 (SC88-OM printed 7-31). */
  uint8_t tone_map_forced;
  uint8_t tone_map_selected;
  uint8_t program;
  uint8_t volume;
  uint8_t expression;
  uint8_t pan;
  uint8_t hold1;
  uint8_t reverb_send;
  uint8_t chorus_send;
  uint8_t delay_send;
  uint8_t cutoff;
  uint8_t resonance;
  /* The envelope-time modifiers, centred at 64. Demo song 1 sets the
     attack twenty-two times, so a part that never receives them cannot
     sound like the recording. */
  uint8_t attack;
  uint8_t decay;
  uint8_t release;
  /* The five part-level sources the controller destination matrix reads,
     at DP:d6a1, d660, d6a0, d760 and d761 (`05_data_model/part_state.md`).
     Channel pressure is held here; poly pressure is per key and is not
     received yet. */
  uint8_t modulation;
  uint8_t channel_pressure;
  /* Part key shift, 0x28..0x58 about 0x40, so +/-24 semitones. Applied
     to the pitch rather than to the note number, which leaves zone and
     kit selection alone. TOXOPLASMA shifts a part down an octave. */
  uint8_t key_shift;
  /* The two assignable controllers and the value each carries. Their
     defaults are CC16 and CC17, and every one of their matrix depths
     is zero after a reset, so an assigned controller is received and
     changes nothing until a song sets a depth. */
  uint8_t cc1_assign, cc2_assign;
  uint8_t cc1_value, cc2_value;
  /* Received and held. Mono mode and portamento need voice-lifecycle
     work this does not do yet; holding them keeps a song's request
     from being counted as unimplemented when it is only unapplied. */
  uint8_t mono_mode;
  uint8_t vibrato_rate, vibrato_depth, vibrato_delay;
  uint8_t portamento_time, portamento_switch, portamento_control;
  uint16_t pitch_bend;
  uint8_t pitch_bend_sensitivity;
  uint8_t rpn_msb;
  uint8_t rpn_lsb;
  uint8_t nrpn_msb;
  uint8_t nrpn_lsb;
  /* The controller destination matrix, `40 2x ss`: six source groups in
     the published order, eleven depths each. Only the cutoff column is
     applied so far; the rest are held so a song's settings are not lost,
     and so the one consumer that does exist - modulation to LFO1 pitch
     depth - reads the same bytes every other destination will. */
  uint8_t matrix_depth[XP_MATRIX_SOURCE_COUNT][XP_MATRIX_DEST_COUNT];
  enum xp_same_note_mode same_note_mode;
};

struct Device {
  uint8_t *control_rom;
  uint8_t *decoded_chips[XP_WAVE_CHIP_COUNT];
  struct xp_wave_bank banks[XP_WAVE_BANK_COUNT];
  struct xp_renderer renderer;
  struct xp_engine engine;
  struct xp_reverb reverb;
  struct xp_chorus chorus;
  struct xp_delay delay;
  struct xp_eq eq;
  struct xp_output output;
  /* The reverb's own parameters. Writing the macro reloads all of them from
     a ROM preset, and a GS reset is the same read with macro 4. */
  uint8_t reverb_character, reverb_level, reverb_time, reverb_pre_lpf;
  uint8_t reverb_predelay, reverb_delay_feedback;
  uint8_t reverb_macro;
  /* The chorus parameters are received and held but not yet rendered: the
     CPU-side transforms are recovered while the DSP's audio algorithm is
     not (`08_effects/chorus.md`), so a chorus here would be invented
     rather than modelled. Holding them keeps a song's settings from being
     silently discarded. */
  uint8_t chorus_macro, chorus_level, chorus_feedback, chorus_delay;
  uint8_t chorus_rate, chorus_depth, chorus_pre_lpf, chorus_send_to_reverb;
  /* 0 single module, 1 double. Several parameters exist only in one mode. */
  /* The ten delay parameters in manual order, and the macro that last
     loaded them. Single-module only. */
  uint8_t delay_params[10];
  uint8_t delay_macro;
  /* The output equaliser, on after a reset. Its four wire bytes are the
     band corners and gains; at the centre gain it is an identity. */
  uint8_t eq_low_frequency, eq_low_gain;
  uint8_t eq_high_frequency, eq_high_gain;
  uint8_t system_mode;
  double output_rate;
  /* The filter's cutoff is a fraction of the sound chip's own 32 kHz,
     so the coefficient has to be recomputed for whatever rate the
     caller renders at; the transfer function is handed this table of
     coefficients at output_rate. */
  struct xp_tvf_coefficients tvf_coefficients;
  /* SysEx writes that parsed correctly but name an address this
     implementation does not act on. Counted rather than dropped quietly,
     so a render can say what it ignored. */
  unsigned long unhandled_sysex;
  /* The output is AC-coupled, as every analogue audio output is. The
     decoder integrates differences, so a sample whose deltas do not
     sum to zero leaves a standing offset, and a sustained note holds
     it: song 1 measured 0.157 of DC against the hardware's 0.001, in
     65 % of its length, and that offset is what pushed the mix into
     clipping. One pole at 10 Hz, which is -0.3 dB by 40 Hz. */
  float *send_bus;
  float *chorus_bus;
  float *delay_bus;
  size_t send_capacity;
  struct ChannelState channels[XP_ENGINE_PART_COUNT];
  uint8_t master_volume;
  uint8_t secondary_level;
  uint8_t master_pan;
  /* How many times a part's pan was substituted because it asked for GS
     random pan, which needs a sound-chip random source that is not
     recovered. Counted rather than hidden: it is a labelled divergence, and
     a render reporting zero is exact in this respect. */
  /* Parts that asked for random pan; the engine draws each voice. */
  unsigned long random_pan_requests;
  /* A device whose voice path is its own (XpDeviceProfile::voiceEngine)
     keeps its state here and the members above are unused: the engine,
     renderer and effects chain in this struct are the shared firmware
     port's, and a device with no dumped firmware has no use for them.
     Null on a device the port serves, which is what selects it. */
  const struct XpVoiceEngineOps *voice_ops;
  void *voice_state;
  bool initialized;
};

bool device_init_raw(Device *device, const uint8_t *controlRom,
                      size_t controlRomSize,
                      const uint8_t *const rawChips[XP_WAVE_CHIP_COUNT],
                      const size_t rawSizes[XP_WAVE_CHIP_COUNT],
                      double outputRate, enum xp_fractional_wrap wrap);
bool device_init_decoded(
  Device *device, const uint8_t *controlRom, size_t controlRomSize,
  const uint8_t *const decodedChips[XP_WAVE_CHIP_COUNT],
  const size_t decodedSizes[XP_WAVE_CHIP_COUNT], double outputRate,
  enum xp_fractional_wrap wrap);
void device_destroy(Device *device);
void device_reset_controllers(Device *device);
/* The GM System On a device with its own voice path acts on, as if the
 * message had arrived. False, with nothing changed, on a device that has
 * no GM mode - which includes every device on the shared firmware port. */
bool device_gm_system_on(Device *device);
/* True on a device whose profile has a host reset enter GM mode; see
 * XpDeviceProfile::hostResetEntersGm. */
bool device_host_reset_enters_gm(const Device *device);
/* Caps simultaneous voices below the device's own polyphony. Dispatches
 * to whichever voice path the loaded device uses, so a caller does not
 * have to know which one that is. */
bool device_set_max_voices(Device *device, unsigned maxVoices);
/* Renders the voices on `pool`'s threads from the next block on, whichever
 * voice path the device uses; null, or a one-thread pool, renders them on
 * the calling thread. The output is the same bytes either way. False, with
 * the device left on the calling thread, if its voice path cannot take the
 * pool. The pool is not owned and must outlive the device or be detached
 * first. */
bool device_set_worker_pool(Device *device, WorkerPool *pool);
void device_set_master_volume(Device *device, uint8_t value);
void device_set_master_pan(Device *device, uint8_t value);

/* One System Exclusive message, with or without its leading `f0` and
 * trailing `f7`. Returns false for a message that is not a well-formed GS
 * DT1 for this device - a wrong manufacturer or model, a bad checksum, a
 * truncated packet - and true once the packet has been applied, whether or
 * not every address in it was one this implementation acts on. */
bool device_sysex(Device *device, uint8_t port, const uint8_t *data,
                   size_t size);

/* MIDI port 0 addresses parts 0..15 and port 1 addresses parts 16..31.
 * Supported channel messages: note on/off, pitch bend, program change,
 * CC0/6/7/10/11/32/64/66/98..101/121, RPN 00/00 bend sensitivity, and
 * NRPN 01/20..21 cutoff/resonance. Unsupported messages return false
 * without changing state. */
bool device_midi(Device *device, uint8_t port, uint8_t status,
                  uint8_t data1, uint8_t data2);

/* One part's cached controller-matrix cutoff word, the value the firmware
 * holds at DP:1c34 + part. Exposed so the arithmetic can be checked on its
 * own; the device recomposes it whenever one of its inputs moves. */
int16_t device_matrix_cutoff_word(const ChannelState *channel);

void device_render(Device *device, float *stereo, size_t frames);

}}  // namespace EmuSC::Xp
#endif

#endif
