/* SPDX-License-Identifier: CC0-1.0 */
#ifndef EMUSC_XP_TVF_H
#define EMUSC_XP_TVF_H

#include "devices/sc88.h"
#include "rom.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
#include <atomic>

extern "C" {
#endif

struct xp_tvf_controls {
  uint8_t part_cutoff;
  uint8_t secondary_cutoff;
  uint8_t part_resonance;
  uint8_t secondary_resonance;
  /* The controller destination matrix's cached cutoff word, held at
     DP:1c34 + part and composed by SC88-CTL 0x11871..0x11982 from
     modulation, pitch bend, channel pressure and the two assignable
     controllers (`04_protocol/controllers.md`). It is not an index term
     like part_cutoff above: 0x6ad0..0x6aff clamps it, scales it and
     halves it into the word accumulated at RAM 30da, so its units are
     the cutoff word's own. Zero when every depth is at its reset 0x40. */
  int16_t matrix_cutoff;
};

/* Exact CPU-prepared XP register state. The physical cutoff, resonance and
 * filter transfer represented by these words remain deliberately unnamed. */
struct xp_tvf_registers {
  uint8_t cutoff_index;
  uint8_t resonance_index;
  uint16_t base_value;
  /* The saturated table entry plus key, controller and LFO terms as
     routine 6ccd holds it before the shift right one. The envelope and
     release are not part of it: the firmware adds those to the halved
     word (tvf_update_frequency). */
  uint16_t base_unshifted;
  uint16_t combined;
  uint32_t frequency_current;
  uint32_t frequency_target;
  uint16_t frequency_interpolation;
  uint32_t resonance_current;
  uint32_t resonance_target;
  uint16_t resonance_interpolation;
  uint16_t filter_select;
  bool fixed_tuple;
};

struct xp_tvf_envelope {
  int16_t targets[4];
  uint16_t initial_phases[4];
  uint16_t increments[4];
  uint16_t depth;
  uint8_t stage;
  uint8_t saved_count;
  uint16_t phase;
  int16_t base;
  int16_t delta;
  int16_t current;
  bool active;
};

struct xp_tvf_release {
  int16_t target;
  int16_t current;
  uint16_t phase;
  uint16_t increment;
  uint16_t scale;
  bool scale_enabled;
  bool active;
};

/* Replaceable audio-side interpretation of the still-undecoded XP words.
 * This state-variable topology is intentionally separate from the exact CPU
 * state above. */
struct xp_tvf_audio_state {
  float integrator_band;
  float integrator_low;
  float section_band[XP_TVF_SECTIONS];
  float section_low[XP_TVF_SECTIONS];
  /* g's own memo: a pure function of the rounded word (see
     tvf_audio_process_provisional), so a repeat word - the common case,
     since the register glides in steps far coarser than one word per
     sample - reuses it without touching the shared coefficient table.
     tvf_audio_reset's memset leaves memo_valid false, which is correct:
     word 0 is a real, distinct key, not "no memo yet". */
  uint32_t memo_word;
  double memo_g;
  bool memo_valid;
  /* section 0's coefficient-clamp bound, similarly memoized: a pure
     function of resonance_current, which (07_synthesis/tvf.md) does not
     move after note-on for the vast majority of components - see the
     note in tvf_audio_process_provisional. */
  uint32_t memo_resonance;
  float memo_bound;
  bool bound_valid;
};

typedef float (*xp_tvf_audio_transfer_fn)(
  void *user, struct xp_tvf_audio_state *state,
  const struct xp_tvf_registers *registers,
  double period_fraction, float input);

#ifdef __cplusplus
}

namespace EmuSC { namespace Xp {

// TVF (cutoff/resonance control law, and the provisional audio-side state-
// variable filter) for the XP-generation-1 engine (see engines/xp/README.md).
// The plain C types above are shared, unrenamed, with EmuSC::Xp::Device
// (device.h), which embeds the per-voice component holding them by value,
// and with xp_tvf_test.c, which reads them directly.
//
// The control-law functions below (prepare_registers through
// advance_registers) are [FW-EXACT]: they reproduce the firmware's own
// integer arithmetic bit for bit. audio_reset/audio_process_provisional are
// not - they are a measured model of the still-undecoded XP audio path, and
// the "_provisional" name is load-bearing, not decorative. That split is
// kept visible in the ordering of this file, not in a second file pair: the
// audio state is a plain struct embedded by value in the per-voice
// component, which is itself embedded by value in EmuSC::Xp::Device for
// device_test.cc's sake, so it cannot yet hold an EmuSC::SVF - the
// topology this filter shares with the SC-55 path's svf.h - directly.
// Replacing this section with an EmuSC::SVF member is the natural
// follow-up whenever that constraint lifts.

/* pre_base_modulation is the wrapped key/dynamic word prepared before the
 * base-table lookup, carrying the two LFO filter terms alongside the key
 * term; the controller matrix's cutoff term is added to it from
 * controls->matrix_cutoff. Envelope and release modulation are added
 * afterward by update_frequency. */
bool tvf_prepare_registers(const struct xp_rom *rom,
                            const struct xp_component *component,
                            int16_t preBaseModulation,
                            const struct xp_tvf_controls *controls,
                            struct xp_tvf_registers *registers);

/* The controller matrix's cached cutoff word, as routine 0x6ad0..0x6aff
 * turns it into a term of the pre-base accumulator: clamped to
 * -4000..+4000, shifted left three, multiplied by 0x8312 keeping the
 * signed high word, and halved. Full scale is 8191 word units, two
 * octaves at the base table's 4096-per-octave word. */
int16_t tvf_matrix_cutoff_term(int16_t cached);

/* One oscillator's filter term, as routine 0x6b7a..0x6bd7 turns its
 * already-faded depth word and its waveform word into a term of the same
 * pre-base accumulator: clamped to -4032..+4032, shifted left three,
 * multiplied by 0x8208 keeping the signed high word, halved, and
 * multiplied by the waveform the firmware's own way. Full scale is 4095
 * word units, one octave at the base table's 4096-per-octave word. */
int16_t tvf_lfo_filter_term(int16_t fadedDepth, int16_t waveform);

/* Signed key-table word times signed component factor, retaining the product
 * high word and applying the firmware's final doubling. */
bool tvf_key_modulation(const struct xp_rom *rom, const struct xp_tone *tone,
                         const struct xp_component *component,
                         uint8_t selectorKey, int16_t *modulation);

/* Exact note-on depth, key/velocity rate scaling, targets and CPU clock.
 * Attack/decay controller modifiers are neutral; softPedal selects the
 * recovered velocity reduction before the depth curve lookup. */
bool tvf_envelope_prepare(const struct xp_rom *rom, const struct xp_tone *tone,
                           const struct xp_component *component,
                           uint8_t selectorKey, uint8_t velocity, bool softPedal,
                           struct xp_tvf_envelope *envelope);
bool tvf_envelope_advance(struct xp_tvf_envelope *envelope,
                           unsigned elapsedPeriods);
bool tvf_release_prepare(const struct xp_rom *rom, const struct xp_tone *tone,
                          const struct xp_component *component,
                          uint8_t selectorKey, uint16_t envelopeDepth,
                          struct xp_tvf_release *release);
bool tvf_release_set_pedal(const struct xp_rom *rom, uint8_t hold1,
                            bool continuousHold, bool keepScaleAtZero,
                            bool sostenutoRetained,
                            struct xp_tvf_release *release);
bool tvf_release_advance(struct xp_tvf_release *release,
                          unsigned elapsedPeriods);

/* Recompose TVF-F from an already prepared accumulator while retaining the
 * note-start Q/type tuple. Fixed negative-mode tuples remain unchanged. */
bool tvf_update_frequency(const struct xp_rom *rom,
                           int16_t postBaseModulation,
                           struct xp_tvf_registers *registers);
void tvf_latch_frequency(struct xp_tvf_registers *registers);

/* Move the frequency register toward its target by its own interpolation
 * word (`0x4100`, written beside the target at `683f`), for the elapsed
 * control periods. Resonance does not move; see tvf.cc. */
void tvf_advance_registers(struct xp_tvf_registers *registers,
                            unsigned periods);

/* The frequency a TVF-F register value asks for, in hertz at the chip's
 * own rate. Slope from the ROM, anchor inferred - see tvf.cc. */
double tvf_word_to_hz(uint32_t word);

/* The filter coefficient g for every TVF-F word below XP_TVF_NYQUIST_WORD
 * at one host rate. Every word a voice can reach is in that range: the
 * register is a 16-bit limit-clamped value, at most 0x7fff, shifted left
 * three. g is a pure function of the word and the rate, so an entry is
 * computed by the same code as an uncached coefficient the first time any
 * voice asks for it and read back from then on - the same value, not an
 * approximation of it. 0 marks an entry not yet computed; no word yields a
 * g of 0. One table per Device, since the rate is fixed per Device.
 *
 * Voices on different render threads read and fill it at once. An entry's
 * value depends on nothing but its word and the rate, so two threads that
 * both find it empty compute and store the same float; the entries are
 * atomic, with relaxed ordering, only so that neither read nor write can
 * tear - a plain load or store of an aligned float on every target. */
struct xp_tvf_coefficients {
  double rate;
  std::atomic<float> *g;
};

/* Allocates the XP_TVF_NYQUIST_WORD-entry table (1 MiB). */
bool tvf_coefficients_init(struct xp_tvf_coefficients *coefficients,
                           double rate);
void tvf_coefficients_destroy(struct xp_tvf_coefficients *coefficients);

void tvf_audio_reset(struct xp_tvf_audio_state *state);
/* user is the Device's struct xp_tvf_coefficients, or NULL for the chip's
 * own rate with nothing cached across voices. */
float tvf_audio_process_provisional(void *user, struct xp_tvf_audio_state *state,
                                     const struct xp_tvf_registers *registers,
                                     double periodFraction, float input);

}}  // namespace EmuSC::Xp
#endif

#endif
