/* SPDX-License-Identifier: CC0-1.0 */
#include "tvf.h"

#include "common/constants.h"
#include "devices/sc88.h"
#include "tva.h"
#include "../common/dsp_kernels.h"

#include <cmath>
#include <cstring>
#include <climits>
#include <cstdlib>
#include <new>

namespace EmuSC { namespace Xp {

namespace {

uint16_t be16(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

int8_t s8(uint8_t value)
{
  return value <= INT8_MAX
    ? (int8_t)value
    : (int8_t)(-1 - (int16_t)(UINT8_MAX - value));
}

int16_t s16(uint16_t value)
{
  return value <= INT16_MAX
    ? (int16_t)value
    : (int16_t)(-1 - (int32_t)(UINT16_MAX - value));
}

int32_t floorDivPow2(int32_t value, unsigned shift)
{
  if (value >= 0)
    return value / (INT32_C(1) << shift);
  return -(int32_t)(((uint32_t)(-value) +
    ((UINT32_C(1) << shift) - 1)) >> shift);
}

int clampIndex(int value)
{
  if (value < 0)
    return 0;
  if (value > 127)
    return 127;
  return value;
}

/* `0x6bb3`..`0x6bd3` and `0x6c97`..`0x6cb7`. The firmware multiplies by
 * magnitude and keeps the unsigned product's high word. Where exactly
 * one operand is negative it negates that word and then subtracts the
 * borrow left by restoring the register it negated, so the result is one
 * below the magnitude's high word negated - floor, except where the
 * product is an exact multiple of 65536. Restoring a zero leaves no
 * borrow, so a zero waveform gives a zero term rather than -1. */
int32_t signedProductHigh(int32_t left, int16_t right)
{
  uint32_t magnitude =
    (uint32_t)(left < 0 ? -left : left) *
    (uint32_t)(right < 0 ? -(int32_t)right : (int32_t)right);

  if ((left < 0) == (right < 0))
    return (int32_t)(magnitude >> 16);
  if (right == 0)
    return 0;
  return -(int32_t)(magnitude >> 16) - 1;
}

bool keyRateScale(const struct xp_rom *rom, const struct xp_tone *tone,
                   const struct xp_component *component,
                   uint8_t selectorKey, uint16_t pointerAt, uint8_t factorAt,
                   uint16_t *scale)
{
  if (!rom || !rom->bytes || !tone || !tone->common || !component ||
      !component->bytes || !scale || selectorKey > 127)
    return false;
  const struct XpDeviceProfile *profile = xp_profile(rom);
  uint32_t curve = ((uint32_t)tone->common[0x21] << 16) |
    be16(component->bytes + pointerAt);
  if (curve + selectorKey >= rom->size ||
      profile->rateScaleTable + 129u * 2 > rom->size)
    return false;
  int keyValue = s8(rom->bytes[curve + selectorKey]);
  int factor = s8((uint8_t)(0u - component->bytes[factorAt]));
  int index = floorDivPow2(keyValue * factor, 8) + 64;
  if (index < 0 || index > 128)
    return false;
  *scale = be16(rom->bytes + profile->rateScaleTable + (uint32_t)index * 2);
  return true;
}

bool velocityRateScale(const struct xp_rom *rom, uint8_t velocity,
                        int factor, uint16_t *scale)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  if (!rom || !rom->bytes || !scale || velocity > 127 ||
      factor < -128 || factor > 127 ||
      profile->rateScaleTable + 129u * 2 > rom->size)
    return false;
  int index = floorDivPow2((2 * ((int)velocity - 64)) * factor, 8) + 64;
  if (index < 0 || index > 128)
    return false;
  *scale = be16(rom->bytes + profile->rateScaleTable + (uint32_t)index * 2);
  return true;
}

bool envelopeDepth(const struct xp_rom *rom, const struct xp_tone *tone,
                    const struct xp_component *component, uint8_t velocity,
                    bool softPedal, uint16_t *depth)
{
  if (!rom || !rom->bytes || !tone || !tone->common || !component ||
      !component->bytes || !depth || velocity > 127)
    return false;
  uint16_t input = be16(component->bytes + 0x48);
  if (input == 0) {
    *depth = 0;
    return true;
  }
  if (softPedal)
    velocity = (uint8_t)(((uint32_t)velocity * UINT16_C(0xb76f)) >> 16);
  uint32_t curve = ((uint32_t)tone->common[0x21] << 16) |
    be16(component->bytes + 0x3a);
  if (curve + velocity >= rom->size)
    return false;
  unsigned curveValue = rom->bytes[curve + velocity];
  int factor = s16(be16(component->bytes + 0x60));
  uint32_t magnitude;
  if (factor < 0) {
    magnitude = (uint32_t)(-factor);
    curveValue = (unsigned)(uint8_t)(0u - curveValue) & 0x7fu;
  } else {
    magnitude = (uint32_t)factor;
  }
  curveValue = (unsigned)(uint8_t)(~curveValue) & 0x7fu;
  uint16_t complement = (uint16_t)~(uint16_t)(magnitude * curveValue);
  *depth = (uint16_t)(((uint32_t)input * complement) >> 16);
  return true;
}

void prepareIncrement(uint16_t tableRate, uint16_t scale, uint16_t *phase,
                       uint16_t *increment)
{
  if (tableRate < 16)
    tableRate = UINT16_MAX;
  uint32_t product = (uint32_t)tableRate * scale;
  if (product >= UINT32_C(0x01000000)) {
    *phase = UINT16_MAX;
    *increment = UINT16_MAX;
  } else {
    *phase = 0;
    *increment = (uint16_t)(product >> 8);
  }
}

int16_t scaleTarget(int16_t target, uint16_t depth)
{
  return s16((uint16_t)floorDivPow2((int32_t)target * depth, 16));
}

/* `67a8` writes the TVF-F target and current value at scratch `1a7c`, and
 * the interpolation pair beside them; `683f: ea 2a 07 41 00` puts
 * `#0x4100` in the low word, which `tva_curve_decode` reads as the LINEAR
 * family at value 256 and `q = 4 * periods` - see tva.cc. */
double frequencyProgress(const struct xp_tvf_registers *registers,
                          double periods)
{
  /* frequency_interpolation has exactly one writer, a few lines below in
     tvf_prepare_registers: always the literal 0x4100 (also asserted by
     xp_rom_test.c, xp_tvf_test.c and xp_renderer_test.c). Decoding
     that fixed word is always the linear family at rate 256/64 = 4.0
     exactly - a power-of-two divisor, so this is that double, not an
     approximation of it - which is what tva_curve_decode/_progress
     reduce to below. Called once per voice per output sample, this skips
     redoing that decode from scratch every time. */
  const double q = 4.0 * periods;
  return q >= 1.0 ? 1.0 : q;
}

}  // namespace

/* The word is the log of sin(pi * f / fs), not the log of f.

   Both machines hold the same cutoff table and the SC-55 holds it in the
   clear. The SC-55 mk1 CPU ROM at 0x7612 is 32768 * sin(pi * f / 32000)
   for f = 440 * 2^((index - 64)/12): fitted over its 116 pre-saturation
   entries the residual is 0.435 LSB rms and the anchor lands on 440.0 Hz
   at index 64 - one semitone per index, saturating where the sine folds.

   The SC-88's base table at 0x78702 is that same quantity in the XP pitch
   register's log domain and decodes to the bit; see the note on
   XP_TVF_NYQUIST_WORD. Reading the word as a log FREQUENCY instead leaves an sd
   of 2072 word units and up to 0.67 octave, because it has no account of
   why the table's steps shrink from 1368 to 0 over its last twelve
   entries. That compression is the sine approaching one; the frequency
   underneath it is a clean note table.

   Hardware agrees, on two stimuli each with its own control and with the
   corner fitted through the realisation the chip has rather than an
   analog prototype. Hardware corner over computed cutoff is 0.97
   (quartiles 0.90/1.10, n = 13) measured against our own --no-filter
   render, and 1.04 (0.99/1.25, n = 9) on a self-differential of two
   instants inside one held note, which carries no render of ours at all.
   Their controls - the same fit on our own output, whose corner is at
   the computed cutoff by construction - read 1.05 and 1.00, so the
   method's own bias is the size of the disagreement.

   0x40000 still means the top of the range and still means fs/2; what
   runs exponentially between is sin(pi * f / fs). Below about 4 kHz the
   two readings differ by exactly pi/2 - 0.651 octave - converging at the
   top. */
double tvf_word_to_hz(uint32_t word)
{
  double sine = std::exp2(((double)word - XP_TVF_NYQUIST_WORD) / kXpTvfOctaveUnits);

  if (sine >= 1.0)
    return 0.5 * kXpNativeRate;
  return (kXpNativeRate / 3.14159265358979323846) * std::asin(sine);
}

namespace {

/* The coefficient the ROM's word already is. At the chip's own rate this
   is 2 * exp2((word - 0x40000)/16384) exactly - the sine is taken out of
   the word by tvf_word_to_hz and put straight back - and at any other host
   rate it is the same analog corner retuned to that rate, which is the
   only part of this that is a choice. */
double coefficientForWord(uint32_t word, double rate)
{
  double cutoff = tvf_word_to_hz(word);
  double nyquist = rate * 0.5;
  if (cutoff > nyquist * 0.99)
    cutoff = nyquist * 0.99;
  return 2.0 * std::sin(3.14159265358979323846 * cutoff / rate);
}

}  // namespace

bool tvf_coefficients_init(struct xp_tvf_coefficients *coefficients,
                           double rate)
{
  if (!coefficients)
    return false;
  coefficients->rate = rate;
  coefficients->g = new (std::nothrow) std::atomic<float>[XP_TVF_NYQUIST_WORD]();
  return coefficients->g != nullptr;
}

void tvf_coefficients_destroy(struct xp_tvf_coefficients *coefficients)
{
  if (!coefficients)
    return;
  delete[] coefficients->g;
  coefficients->g = nullptr;
}

/* `0x6ad0`..`0x6aff`. The matrix's cached word is a controller reading,
   not yet a cutoff term: the firmware clamps it to `0xf060..0x0fa0`
   (`0x6ad4`, `0x6ade`), shifts it left three (`0x6ae6`), multiplies by
   `0x8312` keeping only the signed high word (`0x6aec`..`0x6afb`) and
   halves that (`0x6afd`). Full scale reaches 8191, which is two octaves
   in the base table's 4096-per-octave word - the constant was chosen for
   exactly that, as were the LFO paths' own `+-0xfc0` and `0x8208`.

   A cached word of zero short-circuits at `0x6ad2`, which this returns
   the same value for, so the branch is arithmetic rather than a case. */
int16_t tvf_matrix_cutoff_term(int16_t cached)
{
  int32_t value = cached;

  if (value < -4000)
    value = -4000;
  else if (value > 4000)
    value = 4000;
  value = floorDivPow2((value << 3) * INT32_C(0x8312), 16);
  return (int16_t)floorDivPow2(value, 1);
}

/* `0x6b7a`..`0x6bd7` and `0x6c58`..`0x6cbb`, the two LFO filter terms.
   Each oscillator's faded depth word is a modulation reading, not yet a
   cutoff term: the firmware short-circuits a zero (`0x6b7c`, `0x6c5a`),
   clamps to `0xf040..0x0fc0` (`0x6b7e`, `0x6b88`), shifts left three
   (`0x6b90`), multiplies by `0x8208` keeping the signed high word
   (`0x6b96`..`0x6ba5`), halves that (`0x6ba7`) and multiplies the result
   by the oscillator's own waveform word.

   The clamped depth reaches 8191, two octaves in the base table's 4096
   per octave, and a full-scale waveform brings the term to 4095 - one
   octave before the accumulator's shift right one, half an octave
   after it. */
int16_t tvf_lfo_filter_term(int16_t fadedDepth, int16_t waveform)
{
  int32_t value = fadedDepth;

  if (value == 0)
    return 0;
  if (value < -4032)
    value = -4032;
  else if (value > 4032)
    value = 4032;
  value = floorDivPow2((value << 3) * INT32_C(0x8208), 16);
  value = floorDivPow2(value, 1);
  return (int16_t)signedProductHigh(value, waveform);
}

bool tvf_prepare_registers(const struct xp_rom *rom,
                            const struct xp_component *component,
                            int16_t preBaseModulation,
                            const struct xp_tvf_controls *controls,
                            struct xp_tvf_registers *registers)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  if (!rom || !rom->bytes || rom->size < profile->limitTable + 256u ||
      !component || !component->bytes || !controls || !registers ||
      controls->part_cutoff > 127 || controls->secondary_cutoff > 127 ||
      controls->part_resonance > 127 ||
      controls->secondary_resonance > 127)
    return false;

  std::memset(registers, 0, sizeof *registers);
  registers->frequency_interpolation = 0x4100;
  registers->resonance_interpolation = 0x095f;
  int mode = s8(component->bytes[0x3e]);
  if (mode < 0) {
    registers->resonance_current = 0x20000;
    registers->resonance_target = 0x80000;
    registers->filter_select = 0x0800;
    registers->fixed_tuple = true;
    return true;
  }

  int cutoffIndex = clampIndex(
    component->bytes[0x3c] + controls->part_cutoff +
    controls->secondary_cutoff - 128);
  int resonanceIndex = clampIndex(
    component->bytes[0x3d] - 2 * (controls->part_resonance +
                                  controls->secondary_resonance - 128));
  int resonanceFloor = component->bytes[0x3d] < 4
    ? component->bytes[0x3d] : 4;
  if (resonanceIndex < resonanceFloor)
    resonanceIndex = resonanceFloor;

  /* Base and limit share one domain: both halved. Routine 6ccd shifts
   * the saturated table-plus-modulation sum right one (6d21) and the
   * limit is read from 78802 and shifted right one (68b4, 6984) before
   * the unsigned compare. Halved on both sides the base spans
   * 12561..32767 against a limit of 29811..31744, so the clamp bites
   * only for the top dozen indices. */
  /* `0x6aff`..`0x6cc9` builds RAM 30da as the controller term, the two
     LFO terms and the key term added into one another with plain 16-bit
     `add:g.w`, so the accumulator wraps before the table entry is added
     to it. The caller supplies the key and LFO terms; the controller
     term is the matrix's. */
  int16_t accumulator = s16(
    (uint16_t)((uint16_t)preBaseModulation +
               (uint16_t)tvf_matrix_cutoff_term(controls->matrix_cutoff)));
  int32_t combined = be16(rom->bytes + profile->baseTable + (unsigned)cutoffIndex * 2u);
  combined += accumulator;
  if (combined < 0)
    combined = 0;
  else if (combined > UINT16_MAX)
    combined = UINT16_MAX;
  registers->base_unshifted = (uint16_t)combined;
  combined >>= 1;
  registers->base_value = (uint16_t)combined;
  uint16_t limit = (uint16_t)(be16(
    rom->bytes + profile->limitTable + (unsigned)resonanceIndex * 2u) >> 1);
  if (combined > limit)
    combined = limit;

  registers->cutoff_index = (uint8_t)cutoffIndex;
  registers->resonance_index = (uint8_t)resonanceIndex;
  registers->combined = (uint16_t)combined;
  registers->frequency_current = (uint32_t)combined << 3;
  registers->frequency_target = registers->frequency_current;
  registers->resonance_current = (uint32_t)resonanceIndex << 11;
  registers->resonance_target = (uint32_t)resonanceIndex << 13;
  registers->filter_select = (uint16_t)((unsigned)mode << 8);
  return true;
}

bool tvf_key_modulation(const struct xp_rom *rom, const struct xp_tone *tone,
                         const struct xp_component *component,
                         uint8_t selectorKey, int16_t *modulation)
{
  if (!rom || !rom->bytes || !tone || !tone->common || !component ||
      !component->bytes || !modulation || selectorKey > 127)
    return false;
  uint32_t table = ((uint32_t)tone->common[0x21] << 16) |
    be16(component->bytes + 0x40);
  if (table + (uint32_t)selectorKey * 2 + 2 > rom->size)
    return false;
  int16_t keyValue = s16(be16(rom->bytes + table + (uint32_t)selectorKey * 2));
  int16_t factor = s16(be16(component->bytes + 0x42));
  int32_t high = floorDivPow2((int32_t)keyValue * factor, 16);
  *modulation = s16((uint16_t)((uint16_t)high << 1));
  return true;
}

bool tvf_envelope_prepare(const struct xp_rom *rom, const struct xp_tone *tone,
                           const struct xp_component *component,
                           uint8_t selectorKey, uint8_t velocity, bool softPedal,
                           struct xp_tvf_envelope *envelope)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  if (!rom || !rom->bytes || !tone || !component || !component->bytes ||
      !envelope || selectorKey > 127 || velocity > 127 ||
      profile->envelopeRateTable + 128u * 2 > rom->size ||
      !envelopeDepth(rom, tone, component, velocity, softPedal,
                     &envelope->depth))
    return false;
  {
    uint16_t depth = envelope->depth;
    std::memset(envelope, 0, sizeof *envelope);
    envelope->depth = depth;
  }
  envelope->stage = 4;
  if (envelope->depth == 0)
    return true;
  uint16_t keyScale;
  if (!keyRateScale(rom, tone, component, selectorKey, 0x5a, 0x5e, &keyScale))
    return false;
  for (unsigned stage = 0; stage < 4; ++stage) {
    int factor = s8(component->bytes[stage < 2 ? 0x62 : 0x63]);
    uint16_t velocityScale;
    if (!velocityRateScale(rom, velocity, factor, &velocityScale))
      return false;
    uint16_t finalScale = (uint16_t)(((uint32_t)keyScale * velocityScale) >> 8);
    uint16_t tableRate = be16(rom->bytes + profile->envelopeRateTable +
      (uint32_t)component->bytes[0x54 + stage] * 2);
    prepareIncrement(tableRate, finalScale, envelope->initial_phases + stage,
                      envelope->increments + stage);
    envelope->targets[stage] = scaleTarget(
      s16(be16(component->bytes + 0x4a + stage * 2)), envelope->depth);
  }
  envelope->stage = component->bytes[0x54] == 0 ? 1 : 0;
  envelope->base = envelope->stage == 1 ? envelope->targets[0] : 0;
  envelope->current = envelope->base;
  envelope->delta = s16((uint16_t)(
    (uint16_t)envelope->targets[envelope->stage] - (uint16_t)envelope->base));
  envelope->phase = envelope->initial_phases[envelope->stage];
  envelope->active = true;
  return true;
}

bool tvf_envelope_advance(struct xp_tvf_envelope *envelope,
                           unsigned elapsedPeriods)
{
  if (!envelope || !envelope->active || envelope->stage >= 4 ||
      elapsedPeriods == 0)
    return false;
  uint8_t catchup = (uint8_t)(elapsedPeriods - 1);
  uint16_t remaining = (uint16_t)(envelope->saved_count +
    (catchup <= 127 ? (int)catchup : (int)catchup - 256));
  uint16_t working = envelope->phase;
  uint16_t increment = envelope->increments[envelope->stage];
  for (;;) {
    uint16_t next = (uint16_t)(working + increment);
    if (next < working) {
      envelope->saved_count = (uint8_t)remaining;
      ++envelope->stage;
      if (envelope->stage == 4) {
        envelope->current = s16((uint16_t)(
          (uint16_t)envelope->base + (uint16_t)envelope->delta));
        envelope->phase = 0;
        envelope->active = false;
      } else {
        envelope->base = s16((uint16_t)(
          (uint16_t)envelope->base + (uint16_t)envelope->delta));
        envelope->current = envelope->base;
        envelope->delta = s16((uint16_t)(
          (uint16_t)envelope->targets[envelope->stage] -
          (uint16_t)envelope->base));
        envelope->phase = envelope->initial_phases[envelope->stage];
      }
      return true;
    }
    working = next;
    --remaining;
    if (remaining == UINT16_MAX)
      break;
  }
  envelope->phase = working;
  envelope->saved_count = 0;
  envelope->current = s16((uint16_t)(
    (uint16_t)envelope->base + (uint16_t)floorDivPow2(
      (int32_t)envelope->delta * working, 16)));
  return true;
}

bool tvf_release_prepare(const struct xp_rom *rom, const struct xp_tone *tone,
                          const struct xp_component *component,
                          uint8_t selectorKey, uint16_t envelopeDepthValue,
                          struct xp_tvf_release *release)
{
  if (!rom || !rom->bytes || !tone || !component || !component->bytes ||
      !release || selectorKey > 127 ||
      xp_profile(rom)->envelopeRateTable + 128u * 2 > rom->size)
    return false;
  std::memset(release, 0, sizeof *release);
  release->scale = UINT16_MAX;
  if (envelopeDepthValue == 0)
    return true;
  uint16_t keyScale;
  if (!keyRateScale(rom, tone, component, selectorKey, 0x5c, 0x5f, &keyScale))
    return false;
  uint16_t tableRate = be16(rom->bytes + xp_profile(rom)->envelopeRateTable +
                            (uint32_t)component->bytes[0x58] * 2);
  uint16_t initialPhase;
  prepareIncrement(tableRate, keyScale, &initialPhase, &release->increment);
  release->phase = initialPhase;
  release->target = scaleTarget(
    s16(be16(component->bytes + 0x52)), envelopeDepthValue);
  return true;
}

bool tvf_release_set_pedal(const struct xp_rom *rom, uint8_t hold1,
                            bool continuousHold, bool keepScaleAtZero,
                            bool sostenutoRetained,
                            struct xp_tvf_release *release)
{
  if (!rom || !rom->bytes || !release || hold1 > 127)
    return false;
  release->scale_enabled = true;
  if (sostenutoRetained) {
    release->scale = 0;
  } else {
    release->scale = UINT16_MAX;
    unsigned effective = continuousHold ? hold1 : (hold1 >= 64 ? 127u : 0u);
    if (effective == 0) {
      if (!keepScaleAtZero)
        release->scale_enabled = false;
    } else {
      uint32_t offset = xp_profile(rom)->releasePedalTable + (127u - effective) * 2;
      if (offset + 2 > rom->size)
        return false;
      release->scale = be16(rom->bytes + offset);
    }
  }
  release->active = true;
  return true;
}

bool tvf_release_advance(struct xp_tvf_release *release,
                          unsigned elapsedPeriods)
{
  if (!release || !release->active || elapsedPeriods == 0)
    return false;
  uint16_t step = release->scale_enabled
    ? (uint16_t)(((uint32_t)release->increment * release->scale) >> 16)
    : release->increment;
  uint8_t periods = (uint8_t)elapsedPeriods;
  uint32_t product = (uint32_t)step * periods;
  uint16_t next = (uint16_t)(release->phase + (uint16_t)product);
  if ((product >> 16) != 0 || next < release->phase) {
    release->current = release->target;
    release->active = false;
  } else {
    release->phase = next;
    release->current = scaleTarget(release->target, next);
  }
  return true;
}

bool tvf_update_frequency(const struct xp_rom *rom,
                           int16_t postBaseModulation,
                           struct xp_tvf_registers *registers)
{
  if (!rom || !rom->bytes || !registers ||
      rom->size < xp_profile(rom)->limitTable + 256u)
    return false;
  if (registers->fixed_tuple)
    return true;
  /* The envelope and release outputs join the word after the halving.
     Routine 6ccd saturates the table entry plus the key, controller and
     LFO terms, shifts the sum right one and stores it; 68a1, 68e3 and
     6971 then add the envelope output (and, in the update path, the
     release ramp) to that stored word with a plain 16-bit add, and the
     sum is compared unsigned against the halved limit. One unit of
     envelope is therefore 1/2048 octave in the register - twice the
     weight of one unit of table or key term. The add does not saturate;
     over the held bank the halved word never falls below 12561 and no
     envelope target reaches -9101 from there, so it never wraps. */
  uint16_t combined = (uint16_t)((uint16_t)registers->base_value +
                                 (uint16_t)postBaseModulation);
  uint16_t limit = (uint16_t)(be16(
    rom->bytes + xp_profile(rom)->limitTable +
    (uint32_t)registers->resonance_index * 2) >> 1);
  if (combined > limit)
    combined = limit;
  registers->combined = combined;
  registers->frequency_target = (uint32_t)combined << 3;
  return true;
}

void tvf_latch_frequency(struct xp_tvf_registers *registers)
{
  if (registers)
    registers->frequency_current = registers->frequency_target;
}

void tvf_advance_registers(struct xp_tvf_registers *registers,
                            unsigned periods)
{
  if (!registers || periods == 0)
    return;
  double value = (double)registers->frequency_current +
    frequencyProgress(registers, (double)periods) *
    ((double)registers->frequency_target - registers->frequency_current);
  registers->frequency_current = value <= 0.0 ? 0u : (uint32_t)(value + 0.5);
  /* TVF-Q is not approached: its target is the same value as its current
     with two more fraction bits (see the damping note in the audio path). */
}

void tvf_audio_reset(struct xp_tvf_audio_state *state)
{
  if (state)
    std::memset(state, 0, sizeof *state);
}

float tvf_audio_process_provisional(void *user, struct xp_tvf_audio_state *state,
                                     const struct xp_tvf_registers *registers,
                                     double periodFraction, float input)
{
  if (!state || !registers)
    return input;
  /* A negative mode byte installs the fixed tuple: zero cutoff, type word
     0x0800 - type code 2, the high-pass. A high-pass at zero cutoff
     passes everything, so the tuple is a bypass by its own terms; see the
     type-code note below. */
  if (registers->fixed_tuple)
    return input;
  if (periodFraction < 0.0)
    periodFraction = 0.0;
  else if (periodFraction > 1.0)
    periodFraction = 1.0;
  /* Where the register stands this far into the period, by its own
     interpolation word rather than by a ramp spread over the whole of it. */
  double word = (double)registers->frequency_current +
    frequencyProgress(registers, periodFraction) *
    ((double)registers->frequency_target - registers->frequency_current);
  double g;
  {
    const uint32_t wordInt = (uint32_t)(word + 0.5);
    /* g is a pure function of wordInt at a fixed host rate, and the host
       rate is set once per Device and never changes mid-render (it comes
       from renderer->tvf_audio_user, wired up once at device init) - so
       memoizing on wordInt alone, per voice, and tabulating on it per
       Device are exact, not an approximation of the rate-dependent case.
       Only (float)g reaches the filter below, so a table of floats holds
       everything the double carried that matters. */
    if (state->memo_valid && state->memo_word == wordInt) {
      g = state->memo_g;
    } else {
      struct xp_tvf_coefficients *table = (struct xp_tvf_coefficients *)user;
      if (table && table->g && wordInt < XP_TVF_NYQUIST_WORD) {
        float entry = table->g[wordInt].load(std::memory_order_relaxed);
        if (entry == 0.0f) {
          entry = (float)coefficientForWord(wordInt, table->rate);
          table->g[wordInt].store(entry, std::memory_order_relaxed);
        }
        g = entry;
      } else {
        g = coefficientForWord(wordInt, table ? table->rate : kXpNativeRate);
      }
      state->memo_word = wordInt;
      state->memo_g = g;
      state->memo_valid = true;
    }
  }
  /* Damping from the register the ROM supplies. `07_synthesis/tvf.md`:
   * the companion table at 0x78902 is exactly index * 512, expanded left
   * 2 into TVF-Q current and left 4 into TVF-Q target, and the target is
   * shifted right two bits before use - the target carries two extra
   * fraction bits, so current and target name the same value and Q does
   * not move after note-on. Approaching current toward target as if they
   * shared units ramped Q fourfold over 220 ms and opened every note with
   * a +6 dB peak at the cutoff, which set the drum path 200 Hz too bright
   * (`M-105`). The fixed-tuple path's TVF-Q current 0x20000 is what index
   * 64 produces, so 64 is the neutral resonance; the index runs opposite
   * to the player's setting, so a low index is a high Q. */
  double damping = (double)registers->resonance_current / kXpTvfQUnity;
  /* The register runs to index 127, which asks for 3.97. The topology is
     stable for any positive damping, so the only bound needed is one that
     keeps the denominator away from zero; refusing the overdamped end
     flattened 13 components onto the neutral value. */
  if (damping < 0.05)
    damping = 0.05;
  else if (damping > 4.0)
    damping = 4.0;
  /* The two poles are realised with forward-Euler integrators (svf_step,
     engines/common/dsp_kernels.h), the topology the chip's limit table
     names (see XpDeviceProfile::limitTable) and the one libEmuSC's SC-55
     path already runs. The trapezoidal form this replaced is a bilinear
     transform: it leaves a double zero at
     Nyquist, so the two poles the ROM asks for rolled off like three near
     the top of the band - 0.5 dB darker than its own analog prototype at
     twice the corner for a tone cut off at 1.6 kHz, 3.0 dB at 3.7 kHz and
     7.5 dB at 5.3 kHz. Forward Euler has no zero but the one sample of
     delay, and its error runs the other way. */
  unsigned section;
  float signal = input;
  for (section = 0; section < XP_TVF_SECTIONS; ++section) {
    float d = section == 0 ? (float)damping : 2.0f;
    float f = (float)g;
    float &lp = section == 0 ? state->integrator_low
                             : state->section_low[section];
    float &bp = section == 0 ? state->integrator_band
                             : state->section_band[section];
    /* The chip's own limit table keeps f*f + f*d at or below 2, which is
       well inside this bound; the bound is here because the damping floor
       above and a host sample rate other than the chip's are not the ROM's
       doing and must not be able to put a pole outside the unit circle.
       Run in float: the persistent state either side of this loop is
       already float, and the section arithmetic itself only ever differs
       from a double computation by <=1 LSB of 16-bit output, on 0.045% of
       samples in a 137s SC-88 reference render - below the noise floor of
       the 16-bit format this ships as.

       Section 0's bound is a pure function of resonance_current (via d),
       which - like g above - glides in steps far coarser than one sample,
       so it is memoized the same way. A hypothetical section > 0 (d fixed
       at 2.0f; XP_TVF_SECTIONS is 1 today, so this never runs) is not
       memoized: it would be one constant, but there is no live case to
       verify that against. */
    float bound;
    if (section == 0 && state->bound_valid &&
        state->memo_resonance == registers->resonance_current) {
      bound = state->memo_bound;
    } else {
      bound = 0.99f * (std::sqrt(d * d + 4.0f) - d);
      if (section == 0) {
        state->memo_resonance = registers->resonance_current;
        state->memo_bound = bound;
        state->bound_valid = true;
      }
    }
    if (f > bound)
      f = bound;
    float high = svf_step(signal, f, d, lp, bp);
    /* XP bits 10..11 carry a type code - 0 on 1193 components, 1 on 12,
       2 on 38 - and the name binding proposed from JV-1080
       documentation is LPF/BPF/HPF for 0/1/2. Code 2 takes the
       high-pass output, which the SC-88's own archive recordings show
       directly.

       Breath Noise is the case that cannot be read two ways: its one
       component is code 2 with a static 2.7 kHz cutoff and no envelope,
       and it is broadband, so the filter's shape is the tone. Against
       the archive recording at C4, level-matched over 17 log bands, the
       high-pass output lands within 1.1 dB from 256 Hz to 7.2 kHz. The
       same render with the filter removed is 36 dB too loud at 256 Hz
       and 22 dB at 590 Hz: the hardware has less low end than the raw
       sample, which neither a low-pass nor a bypass can produce, and
       the low-pass output is 20 dB short at 8.9 kHz on top of that.
       Seashore, also code 2, moves from 20.0 to 1.6 dB median band
       error at the same key. In the 63-note set the Fiddle goes from
       19.19 to 1.75 dB and Halo Pad from 8.34 to 2.24.

       The earlier reading - every code takes the low-pass output -
       rested on M-100, which put the Fiddle's cutoff at 8.1 kHz, where
       a high-pass would indeed have removed its fundamental. The
       firmware-exact cutoff law puts that same component at 439 Hz at
       C5, where a two-pole high-pass leaves the 523 Hz fundamental at
       +1.0 dB and everything above it flat. M-100's measurement stands;
       what it refutes does not survive the law it was read under.

       This also settles the negative-mode tuple, which installs type
       code 2 with a cutoff word of zero: a high-pass at zero passes
       everything, so `fixed_tuple` returning the input untouched is the
       type code's own behaviour rather than an assumption about it.

       Code 1 keeps the low-pass output. No component in any archive
       recording carries it - the only one in the GM bank is Slap Bass
       2, which is not in the single-note set - so band-pass is
       unverified here and is not adopted on the strength of the
       binding alone. */
    signal = ((registers->filter_select >> 10) & 3u) == 2u ? high : lp;
  }
  return signal;
}

}}  // namespace EmuSC::Xp
