/* SPDX-License-Identifier: CC0-1.0 */
#include "oscillator.h"

#include "common/constants.h"

#include <cmath>

namespace EmuSC { namespace Xp {

double pitch_word_rate(uint32_t pitchWord, double outputRate)
{
  if (outputRate <= 0.0)
    return 0.0;
  return (kXpNativeRate / outputRate) *
    std::pow(2.0, ((double)pitchWord - kXpPitchUnity) / kXpPitchUnitsPerOctave);
}

/* Fractional bits in the fixed-point phase accumulator, of the 64 it has.
   40 leaves 24 for the sample index, against the 20 a 1 Mi-sample bank -
   the widest any XP device profile declares - actually needs, and puts the
   playback rate's own quantization at 2^-41 of a sample: a relative rate
   error of 6.5e-13 at unity, or 1.1e-9 cent, against the 0.0732 cent that
   one unit of the pitch register is worth. That margin is what keeps the
   accumulator's drift away from the double path's below the point where
   the converted 16-bit output can tell the two apart. */
#ifndef XP_PHASE_FRACTION_BITS
#define XP_PHASE_FRACTION_BITS 40
#endif
static constexpr unsigned kPhaseFractionBits = XP_PHASE_FRACTION_BITS;
static constexpr double kPhaseScale =
  (double)(UINT64_C(1) << kPhaseFractionBits);
/* Bits of the fraction the float kernel is handed - every one of which a
   float's significand holds exactly, so the conversion below is exact and
   introduces nothing the kernel would not introduce anyway. */
static constexpr unsigned kPhaseFloatFractionBits = 24;

double oscillator_phase(const struct xp_oscillator *oscillator)
{
  if (!oscillator)
    return 0.0;
#ifdef EMUSC_LEGACY_DSP_FAST
  return (double)oscillator->phase_fixed / kPhaseScale;
#else
  return oscillator->phase;
#endif
}

void oscillator_set_phase(struct xp_oscillator *oscillator, double phase)
{
  if (!oscillator)
    return;
  if (phase < 0.0)
    phase = 0.0;
  oscillator->phase = phase;
  oscillator->phase_fixed = (uint64_t)(phase * kPhaseScale + 0.5);
}

void oscillator_set_step(struct xp_oscillator *oscillator, double step)
{
  if (!oscillator)
    return;
  if (step < 0.0)
    step = 0.0;
  oscillator->step = step;
  /* Rounded rather than truncated: truncation would leave every rate
     short of the one asked for, which is a pitch that is always flat
     rather than one off by half a unit either way. */
  oscillator->step_fixed = (uint64_t)(step * kPhaseScale + 0.5);
}

namespace {

/* A ping-pong turn in a DIFFERENTIAL format is not a time reversal.

   The wave ROM stores deltas, and the decoder is an accumulator.  Walking
   the address back down the stream while still ADDING what it reads gives,
   from the turn at address_c,

       y[m] = x[c] + sum(d[c] .. d[c-m+1]) = 2*x[c] - x[c-m]

   - the loop's own waveform, backwards AND reflected about the value it
   turned at.  That is continuous in value and in SLOPE at the turn, where a
   plain time reversal puts a corner and a forward wrap puts a phase jump.

   The ROM says this is the shape the format is cut for: over all 1733
   looping descriptors the deltas from address_b to address_c sum to exactly
   zero, so x[c] == x[b-1] without exception.  That single fact makes the
   reflected pass land exactly on x[b-1] when the address reaches b-1, and
   the cycle closes with no step and no drift - which a reflection about any
   other value would not do.

   Cycle of 2*span positions, span = c - b + 1:
     index < span        address c-1 down to b-1, value reflected
     index >= span       address b up to c, value as decoded

   The descending pass's last position is b-1, where the reflection returns
   x[c] by the invariant above, so it is answered directly rather than read:
   b-1 can sit before the first decoded frame when a zone loops from its own
   start.  */
uint32_t cycle_address(const struct xp_oscillator *oscillator, size_t index)
{
  const size_t span = (size_t)(oscillator->end - oscillator->loop) + 1;
  index %= oscillator->cycle_count;
  if (oscillator->mode == XP_WAVE_FORWARD_LOOP)
    return oscillator->loop + (uint32_t)index;
  if (index < span)
    return oscillator->end - 1 - (uint32_t)index;
  return oscillator->loop + (uint32_t)(index - span);
}

/* True while the cycle is on its reflected descending pass. */
bool cycle_reflected(const struct xp_oscillator *oscillator, size_t index)
{
  const size_t span = (size_t)(oscillator->end - oscillator->loop) + 1;
  if (oscillator->mode != XP_WAVE_PING_PONG_LOOP || !oscillator->cycle_count)
    return false;
  return (index % oscillator->cycle_count) < span;
}

bool reflected(const struct xp_oscillator *oscillator, size_t index)
{
  if (oscillator->initial) {
    if (index < oscillator->initial_count || !oscillator->cycle_count)
      return false;
    return cycle_reflected(oscillator, index - oscillator->initial_count);
  }
  return cycle_reflected(oscillator, index);
}

uint32_t address_at(const struct xp_oscillator *oscillator, size_t index)
{
  if (oscillator->initial) {
    if (index < oscillator->initial_count) {
      if (oscillator->mode == XP_WAVE_REVERSE_ONE_SHOT)
        return oscillator->start - (uint32_t)index;
      return oscillator->start + (uint32_t)index;
    }
    if (oscillator->cycle_count)
      return cycle_address(oscillator, index - oscillator->initial_count);
    return oscillator->mode == XP_WAVE_REVERSE_ONE_SHOT
      ? oscillator->end + 1 : oscillator->end;
  }
  return cycle_address(oscillator, index);
}

bool contains(const struct xp_oscillator *oscillator, uint32_t address)
{
  return address >= oscillator->pcm_base &&
    (size_t)(address - oscillator->pcm_base) < oscillator->pcm_count;
}

}  // namespace

bool oscillator_init(const struct XpDeviceProfile *profile,
                      struct xp_oscillator *oscillator,
                      const int32_t *pcm24, size_t pcmCount,
                      uint32_t pcmBase,
                      const struct xp_wave_registers *registers,
                      enum xp_wave_loop_type mode,
                      uint32_t pitchWord, double outputRate,
                      enum xp_fractional_wrap wrap)
{
  if (!profile)
    profile = &SC88_PROFILE;
  if (!oscillator || !pcm24 || !pcmCount || !registers ||
      outputRate <= 0.0 || pitchWord > kXpPitchSaturation ||
      wrap < XP_WRAP_FULL_CARRY || wrap > XP_WRAP_FRACTION_ONLY ||
      registers->start >= profile->waveBankSize ||
      registers->loop >= profile->waveBankSize ||
      registers->end >= profile->waveBankSize)
    return false;

  oscillator->pcm24 = pcm24;
  oscillator->pcm_count = pcmCount;
  oscillator->pcm_base = pcmBase;
  oscillator->start = registers->start;
  oscillator->loop = registers->loop;
  oscillator->end = registers->end;
  oscillator->mode = mode;
  oscillator->wrap = wrap;
  oscillator_set_phase(oscillator, 0.0);
  oscillator_set_step(oscillator, pitch_word_rate(pitchWord, outputRate));
  oscillator->initial = true;
  oscillator->ended = false;
  oscillator->cycle_count = 0;

  uint32_t last;
  if (mode == XP_WAVE_REVERSE_ONE_SHOT) {
    last = registers->end + 1;
    if (last > registers->start)
      return false;
    oscillator->initial_count = (size_t)(registers->start - last) + 1;
  } else {
    if (registers->start > registers->end)
      return false;
    oscillator->initial_count =
      (size_t)(registers->end - registers->start) + 1;
    if (mode == XP_WAVE_FORWARD_LOOP ||
        mode == XP_WAVE_PING_PONG_LOOP) {
      if (registers->loop > registers->end)
        return false;
      size_t span = (size_t)(registers->end - registers->loop) + 1;
      oscillator->cycle_count = mode == XP_WAVE_FORWARD_LOOP
        ? span : span * 2;
    }
  }

  last = mode == XP_WAVE_REVERSE_ONE_SHOT
    ? registers->end + 1 : registers->end;
  return contains(oscillator, registers->start) &&
    contains(oscillator, last) &&
    (oscillator->cycle_count == 0 || contains(oscillator, registers->loop));
}

namespace {

/* THE PHASE CARRIES ACROSS A LOOP WRAP, and the ROM says so.

   THE FIRMWARE IS SILENT, and that is worth establishing rather than
   assuming.  Exactly four routines in the 0x80000 image select the XP's
   address domain - `ldc.b #0xe:8,ep` at SC88-CTL 0x4f51, 0x51cd, 0x5278 and
   0x559b.  0x51cd and 0x5278 write only the voice enable bitmap at XP
   `0x3900`; 0x559b is the per-period service and rewrites only the TVF and
   TVA triplets.  The voice uploader at 0x4f51 is the only one that writes a
   wave address at all, and it runs once, at note start: `0x0100` at 0x4f9e,
   `0x0200` at 0x4fbd, `0x0300` at 0x4fcb, the per-voice state `0x0c00` at
   0x4fd9, and `0x0e00` cleared to zero at 0x4f82/0x4f87.  So no instruction
   rewrites an address register while a voice sounds: the wrap happens inside
   the XP and the control ROM cannot express a rule for it.  The identical XP
   part in the JV-1080 is written the same way, once per note start.

   THE WAVE DIRECTORY IS NOT SILENT.  A loop of `span` logical samples read
   at `step` samples per output sample lasts `span / step` output samples if
   the accumulator carries its remainder, and `ceil(span / step)` if the
   remainder is thrown away - because then the loop has to end on an output
   sample.  The ROM's own descriptors are cut for one of those two.

   Each descriptor carries a root key at +6 and a base pitch correction at
   +4, 16384 units to the octave.  Read the correction as the rate at the
   root key and the loop as `span / step`: on 1005 of the 1520 forward-loop
   descriptors that is within ONE CENT of a whole number of periods of that
   descriptor's own root key - median residual 0.28 cent, 85.5 % within five.
   Zero the +4 correction and 4.0 % are, median 16.35 cents.  So the field is
   cut to make that identity hold, at 0.0732 cent per unit.  Take the loop as
   `ceil(span / step)` instead and only 20.5 % of those same 1005 survive a
   cent; the median residual becomes 7.49 cents and the 99th percentile 182.6.
   A correction finer than the grid it is quantised onto is not a correction.

   The same arithmetic over the 51166 (zone, key) pairs the 549 directories
   name: clearing the phase puts 18.2 % of them more than 20 cents flat and
   6.9 % more than a semitone, always flat, worst -5800 cents on a loop the
   ROM cuts at four samples.  Dropping only the integer part of the overshoot
   is identical to carrying whenever `step <= 1`, and leaves 26.9 % of the
   upward-transposed pairs more than 20 cents flat.  Carrying the whole
   remainder is the only one of the three that plays every zone at the rate
   its own descriptor asks for.

   HORN_B's key-60 zone is the short loop this was opened on (emusc-match
   TASK-162; descriptor at 0x3820c, span 86, forward, 2.7 ms at its own
   rate).  Carrying gives 115.651 output samples a traversal; clearing gives
   exactly 116, which is the sustain 5.22 cents flat of the attack it grew
   out of, and a fresh discontinuity at every one of its 1103 wraps -
   45 times the note's own departure from its neighbours, where carrying is
   5.9 against the 4.2 the same waveform reaches at other phases.  The
   opposite reading came from a metric that rewarded an output for repeating
   on the integer sample grid, which is exactly what clearing the phase
   manufactures.

   The fixed-point form below says the same three things about the
   accumulator EMUSC_LEGACY_DSP_FAST advances: the carry is a modulo, and
   one subtraction is all it takes unless the step is longer than the
   cycle itself - a loop the transposition traverses more than once per
   output sample, where the wide divide is what fmod costs here too.
   Dropping the integer part is masking off the bits above the
   fraction. */
#ifndef EMUSC_LEGACY_DSP_FAST
double wrapped_phase(const struct xp_oscillator *oscillator, double overflow)
{
  switch (oscillator->wrap) {
  case XP_WRAP_FULL_CARRY:
    return std::fmod(overflow, (double)oscillator->cycle_count);
  case XP_WRAP_FULL_RESET:
    return 0.0;
  case XP_WRAP_FRACTION_ONLY:
    return overflow - std::floor(overflow);
  }
  return 0.0;
}
#else
uint64_t wrapped_phase_fixed(const struct xp_oscillator *oscillator,
                              uint64_t overflow)
{
  switch (oscillator->wrap) {
  case XP_WRAP_FULL_CARRY: {
    const uint64_t cycle =
      (uint64_t)oscillator->cycle_count << kPhaseFractionBits;
    if (overflow < cycle)
      return overflow;
    overflow -= cycle;
    return overflow < cycle ? overflow : overflow % cycle;
  }
  case XP_WRAP_FULL_RESET:
    return 0;
  case XP_WRAP_FRACTION_ONLY:
    return overflow & ((UINT64_C(1) << kPhaseFractionBits) - 1);
  }
  return 0;
}
#endif

/* The decoded value the oscillator is standing on, with the reflected
   descending pass of a ping-pong applied.  The one position the reflection
   cannot read is address_b - 1, which can precede the first decoded frame;
   the ROM's zero-sum invariant answers it as x[c] exactly. */
/* pcm24 samples are int32_t in [-2^23, 2^23), and a float's 24-bit
   significand represents every integer in that range exactly, so this
   float carries the sample - and the plain integer arithmetic below it -
   with no precision loss versus double. */
bool value_at(const struct xp_oscillator *oscillator, size_t index,
              float *out)
{
  uint32_t address = address_at(oscillator, index);

  if (reflected(oscillator, index)) {
    float anchor = (float)oscillator->pcm24[oscillator->end -
                                             oscillator->pcm_base];
    if (contains(oscillator, address)) {
      *out = 2.0f * anchor -
        (float)oscillator->pcm24[address - oscillator->pcm_base];
      return true;
    }
    /* address_b - 1 only: it can precede the first decoded frame when a
       zone loops from its own start, and the ROM's zero-sum invariant
       answers it as x[c] exactly. */
    if (address + 1 == oscillator->loop) {
      *out = anchor;
      return true;
    }
    return false;
  }
  if (!contains(oscillator, address))
    return false;
  *out = (float)oscillator->pcm24[address - oscillator->pcm_base];
  return true;
}

/* THE FOUR-POINT READ.  The response is measured on the SC-88's own
   recordings; the kernel that realises it is measured on the silicon.

   The wave is read with a kernel, and a kernel is one fixed filter in the
   WAVE's own time base.  A tone transposed by `ratio` puts output
   frequency f at the normalised wave frequency

       theta = f / (32000 * ratio)      cycles per wave sample

   so two tones at different transpositions reach the same theta from
   different output frequencies, and a real kernel has to give them the
   same answer.  That over-determination is what separates a measurement
   from a curve fitted to a residual.

   THE RESPONSE, off the SC-88 itself, indirectly.  63 archive single notes
   against our render, every band the reference carries above its own noise
   floor, 1101 points.  At FIXED theta the excess does not depend on output
   frequency at all - the slope is +0.15, -0.02 and -0.95 dB per octave in
   three theta bands, every |t| <= 0.3, over f from 726 Hz to 13.5 kHz.  At
   FIXED output frequency it depends strongly on the transposition: -2.76
   dB per octave of ratio from 5 to 9.5 kHz and -5.36 above.  Fitting both
   at once over every band above 1 kHz gives +0.90 +- 0.21 dB per octave of
   frequency and -1.25 +- 0.43 per octave of ratio; an output-stage
   response requires the second to be zero and it is 2.9 sigma away, while
   a kernel in the wave's time base requires it to be the negative of the
   first, which it is.  Recovered on the r < 0.90 group alone and expressed
   against theta it PREDICTS the held-out 0.90 <= r < 1.25 group to a mean
   0.81 dB over nine shared bins, eight of nine 68 % intervals overlapping,
   at output frequencies an octave apart; the same points binned by output
   frequency instead miss by 2.56 dB with none of ten overlapping.

       theta         0.12   0.16   0.20   0.24   0.28   0.32   0.43   0.50
       reference dB  -0.74  -1.02  -1.64  -2.45  -3.78  -5.12  -9.74 -11.08
       two-point     -0.43  -0.75  -1.18  -1.71  -2.35  -3.10  -5.79  -7.53

   Written as sinc**n that curve is n = 3.25 [2.99, 3.42] at 68 %,
   resampled over tones; two-point linear is n = 2.  It names a RESPONSE
   and not a kernel, and the SC-88's own ROM names no kernel either: the
   control ROM holds no phase-indexed unity-summing table under any of four
   numeric readings, against a search that recovers three of three and four
   of four planted ones, and the 28-register XP voice upload carries pitch,
   TVF, TVA, pan and sends - no coefficient bank, no interpolation-mode
   field.  The interpolation is XP silicon.

   THE KERNEL, off that silicon, directly.  The JV-1080 carries the same
   part - MBCS30109, Roland 15239239 - and its wave ROM is dumped and
   decoded, so on that machine the source spectrum is KNOWN instead of
   being differenced away.  Two single-element, filter-off,
   neutral-fine-tune waves, 13-16 slots at ratio 0.25-0.60, each candidate
   compared against the recovered response over 42 dB of span and allowed
   only its own constant offset:

       cubic B-spline      0.63 / 0.59 dB rms
       quartic B-spline    2.77 / 1.83
       quadratic B-spline  2.80 / 2.28
       Catmull-Rom         5.15 / 4.47
       two-point linear    5.48 / 4.23
       nearest sample      8.18 / 6.19

   Two independent waves, one winner, by a factor of three to nine.  The
   cubic B-spline's response is |sinc|^4 - n = 3, inside the interval the
   SC-88's own recordings give - and its weights at fraction 0 are
   [1/6, 2/3, 1/6, 0], which is the SC-55 PCM chip's own four-point table
   [0.174, 0.653, 0.173, 0] to 0.007.  One kernel across both chip
   families.  In silicon it is three linear interpolations (de Boor), so it
   is nothing exotic for a 1994 ASIC.

   [MEASURED ON A JV-1080] (P-xxxx).  The SC-88 shares the part, and the
   SC-88's own response interval contains this kernel, but a JV-1080 is
   still another device: this is a strong lead for the SC-88 and it is NOT
   firmware-exact for it.

   IT DOES NOT INTERPOLATE.  At fraction 0 it does not hand value0 through,
   it convolves by [1/6, 2/3, 1/6].  So it runs at EVERY ratio, unity
   included, and a sample played at its own root key is smoothed like any
   other; there is deliberately no fast path for fraction 0.  That is also
   how an earlier measurement on this same silicon read the chip as
   two-point linear and had to be withdrawn - all of its candidates were
   interpolating ones, identity at fraction 0, so the integer-ratio control
   where they all agree was precisely the case none of them could see.

   Whole board, 63 archive single notes, dry, all three kernels rendered
   from this engine.  Per-band median residual, ours minus the archive:

       band Hz          3.1k  3.9k  4.8k  5.9k  7.2k  8.9k 11.0k 13.5k
       two-point linear  0.1   0.2   0.4   0.6   1.1   1.7   2.8   4.9
       quadratic         0.0  -0.0   0.0   0.1   0.1   0.4   0.7   2.0
       this, cubic      -0.2  -0.4  -0.5  -0.7  -0.8  -0.9  -1.2  -0.9

   On the r < 0.90 group alone - the transposed-down tones the excess was
   found on - 0.3 0.2 0.5 0.8 1.8 2.8 4.6 8.6 becomes -0.2 -0.5 -0.8 -1.0
   -1.2 -1.5 -3.2 -1.2, and the correlation of the excess against
   log2(ratio) that named the wave's time base in the first place
   collapses with it: -0.38 -> -0.12 -> +0.20 at 13.5 kHz and
   -0.31 -> -0.14 -> +0.05 at 11.0 across the three kernels.  The whole
   board's spectral audit reads the last step as level: median MAD
   1.3 -> 0.9 -> 0.9 dB, 55 of 63 within 3 dB under both B-splines,
   tilted bright 10 -> 4 -> 4.

   AND IT GOES PAST THE ARCHIVE, which is worth saying plainly.  Recovered
   from this very render the archive's curve is sinc^n with
   n = 3.28 [3.12, 3.48], and a quadratic render returns the same number
   (3.20 [3.03, 3.38]) as it must - so the SC-88's own recordings want
   something BETWEEN sinc^3 and sinc^4, nearer the quadratic, and this
   kernel sits about 0.7 dB past them: median miss +0.72 dB against the
   quadratic's -0.15.  Per tone, 18 of 63 improve and 33 worsen, the
   improvements concentrated in the deeply transposed-down tones the
   response was found on.  The kernel stays, because the two numbers are
   not the same class of evidence.  The JV-1080 figure is a structural
   measurement of THIS quantity against a source spectrum that is known;
   the archive figure is a residual of our WHOLE render against recordings
   whose own provenance is not established, so it carries every other
   high-end error we still have.  If one of those is found later, this
   residual should move towards the cubic rather than away from it.  */

/* The position before this one.  Inside a cycle, position zero's
   predecessor is the cycle's LAST position and not something before the
   note: a read that folded back there would put a step into every loop
   turn, at the loop's own rate.  It answers the four-point read's left
   outer tap unchanged, because at the initial-to-cycle handover the
   cycle's last position and the initial pass's last position are the same
   address - `end` - for both the forward and the ping-pong loop.

   Before the note's FIRST position there is genuinely nothing: the zone's
   decode starts at `start`, and in a differential format the frame before
   it is not a sample of this wave at all.  The caller folds back. */
bool previous_index(const struct xp_oscillator *oscillator, size_t index,
                    size_t *out)
{
  if (index > 0) {
    *out = index - 1;
    return true;
  }
  if (oscillator->initial || !oscillator->cycle_count)
    return false;
  *out = oscillator->cycle_count - 1;
  return true;
}

/* One of the four-point read's two outer taps, two positions apart across
   the fraction.  Both are optional and the INNER tap on the same side
   stands in for a missing one, which repeats a sample the note really has
   rather than inventing one, and leaves the four weights summing to one so
   a boundary cannot put a gain step or a DC offset into the output.  The
   two inner taps stay mandatory, so the one-shot termination test is
   exactly the one the two-point read used.

   Which of the two can actually be missing is not symmetric.  The forward
   tap, index + 2, is answered for every zone `xp_oscillator_init`
   accepts: a loop wraps it modulo the cycle, a one-shot holds it at `end`
   (`end + 1` reversed) and init has already checked that address is inside
   the zone, and the ping-pong's one unreadable position is answered by the
   ROM's zero-sum invariant in `xp_oscillator_value`.  Its fallback is a
   guard, not a case that arises.  The backward tap goes missing exactly
   once per note - at position zero of the initial pass - and the value it
   falls back to is a CHOICE, not a recovered one: nothing measured here
   says what the chip reads when its address counter is still on the zone's
   first frame.  It costs at most the note's first 1/step output samples
   and weight (1-f)^3/6 of one of them. */
float outer_tap(const struct xp_oscillator *oscillator, size_t index,
                 bool back, float inner)
{
  size_t at;

  if (back) {
    if (!previous_index(oscillator, index, &at))
      return inner;
  } else {
    at = index + 1;
  }
  float value;
  return value_at(oscillator, at, &value) ? value : inner;
}

/* The four-point read is always centred on the span the phase is in, so
   unlike a three-tap read it needs no folding to a nearer position:
   `fraction` and its complement carry the whole symmetry.  The uniform
   cubic B-spline basis, with `rest` = 1 - fraction:

     index - 1   rest^3 / 6
     index       2/3 - fraction^2 + fraction^3 / 2
     index + 1   2/3 - rest^2     + rest^3 / 2
     index + 2   fraction^3 / 6

   which is [1/6, 2/3, 1/6] at fraction 0 - a smoother, not an identity -
   and sums to one at every fraction.  8388608 is the pcm24 full scale.

   One kernel in the arithmetic the tier in force evaluates it in:
   double, exactly as the measurement that recovered it was made, or -
   under EMUSC_LEGACY_DSP_FAST - float, where the two divisions become
   reciprocal multiplies and a Pentium III-class target reaches the whole
   expression with scalar SSE1 instead of x87, which carries no float at
   all below SSE2. Same kernel either way. */
/* The four taps where they are four neighbouring frames of one pass - the
   initial pass, or one leg of a cycle - each the value value_at reads
   there, without its per-tap cycle arithmetic (two divisions a tap, and a
   library call each on an ARMv7 core without a divider). False wherever a
   tap would wrap, turn, cross into the cycle, or leave the decoded data:
   the general reads answer those. */
inline bool plain_taps(const struct xp_oscillator *oscillator, size_t index,
                       float *left, float *value0, float *value1,
                       float *right)
{
  if (index < 1)
    return false;
  int64_t first;       /* address of the left tap */
  int64_t dir;         /* address step from one tap to the next */
  bool reflect = false;
  if (oscillator->initial) {
    if (index + 2 >= oscillator->initial_count)
      return false;
    dir = oscillator->mode == XP_WAVE_REVERSE_ONE_SHOT ? -1 : 1;
    first = (int64_t)oscillator->start + dir * ((int64_t)index - 1);
  } else {
    if (index + 2 >= oscillator->cycle_count)
      return false;
    const size_t span = (size_t)(oscillator->end - oscillator->loop) + 1;
    if (oscillator->mode == XP_WAVE_FORWARD_LOOP) {
      dir = 1;
      first = (int64_t)oscillator->loop + (int64_t)index - 1;
    } else if (oscillator->mode != XP_WAVE_PING_PONG_LOOP) {
      return false;
    } else if (index + 2 < span) {
      dir = -1;
      first = (int64_t)oscillator->end - 1 - ((int64_t)index - 1);
      reflect = true;
    } else if (index - 1 >= span) {
      dir = 1;
      first = (int64_t)oscillator->loop + (int64_t)(index - 1 - span);
    } else {
      return false;
    }
  }
  const int64_t last = first + 3 * dir;
  const int64_t lo = (first < last ? first : last) - oscillator->pcm_base;
  const int64_t hi = (first < last ? last : first) - oscillator->pcm_base;
  if (lo < 0 || hi >= (int64_t)oscillator->pcm_count)
    return false;
  const int32_t *at = oscillator->pcm24 + (first - oscillator->pcm_base);
  float taps[4];
  if (reflect) {
    const float anchor =
      (float)oscillator->pcm24[oscillator->end - oscillator->pcm_base];
    for (int k = 0; k < 4; ++k)
      taps[k] = 2.0f * anchor - (float)at[-k];
  } else {
    for (int k = 0; k < 4; ++k)
      taps[k] = (float)at[k * dir];
  }
  *left = taps[0];
  *value0 = taps[1];
  *value1 = taps[2];
  *right = taps[3];
  return true;
}

#ifdef EMUSC_LEGACY_DSP_FAST
float bspline4(float fraction, float left, float value0, float value1,
                float right)
{
  const float rest = 1.0f - fraction;
  return (rest * rest * rest * (1.0f / 6.0f) * left +
          (2.0f / 3.0f - fraction * fraction *
           (1.0f - fraction * 0.5f)) * value0 +
          (2.0f / 3.0f - rest * rest *
           (1.0f - rest * 0.5f)) * value1 +
          fraction * fraction * fraction * (1.0f / 6.0f) * right) *
         (1.0f / 8388608.0f);
}
#else
float bspline4(double fraction, float left, float value0, float value1,
                float right)
{
  const double rest = 1.0 - fraction;
  return (float)((rest * rest * rest / 6.0 * left +
                  (2.0 / 3.0 - fraction * fraction *
                   (1.0 - fraction * 0.5)) * value0 +
                  (2.0 / 3.0 - rest * rest *
                   (1.0 - rest * 0.5)) * value1 +
                  fraction * fraction * fraction / 6.0 * right) /
                 8388608.0);
}
#endif

}  // namespace

bool oscillator_next(struct xp_oscillator *oscillator, float *sample)
{
  if (!oscillator || !sample || oscillator->ended)
    return false;
#ifdef EMUSC_LEGACY_DSP_FAST
  /* The index is the accumulator's high half and the fraction its low
     half: no conversion, and so none of the FPU control-word juggling a
     double-to-integer cast costs on the 32-bit x87 target. The fraction
     is taken to kPhaseFloatFractionBits, every one of which a float's
     significand holds exactly, so this carries as much of it as the float
     kernel below can use. The accumulator itself keeps all of
     kPhaseFractionBits, which is what the playback rate rides on. */
  const size_t index = (size_t)(oscillator->phase_fixed >> kPhaseFractionBits);
  const float fraction =
    (float)(int32_t)((oscillator->phase_fixed >>
                      (kPhaseFractionBits - kPhaseFloatFractionBits)) &
                     ((UINT32_C(1) << kPhaseFloatFractionBits) - 1)) *
    (1.0f / (float)(UINT32_C(1) << kPhaseFloatFractionBits));
#else
  /* oscillator->phase is provably never negative: it starts at 0.0,
     advances by a non-negative step (pitch_word_rate can't return a
     negative rate), and every wrapped_phase branch also returns a
     non-negative value. A truncating cast of a non-negative double is
     the same integer as std::floor of it, so this skips std::floor's
     FPU-control-word juggling (real cost on the 32-bit x87 target;
     invisible on x86-64, which lowers std::floor to a single SSE2
     instruction) for a value that already truncates to the same place. */
  const size_t index = (size_t)oscillator->phase;
  const double fraction = oscillator->phase - (double)index;
#endif
  float left, value0, value1, right;
  if (!plain_taps(oscillator, index, &left, &value0, &value1, &right)) {
    if (!value_at(oscillator, index, &value0) ||
        !value_at(oscillator, index + 1, &value1))
      return false;
    left = outer_tap(oscillator, index, true, value0);
    right = outer_tap(oscillator, index + 1, false, value1);
  }
#ifdef EMUSC_LEGACY_DSP_FAST
  *sample = bspline4(fraction, left, value0, value1, right);

  oscillator->phase_fixed += oscillator->step_fixed;
  if (oscillator->initial &&
      (oscillator->phase_fixed >> kPhaseFractionBits) >=
        (uint64_t)oscillator->initial_count) {
    const uint64_t overflow = oscillator->phase_fixed -
      ((uint64_t)oscillator->initial_count << kPhaseFractionBits);
    if (!oscillator->cycle_count) {
      oscillator->ended = true;
    } else {
      oscillator->initial = false;
      oscillator->phase_fixed = wrapped_phase_fixed(oscillator, overflow);
    }
  } else if (!oscillator->initial &&
             (oscillator->phase_fixed >> kPhaseFractionBits) >=
               (uint64_t)oscillator->cycle_count) {
    oscillator->phase_fixed = wrapped_phase_fixed(
      oscillator, oscillator->phase_fixed -
        ((uint64_t)oscillator->cycle_count << kPhaseFractionBits));
  }
#else
  *sample = bspline4(fraction, left, value0, value1, right);

  oscillator->phase += oscillator->step;
  if (oscillator->initial &&
      oscillator->phase >= (double)oscillator->initial_count) {
    double overflow = oscillator->phase - oscillator->initial_count;
    if (!oscillator->cycle_count) {
      oscillator->ended = true;
    } else {
      oscillator->initial = false;
      oscillator->phase = wrapped_phase(oscillator, overflow);
    }
  } else if (!oscillator->initial &&
             oscillator->phase >= (double)oscillator->cycle_count) {
    oscillator->phase = wrapped_phase(
      oscillator, oscillator->phase - oscillator->cycle_count);
  }
#endif
  return true;
}

}}  // namespace EmuSC::Xp
