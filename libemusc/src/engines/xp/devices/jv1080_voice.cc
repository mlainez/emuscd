/* SPDX-License-Identifier: CC0-1.0 */
/*
 *  Roland JV-1080 behavioural voice model for the XP engine (engines/xp/).
 *
 *  Separate from jv1080.cc, which is pure profile data that the engine's
 *  identification links in unconditionally: a test that only needs this
 *  device's profile to exist should not have to link a voice model and the
 *  packed-record reader behind it.
 */
#include "jv1080.h"

/* ======================================================================
 *  The behavioural voice model.
 *
 *  Every law below carries the measurement it comes from. None of it is
 *  firmware-exact and none of it may be cited as such: this device's
 *  synthesis engine is in the SH7034's undumped 64 KB internal mask ROM,
 *  so what exists is the transfer function measured at the machine's
 *  output, not the code that produces it. Where a law is measured at too
 *  few points to pin its shape, that is written down next to it rather
 *  than smoothed over.
 * ====================================================================== */

#include "../common/constants.h"
#include "../packed_rom.h"
#include "../rom.h"
#include "../wave_cache.h"

#include <cmath>
#include <cstring>

namespace EmuSC { namespace Xp {

namespace {

/* @provenance class=MEASURED devices=JV-1080 ref=M-018
   The A-ENV's level field is its own table, not the
   level fields' square law - the two differ by 10.08 dB at worst. These are
   the eleven points, in dB relative to value 127, read with all four times
   at zero so the note is a rectangle at one level and the envelope is
   standing still. Value 0 is the noise floor.

   Its closed form is not established; the eleven points ARE the table, and
   between them this interpolates in dB, which is interpolation and not a
   recovered law. The slope converges to 0.391 dB per step at the top and
   steepens to 0.96 dB per step at value 8, so it is neither the square law
   nor a constant-dB table. */
const struct { uint8_t value; double db; } kAmpEnvLevelTable[] = {
  {   8u, -56.65 }, {  16u, -48.98 }, {  24u, -43.60 }, {  32u, -39.24 },
  {  48u, -31.71 }, {  64u, -24.93 }, {  80u, -18.47 }, {  96u, -12.13 },
  { 112u,  -5.86 }, { 127u,   0.00 },
};

/* @provenance class=MEASURED devices=JV-1080 ref=M-015,M-023
   Pan is a constant-power law, antisymmetric
   about value 64, and the tone and part fields index one table. These are
   the measured channel differences in dB at each distance from centre,
   after the 0.97 dB that belongs to the capture interface was cancelled by
   recording the sweep twice with L and R interchanged.

   It is a TABLE and not a trigonometric or linear law: the calibrated curve
   misses cos/sin by 2.05 dB and a linear pan by 2.07. So this interpolates
   between the measured distances rather than evaluating a formula.

   The two points past 48 are this lane's own measurement on the device, a
   tone-pan sweep of one dry part with the part's pan centred, and they
   exist because a straight line from 48 to 64 was 17 dB too wide at 56.
   The sweep also re-measures the four points above, independently and in
   the other direction: 2.21, 4.50, 9.23 and 15.50 dB against `M-015`'s
   2.20, 4.50, 9.20 and 15.60, after the 0.46 dB the interface contributes
   at centre was removed. The 60 point is the same rig reading a PART pan
   of 60 rather than a tone pan, which is a table point only under
   `M-002`/`M-048`'s finding that the two fields index one table.

   The last entry is NOT a measured difference. At a distance of 63 the
   quiet channel sits on the recorder's own noise floor (-95.9 dBFS, the
   same figure as the silence between notes), so all that is established is
   that the separation is at least 45 dB and the corner is effectively
   hard; 65 dB is a stand-in for "hard" and the curve's true shape over the
   last few steps is not recovered. */
const struct { uint8_t distance; double difference_db; } kPanTable[] = {
  {  0u,  0.00 }, {  8u,  2.20 }, { 16u,  4.50 }, { 32u,  9.20 },
  { 48u, 15.60 }, { 56u, 23.22 }, { 60u, 30.57 }, { 64u, 65.00 },
};


double interpolate_points(const double *xs, const double *ys, unsigned count,
                           double x)
{
  if (x <= xs[0])
    return ys[0];
  for (unsigned i = 1; i < count; ++i) {
    if (x <= xs[i]) {
      double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
      return ys[i - 1] + t * (ys[i] - ys[i - 1]);
    }
  }
  return ys[count - 1];
}

/* THE LAST STEPS ARE NOT SYMMETRIC (`M-047`, `P-xxxx`): at a distance of
   63 the right-hand side is already hard - tone pan 127 reads -65.4 dB, on
   the recorder's floor - where the left-hand side is not: the alternate-pan
   takes land a note at tone pan 1 at +44.5, 21 dB inside its own hard
   corner (pan 0, +65.7). So a right-hand distance of 63 is the stand-in
   "hard" value, and a left-hand one is 44.5 dB. */
const double kPanLeft63Db = 44.5;

double pan_difference_asymmetric(double offset)
{
  const unsigned count = (unsigned)(sizeof kPanTable / sizeof kPanTable[0]);
  double xs[sizeof kPanTable / sizeof kPanTable[0] + 1];
  double ys[sizeof kPanTable / sizeof kPanTable[0] + 1];
  double d = offset < 0.0 ? -offset : offset;
  unsigned n = 0;
  for (unsigned i = 0; i < count; ++i) {
    if (kPanTable[i].distance == 64u) {
      xs[n] = 63.0;
      ys[n++] = offset < 0.0 ? kPanLeft63Db : kPanTable[i].difference_db;
    }
    xs[n] = kPanTable[i].distance;
    ys[n++] = kPanTable[i].difference_db;
  }
  double m = interpolate_points(xs, ys, n, d);
  return offset < 0.0 ? -m : m;
}

/* interpolate_points over the level table, read in place: this runs once
   per sample per voice inside an envelope segment. */
double amp_env_level_db(double units)
{
  const unsigned count =
    (unsigned)(sizeof kAmpEnvLevelTable / sizeof kAmpEnvLevelTable[0]);
  if (units <= (double)kAmpEnvLevelTable[0].value)
    return kAmpEnvLevelTable[0].db;
  for (unsigned i = 1; i < count; ++i) {
    const double x1 = kAmpEnvLevelTable[i].value;
    if (units <= x1) {
      const double x0 = kAmpEnvLevelTable[i - 1].value;
      const double y0 = kAmpEnvLevelTable[i - 1].db;
      const double t = (units - x0) / (x1 - x0);
      return y0 + t * (kAmpEnvLevelTable[i].db - y0);
    }
  }
  return kAmpEnvLevelTable[count - 1].db;
}

/* A level in the record's own 0-127 units as linear amplitude, through
   the level table, for any point between two values. Below the table's
   first entry, 8, the amplitude is taken as linear down to zero: that
   stretch is not measured. */
double amp_env_units_amplitude(double units)
{
  if (units <= 0.0)
    return 0.0;
  const double first = kAmpEnvLevelTable[0].value;
  if (units < first)
    return std::pow(10.0, kAmpEnvLevelTable[0].db / 20.0) * units / first;
  return std::pow(10.0, amp_env_level_db(units) / 20.0);
}

#ifdef EMUSC_LEGACY_DSP_FAST
/* Samples between two exact level reads: a millisecond at the engine's
   own 32 kHz. */
const unsigned kEnvelopeResyncSamples = 32u;

/* Which interval of the level table `units` falls in: i for
   (value[i-1], value[i]], 0 at or below the first entry and the table's
   count above the last. Inside one interval the level is linear in dB, so
   a segment moving linearly in units moves by one ratio per sample. */
unsigned amp_env_level_interval(double units)
{
  const unsigned count =
    (unsigned)(sizeof kAmpEnvLevelTable / sizeof kAmpEnvLevelTable[0]);
  if (units <= (double)kAmpEnvLevelTable[0].value)
    return 0u;
  for (unsigned i = 1; i < count; ++i)
    if (units <= (double)kAmpEnvLevelTable[i].value)
      return i;
  return count;
}

/* amp_env_units_amplitude for a voice in a moving segment: the exact read
   once per kEnvelopeResyncSamples and whenever the interval changes, and
   in between the last read scaled by the segment's constant ratio. `step`
   is the segment's change in units per sample. Below the table and above
   it there is no pow() to remove, and those reads stay exact. */
double amp_env_units_amplitude_carried(struct XpJv1080Voice *voice,
                                       double units, double step)
{
  const unsigned count =
    (unsigned)(sizeof kAmpEnvLevelTable / sizeof kAmpEnvLevelTable[0]);
  const unsigned interval = amp_env_level_interval(units);
  if (voice->envelope_ratio_valid && voice->envelope_ratio_countdown &&
      interval == voice->envelope_ratio_interval) {
    --voice->envelope_ratio_countdown;
    return voice->envelope_units_amplitude * voice->envelope_ratio;
  }
  const double amplitude = amp_env_units_amplitude(units);
  if (interval == 0u || interval >= count) {
    voice->envelope_ratio_valid = false;
    return amplitude;
  }
  if (!voice->envelope_ratio_valid ||
      interval != voice->envelope_ratio_interval) {
    const double dbPerUnit =
      (kAmpEnvLevelTable[interval].db - kAmpEnvLevelTable[interval - 1u].db) /
      ((double)kAmpEnvLevelTable[interval].value -
       (double)kAmpEnvLevelTable[interval - 1u].value);
    voice->envelope_ratio = std::pow(10.0, dbPerUnit * step / 20.0);
    voice->envelope_ratio_interval = interval;
    voice->envelope_ratio_valid = true;
  }
  voice->envelope_ratio_countdown = kEnvelopeResyncSamples - 1u;
  return amplitude;
}
#endif

/* The inverse: where between 0 and 127 a linear amplitude stands. */
double amp_env_amplitude_units(double amplitude)
{
  if (amplitude <= 0.0)
    return 0.0;
  const double first = kAmpEnvLevelTable[0].value;
  const double floor = std::pow(10.0, kAmpEnvLevelTable[0].db / 20.0);
  if (amplitude < floor)
    return first * amplitude / floor;
  double db = 20.0 * std::log10(amplitude);
  double xs[10], ys[10];
  for (unsigned i = 0; i < 10; ++i) {
    xs[i] = kAmpEnvLevelTable[i].db;
    ys[i] = kAmpEnvLevelTable[i].value;
  }
  return interpolate_points(xs, ys, 10u, db);
}

/* The RIGHT channel's level minus the LEFT channel's, in dB, for an offset
   that is positive to the right. */
double pan_difference_db(int offset)
{
  return pan_difference_asymmetric((double)offset);
}

/* @provenance class=MEASURED devices=JV-1080 ref=M-009,M-019,M-048
   One square-law table, indexed by
   five different fields - tone level, patch level, part level, tone output
   level and CC7 - to a worst deviation of 0.41 dB and often under 0.15.
   `gain = (value/127)^2`. */
double square_law_gain(unsigned value)
{
  double v = (double)(value > 127u ? 127u : value) / 127.0;
  return v * v;
}

/* MEASURED (`M-081`, `closeout/cc7_residual`): CC7 indexes the same
   square law from value 9 up, and below it the machine's own points: 0, 1
   and 2 all read -81.7 dB, the floor, and 3, 4, 6 and 8 read -70.7, -62.8,
   -54.2 and -48.7 dB, running 5.6, 2.8, 1.2 and 0.7 dB under the law as
   they converge on it. Between those points the level is interpolated in
   dB, which is not recovered. */
double cc7_gain(unsigned value)
{
  static const double xs[] = { 2.0, 3.0, 4.0, 6.0, 8.0 };
  static const double ys[] = { -81.7, -70.7, -62.8, -54.2, -48.7 };
  if (value > 127u)
    value = 127u;
  if (value <= 8u)
    return std::pow(10.0, interpolate_points(xs, ys, 5u, (double)value) / 20.0);
  double v = (double)value / 127.0;
  return v * v;
}

/* MEASURED (`M-011`): the A-ENV's times 2, 3 and 4 index one exponential
   table - the same table to within 5 % - and a 20 dB fall takes 50 ms at
   value 16, 675 ms at 64 and 4.3 s at 104, doubling every 13.2 value steps
   as a fit. So the field is a RATE in dB per second, and a segment's
   duration is how far it has to travel at that rate.

   MEASURED, the table that fit stands for: the time for a 20 dB fall
   read off every take on disk that holds one segment, `aenv_t2_short`,
   `aenv_t3_short` and `aenv_t4_*` for values 0 to 64 in eights and
   `aenv_t2_080/104/127` and `aenv_t4_096/127` above, on an envelope
   smoothed over one period of the note. Where two or three segments were
   read at one value they agree to 5 % - 18.0 to 21.0 ms at 8, 656 to
   674 ms at 64, 12.2 to 12.5 s at 127 - and the mean is what is listed.

   The fit is off at both ends: 0 is instant here (within the smoothing's
   4 ms) where it asks for 23 ms, 8 is 19.7 ms against 36, and 127 is
   12.3 s against 18.5; the table doubles every 7 to 11 steps below 32 and
   every 14 to 15 above 64. Between the listed values the time is
   interpolated in its logarithm, and from 0 to 8 in the time itself; that
   is not recovered, and the machine's own table is in its internal ROM. */
struct FallPoint {
  unsigned value;
  double ms;
};
const struct FallPoint kAmpEnvFallTable[] = {
  { 0u, 0.0 },      { 8u, 19.7 },     { 16u, 43.5 },    { 24u, 76.7 },
  { 32u, 128.2 },   { 40u, 201.3 },   { 48u, 303.7 },   { 56u, 456.3 },
  { 64u, 667.2 },   { 80u, 1424.0 },  { 96u, 2906.0 },  { 104u, 4290.0 },
  { 127u, 12317.0 },
};

double amp_env_fall_seconds_per_20db(unsigned value)
{
  const unsigned count =
    (unsigned)(sizeof kAmpEnvFallTable / sizeof kAmpEnvFallTable[0]);
  if (value >= kAmpEnvFallTable[count - 1].value)
    return kAmpEnvFallTable[count - 1].ms / 1000.0;
  for (unsigned i = 1; i < count; ++i) {
    const FallPoint &b = kAmpEnvFallTable[i];
    if (value > b.value)
      continue;
    const FallPoint &a = kAmpEnvFallTable[i - 1];
    double t = (double)(value - a.value) / (double)(b.value - a.value);
    double ms = a.ms > 0.0 ? a.ms * std::pow(b.ms / a.ms, t)
                           : a.ms + t * (b.ms - a.ms);
    return ms / 1000.0;
  }
  return 0.0;
}

/* MEASURED ON THE DEVICE, fourteen points across the field, with the wave's
   own onset taken out of the way.
 *
 *   The stimulus is the internal `Sine` wave - a looped sine, so it has no
 *   attack of its own and the output IS the envelope - played at key 84
 *   (1046.5 Hz) with the filter switched off, every LFO depth zeroed, the
 *   two later envelope levels at the top so the segment ends on a plateau,
 *   and the release at zero so notes cannot run into each other. The
 *   envelope is read off by quadrature demodulation at the carrier, which
 *   gives amplitude at about 1 ms resolution.
 *
 *   These are the full travels, in seconds, from silence to the level the
 *   segment is heading for. They are NOT the 0-to-90 % rise `M-011`
 *   reports, which is 0.74 of this; converting M-011's three points that
 *   way, its 1210 ms at value 64 against this sweep's 1183 ms agrees to
 *   2 %, its 2560 ms at 80 against 2290 ms to 11 %, and its 35 ms at 8
 *   against 29 ms to 17 % - an independent corroboration on a different
 *   wave and rig, which is why the two disagreeing at the BOTTOM of the
 *   field matters: extrapolating one exponential through M-011's points
 *   put value 0 at 21 ms where the machine takes about 2, and that is a
 *   pick transient smeared into a whisper on every tone that asks for the
 *   fastest attack. Six of the first factory song's tones do.
 *
 *   NOT ONE EXPONENTIAL, and now it is clear why nobody could fit one: the
 *   doubling interval widens steadily up the field - about 10.7 value steps
 *   between 8 and 64, 12.0 between 16 and 64, 13.4 between 32 and 64 and
 *   14.1 between 48 and 64. So this interpolates the measured points in log
 *   time, and above the top of the table continues at that last measured
 *   slope, which is an extrapolation and says so.
 *
 *   The first entry is at the measurement's own floor: 2 ms is one
 *   demodulator window, so all that is established at value 0 is "at or
 *   below 2 ms", i.e. instant for any purpose. */
const struct { uint8_t value; double seconds; } kAmpEnvAttackTable[] = {
  {  0u, 0.00204 }, {  1u, 0.00254 }, {  2u, 0.00817 }, {  3u, 0.01001 },
  {  4u, 0.01831 }, {  6u, 0.03137 }, {  8u, 0.03946 }, { 12u, 0.06668 },
  { 16u, 0.09821 }, { 24u, 0.18481 }, { 32u, 0.30751 }, { 48u, 0.73174 },
  { 64u, 1.60289 }, { 80u, 3.09262 },
};

double amp_env_attack_seconds(unsigned value)
{
  const unsigned count =
    (unsigned)(sizeof kAmpEnvAttackTable / sizeof kAmpEnvAttackTable[0]);
  double v = (double)value;
  if (v <= kAmpEnvAttackTable[0].value)
    return kAmpEnvAttackTable[0].seconds;
  for (unsigned i = 1; i < count; ++i) {
    if (v <= kAmpEnvAttackTable[i].value) {
      double x0 = kAmpEnvAttackTable[i - 1].value;
      double x1 = kAmpEnvAttackTable[i].value;
      double y0 = std::log(kAmpEnvAttackTable[i - 1].seconds);
      double y1 = std::log(kAmpEnvAttackTable[i].seconds);
      return std::exp(y0 + (v - x0) * (y1 - y0) / (x1 - x0));
    }
  }
  /* Past the last measured point, continue at the slope of the last
     measured interval. Extrapolation, not measurement. */
  double x0 = kAmpEnvAttackTable[count - 2].value;
  double x1 = kAmpEnvAttackTable[count - 1].value;
  double y0 = std::log(kAmpEnvAttackTable[count - 2].seconds);
  double y1 = std::log(kAmpEnvAttackTable[count - 1].seconds);
  return std::exp(y1 + (v - x1) * (y1 - y0) / (x1 - x0));
}

/* MEASURED ON THE DEVICE, from the same sweep: the SHAPE the attack segment
   traverses, as the fraction of its travel reached at each fraction of its
   duration. It is not a straight amplitude ramp - it is front-loaded, half
   the travel being done in the first 31 % of the time.
 *
 *   Averaged over the six segment durations from value 12 to value 64,
 *   which span 24x in time, the fifteen points below have a worst standard
 *   deviation of 0.016 and are under 0.005 over most of the range - so the
 *   shape is scale-invariant, which is itself the finding. It is close to
 *   sin(pi t / 2T) but not equal to it: that curve misses by up to 0.019,
 *   consistently high past the middle, so the measured points are kept
 *   rather than the formula they resemble.
 *
 *   Measured only at full travel, from silence to the first level, which is
 *   what the stimulus set up; whether a shorter travel uses the same curve
 *   is not established. */
const double kAmpEnvAttackShape[15] = {
  0.116, 0.218, 0.308, 0.394, 0.476, 0.556, 0.630, 0.697,
  0.760, 0.816, 0.866, 0.905, 0.938, 0.966, 0.983,
};

/* One sample of the element, `offset` positions from the read head, with a
   loop read as the cycle it is: a position past the loop's end comes back
   round to its start, and the position one before its start is its end -
   but only once the head is inside the loop, since before that the element
   is still playing straight through and the sample behind really is the one
   behind it in memory. Where there is no such sample - a one-shot's two
   ends - the caller's inner tap stands in, which repeats a sample the note
   really has rather than inventing one and leaves the four weights summing
   to one, so a boundary cannot put a gain step or a DC offset into the
   output. The same choice `oscillator.cc` makes for the sibling engine. */
/* One position of a ping-pong loop's cycle, as a value.

   A ping-pong turn in a DIFFERENTIAL format is not a time reversal. The
   decoder accumulates deltas, so running the address back down the stream
   while still adding what it reads gives

       y[m] = x[c] + d[c] + ... + d[c-m+1] = 2*x[c] - x[c-m]

   - the loop backwards AND reflected about the value it turned at, which is
   continuous in value and in SLOPE. A plain time reversal leaves a corner
   and a forward wrap leaves a phase jump.

   The cycle is `2*span` long, `span = loop_last - loop_first + 1`:

     index < span     address loop_last-1 down to loop_first-1, REFLECTED
     index >= span    address loop_first up to loop_last, as decoded

   This ROM satisfies the invariant the turn needs. The reflected pass lands
   on x[loop_first-1] as its address reaches loop_first-1, and the cycle
   closes with no step and no drift only if x[loop_last] == x[loop_first-1].
   Decoded through this engine's own FCE decoder that holds with INTEGER
   EQUALITY on 204 OF 204 of this ROM's loop-type-1 elements - every one -
   against 1029 of 1031 for loop type 0. So loop_first-1 is answered with
   x[loop_last] directly rather than read: it can sit before the element's
   first decoded frame when a zone loops from its own start.

   THE SIBLING'S TELL-TALE IS ABSENT HERE, which is why the ROM alone could
   not settle it. On the SC-88 a forward read leaves a visible step and a
   phase jump at the join - correlation across it +0.197 against +0.968 for
   its forward loops. On this device both ends of the loop sit at EXACTLY
   ZERO: on the loop-type-1 elements examined sample by sample, x[loop_last]
   and x[loop_first-1] are both 0, and the step a FORWARD wrap would leave
   measures 0.23, 0.58 and 0.48 times the loop's own median sample-to-sample
   delta. A forward read is continuous in VALUE here, so the step says
   nothing either way.

   MEASURED ON THIS DEVICE'S OWN HARDWARE RECORDINGS (`TASK-346` AC#2), and
   the answer is the reflection. Rendering two preset rhythm kits both ways
   and comparing each against the machine's recording of the same key, in the
   key's own 520 ms slot: on the ELEVEN keys where the two readings differ at
   all, the reflected read matches the machine's sustained spectrum better on
   TEN, the one exception being a key whose correlation is 0.19 either way.
   The two tonal ones settle it outright. On the Long Whistle the machine's
   strongest partial is 2047.0 Hz: the reflected read puts it at 2038.9,
   -6.8 cents away, while the forward read's strongest is 2269.6 - it
   promotes to dominant what is only the machine's SECOND partial at 2277.
   On the Short Whistle the forward read adds partials at 3692 Hz that the
   machine does not have at all, where the reflected read's partial set is
   the machine's, 9418-9422 against 9416-9419. Both reproduce identically
   from two different kits, the same element through two banks. */
/* The ping-pong cycle of a voice whose loop lies inside its decoded span,
   with what every read of it shares worked out once. */
struct PingPongCycle {
  const int32_t *pcm;
  long long first;
  long long last;
  long long span;
  long long cycle;
  double turn;
};

bool ping_pong_cycle(const struct XpJv1080Voice *voice,
                     struct PingPongCycle *c)
{
  if (voice->loop_last >= voice->pcm_count ||
      voice->loop_last < voice->loop_first)
    return false;
  c->pcm = voice->pcm;
  c->first = (long long)voice->loop_first;
  c->last = (long long)voice->loop_last;
  c->span = c->last - c->first + 1;
  c->cycle = 2 * c->span;
  c->turn = (double)voice->pcm[voice->loop_last];
  return true;
}

/* `index` reduced into [0, cycle). Every per-sample caller passes an index
   at most one cycle outside it, where one add or subtract is the
   remainder; the division is kept for any other index. */
long long cycle_wrap(long long index, long long cycle)
{
  if (index < 0) {
    index += cycle;
    if (index < 0) {
      index %= cycle;
      if (index < 0)
        index += cycle;
    }
  } else if (index >= cycle) {
    index -= cycle;
    if (index >= cycle)
      index %= cycle;
  }
  return index;
}

/* `index` in [0, cycle). The forward pass reads loop_first..loop_last and
   the reflected one loop_last-1 down to loop_first-1, all inside the
   decoded span but for loop_first-1, which is the invariant's own answer,
   the turn value. */
double cycle_value(const struct PingPongCycle *c, long long index)
{
  if (index >= c->span)
    return (double)c->pcm[(size_t)(c->first + (index - c->span))];
  long long at = c->last - 1 - index;
  if (at < c->first)
    return c->turn;
  return 2.0 * c->turn - (double)c->pcm[(size_t)at];
}

double cycle_sample(const struct XpJv1080Voice *voice, long long index)
{
  struct PingPongCycle c;
  if (!ping_pong_cycle(voice, &c))
    return 0.0;
  return cycle_value(&c, cycle_wrap(index, c.cycle));
}

double wave_tap(const struct XpJv1080Voice *voice, size_t index, int offset,
                 double fallback)
{
  long long at = (long long)index + offset;
  /* A ping-pong element has not wrapped when the head runs past the loop's
     end - it has TURNED, so the sample after loop_last is the cycle's first
     reflected one. Behind the head it is still plain memory, because this
     branch is only reached before the first turn. */
  if (voice->ping_pong && voice->loop_last >= voice->loop_first &&
      at > (long long)voice->loop_last)
    return cycle_sample(voice, at - (long long)voice->loop_last - 1);
  if (!voice->ping_pong && voice->looping &&
      voice->loop_last >= voice->loop_first) {
    long long first = (long long)voice->loop_first;
    long long last = (long long)voice->loop_last;
    long long length = last - first + 1;
    if (at > last)
      at = first + (at - first) % length;
    else if (at < first && (long long)index >= first)
      at = last + 1 - (first - at);
  }
  if (at < 0 || (size_t)at >= voice->pcm_count)
    return fallback;
  return (double)voice->pcm[(size_t)at];
}

double amp_env_attack_shape(double done)
{
  if (done <= 0.0)
    return 0.0;
  if (done >= 1.0)
    return 1.0;
  double u = done * 16.0;                /* the table is on sixteenths */
  unsigned i = (unsigned)u;
  double f = u - (double)i;
  double a = i == 0u ? 0.0 : kAmpEnvAttackShape[i - 1u];
  double b = i >= 15u ? 1.0 : kAmpEnvAttackShape[i];
  return a + (b - a) * f;
}

/* @provenance class=MEASURED devices=JV-1080 ref=M-035
   Wave gain is exactly the display enum,
   -6 / 0 / +6 / +12 dB, within 0.06 dB. */
double wave_gain(unsigned raw)
{
  static const double db[4] = { -6.0, 0.0, 6.0, 12.0 };
  return std::pow(10.0, db[raw & 3u] / 20.0);
}

/* MEASURED (`M-012`, `M-118`): the two-pole section's natural frequency
   per cutoff value, which is where the resonant peak sits (`M-021`: 59, 293,
   809 Hz at cutoffs 40, 64, 80) and not the -3 dB corner. At resonance 0 the
   section's Q is 0.84 (see tvf_q), which puts the -3 dB corner 1.155x above
   the natural frequency - and M-012's corners, 335 Hz at cutoff 64 to 5930
   at 104, read exactly that far above these points. Two poles, -12.2 dB
   per octave above the corner (`M-012`, `M-076`).

   Cutoffs 40, 48 and 56 are two-pole fits to the res-0 white-noise sweep
   (`tvf/cutoff_lpf_res000`); 64 to 104 are M-012's saw-carrier corners
   divided by 1.155, which the same noise fits reproduce to 1.5 % at 64 to
   88. Outside 40-104 nothing is resolved - the interface's roll-off below,
   the fits' breakdown above - and the ends extend at 10 steps per octave,
   the -3 dB corners' own slope. Below cutoff about 20 the machine emits
   digital silence, as the extension does. At cutoffs 24 and 32 the saw
   take's harmonics, fitted the same way, read 22.1 and 38.6 Hz - 0.11 and
   0.12 octave above the extension, which is the margin by which that
   method reads above the noise fit at 40 (66.8 against 62 Hz), so the
   extension is not moved on it. What the saw take shows beyond that at
   low cutoffs sits within 15 dB of its own noise floor, 100 dB and more
   under full scale.

   These are the resonance-0 frequencies; with resonance the machine's
   natural frequency moves, which tvf_natural_hz applies.

   Measured on the low-pass; the other three types take the same frequency,
   which is not measured. */
const double kTvfNaturalHz[][2] = {
  { 40.0, 62.0 }, { 48.0, 104.0 }, { 56.0, 174.0 }, { 64.0, 290.0 },
  { 72.0, 499.0 }, { 80.0, 842.0 }, { 88.0, 1457.0 }, { 96.0, 2589.0 },
  { 104.0, 5134.0 } };

double tvf_cutoff_hz(double cutoff)
{
  const unsigned n = sizeof kTvfNaturalHz / sizeof kTvfNaturalHz[0];
  if (cutoff <= kTvfNaturalHz[0][0])
    return kTvfNaturalHz[0][1] *
      std::pow(2.0, (cutoff - kTvfNaturalHz[0][0]) / 10.0);
  if (cutoff >= kTvfNaturalHz[n - 1][0])
    return kTvfNaturalHz[n - 1][1] *
      std::pow(2.0, (cutoff - kTvfNaturalHz[n - 1][0]) / 10.0);
  unsigned i = 1;
  while (kTvfNaturalHz[i][0] < cutoff)
    ++i;
  double t = (cutoff - kTvfNaturalHz[i - 1][0]) /
    (kTvfNaturalHz[i][0] - kTvfNaturalHz[i - 1][0]);
  return std::exp(std::log(kTvfNaturalHz[i - 1][1]) +
                  t * (std::log(kTvfNaturalHz[i][1]) -
                       std::log(kTvfNaturalHz[i - 1][1])));
}

/* MEASURED (`M-118`, `M-123`): with resonance the natural frequency falls
   under the resonance-0 one, and further at higher cutoffs. As a factor on
   tvf_cutoff_hz, per cutoff at resonance 32, 64, 96, 112 and 120:

     cutoff 64   1 throughout - the noise fits read 290-293 Hz at every
                 resonance.
     cutoff 80   0.971 0.957 0.957 0.957 0.957 - two-pole fits to the
                 white-noise resonance sweep (818, 806 Hz against 842).
     96, 104, 112  sine sweeps, one note per semitone against the same
                 file's filter-OFF sweep; the peak located by a log-parabola
                 and taken to the natural frequency by the two-pole's own
                 peak offset at the resonance's Q (0.968 at 32, 0.996 at 64,
                 1 above):
       96   0.890 0.865 0.857 0.856 0.856
       104  0.773 0.732 0.724 0.722 0.721
       112  0.797 0.713 0.697 0.693 0.691

   At cutoff 96 the machine is a two-pole to within 0.3 dB rms and these
   factors carry its response. AT 104 AND 112 IT IS NOT: a two-pole fit
   there leaves 2 to 4.5 dB rms, so the peak lands where the machine's does
   but the shape around it does not follow; the structure behind it is not
   resolved (`L-05`). Resonance 127 takes 120's factor (its own peak reading
   at 112 is not usable); cutoffs above 112 take 112's, and below 64 none.
   Between resonance 0 and 32 nothing is measured and the factor runs
   straight from 1. */
const double kDriftCutoff[] = { 64.0, 80.0, 96.0, 104.0, 112.0 };
const double kDriftResonance[] = { 0.0, 32.0, 64.0, 96.0, 112.0, 120.0 };
const double kDrift[5][6] = {
  { 1.0, 1.0,   1.0,   1.0,   1.0,   1.0   },
  { 1.0, 0.971, 0.957, 0.957, 0.957, 0.957 },
  { 1.0, 0.890, 0.865, 0.857, 0.856, 0.856 },
  { 1.0, 0.773, 0.732, 0.724, 0.722, 0.721 },
  { 1.0, 0.797, 0.713, 0.697, 0.693, 0.691 } };

/* THE TOP OF THE RANGE, MEASURED (`P-xxxx`, TASK-381, the same takes as
   set_biquad's). The cutoff saturates: at resonance 0 every type reads the
   same at 112, 120 and 127, band for band; at resonance 64 and 127, 120
   and 127 are identical and 112 is not. Between 0 and 64 nothing is
   measured, and every resonance above 0 takes 120. With resonance the
   peak then stops at 7781 Hz, on `Synth Saw 2` and on White Noise alike -
   `M-077`'s 7809 Hz and `M-078`'s 7891, which read it as a property of the
   wave; it is the filter's - and the natural frequency is held at 7809.
   The low-pass at resonance 0 from 112 up is the filter OFF, to 0.0 dB
   (tvf_bypassed). The other three types there are not reproduced by these
   sections at any natural frequency: at the extension's 8440 Hz the HPF
   reads within 2.7 dB from 350 Hz up and the BPF 5 to 10 dB under the
   machine; that regime is not resolved. The PKG at resonance 0 there is a
   high shelf, not any section at a natural frequency (set_pkg_top_shelf). */
const double kTvfPeakCeilingHz = 7809.0;

double tvf_natural_hz(double cutoff, unsigned resonance)
{
  double top = resonance ? 120.0 : 112.0;
  if (cutoff > top)
    cutoff = top;
  double hz = tvf_cutoff_hz(cutoff);
  if (!resonance)
    return hz;
  if (cutoff > kDriftCutoff[0]) {
    double r = (double)(resonance > 120u ? 120u : resonance);
    double at[5];
    for (unsigned c = 0; c < 5u; ++c)
      at[c] = interpolate_points(kDriftResonance, kDrift[c], 6u, r);
    hz *= interpolate_points(kDriftCutoff, at, 5u, cutoff);
  }
  return hz > kTvfPeakCeilingHz ? kTvfPeakCeilingHz : hz;
}

bool tvf_bypassed(int type, double cutoff, unsigned resonance)
{
  return (type == 1 || type > 4) && !resonance && cutoff >= 112.0;
}

/* MEASURED (`M-082`, P-xxxx TASK-407): the seven F-ENV velocity curves,
   ten points each, as the fraction of the envelope's travel IN CUTOFF
   UNITS - each note's resonant peak taken back through tvf_natural_hz at
   resonance 80, less the record's cutoff, over kFilterEnvDepthScale times
   the depth.

   Both takes behind the table are recorded at velocity sensitivity +50
   (their generators set it; one stimulus description says 74, which is
   not what the take carries), so this IS the response at +50 - the case
   filter_env_sensed_fraction reads without moving the velocity. Read from
   `closeout/fenv_vel_curve_shallow_c0..c6` (cutoff 24, depth +30, nothing
   saturates) and `tvf/fenv_vel_curve_c0..c6` (cutoff 40, depth +63, which
   resolves the low velocities the shallow take loses under the rig's
   33 Hz floor, and is clipped against the 7809 Hz ceiling above them).
   Where both resolve a point they agree within 0.01 - curve 0 at 32 and
   48 reads 0.264/0.258 and 0.382/0.382, curve 1 at 64 and 80 0.240/0.241
   and 0.356/0.362 - and the table takes their mean; elsewhere the one
   take that resolves it. Curve 0 is linear in velocity: 0.124 per 16 from
   32 up.

   Between the ten velocities this interpolates linearly, which is
   interpolation and not a recovered law. The zeros at the bottom of
   curves 0, 1, 2 and 5 are the deep take's readings at the record's own
   cutoff, within 1.5 units of it. */
const uint8_t kFilterEnvCurveVelocity[10] = {
  1u, 8u, 16u, 32u, 48u, 64u, 80u, 96u, 112u, 127u,
};

const double kFilterEnvCurve[7][10] = {
  { 0.0,   0.070, 0.130, 0.261, 0.382, 0.510, 0.634, 0.758, 0.882, 1.0 },
  { 0.0,   0.023, 0.034, 0.090, 0.157, 0.240, 0.359, 0.522, 0.734, 1.0 },
  { 0.0,   0.0,   0.0,   0.0,   0.034, 0.062, 0.121, 0.247, 0.510, 1.0 },
  { 0.034, 0.206, 0.330, 0.522, 0.643, 0.748, 0.829, 0.893, 0.952, 1.0 },
  { 0.208, 0.522, 0.634, 0.758, 0.828, 0.877, 0.918, 0.950, 0.977, 1.0 },
  { 0.0,   0.0,   0.023, 0.054, 0.151, 0.522, 0.864, 0.950, 0.985, 1.0 },
  { 0.062, 0.271, 0.359, 0.437, 0.465, 0.504, 0.529, 0.566, 0.648, 1.0 },
};

/* MEASURED ON THE DEVICE - a depth sweep with the filter's corner read
   directly.

   The stimulus isolates the scale from everything it used to be tangled
   with: the internal `White Noise` wave at its own root key, so the source
   spectrum is flat and known; LPF with resonance 100, so the corner shows
   as a peak; base cutoff 40; the F-ENV holding at its top with level 1 at
   127 and time 1 at 0; and - the point of it - THE VELOCITY SENSITIVITY AT
   ZERO, so the depth's own scale is observed rather than its product with
   the sensitivity's. The corner is read as the peak of the note's spectrum
   divided by the same wave's spectrum with the filter switched off.

   Depths +6, +12, +18 and +24 put the peak at 181.6, 521.5, 1502.9 and
   4347.7 Hz. The scale is in the units THIS ENGINE turns into a frequency,
   so the peaks are taken back through tvf_natural_hz at resonance 100
   (P-xxxx, TASK-403): cutoff 56.67, 73.05, 89.98 and 106.44, a straight
   line of

       2.771 cutoff units per unit of depth

   whose intercept, 39.98, is the 40 the cutoff was set to. A fifth point
   at depth +30 is excluded: its corner lands at 12 kHz where the machine's
   own 32 kHz output and its reconstruction filter flatten the peak.

   Three readings on other takes, all converted the same way, agree:
   `M-082`'s velocity-127 endpoint (depth +30 from cutoff 24, resonance
   80, 6.80 octaves over a velocity-1 note at 41 Hz) gives 2.769; the
   resonant peak of `tvf/fenv_level_sweep` (depth +63, levels 48 and 64
   held) 2.75 and 2.77; and where the attack and decay lines of
   `tvf/fenv_depth_p32` and `_p63` meet, 2.80 and 2.79. Read through
   `M-012`'s `fc = 341 * 2^((cutoff-64)/10)` instead, the same four peaks
   give 2.545 - a law of ten units per octave that tvf_cutoff_hz does not
   follow above cutoff 80, where it runs eight to ten.

   Measured with the SENSITIVITY AT ZERO, and that case is confirmed
   rather than assumed: at depth +30 with sensitivity 0 the corner sits at
   8003.9, 8001.0 and 8003.9 Hz for velocities 1, 64 and 127: sensitivity
   zero is full depth at every velocity (filter_env_sensed_fraction). */
inline constexpr double kFilterEnvDepthScale = 2.771;

/* The record's velocity curve, interpolated between the table's ten
   points. A record type with no curve field is rendered on curve 0; which
   curve such a record uses is not established. */
double filter_env_curve_fraction(unsigned curve, double velocity)
{
  if (curve > 6u)
    curve = 0u;
  double xs[10], ys[10];
  for (unsigned i = 0; i < 10u; ++i) {
    xs[i] = kFilterEnvCurveVelocity[i];
    ys[i] = kFilterEnvCurve[curve][i];
  }
  return interpolate_points(xs, ys, 10u, velocity);
}

/* How far velocity lets the envelope travel, from the record's velocity
   curve and its velocity sensitivity (decoded -50..+75).

   MEASURED at four settings (P-xxxx, TASK-407), each note's resonant peak
   read as above, `White Noise` through the LPF at cutoff 40, resonance 80,
   depth +63, curve 0:
     sensitivity   0 (`tvf/fenv_vel_sens_000`): flat - every velocity at
                      the peak ceiling, as `M-146`'s depth +30 check reads
                      at 8 kHz for velocities 1, 64 and 127.
     sensitivity +50 (the curve takes above): the curve itself.
     sensitivity +75 (`tvf/fenv_vel_sens_p75`): velocities 1, 16 and 32 at
                      the record's cutoff, 64 at 41.5, 96 and 127 at the
                      ceiling. That is the curve read at 127 - 2 (127 - v):
                      64 maps to 1 (predicted 41.6), 96 to 65 (0.518 of
                      the travel, past the ceiling). Scaling the +50 result
                      by one and a half instead would put 64 at 1.1 kHz.
     sensitivity -50 (`tvf/fenv_vel_sens_m50`): the +50 result turned
                      over - 1 minus the curve - 96 at 82.1 against 82.2
                      predicted, 127 at the record's cutoff, 64 and below
                      at the ceiling.

   That is the A-ENV's law (sensed_velocity) - k 0, 1 and 2 at 0, 50 and
   75, the curve read at the velocity 127 - k (127 - v) - with the negative
   half taken as one minus the positive result at the same magnitude.

   NOT RECOVERED: k between the four settings is interpolated linearly as
   the A-ENV's is, and the factory library sits mostly between them (30 to
   75); and every take here is curve 0, which is linear, so whether a
   curve other than 0 is read at the moved velocity (this form) or has its
   result scaled about 1 (the 512-slot sweep scores the two within 0.1 dB
   median) is not settled by them. A velocity mapped below 1 reads the
   curve at 1. */
double filter_env_sensed_fraction(unsigned curve, int sensitivity,
                                  unsigned velocity)
{
  if (!sensitivity)
    return 1.0;
  int magnitude = sensitivity < 0 ? -sensitivity : sensitivity;
  double k = magnitude <= 50 ? magnitude / 50.0
                             : 1.0 + (magnitude - 50) / 25.0;
  double v = 127.0 - k * (127.0 - (double)velocity);
  double g = filter_env_curve_fraction(curve, v < 1.0 ? 1.0 : v);
  return sensitivity > 0 ? g : 1.0 - g;
}

/* MEASURED (`P-xxxx`, TASK-403): a filter envelope segment lasts 2.50
   times the amplitude envelope's 20 dB fall time at the same value, and
   moves the cutoff parameter linearly over that time.

   Read as the resonant peak's position in cutoff units (resonance 90 on
   `Synth Saw 2`, the peak located on the saw's harmonic comb and taken
   through tvf_natural_hz), fitted as a line over cutoff 66 to 115:
     attack, `tvf/fenv_t1_sweep` (depth +63): T1 32, 48, 64, 96 climb 545,
       230, 105 and 23.4 units per second;
     release, `gaps/fenv_t4_hold` (depth +63, the A-ENV held open): T4 32,
       48, 64 fall 531, 227 and 105;
     decay, `tvf/fenv_depth_p32` and `_p63`: T2 60 falls 65.6 and 131 -
       twice the rate at twice the depth, so the segment's duration does not
       depend on how far it moves.
   Each line is straight to 0.5-0.9 units rms. At kFilterEnvDepthScale
   (174.6 units at depth +63) those are segments of 0.32, 0.76, 1.66
   and 7.46 s for T1, 0.33, 0.77 and 1.66 s for T4 and 1.35 and 1.33 s for
   T2 at 60: 2.42 to 2.57 times kAmpEnvFallTable at every one of the nine,
   and 2.50 is their middle. The times 1, 2 and 4 read one table. Values below
   32 are faster than a 32 ms analysis window resolves and are not
   measured here; they take the same ratio, and value 0 is instant as the
   table's is.

   What is observed is the RATE, which is depth scale over time: the 2.50
   is correct together with kFilterEnvDepthScale and would move with it.
   The exact constant is not recovered - the nine readings spread 6 % -
   and the machine's own time table is in its internal ROM.

   `M-040`'s ratio of 1.64 to 1.92 is a different quantity: it compares
   20 dB falls in the energy above 1 kHz, which depend on how far the corner
   has to travel to take that energy away, not the envelope's own time.

   Every take above runs a segment over the whole 0-to-127 range. A segment
   that runs over PART of it lasts the same time (`P-xxxx`, TASK-416). Read
   on the factory patches whose decay stops short of level 0, as octave
   band levels in 5 ms steps on their corpus slots, the upper bands reach
   their held value where the segment's WHOLE time says, not its share of
   the range: Synth Bass 1 (T2 20, level 127 to 50) at about 160 ms, where
   the whole time is 145 and the share 88; Euro Bass (T2 15, 127 to 42,
   then T3 0) at 100 to 120 ms against 100 and 67; Moist Bass (T2 12 then
   T3 10, 97 to 53 to 0) at about 145 ms against 135 and 51. Rendered this
   way, those two and BritelowBass (T2 17, 127 to 72) follow the takes'
   band levels to within 1 to 3 dB through the decay. The amplitude
   envelope (amp_env_segment_seconds) and the pitch envelope
   (pitch_env_seconds) are measured to follow the same law, and the pitch
   envelope's times are these same numbers: 322, 767, 1681 and 7447 ms at
   32, 48, 64 and 96. */
inline constexpr double kFilterEnvTimeScale = 2.50;

double filter_env_segment_seconds(unsigned value)
{
  return kFilterEnvTimeScale * amp_env_fall_seconds_per_20db(value);
}

/* MEASURED (`M-069`): time key follow is one law on all three envelopes -
   a factor of two per octave of key at +-100 %, pivoting exactly on key
   60. `kf` is the field's percentage as a fraction, read from the device's
   own list by time_key_follow(). `M-066` measured on the A-ENV that it
   does NOT scale the attack; whether the filter envelope's own attack is
   likewise exempt was not measured, and this follows the amplitude
   envelope's rule. */
double time_key_follow_scale(double kf, unsigned key)
{
  return std::pow(2.0, -kf * ((double)key - 60.0) / 12.0);
}

/* MEASURED (`M-070`): velocity-time sensitivity pivots on velocity 64 and,
   at the enum's extremes, spans a factor of 1.30 in time from velocity 1
   to 127 - on the pitch and filter envelopes alike, to within 0.4 %. That
   is 0.39 of an octave across the WHOLE range, half of it each side of 64:
   `t = t_64 * 2^(-vs * 0.39 * (vel - 64) / 126)` with vs from -1 to +1.
   Read on `closeout/penv_vel_t1_i{00,14}` at 4.5 s into a time-96 ramp,
   the hardware travels 846 to 641 cents from velocity 1 to 127 at index 0
   and 644 to 838 at index 14 - spans of 1.32 and 1.30. */
double velocity_time_scale(unsigned enumValue, unsigned velocity)
{
  double vs = ((double)(enumValue > 14u ? 14u : enumValue) - 7.0) / 7.0;
  return std::pow(2.0, -vs * 0.39 * ((double)velocity - 64.0) / 126.0);
}

/* MEASURED (`P-xxxx`, TASK-402): the A-ENV's velocity-time sensitivity
   scales the attack LINEARLY in time with velocity, pivoting on 64:
   `t = t_64 * (1 - vs * 0.0090 * (vel - 64))`, vs from -1 to +1 across the
   15-entry enum. At the extremes that is x1.57 at velocity 1 and x0.43 at
   127 - a span of 3.7, not the 1.30 `M-070` measured on the pitch and
   filter envelopes' own fields, which `closeout/fenv_vel_t1_i*` confirm
   (the band-crossing times span 1.30 there too).

   Read on `envelopes/aenv_vel_t1_i{00,07,14}` (A-ENV T1 64, level velocity
   sensitivity 0, filter off) as the 10-90 % amplitude rise: index 7 is
   1.042 s at all six velocities; index 14 gives 1.606, 1.480, 1.330,
   1.028, 0.712 and 0.432 s at velocities 1, 16, 32, 64, 96 and 127, and
   index 0 their mirror about 64 (0.436 ... 1.610). As a ratio to index 7
   the points lie on one straight line in velocity to 0.017 rms, and not on
   one in log time (log2 ratio 0.62, 0.51, 0.35, -0.02, -0.55, -1.27). A
   free line puts the pivot at 63.6 and the slope at 0.0090 per velocity;
   the index-14 and index-0 end points alone give 0.0091.

   NOT RECOVERED: every take is time 1 = 64, so that the scale is a factor
   on the time (rather than an offset in seconds) at other values is
   assumed, as for the other envelopes; indices between 0, 7 and 14 are
   interpolated linearly; and the rhythm note's one field is unmeasured.
   The time-4 field is NOT applied: `envelopes/aenv_vel_t4_i{00,07,14}`
   (T4 64, each note released at 800 ms) fall 10, 20 and 40 dB in 0.34,
   0.68 and 1.28 s at every velocity and index, within 1.5 %. Their
   note-offs all carry velocity 64, so whether the note-OFF velocity
   drives that field is not measured. */
inline constexpr double kAmpEnvVelocityTimeSlope = 0.0090;

double amp_env_velocity_time_scale(unsigned enumValue, unsigned velocity)
{
  double vs = ((double)(enumValue > 14u ? 14u : enumValue) - 7.0) / 7.0;
  return 1.0 - vs * kAmpEnvVelocityTimeSlope * ((double)velocity - 64.0);
}

/* Resonance as the two-pole section's Q, in dB. The peak gain of a
   two-pole section is its Q for a Q well above unity; the chip's own
   coefficient form is unknown (`L-05`) and is not what this reproduces.

   MEASURED (`M-021`, `M-118`): from value 8 to 96 about 0.28 dB per step -
   two-pole fits to the white-noise resonance sweeps read within 0.6 dB of
   it at cutoffs 40, 64 and 80 - and at value 0 a Q of 0.84 (-1.76, -1.36
   and -1.56 dB there), not 1. Between 0 and 8 nothing is measured and the
   dB runs straight between the two.

   Above 96 the peak is narrower than those takes' spectra resolve, and
   M-021's peak readings there (46.7 dB at 127 on cutoff 64, 67.4 on 112)
   are resolution-limited. It is read instead from each note's ENERGY: our
   render of the same file with the filter off, passed through this section
   from the note-on over the same window, gives the energy any Q produces
   against Q 0.84, and the Q that matches the take's own ratio is the
   take's. The method reads our engine's own Q back to 0.25 dB. On the
   hardware it reads the same increments over value 96 at cutoffs 64, 80,
   96 and 112, within 0.5 dB: +3.5 at 104, +6.0 at 112, +12.4 at 120, and at
   127 at least +42 - the method's own ceiling, the machine at the edge of
   self-oscillation. Cutoff 40 reads lower and is not used: its peak, at
   59 Hz, is inside the interface's roll-off. The increments are added to
   the law's value at 96; between the measured values the dB runs straight.

   On the absolute level at 88 and 96 the two methods part by up to 2.5 dB
   (energy below the law, the fits above it at cutoff 64), which says the
   machine's peak is not exactly a two-pole's shape; that is not resolved. */
const double kTvfTopResonance[] = { 96.0, 104.0, 112.0, 120.0, 127.0 };
const double kTvfTopDb[] = { 26.88, 30.38, 32.88, 39.28, 68.88 };

double tvf_q(unsigned resonance)
{
  const double zeroDb = 20.0 * std::log10(0.84);
  double peakDb = resonance < 8u
    ? zeroDb + (0.28 * 8.0 - zeroDb) * (double)resonance / 8.0
    : resonance <= 96u
    ? 0.28 * (double)resonance
    : interpolate_points(kTvfTopResonance, kTvfTopDb, 5u,
                         (double)(resonance > 127u ? 127u : resonance));
  return std::pow(10.0, peakDb / 20.0);
}

/* A two-pole section per filter type. MEASURED (`M-017`): all four types
   are active and each has the response its name says - the type register
   reading zero says the type is carried elsewhere, not that the types are
   unused.

   THE REALISATION, MEASURED at the machine's 32 kHz (`P-xxxx`, TASK-381,
   `tvf/cutoff_{lpf,hpf,bpf,pkg}_res000` and `_res064`, White Noise, each
   slot against `filter_type_all_res000`'s filter-OFF note - the two
   sessions agree to 0.0 dB where a slot is transparent). Candidate
   sections share these poles and differ only in their zeros; on cutoffs
   64 to 104, in third-octave bands to 14 kHz:
     LPF  no zeros, unity at DC. Within 0.6 dB at 80, 88 and 96, where a
          bilinear section (a double zero at Nyquist) is 26 dB under at
          14 kHz. On the cookbook's poles it read up to 2.7 dB bright
          between 5 and 9 kHz at 104; its poles are the matched ones in
          set_biquad, which read up to 1.5 dB bright there (2.4 at
          11 kHz, where the filter-off slots read 1.6 bright as well).
     HPF  a double zero at DC and no scaling, so the passband rises over
          0 dB toward Nyquist as the machine's does (+5.6 dB at 9 kHz at
          104): 1.06 and 0.82 dB rms at resonance 0 and 64, against 2.61
          and 1.04 for a section scaled to unity at Nyquist.
     BPF  one zero, at DC, with the section's Q at the natural frequency:
          0.50 dB rms at resonance 64, where a peak held at 0 dB is 18 dB
          off; 1.36 at resonance 0, where its level runs 0.3 dB under at 64
          and 5 dB under at 104. Up to resonance 96 the peak of Q stands
          on the band-pass's own take at 64 and on the section's peak law,
          which the low-pass's resonance-96 take confirms to 0.0 to 1.3 dB
          rms. Above 96 the peak is held at 0 dB, on two poles and zeros at
          DC and Nyquist. What that boundary rests on is patches, not a
          take: a peak of Q is right for `Bassoon` (resonance 70) and
          `Velo Tekno 1` (90) and puts `Dissimilate` and `Tortured` (100)
          and `Biosphere` (127) over theirs, `Biosphere` by 19 dB, so the
          transition lies between 90 and 100. 96 is the nearest resonance
          with a measured law, not a measurement of the transition.
     PKG  as below; the one-zero variant reads 0.82 and 0.72 against this
          form's 1.23 and 0.49, so neither is preferred and this one stays.
          Both stay under the machine above its peak at 96 and 104, by up
          to 6.6 dB at 14 kHz. From cutoff 112 up at resonance 0 the PKG
          is a high shelf instead (set_pkg_top_shelf).
   These are the outputs of a state-variable section, the topology the
   sibling engine's own chip runs (tvf.cc). The poles are the cookbook
   biquad's at the natural frequency and Q below for HPF, BPF and PKG, and
   the analog section's mapped by z = e^(sT) for the LPF (set_biquad),
   which is this model's choice; the chip's own coefficient form is
   internal (`L-05`).

   This writes the coefficients and leaves the delay line alone, because
   the filter envelope re-solves it while the note is sounding and
   restarting the section every millisecond would put a step in the output
   at every control block. */
/* THE SECTION RUNS AS A STATE-VARIABLE FILTER. The coefficients above
   define the transfer function; what realises it is a trapezoidal
   state-variable section (the Simper form) whose poles and mix are solved
   from them exactly, so for a held cutoff the response is the direct
   form's to rounding. The two differ only while the coefficients move: a
   direct form's state is the last outputs, and when a square LFO or a
   step throws the corner from 1.5 kHz to 4 Hz in one control block that
   state drives a transient hundreds of times full scale. The state here
   is the two integrators', which carry over a coefficient change without
   gaining energy. What structure the machine's own filter uses is not
   known; this one is chosen because it cannot run away, which the
   hardware plainly does not. On `tvf/fenv_depth_p63` the falling sweep
   after the level peak reads within 1 dB of the hardware take, where the
   direct form reads 9 to 10 dB high (`M-142`).

   With s = (1/g)(z-1)/(z+1) on s^2 + k s + 1, the denominator is
   z^2 + a1 z + a2 when g^2 = (1+a1+a2)/(1-a1+a2) and
   g k = 4/(1-a1+a2) - 1 - g^2; the numerator b0 z^2 + b1 z + b2 is then
   A/4 (b0-b1+b2) of the high-pass output, A/(2g) (b0-b2) of the band-pass
   and A/(4g^2) (b0+b1+b2) of the low-pass, A = 4/(1-a1+a2). */
/* The trapezoidal integrators' three gains, a function of g and k alone,
   so they change here rather than per sample. */
void set_svf_gains(struct XpJv1080Voice *voice)
{
  const double g = voice->svf_g, k = voice->svf_k;
  voice->svf_h1 = 1.0 / (1.0 + g * (g + k));
  voice->svf_h2 = g * voice->svf_h1;
  voice->svf_h3 = g * voice->svf_h2;
}

void set_svf(struct XpJv1080Voice *voice)
{
  double lo = 1.0 + voice->a1 + voice->a2;
  double hi = 1.0 - voice->a1 + voice->a2;
  if (lo <= 0.0 || hi <= 0.0) {
    /* Not a stable pole pair: no section this engine builds is one. */
    voice->svf_g = 1.0;
    voice->svf_k = 2.0;
    set_svf_gains(voice);
    voice->m_hp = voice->m_lp = 1.0;
    voice->m_bp = 2.0;
    return;
  }
  double g2 = lo / hi;
  double g = std::sqrt(g2);
  double big = 4.0 / hi;
  voice->svf_g = g;
  voice->svf_k = (big - 1.0 - g2) / g;
  set_svf_gains(voice);
  voice->m_hp = big * (voice->b0 - voice->b1 + voice->b2) / 4.0;
  voice->m_bp = big * (voice->b0 - voice->b2) / (2.0 * g);
  voice->m_lp = big * (voice->b0 + voice->b1 + voice->b2) / (4.0 * g2);
}

void set_biquad(struct XpJv1080Voice *voice, int type, double fc,
                 double q, double rate, unsigned resonance)
{
  double nyquist = rate * 0.5;
  if (fc > nyquist * 0.99)
    fc = nyquist * 0.99;
  if (fc < 1.0)
    fc = 1.0;
  double w = 2.0 * 3.14159265358979323846 * fc / rate;
  double cs = std::cos(w);
  double sn = std::sin(w);
  double alpha = sn / (2.0 * q);
  double a0 = 1.0 + alpha;

  const double a1 = (-2.0 * cs) / a0;
  const double a2 = (1.0 - alpha) / a0;
  switch (type) {
  case 2: {                      /* BPF */
    if (resonance > 96u) {
      /* Past the measured resonances the peak is held at 0 dB. */
      voice->b0 = alpha / a0;
      voice->b1 = 0.0;
      voice->b2 = -alpha / a0;
      break;
    }
    /* One zero, at DC, and the section's Q at the natural frequency: the
       numerator's scale is what puts |H| there at q. */
    double re = 1.0 + a1 * cs + a2 * std::cos(2.0 * w);
    double im = a1 * sn + a2 * std::sin(2.0 * w);
    double k = q * std::sqrt(re * re + im * im) / (2.0 * std::sin(0.5 * w));
    voice->b0 = k;
    voice->b1 = -k;
    voice->b2 = 0.0;
    break;
  }
  case 3:                        /* HPF */
    voice->b0 = 1.0;
    voice->b1 = -2.0;
    voice->b2 = 1.0;
    break;
  case 4:
    /* PKG. MEASURED (`M-017`, `M-121`): a bump of TWICE the section's Q
       at the natural frequency, on the same poles - +4.83 dB and 2.17
       octaves wide at half height at resonance 0, +24.27 dB and 0.75
       octave at 64 (white noise, cutoff 64), where a gain of 2Q predicts
       4.51 dB / 2.05 octaves and 23.94 dB / 0.72, the widths unfitted.
       Across cutoffs 48 to 104 at resonance 64 the bump stays 6.3 to 6.9
       dB over Q; above 104 it grows further, which is not modelled. */
    voice->b0 = (1.0 + 2.0 * alpha * q) / a0;
    voice->b1 = (-2.0 * cs) / a0;
    voice->b2 = (1.0 - 2.0 * alpha * q) / a0;
    break;
  default: {                     /* LPF */
    /* THE LOW-PASS POLES ARE THE ANALOG SECTION'S, MAPPED BY z = e^(sT),
       not the cookbook's. MEASURED (`P-xxxx`, TASK-420): the two mappings
       agree to 0.2 dB up to cutoff 96 and part as the corner nears the
       7.8 kHz ceiling, where the cookbook poles over-peak a low-Q section.
       `tvf/resonance_cut112` (White Noise, each slot against the same
       take's resonance-0 slot, which is the filter off on both): at
       resonance 8 the machine peaks +2.9 dB at 7.5-8.5 kHz, cookbook
       poles +7.8, these +4.3. Third-octave rms ours-hw over 1-12 kHz,
       cookbook -> these, with nothing adjusted:
         resonance_cut112     res 8 2.39 -> 0.98, 16 2.01 -> 0.72,
                              24 1.83 -> 0.77, 32 1.40 -> 0.64
         cutoff_lpf_res000    cutoff 104 1.98 -> 1.05
         cutoff_lpf_res032    112 1.39 -> 0.64, 120 1.33 -> 0.47
         cutoff_lpf_res064    120 0.92 -> 0.59
       and no slot of those takes, `resonance_cut096` or
       `cutoff_lpf_res096` reads worse. At resonance 64 and above the two
       mappings are within 1 dB and both match the machine. The residual at
       cutoff 112 resonance 8 (+1.4 dB at the peak) says the machine's own
       form is still not this one (`L-05`). The other three types keep the
       cookbook poles: their high-cutoff regime is not resolved either way. */
    double r = std::exp(-w / (2.0 * q));
    double wd = w * std::sqrt(1.0 - 1.0 / (4.0 * q * q));
    double m1 = -2.0 * r * std::cos(wd);
    double m2 = r * r;
    voice->b0 = 1.0 + m1 + m2;
    voice->b1 = 0.0;
    voice->b2 = 0.0;
    voice->a1 = m1;
    voice->a2 = m2;
    set_svf(voice);
    return;
  }
  }
  voice->a1 = a1;
  voice->a2 = a2;
  set_svf(voice);
}

/* Where the filter envelope currently puts the cutoff parameter, clamped
   to the field's own 0..127 range. The top of that range saturates
   further, in tvf_natural_hz. */
double filter_env_cutoff(const struct XpJv1080Voice *voice)
{
  double cutoff = voice->cutoff_base + voice->cutoff_offset * voice->fenv_value +
    voice->lfo_cutoff + voice->matrix_cutoff;
  if (cutoff < 0.0)
    return 0.0;
  return cutoff > 127.0 ? 127.0 : cutoff;
}

/* PKG AT THE SATURATED TOP, RESONANCE 0, MEASURED (`P-xxxx`, TASK-429):
   from cutoff 112 up the machine's PKG is not a bump but a HIGH SHELF.
   On `tvf/cutoff_pkg_res000` (White Noise, key 60, velocity 100), each
   slot against the same take's cutoff-0 slot - which reads as the filter
   OFF on the machine, to 0.1 dB against `filter_type_all_res000`'s OFF
   slot, so the interface's own response cancels - the slots at 112, 120
   and 127 are identical to 0.01 dB and rise +0.34 dB at 1 kHz, +1.26 at
   2 kHz, +4.08 at 4 kHz, +7.56 at 6.4 kHz, +9.63 at 8 kHz, +11.66 at
   10.2 kHz and +13.22 at 12.8 kHz, still climbing at the top of the
   machine's 16 kHz band. The two-pole bump set_biquad builds reaches
   +4.5 dB there, 13 dB under.

   A shelf of the audio-EQ cookbook's form - corner 5850 Hz, +13.7 dB,
   slope 0.60 - designed at the machine's 32 kHz reproduces the take's
   37 sixth-octave bands from 200 Hz to 12.8 kHz to 0.05 dB rms (worst
   0.1 dB). Those three numbers are a fit of that form to the take, not
   values read from the ROM; the chip's own coefficients are internal
   (`L-05`). At another output rate the same corner, gain and slope are
   designed at that rate.

   The factory songs carry it: STORM's own rhythm set puts PKG, cutoff
   127, resonance 0 on most of the keys it plays, and with this shelf on
   those keys alone its 2-16 kHz reads within about 1.5 dB of both
   hardware takes where the bump left it 3 to 9 dB dark.

   NOT RESOLVED: the machine's PKG between cutoffs 104 and 112, where the
   take has no slot - at 96 and 104 it is a bump that settles to a plateau
   of +3.3 and +6.8 dB above its peak rather than returning to 0 dB, which
   set_biquad does not reproduce - and resonance above 0 at the top, which
   `cutoff_pkg_res064` holds and this does not read. Both stay on
   set_biquad's section. */
inline constexpr double kPkgTopShelfHz = 5850.0;
inline constexpr double kPkgTopShelfDb = 13.7;
inline constexpr double kPkgTopShelfSlope = 0.60;

bool pkg_top_shelf(int type, double cutoff, unsigned resonance)
{
  return type == 4 && !resonance && cutoff >= 112.0;
}

void set_pkg_top_shelf(struct XpJv1080Voice *voice, double rate)
{
  double fc = kPkgTopShelfHz;
  if (fc > rate * 0.5 * 0.99)
    fc = rate * 0.5 * 0.99;
  double a = std::pow(10.0, kPkgTopShelfDb / 40.0);
  double w = 2.0 * 3.14159265358979323846 * fc / rate;
  double cs = std::cos(w);
  double alpha = std::sin(w) / 2.0 *
    std::sqrt((a + 1.0 / a) * (1.0 / kPkgTopShelfSlope - 1.0) + 2.0);
  double sq = 2.0 * std::sqrt(a) * alpha;
  double a0 = (a + 1.0) - (a - 1.0) * cs + sq;
  voice->b0 = a * ((a + 1.0) + (a - 1.0) * cs + sq) / a0;
  voice->b1 = -2.0 * a * ((a - 1.0) + (a + 1.0) * cs) / a0;
  voice->b2 = a * ((a + 1.0) + (a - 1.0) * cs - sq) / a0;
  voice->a1 = 2.0 * ((a - 1.0) - (a + 1.0) * cs) / a0;
  voice->a2 = ((a + 1.0) - (a - 1.0) * cs - sq) / a0;
  set_svf(voice);
}

/* The section for where the voice's cutoff and resonance now stand. */
void set_filter(struct XpJv1080Voice *voice, double rate)
{
  double cutoff = filter_env_cutoff(voice);
  if (tvf_bypassed(voice->filter_type, cutoff, voice->resonance_value)) {
    voice->b0 = 1.0;
    voice->b1 = voice->b2 = voice->a1 = voice->a2 = 0.0;
    set_svf(voice);
    return;
  }
  if (pkg_top_shelf(voice->filter_type, cutoff, voice->resonance_value)) {
    set_pkg_top_shelf(voice, rate);
    return;
  }
  set_biquad(voice, voice->filter_type,
             tvf_natural_hz(cutoff, voice->resonance_value),
             voice->resonance_q, rate, voice->resonance_value);
}

/* Enter a segment, from wherever the envelope currently stands. It lasts
   its own time however far it has to go. */
void filter_env_enter(struct XpJv1080Voice *voice, unsigned segment)
{
  voice->fenv_segment = segment;
  voice->fenv_start = voice->fenv_value;
  voice->fenv_total = voice->fenv_time[segment];
  voice->fenv_remaining = voice->fenv_total;
}

/* Step the envelope on by one control block. Segments 0 to 2 run from
   note-on; segment 3 is the release, entered by jv1080_voice_release.
   Several segments may finish inside one block - every time field reaches
   down to a few milliseconds - so this consumes the block rather than
   assuming one segment survives it. */
void filter_env_advance(struct XpJv1080Voice *voice, double seconds)
{
  while (voice->fenv_segment < 4u && seconds > 0.0) {
    if (voice->fenv_remaining > seconds) {
      voice->fenv_remaining -= seconds;
      double done = voice->fenv_total > 0.0
        ? 1.0 - voice->fenv_remaining / voice->fenv_total : 1.0;
      double target = voice->fenv_level[voice->fenv_segment];
      voice->fenv_value =
        voice->fenv_start + (target - voice->fenv_start) * done;
      return;
    }
    seconds -= voice->fenv_remaining;
    voice->fenv_value = voice->fenv_level[voice->fenv_segment];
    if (voice->releasing || voice->fenv_segment >= 2u) {
      voice->fenv_segment = 4u;    /* holding, or released to level 4 */
      return;
    }
    filter_env_enter(voice, voice->fenv_segment + 1u);
  }
}

/* MEASURED (`P-xxxx`, `pitch/random_pitch_i{00,10,20,30}`: sixteen notes
   each, the steady pitch in cents): random pitch depth offsets each note by
   a draw spread across the depth's displayed cents, centred on the key's
   pitch. Around the index-0 notes' own pitch (flat to 0.5 cents), index 20
   (displayed 200) spreads -93 to +89 and index 30 (displayed 1200) -567 to
   +544 - half the displayed figure each way; their spreads' standard
   deviations, 60.1 and 356.9 cents, are a uniform draw's (57.7 and 346.4).
   The draw is taken as uniform and fresh per note; sixteen hits do not pin
   its shape, and the machine's own sequence is not reproduced. Index 10
   (displayed 10) reads as a cluster at 0 with notes at +15 to +16 on the
   hardware - and our render, drawing within +-5, reads the same way
   through the same analysis, so that is the reading, not the machine. The
   index to cents list is the panel's (1 to 10 in ones, 20 to 100 in tens,
   200 to 1200 in hundreds). */
const double kRandomPitchCents[31] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100,
  200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200 };

double random_pitch_cents(unsigned index, uint32_t serial)
{
  if (!index)
    return 0.0;
  double width = kRandomPitchCents[index > 30u ? 30u : index];
  uint32_t seed = serial * 2246822519u + 0x9e3779b9u;
  seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
  seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
  return width * ((double)(seed >> 8) / 16777216.0 - 0.5);
}

/* THE PITCH ENVELOPE. MEASURED (`P-xxxx`, the `pitch/penv_*` takes on
   `Sine`, read as the instantaneous frequency in cents):

     depth and levels  a level's offset is depth x 100 cents x level/63:
                       depth +-6 and +-12 peak at +603/-600 and +1202/-1223
                       at level +63, and level -63, -42, -21, 0, +21, +42,
                       +63 at depth +12 hold -1222, -798, -400, 0, +402,
                       +799, +1201.
     times             each segment moves linearly in cents over a duration
                       its time field names, however far it goes: time 1 at
                       8, 16, 32, 48, 64, 96, 127 takes 44, 107, 322, 767,
                       1681, 7447 and 31183 ms from 0 to +1200 cents (to
                       within 7 cents rms of a straight line), and time 2
                       at 96 carries -1200 to +1200 at twice that rate, in
                       7.5 s. Between the measured values the milliseconds
                       are interpolated in log; under 8 they run straight
                       to 0, which is not measured.
     shape             it starts at centre, moves through levels 1 to 3,
                       holds level 3, and from the note-off moves to level 4
                       over time 4.

     velocity          the depth scales linearly with the velocity the
                       A-ENV's sensitivity law reads (sensed_velocity), as
                       (v - 1) / 126: at sensitivity -50 velocities 1, 32,
                       64, 96, 127 peak at 1187, 890, 588, 299 and 12 cents
                       (the law: 1200, 905, 600, 295, 0), at +75 at 12, 12,
                       29, 622, 1200 (0, 0, 0, 610, 1200), and at 0 flat at
                       1203-1205 (`pitch/penv_vel_sens_*`). Between those
                       settings the sensitivity is interpolated as the
                       A-ENV's is, which is not measured.
     time key follow   times 2 to 4 scale by 2^(-kf (key - 60)/12)
                       (`M-069`, measured on time 2). Time 1 is left alone,
                       as `M-066` measured for the A-ENV's attack; the
                       P-ENV's own attack is not measured.
     velocity time     time 1 by `M-070`'s law, measured on the P-ENV; time
                       4 by the same law, which is not measured.

   Measured on the tone record. A rhythm record carries the same fields and
   is taken to read them the same way, which is not measured; its one
   velocity-time field is given to time 1, as the F-ENV's is. */
const double kPitchEnvTimeValue[] = { 8.0, 16.0, 32.0, 48.0, 64.0, 96.0, 127.0 };
const double kPitchEnvTimeMs[] = { 44.0, 107.0, 322.0, 767.0, 1681.0, 7447.0, 31183.0 };

double pitch_env_seconds(unsigned value)
{
  double v = (double)(value > 127u ? 127u : value);
  if (v < kPitchEnvTimeValue[0])
    return kPitchEnvTimeMs[0] * v / kPitchEnvTimeValue[0] / 1000.0;
  unsigned i = 1;
  while (i < 6u && kPitchEnvTimeValue[i] < v)
    ++i;
  double t = (v - kPitchEnvTimeValue[i - 1]) /
    (kPitchEnvTimeValue[i] - kPitchEnvTimeValue[i - 1]);
  return std::exp(std::log(kPitchEnvTimeMs[i - 1]) +
                  t * (std::log(kPitchEnvTimeMs[i]) -
                       std::log(kPitchEnvTimeMs[i - 1]))) / 1000.0;
}

void pitch_env_enter(struct XpJv1080Voice *voice, unsigned segment)
{
  voice->penv_segment = segment;
  voice->penv_start = voice->penv_value;
  voice->penv_total = voice->penv_time[segment];
  voice->penv_remaining = voice->penv_total;
}

void pitch_env_advance(struct XpJv1080Voice *voice, double seconds)
{
  while (voice->penv_segment < 4u) {
    if (voice->penv_remaining > seconds) {
      voice->penv_remaining -= seconds;
      double done = 1.0 - voice->penv_remaining / voice->penv_total;
      voice->penv_value = voice->penv_start +
        (voice->penv_level[voice->penv_segment] - voice->penv_start) * done;
      return;
    }
    seconds -= voice->penv_remaining;
    voice->penv_value = voice->penv_level[voice->penv_segment];
    if (voice->releasing || voice->penv_segment >= 2u) {
      voice->penv_segment = 4u;    /* holding level 3, or released to 4 */
      return;
    }
    pitch_env_enter(voice, voice->penv_segment + 1u);
  }
}

int tone_field(const struct xp_rom *rom, const uint8_t *tone, unsigned index)
{
  (void)rom;
  return tone[index];
}

/* The key every key-scaling field pivots on (`M-066`, `M-069`, `M-077`). */
const double kKeyFollowPivot = 60.0;

/* A key follow field's value as a fraction, the device's own value string
   parsed: of one semitone per key for pitch, of one octave of corner per
   octave of key for cutoff. `absent` where the record has no such field or
   the device no list. */
/* Entry `index` of a list of `count` fixed-width signed percentage strings
   at `table`, as a fraction; `absent` where there is no such entry. */
double percent_list_entry(const struct xp_rom *rom, uint32_t table,
                          unsigned count, unsigned width, unsigned index,
                          double absent)
{
  if (!table || !width || index >= count)
    return absent;
  uint32_t at = table + (uint32_t)index * width;
  if (at + width > rom->size)
    return absent;
  int sign = 1, value = 0;
  for (unsigned c = 0; c < width; ++c) {
    char ch = (char)rom->bytes[at + c];
    if (ch == '-')
      sign = -1;
    else if (ch >= '0' && ch <= '9')
      value = value * 10 + (ch - '0');
  }
  return sign * value / 100.0;
}

double key_follow(const struct xp_rom *rom, uint16_t which,
                  const uint8_t *record, double absent)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  if (which == XP_VOICE_FIELD_NONE)
    return absent;
  return percent_list_entry(rom, profile->keyFollowTable,
                            profile->keyFollowCount, profile->keyFollowWidth,
                            record[which], absent);
}

/* An envelope's time key follow as a fraction, the device's own value
   string parsed; none where the record has no such field. */
double time_key_follow(const struct xp_rom *rom, uint16_t which,
                       const uint8_t *record)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  if (which == XP_VOICE_FIELD_NONE)
    return 0.0;
  return percent_list_entry(rom, profile->timeKeyFollowTable,
                            profile->timeKeyFollowCount,
                            profile->timeKeyFollowWidth, record[which], 0.0);
}

/* MEASURED: a fall, a decay or a release lasts ONE DURATION for its time
   value, whatever levels it runs between, and moves linearly in the
   record's level units - the level table turning that into dB.

   `aenv_l2_sweep` and `aenv_l3_sweep` hold T2 or T3 at 60 and sweep the
   level they fall to: fitting that whole shape to every one of the twelve
   falls gives 1.385 to 1.395 s at every target from 0 to 96, at 0.01 to
   0.12 dB rms over the fall, where a fall timed by its span in dB would
   have run 0.30 s to 12 dB and 1.32 s to 49. Releases from a sustain of
   64 and of 48 at T4 = 64 (`structure/tone_delay_key_off_release`, `_t3`)
   last 1.645 and 1.640 s, where a rate in level units would have given
   0.84 and 0.63.

   The same model is what the fall table above was read through: on a full
   127-to-0 traverse the level table reaches -20 dB 40.0 % of the way, so
   a segment's duration is its time for 20 dB over that fraction - 1.380 s
   at 60, against the 1.39 fitted directly. It also puts -40 dB at 1.90
   times the -20 dB time, which the T4 takes read as 1.91 and 1.92. */
double amp_env_segment_seconds(double seconds_per_20db)
{
  static const double fraction =
    (127.0 - amp_env_amplitude_units(0.1)) / 127.0;
  return seconds_per_20db / fraction;
}

/* MEASURED (`envelopes/aenv_vel_sens_*`, four sensitivities by six
   velocities, levels against each take's velocity-127 note): the A-ENV's
   velocity sensitivity decides which velocity the level law above reads.

     sensitivity   0: no velocity dependence at all - flat to 0.0 dB from
                      velocity 1 to 127.
     sensitivity +50: the velocity itself - -35.1, -23.6, -11.8, -4.8 dB at
                      16, 32, 64, 96 against the law's -36.0, -23.9, -11.9,
                      -4.9.
     sensitivity -50: the law turned over, 128 - velocity - velocity 1 at
                      full level, then -2.2, -4.9, -12.1, -24.3 dB at 16,
                      32, 64, 96 against -2.2, -4.9, -11.9, -23.9, and 127
                      at the floor.
     sensitivity +75: twice the velocity's distance from 127 - 96 reads
                      -11.5 dB against -11.6, and 64 and below reach the
                      floor, where one and a half times the dB of +50 would
                      have read -7.3 at 96.

   So the sensitivity scales how far the velocity is taken from the end the
   law starts at: 127 - k (127 - v) for positive settings and 127 - k (v - 1)
   for negative ones, with k 0, 1 and 2 at 0, 50 and 75 and 1 at -50.
   Between those points k is interpolated linearly, which is not
   recovered: the takes hold these four settings only. */
unsigned sensed_velocity(unsigned velocity, int sensitivity)
{
  double k;
  if (sensitivity >= 0)
    k = sensitivity <= 50 ? sensitivity / 50.0
                          : 1.0 + (sensitivity - 50) / 25.0;
  else
    k = -sensitivity / 50.0;
  double v = sensitivity >= 0 ? 127.0 - k * (127.0 - (double)velocity)
                              : 127.0 - k * ((double)velocity - 1.0);
  if (v <= 0.0)
    return 0u;
  return v >= 127.0 ? 127u : (unsigned)std::lround(v);
}

/* MEASURED (`M-029`; `envelopes/aenv_vel_curve_c0..c6`, re-read for all
   ten points): the A-ENV's seven velocity curves, as dB below each
   curve's own velocity-127 level at velocity sensitivity +50, rms 50 to
   300 ms after each onset at key 60. Curve 0 is the level law itself
   (`square_law_gain`, within 1.4 dB of the take at every point above its
   floor) and is not tabled. Points listed as -64.6 are the take's noise
   floor, 64.6 dB under velocity 127: the level there is at or below it and
   is not recovered. Between the ten velocities the level is interpolated
   in dB, which is not recovered either.

   A record whose sensitivity is not +50 reads its curve at the velocity
   that sensitivity maps it to (sensed_velocity). The curves were measured
   at +50 only, so how a curve and another sensitivity compose on the
   machine is NOT measured; this is the composition that leaves curve 0 as
   it was measured. */
const double kVelocityCurveVelocities[10] = {
  1, 8, 16, 32, 48, 64, 80, 96, 112, 127 };
const double kVelocityCurveDb[6][10] = {
  { -64.6, -64.0, -56.7, -42.0, -32.3, -24.6, -17.6, -11.4,  -5.4, 0.0 },
  { -64.6, -64.6, -64.6, -64.4, -59.9, -48.4, -36.2, -23.8, -11.5, 0.0 },
  { -58.4, -27.7, -19.0, -11.3,  -7.5,  -5.1,  -3.3,  -1.9,  -0.8, 0.0 },
  { -27.4, -11.4,  -7.9,  -4.8,  -3.3,  -2.2,  -1.5,  -0.9,  -0.4, 0.0 },
  { -64.6, -64.6, -62.8, -49.5, -32.7, -11.4,  -2.6,  -0.9,  -0.3, 0.0 },
  { -47.3, -22.3, -17.6, -14.4, -13.0, -11.9, -11.0,  -9.8,  -7.5, 0.0 },
};

double velocity_curve_gain(unsigned curve, unsigned velocity)
{
  if (!curve || curve > 6u)
    return square_law_gain(velocity);
  if (!velocity)
    return 0.0;
  double db = interpolate_points(kVelocityCurveVelocities,
                                 kVelocityCurveDb[curve - 1u], 10u,
                                 (double)velocity);
  return std::pow(10.0, db / 20.0);
}

/* ---- The LFOs ------------------------------------------------------

   MEASURED (`M-013`, `M-022`, `M-041`): both LFOs read one rate law over
   the whole field, 0.0494 Hz * 2^(v / 14.13), 0.0488 to 25.0 Hz. */
double lfo_rate_hz(unsigned value)
{
  return 0.0494 * std::pow(2.0, (double)value / 14.13);
}

/* MEASURED (`M-099`, `M-103`): with EXT SYNC on (CLOCK or TAP, identical
   while Tap control source is OFF) the rate field is a PERIOD in MIDI-clock
   pulses, 24 to the quarter note at the tempo in force, value 0 fastest.
   The points below are the measured counts; between them the count is
   interpolated linearly and rounded to a whole pulse, which is not
   measured - every measured count sits within 0.4 of an integer, but only
   these seventeen settings were read. Value 0 is a fixed 40 ms at any
   tempo, the free-run law's own 25 Hz ceiling. */
const struct { unsigned value; double pulses; } kLfoSyncPulses[] = {
  { 10, 5 }, { 20, 10 }, { 30, 15 }, { 32, 16 }, { 40, 20 }, { 50, 26 },
  { 60, 36 }, { 64, 40 }, { 70, 46 }, { 80, 64 }, { 90, 84 }, { 96, 95 },
  { 100, 111 }, { 110, 151 }, { 120, 192 }, { 127, 218 } };

double lfo_sync_hz(unsigned value, double bpm)
{
  if (value == 0u)
    return 25.0;
  if (bpm <= 0.0)
    bpm = 120.0;                 /* a caller that names no tempo */
  const unsigned n = (unsigned)(sizeof kLfoSyncPulses / sizeof kLfoSyncPulses[0]);
  double pulses;
  if (value <= kLfoSyncPulses[0].value) {
    /* Below the first measured point the count is taken as proportional to
       the value, which the first five points (0.5 pulse per step) follow. */
    pulses = kLfoSyncPulses[0].pulses * value / kLfoSyncPulses[0].value;
  } else {
    pulses = kLfoSyncPulses[n - 1].pulses;
    for (unsigned i = 1; i < n; ++i)
      if (value <= kLfoSyncPulses[i].value) {
        const double v0 = kLfoSyncPulses[i - 1].value;
        const double p0 = kLfoSyncPulses[i - 1].pulses;
        pulses = p0 + (kLfoSyncPulses[i].pulses - p0) *
          ((double)value - v0) / ((double)kLfoSyncPulses[i].value - v0);
        break;
      }
  }
  pulses = std::round(pulses);
  if (pulses < 1.0)
    pulses = 1.0;
  double period = pulses * 60.0 / (24.0 * bpm);
  return period < 0.040 ? 25.0 : 1.0 / period;
}

/* Interpolation through a table of (value, y) points, linear in y. */
struct LawPoint { double x, y; };
double law(const LawPoint *p, unsigned n, double x)
{
  if (x <= p[0].x)
    return p[0].y;
  for (unsigned i = 1; i < n; ++i)
    if (x <= p[i].x)
      return p[i - 1].y + (p[i].y - p[i - 1].y) * (x - p[i - 1].x) /
                            (p[i].x - p[i - 1].x);
  return p[n - 1].y;
}

/* MEASURED (`M-071`, `M-085`): the delay before an LFO acts and the time
   it fades over, one 62 s note per top value. Between the measured values
   the time is interpolated in its logarithm above 32 and linearly below,
   which is not recovered; `M-071` read no fade at all at 32. */
double lfo_time_seconds(const LawPoint *p, unsigned n, unsigned value)
{
  double v = (double)value;
  for (unsigned i = 1; i < n; ++i)
    if (v <= p[i].x) {
      if (p[i - 1].y > 0.0)
        return p[i - 1].y * std::pow(p[i].y / p[i - 1].y,
                                     (v - p[i - 1].x) / (p[i].x - p[i - 1].x));
      return p[i - 1].y + (p[i].y - p[i - 1].y) * (v - p[i - 1].x) /
                            (p[i].x - p[i - 1].x);
    }
  return p[n - 1].y;
}
const LawPoint kLfoDelay[] = {
  { 0, 0.0 }, { 32, 0.255 }, { 64, 1.610 }, { 80, 3.500 }, { 96, 7.380 },
  { 112, 16.320 }, { 127, 32.700 } };
const LawPoint kLfoFade[] = {
  { 0, 0.0 }, { 32, 0.0 }, { 64, 1.135 }, { 80, 2.800 }, { 96, 6.380 },
  { 112, 14.300 }, { 127, 28.880 } };

/* MEASURED (`M-114`): how far each depth moves each destination, as the
   peak of a triangle at rate 64 read by lock-in on the owner's unit,
   2026-09-23, with the corpus's own depth +63 takes replayed as a
   known-answer control (within 0.6 %).

   PITCH, cents: quadratic in depth - 0.455 d^2 to 1-2 % from depth 8 up.
   Below 8 the take is under its own noise, so there the quadratic is
   extrapolated and not measured.
   AMPLITUDE, dB: an ATTENUATION only, and only while the waveform is
   negative - the sweep's top sits on the unmodulated level at every depth,
   and with key trigger on a triangle holds the full level for its whole
   first, positive half cycle before falling to twice the listed peak at
   its bottom (the phase takes of 2026-09-23). The listed peak is the
   fundamental the sweep's lock-in reads, which this law and a plain
   triangle share. With a level offset other than 0 the law is not
   measured; it is applied to the offset waveform here.
   FILTER, octaves of the corner: read on two base cutoffs (60 and 86)
   whose clean halves agree to 3 % from depth 12 to 48; above 48 each take
   clips one half against the analysis floor or the 7891 Hz resonant
   ceiling, so the quadratic that runs through the measured points is
   carried on from 48, and below 8 likewise. Neither is recovered.
   PAN is taken as one pan-table unit of distance per step of depth, which
   the take matches to 1 % up to depth 32 read through the pan table
   (2.19, 4.50, 9.26 dB at 8, 16, 32 against the table's 2.20, 4.50,
   9.20). Depth 1 does not move the pan at all on the machine. */
const LawPoint kLfoPitchCents[] = {
  { 8, 29.3 }, { 12, 65.6 }, { 16, 116.5 }, { 24, 259.9 }, { 32, 451.3 },
  { 40, 722.4 }, { 48, 1053.9 }, { 56, 1416.6 }, { 63, 1805.3 } };
const LawPoint kLfoAmpDb[] = {
  { 0, 0.0 }, { 1, 0.034 }, { 2, 0.070 }, { 4, 0.138 }, { 8, 0.275 },
  { 12, 0.431 }, { 16, 0.573 }, { 24, 0.840 }, { 32, 1.253 }, { 40, 1.649 },
  { 48, 1.972 }, { 56, 2.257 }, { 63, 2.745 } };
const LawPoint kLfoFilterOct[] = {
  { 8, 0.088 }, { 12, 0.194 }, { 16, 0.308 }, { 24, 0.732 }, { 32, 1.268 },
  { 40, 2.047 }, { 48, 2.977 } };

double signed_law(double depth, double (*f)(double))
{
  return depth < 0.0 ? -f(-depth) : f(depth);
}
double pitch_depth_cents(double d)
{
  return d < 8.0 ? 0.455 * d * d : law(kLfoPitchCents, 9u, d);
}
double amp_depth_db(double d)
{
  return law(kLfoAmpDb, 13u, d);
}
double filter_depth_octaves(double d)
{
  if (d < 8.0)
    return 0.088 * (d / 8.0) * (d / 8.0);
  if (d > 48.0)
    return 2.977 * (d / 48.0) * (d / 48.0);
  return law(kLfoFilterOct, 7u, d);
}
double pan_depth_units(double d)
{
  return d <= 1.0 ? 0.0 : d;
}

uint32_t lfo_random(uint32_t *seed)
{
  uint32_t x = *seed ? *seed : 0x9e3779b9u;
  x ^= x << 13; x ^= x >> 17; x ^= x << 5;
  *seed = x;
  return x;
}

/* MEASURED (`M-034`, `M-054`, `M-074`, and the form takes read cycle by
   cycle): TRI, SIN and SAW are the ideal waveforms, SAW rising; SQR is a
   symmetric +-1 square at half duty; TRP is a trapezoid - a quarter of the
   cycle at each extreme and a quarter on each ramp, which is a triangle
   doubled and clipped. MEASURED with key trigger on (`M-071`, and the
   phase takes of 2026-09-23): TRI starts at zero and rising, SQR on its
   top half, and SAW at its bottom, rising.

   S&H, RND and CHS DRAW rather than follow a curve. They are not
   recovered: S&H and RND hold a new random value for each LFO period here,
   and CHS, which `M-054` found running fifteen times the rate field's
   frequency, does the same at fifteen times the rate. */
double lfo_triangle(double ph)
{
  return ph < 0.25 ? 4.0 * ph : ph < 0.75 ? 2.0 - 4.0 * ph : 4.0 * ph - 4.0;
}
double lfo_wave(const struct XpJv1080Lfo *lfo)
{
  double ph = lfo->phase;
  switch (lfo->form) {
  case 0: return lfo_triangle(ph);
  case 1: return std::sin(2.0 * 3.14159265358979323846 * ph);
  case 2: return 2.0 * ph - 1.0;
  case 3: return ph < 0.5 ? 1.0 : -1.0;
  case 4: {
    double t = 2.0 * lfo_triangle(ph);
    return t > 1.0 ? 1.0 : (t < -1.0 ? -1.0 : t);
  }
  default: return lfo->held;
  }
}

/* How much of its depth an LFO has, from its delay, its fade time and its
   fade mode: ON-IN waits the delay and fades in, ON-OUT acts at once and
   fades out after the delay, OFF-IN and OFF-OUT do the same from the
   note-off - what the four `lfo1_fade_*` takes show. The fade is taken as
   a straight ramp in depth, which is not measured. */
double lfo_amount(const struct XpJv1080Lfo *lfo)
{
  bool fadeIn = lfo->fade_mode == 0u || lfo->fade_mode == 2u;
  double t = lfo->since_on;
  if (lfo->fade_mode >= 2u) {
    if (lfo->since_off < 0.0)
      return fadeIn ? 0.0 : 1.0;
    t = lfo->since_off;
  }
  double ramp;
  if (t < lfo->delay)
    ramp = 0.0;
  else if (lfo->fade <= 0.0 || t >= lfo->delay + lfo->fade)
    ramp = 1.0;
  else
    ramp = (t - lfo->delay) / lfo->fade;
  return fadeIn ? ramp : 1.0 - ramp;
}

void lfo_advance(struct XpJv1080Lfo *lfo, double seconds)
{
  double rate = lfo->form == 7u ? 15.0 * lfo->frequency : lfo->frequency;
  lfo->phase += rate * seconds;
  if (lfo->phase >= 1.0) {
    lfo->phase -= std::floor(lfo->phase);
    if (lfo->form >= 5u)
      lfo->held = (double)(lfo_random(&lfo->seed) >> 8) / 8388608.0 - 1.0;
  }
  lfo->since_on += seconds;
  if (lfo->since_off >= 0.0)
    lfo->since_off += seconds;
}

/* The pan-table difference for a pan distance that need not be whole. */
double pan_difference_db_at(double offset)
{
  return pan_difference_asymmetric(offset);
}

/* THE PAN SLEWS (`P-xxxx`, TASK-434). A pan that changes under a sounding
   note does not jump: each channel's gain walks toward its new value in a
   staircase, a fixed amount per 10 ms tick, and the two channels walk
   independently - hard right to centre, the right channel (1 -> 0.71)
   arrives in 4 ticks while the left (0 -> 0.71) takes 8. MEASURED, read in
   linear amplitude per channel off one period of the tone:

     closeout/matrix3 pan_clamp   square pan LFO + matrix PAN, centre <->
                                  hard right, six edges each way
     routing/matrix_dest_all_*    matrix PAN under CC1 stepped every 200 ms,
                                  moves of about 18 units, both directions
     sysex/dt1_sounding_tone_pan  a DT1 to tone pan, centre <-> hard left

   The step is 0.0948 of full-scale pan gain (0.0946-0.0950 across the three
   takes, each channel read against its own hard-panned level), and the tick
   10.0 ms (per-edge means 9.97-10.11 ms). The step is a fraction of the pan
   gain, not of the output: voices 12 dB apart give the same fraction. The
   first tick of a move from rest is 2/3 of a step (0.664-0.669 on all
   thirteen matrix and LFO moves read), then whole steps, and the last stops
   on the target. The DT1 path's first tick is 4/3 of a step instead; that path
   does not reach a sounding voice in this engine. The tick's phase against
   the note-on is not measured; here it runs from the note-on. A note starts
   on its pan at once (pan_clamp's onset lands hard right within 2 ms). */
const double kPanSlewTickSeconds = 0.010;
const double kPanSlewStep = 0.0948;
const double kPanSlewFirstStep = kPanSlewStep * 2.0 / 3.0;

void set_pan(struct XpJv1080Voice *voice, double offset)
{
  if (offset < -64.0)
    offset = -64.0;
  if (offset > 63.0)
    offset = 63.0;
  double ratio = std::pow(10.0, pan_difference_db_at(offset) / 20.0);
  double left = std::sqrt(1.0 / (1.0 + ratio * ratio));
  voice->pan_target_left = left;
  voice->pan_target_right = ratio * left;
}

double pan_slew(double gain, double target, bool *moving)
{
  double d = target - gain;
  if (d == 0.0) {
    *moving = false;
    return gain;
  }
  double step = *moving ? kPanSlewStep : kPanSlewFirstStep;
  *moving = true;
  if (std::fabs(d) <= step)
    return target;
  return d > 0.0 ? gain + step : gain - step;
}

/* One pan tick: the first snaps to the note's pan, every later one takes
   each channel one slew step toward its target. */
void pan_tick(struct XpJv1080Voice *voice)
{
  if (!voice->pan_started) {
    voice->gain_left = voice->pan_target_left;
    voice->gain_right = voice->pan_target_right;
    voice->pan_started = true;
    return;
  }
  voice->gain_left = pan_slew(voice->gain_left, voice->pan_target_left,
                              &voice->pan_moving_left);
  voice->gain_right = pan_slew(voice->gain_right, voice->pan_target_right,
                               &voice->pan_moving_right);
}

/* Once per control block: advance both LFOs and form what they do to the
   pitch, the level, the cutoff and the pan. */
void lfo_update(struct XpJv1080Voice *voice, double seconds)
{
  double cents = 0.0, db = 0.0, cutoff = 0.0, pan = 0.0;
  for (unsigned i = 0; i < 2u; ++i) {
    struct XpJv1080Lfo *lfo = voice->lfo + i;
    lfo_advance(lfo, seconds);
    double v = lfo_wave(lfo) + lfo->offset;
    double k = lfo_amount(lfo);
    cents += voice->lfo_pitch_cents[i] * v * k;
    cutoff += voice->lfo_cutoff_units[i] * v * k;
    pan += voice->lfo_pan_units[i] * v * k;
    /* Attenuation only, while the waveform is below zero. */
    double a = voice->lfo_amp_db[i];
    double signedV = a >= 0.0 ? v : -v;
    if (signedV < 0.0)
      db += 2.0 * std::fabs(a) * signedV * k;
  }
  voice->lfo_pitch_ratio = std::pow(2.0, cents / 1200.0);
  voice->lfo_gain = std::pow(10.0, db / 20.0);
  voice->lfo_cutoff = cutoff;
  if (voice->lfo_pan_units[0] != 0.0 || voice->lfo_pan_units[1] != 0.0)
    set_pan(voice, (double)voice->pan_offset + voice->matrix_pan + pan);
}

/* A field this record type has, or `absent` where it does not have one. */
unsigned field_or(const struct XpVoiceFieldMap *fields, uint16_t which,
                   const uint8_t *record, unsigned absent)
{
  return which == XP_VOICE_FIELD_NONE ? absent : record[which];
}

/* THE PER-NOTE PAN SOURCES, as offsets from centre in pan units added to
   the tone, patch and part pans (`M-002`). MEASURED (`M-047`, `P-xxxx`, the
   `tva/` takes read as left minus right per note, each placed on the
   hardware's own `tone_pan_sweep`):

     alternate pan   (value - 64) units, the sign flipping on each note of
                     the part and positive on its first: value 1 lands its
                     notes at pan 1 and 127 in turn (+44.5 / -65.4 dB), 127
                     the reverse, 64 all at centre. Only the ends and the
                     middle are measured; between them it is taken as
                     linear. THE COUNTER (`P-xxxx`, 164 notes over eleven
                     files, none contradicting it): one per part, flipped
                     once by every note-on - whatever number of tones it
                     starts, none included, so all of a note's tones throw
                     the same way and a chord's keys alternate - and put
                     back on "+" by a program change on that part. Other
                     parts' notes and program changes, a bank select alone,
                     DT1 writes to tone, patch or part, and idle time up to
                     8 s leave it where it is.
     random pan      a fresh draw uniform across twice the depth each way:
                     depth 32 spreads its sixteen notes from 60 units left to
                     60 right; depth 63 puts eleven of sixteen on a rail (9
                     left, 2 right). Uniform is taken, not recovered.
     pan key follow  kf x (key - 60) x 3.58 units, kf -1..+1 across the
                     panel's list: at index 0 (-100 %) keys 48 and 72 sit 43
                     and 42 units right and left of centre, keys 24, 36, 84
                     and beyond on the rails, index 7 centred at every key;
                     index 14 the mirror. The 3.58 is read at +-12 keys
                     alone; the list between the ends is the panel's. */
const double kPanKeyFollow[15] = { -1.0, -0.7, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0,
                                   0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1.0 };

int note_pan_offset(const struct XpVoiceFieldMap *fields, const uint8_t *tone,
                    unsigned key, const struct XpJv1080PartControls *controls)
{
  double offset = 0.0;
  unsigned kf = field_or(fields, fields->panKeyFollow, tone, 7u);
  offset += kPanKeyFollow[kf > 14u ? 14u : kf] * ((double)key - 60.0) * 3.58;
  unsigned alt = field_or(fields, fields->alternatePanDepth, tone, 64u);
  offset += (double)((int)alt - 64) * (double)(controls->alternate_phase < 0 ? -1 : 1);
  unsigned depth = field_or(fields, fields->randomPanDepth, tone, 0u);
  if (depth) {
    uint32_t seed = controls->lfo_seed * 3266489917u + 0x85ebca6bu;
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
    offset += 2.0 * (double)depth * (2.0 * ((double)(seed >> 8) / 16777216.0) - 1.0);
  }
  return (int)std::lround(offset);
}

/* Which key selects the zone, and which key the wave is played at. They are
   the same on a record that transposes and differ on one that names its own
   source key. */
unsigned playback_key(const struct XpVoiceFieldMap *fields,
                       const uint8_t *record, unsigned key)
{
  return field_or(fields, fields->sourceKey, record, key);
}

/* The five links from a record's three wave fields to a wave-element
   record, shared by the span query and the note-on below so the two cannot
   drift. */
/* The key a record's multisample zone is chosen by. MEASURED (`P-xxxx`,
   `pitch/coarse_semitones`, `coarse_octaves`, `part_coarse`): a tone's
   coarse tune and the part's key shift move it with the transposition. On
   the INT-B `Sine`, whose elements carry their own second-harmonic level,
   the take's h2 steps at coarse tune -9, -3, +5 and +10 - exactly where
   the transposed key crosses the multisample's splits at 50, 56, 64 and 69
   - and reads within 0.9 dB of the element that key selects at every step;
   the part key shift's takes step the same way. With the key alone the
   element never changes and h2 sits at -32.1 dB throughout. A record that
   names its own source key - a drum - keeps it: how its coarse tune bears
   on its zone is not measured. */
unsigned zone_key(const struct XpVoiceFieldMap *fields, const uint8_t *record,
                  unsigned key, int keyShift)
{
  if (fields->sourceKey != XP_VOICE_FIELD_NONE)
    return playback_key(fields, record, key);
  int shifted = (int)key + keyShift +
    (int)(int8_t)(uint8_t)field_or(fields, fields->coarseTune, record, 0);
  return shifted < 0 ? 0u : (shifted > 127 ? 127u : (unsigned)shifted);
}

bool resolve_element(const struct xp_rom *rom,
                      const struct XpVoiceFieldMap *fields,
                      const uint8_t *record, unsigned key, int keyShift,
                      struct xp_wave_element *element)
{
  unsigned source = 0;
  uint8_t msBank = 0;
  uint16_t msRow = 0;
  struct xp_wave_zone zone;
  return wave_source_select(rom, (unsigned)record[fields->waveGroup],
                             (unsigned)record[fields->waveGroupId],
                             &source) &&
    wave_number_resolve(rom, source, (unsigned)record[fields->waveNumber],
                         &msBank, &msRow) &&
    multisample_select(rom, msBank, msRow,
                        zone_key(fields, record, key, keyShift), &zone) &&
    wave_element_open(rom, zone.directory, zone.element, element);
}

/* How much of a tone its velocity range lets through, 0..1. Outside the
   range a tone with a velocity cross fade depth fades out over that many
   velocity steps rather than stopping at the edge; depth 0 is the hard
   gate. The amplitude d steps outside the range is (1 - d/depth)^2.
   MEASURED (`P-xxxx`, TASK-425) on a dedicated hardware capture: two
   tones ranged 1-64 and 65-127, stepped velocities at depths 32, 64 and
   127. A free exponent fits 1.999 over all 75 points (1.98-2.005 per
   depth and edge), 0.04 dB RMS from the square against 4.41 dB from the
   straight line. The gain depends on d/depth alone - the same fraction
   reads the same dB at every depth - and the two edges agree to 0.02 dB.
   Not explained: at depth 127 the two tones differ by 0.1-0.3 dB, which
   a /128 divisor does not remove. */
double velocity_fade_gain(const struct XpVoiceFieldMap *fields,
                          const uint8_t *record, unsigned velocity)
{
  unsigned lo = field_or(fields, fields->velocityRangeLow, record, 1);
  unsigned hi = field_or(fields, fields->velocityRangeHigh, record, 127);
  unsigned d = velocity < lo ? lo - velocity
    : (velocity > hi ? velocity - hi : 0u);
  if (!d)
    return 1.0;
  unsigned depth = field_or(fields, fields->velocityCrossFade, record, 0u);
  if (d >= depth)
    return 0.0;
  double g = 1.0 - (double)d / (double)depth;
  return g * g;
}

/* The TVA key bias, as a factor on the tone's level field before the
   square law: the field reads (level/127)^2 (`M-029`), and the bias scales
   its `level/127` by f = 1 + p * d / 12, clamped to 0..1, so the gain is
   f^2. p is the level enum's percentage (-100, -70, -50, -40, -30, -20,
   -10, 0, +10, +20, +30, +40, +50, +70, +100; `partial_schema.md`) and d
   the key's distance past the point on the side the direction names:
   LOWER below it, UPPER above it, LOWER&UPPER both, and ALL the signed
   distance key - point, so the one slope rises through the point.

   MEASURED (`M-046`, the eleven `tva/bias_*` takes, keys 24-108 in
   octaves): -100 % mutes from one octave past the point on the named side
   (UPPER above, LOWER below, L&U both, ALL above), +100 % on ALL mutes one
   octave below, and +100 % on UPPER, LOWER and L&U changes nothing - no
   key is ever louder than the unbiased tone, hence the clamp at 1.
   MEASURED on an intermediate value (`P-xxxx`, TASK-435 `santur_keys`: GM
   Santur's Gtr Harm A tone alone, UPPER, point 48, -50 %): -12.5 dB at key
   60 and -29.6 at key 68 against the unbiased engine, where this law gives
   -12.0 and -31.1; a slope linear in dB would give -20.8 at key 68. That p
   counts per OCTAVE is the reading these points support, not a value
   recovered from the ROM. The key is the note the record plays, octave
   shift included and coarse tune not - which of the two the machine uses
   is not measured. */
double bias_gain(const struct XpVoiceFieldMap *fields, const uint8_t *record,
                 unsigned key)
{
  if (fields->biasDirection == XP_VOICE_FIELD_NONE ||
      fields->biasPoint == XP_VOICE_FIELD_NONE ||
      fields->biasLevel == XP_VOICE_FIELD_NONE)
    return 1.0;
  return jv1080_bias_gain(record[fields->biasDirection],
                          record[fields->biasPoint],
                          record[fields->biasLevel], key);
}

/* The record's own gates. One that is off, or whose key range or velocity
   fade excludes this note, does not sound - which is not an error: a
   patch's four tones routinely split the keyboard between them. A record
   type with no range fields gates on its switch alone. */
bool record_sounds(const struct XpVoiceFieldMap *fields,
                    const uint8_t *record, unsigned key, unsigned velocity)
{
  if (!record[fields->enable])
    return false;
  if (key < field_or(fields, fields->keyRangeLow, record, 0) ||
      key > field_or(fields, fields->keyRangeHigh, record, 127))
    return false;
  return velocity_fade_gain(fields, record, velocity) > 0.0;
}

}  // namespace

double jv1080_filter_env_curve(unsigned curve, unsigned velocity)
{
  return filter_env_curve_fraction(curve, (double)velocity);
}

double jv1080_bias_gain(unsigned direction, unsigned point, unsigned level,
                        unsigned key)
{
  static const double kBiasPercent[15] = {
    -100.0, -70.0, -50.0, -40.0, -30.0, -20.0, -10.0, 0.0,
    10.0, 20.0, 30.0, 40.0, 50.0, 70.0, 100.0,
  };
  const double p = kBiasPercent[level > 14u ? 14u : level] / 100.0;
  const double signedKeys = (double)key - (double)point;
  double d = 0.0;
  switch (direction & 3u) {
  case 0u: d = signedKeys < 0.0 ? -signedKeys : 0.0; break;      /* LOWER */
  case 1u: d = signedKeys > 0.0 ? signedKeys : 0.0; break;       /* UPPER */
  case 2u: d = signedKeys < 0.0 ? -signedKeys : signedKeys; break;  /* L&U */
  default: d = signedKeys; break;                                  /* ALL */
  }
  double f = 1.0 + p * d / 12.0;
  if (f < 0.0)
    f = 0.0;
  if (f > 1.0)
    f = 1.0;
  return f * f;
}

double jv1080_amp_env_velocity_time_scale(unsigned enumValue,
                                          unsigned velocity)
{
  return amp_env_velocity_time_scale(enumValue, velocity);
}

double jv1080_filter_env_segment_seconds(unsigned value)
{
  return filter_env_segment_seconds(value);
}

double jv1080_filter_env_offset(const struct XpVoiceFieldMap *fields,
                                 const uint8_t *record, unsigned velocity)
{
  if (!fields || !record || fields->filterEnvDepth == XP_VOICE_FIELD_NONE)
    return 0.0;
  int depth = (int8_t)record[fields->filterEnvDepth];
  if (!depth)
    return 0.0;
  /* A record type with no velocity-curve field of its own is rendered on
     curve 0; which curve such a record uses is not established. */
  unsigned curve = field_or(fields, fields->filterEnvVelCurve, record, 0u);
  int sensitivity =
    (int8_t)(uint8_t)field_or(fields, fields->filterEnvVelSens, record, 0u);
  return kFilterEnvDepthScale * (double)depth *
    filter_env_sensed_fraction(curve, sensitivity, velocity);
}

double jv1080_tvf_response_db(int type, double cutoff, unsigned resonance,
                              double rate, double hz)
{
  struct XpJv1080Voice *voice = new XpJv1080Voice();
  voice->filter_type = type;
  voice->cutoff_base = cutoff;
  voice->resonance_value = resonance;
  voice->resonance_q = tvf_q(resonance);
  set_filter(voice, rate);
  double w = 2.0 * 3.14159265358979323846 * hz / rate;
  double nr = voice->b0 + voice->b1 * std::cos(w) + voice->b2 * std::cos(2.0 * w);
  double ni = -voice->b1 * std::sin(w) - voice->b2 * std::sin(2.0 * w);
  double dr = 1.0 + voice->a1 * std::cos(w) + voice->a2 * std::cos(2.0 * w);
  double di = -voice->a1 * std::sin(w) - voice->a2 * std::sin(2.0 * w);
  delete voice;
  return 10.0 * std::log10((nr * nr + ni * ni) / (dr * dr + di * di));
}

bool jv1080_voice_span(const struct xp_rom *rom,
                        const struct XpVoiceFieldMap *fields,
                        const uint8_t *tone, unsigned key, unsigned velocity,
                        int keyShift, size_t *samples)
{
  struct xp_wave_element element;
  if (!rom || !fields || !tone || !samples || key > 127u || velocity == 0u ||
      velocity > 127u || !record_sounds(fields, tone, key, velocity) ||
      !resolve_element(rom, fields, tone, key, keyShift, &element))
    return false;
  *samples = (size_t)(element.bank_end - (element.bank_start & ~0x0fu)) + 1u;
  return true;
}

bool jv1080_voice_decode_span(const struct xp_rom *rom,
                               const struct XpVoiceFieldMap *fields,
                               const uint8_t *tone, unsigned key,
                               int keyShift, unsigned *bank,
                               uint32_t *first, uint32_t *last)
{
  struct xp_wave_element element;
  if (!rom || !fields || !tone || !bank || !first || !last || key > 127u ||
      !resolve_element(rom, fields, tone, key, keyShift, &element) ||
      element.bank >= XP_WAVE_BANK_COUNT)
    return false;
  *bank = element.bank;
  *first = element.bank_start;
  *last = element.bank_end;
  return true;
}

bool jv1080_patch_tone(const struct xp_rom *rom,
                        const struct xp_packed_record *patch, unsigned index,
                        uint8_t *tone, unsigned *patchLevel,
                        unsigned *patchPan)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  if (!patch || !tone || !patchLevel || !patchPan ||
      index >= patch->part_count)
    return false;
  for (unsigned f = 0; f < XP_JV1080_TONE_FIELDS; ++f) {
    int value = 0;
    if (!packed_part_field(rom, patch, index, f, &value))
      return false;
    /* The decoder stores each field in a signed byte, so the group's six
       eight-bit fields come back negative and every consumer re-widens
       them. Keeping the unwrapped byte is the same value. */
    tone[f] = (uint8_t)(value & 0xff);
  }
  int level = 0;
  int pan = 0;
  if (!packed_common_field(rom, patch, profile->patchFieldLevel, &level) ||
      !packed_common_field(rom, patch, profile->patchFieldPan, &pan))
    return false;
  *patchLevel = (unsigned)(level < 0 ? 0 : level);
  *patchPan = (unsigned)(pan < 0 ? 0 : pan);
  return true;
}

bool jv1080_voice_start(const struct xp_rom *rom,
                         const struct XpVoiceFieldMap *fields,
                         const uint8_t *tone,
                         const struct XpJv1080PartControls *controls,
                         unsigned key, unsigned velocity,
                         const uint8_t *const banks[XP_WAVE_BANK_COUNT],
                         const size_t bankSizes[XP_WAVE_BANK_COUNT],
                         struct xp_wave_cache *cache,
                         double outputRate, struct XpJv1080Voice *voice)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  if (!rom || !fields || !tone || !controls || !banks || !bankSizes ||
      !voice ||
      key > 127u || velocity > 127u || velocity == 0u || outputRate <= 0.0)
    return false;
  std::memset(voice, 0, sizeof *voice);

  struct xp_wave_element element;
  if (!record_sounds(fields, tone, key, velocity) ||
      !resolve_element(rom, fields, tone, key, controls->key_shift, &element))
    return false;
  if (element.bank >= XP_WAVE_BANK_COUNT || !banks[element.bank])
    return false;

  /* Decode the element. The whole span is decoded at note-on rather than
     streamed: the accumulator is differential, so a sample's value depends
     on every delta before it in the block, and a probe that holds the span
     is simpler than one that reseeds. The span is the one fce_decode_storage
     decodes for a descriptor from bank_start to bank_end - the 16-sample
     block holding bank_start through bank_end - which is what the cache
     keys on. */
  struct xp_wave_descriptor span;
  std::memset(&span, 0, sizeof span);
  span.address_a = element.bank_start;
  span.address_c = element.bank_end;
  const int32_t *pcm = nullptr;
  uint32_t base = 0;
  size_t needed = 0;
  if (!wave_cache_acquire(cache, profile, banks[element.bank],
                          bankSizes[element.bank], &span, &pcm, &base,
                          &needed))
    return false;

  voice->pcm = pcm;
  voice->pcm_count = needed;
  voice->reverse = element.reverse;
  voice->looping = element.mode == XP_WAVE_FORWARD_LOOP ||
    element.mode == XP_WAVE_PING_PONG_LOOP;
  voice->ping_pong = element.mode == XP_WAVE_PING_PONG_LOOP;
  voice->in_cycle = false;
  voice->loop_first = (size_t)(element.bank_loop - base);
  voice->loop_last = (size_t)(element.bank_end - base);
  /* MEASURED (`M-090`): a reversed element plays its last N samples
     backwards - the reversed array's first N, not the first N reversed - so
     the read head starts at the far end and walks down. A reversed element
     does not loop. */
  voice->position = voice->reverse
    ? (double)(voice->pcm_count - 1u)
    : (double)(element.bank_start - base);
  if (voice->reverse) {
    voice->looping = false;
    voice->ping_pong = false;
  }

  /* MEASURED (`M-014`), all exact: coarse tune is `value - 48` semitones to
     within 0.3 cents over the whole range and fine tune is `value - 50`
     cents to within 0.1. The bias is already applied by the descriptor, so
     the decoded bytes are signed.

     The element record's own fine-tune field (+0x0E) is NOT applied: its
     units are open (`U-R3-03`), and a guess at them would be a tuning error
     on every note rather than on none. */
  int coarse = (int8_t)(uint8_t)field_or(fields, fields->coarseTune, tone, 0);
  int fine = (int8_t)(uint8_t)field_or(fields, fields->fineTune, tone, 0);
  /* The part's key shift and the coarse tune are pitch terms below; the
     zone they move is zone_key's. */
  const unsigned soundedKey = playback_key(fields, tone, key);
  /* The patch's octave shift arrives in `key` itself (the engine shifts the
     note it starts), so it is not a pitch term here. MEASURED on the
     device: twelve semitones per unit, adding to both coarse tunes. */
  /* Key follow scales the key's distance from the pivot. The part's key
     shift below stays outside it: whether the machine scales a shift by
     key follow is not measured. */
  const double trackedKey =
    kKeyFollowPivot + key_follow(rom, fields->pitchKeyFollow, tone, 1.0) *
                        ((double)soundedKey - kKeyFollowPivot);
  double keyHz = 440.0 * std::pow(2.0, (trackedKey - 69.0) / 12.0) *
    std::pow(2.0, (double)coarse / 12.0) *
    std::pow(2.0, (double)controls->key_shift / 12.0) *
    std::pow(2.0, (double)controls->fine_tune / 1200.0) *
    std::pow(2.0, (double)fine / 1200.0) *
    std::pow(2.0, controls->tune_cents / 1200.0) *
    std::pow(2.0, random_pitch_cents(
      field_or(fields, fields->randomPitchDepth, tone, 0u),
      controls->lfo_seed) / 1200.0);
  double rootHz = 440.0 * std::pow(2.0,
                                    ((double)element.root_key - 69.0) / 12.0);
  /* MEASURED, from the ROM against itself: the wave-element record's
     `+0x0E` field is a SIGNED PITCH OFFSET of the recorded sample from its
     own root key - neutral at 1024, one semitone per 1024 units - and
     without it every instrument is detuned by its own amount.

     `wave_metadata.md` carries this field as "fine tune, typical
     0x03xx-0x06xx" with its units open (`U-R3-03`). They are recovered
     here by measuring the ROM's own sample data: for each element, the
     pitch of its loop region read at the wave ROM's 32 kHz against the
     frequency its root key names. Over the 26 elements the first factory
     song plays, 23 fit

         cents = -(fine - 1024) * 100 / 1024

     with a residual rms of 3.2 cents across a measured spread of 81 cents
     peak to peak. The three that do not are the three whose pitch the
     measurement cannot pin - `Orch. Hit`, which is a stab with no pitch,
     `Rot.Org Fst`, whose second harmonic is louder than its fundamental,
     and `EG Harm`, a guitar harmonic.

     This is what "the instruments are out of tune" was: not a transpose,
     which is why the whole mix still aligned with the hardware take to
     0 cents, but every element sitting at its own offset - from -23 to
     +48 cents in this song alone - so the parts disagree with each other. */
  rootHz *= std::pow(2.0, -((double)element.fine_tune - 1024.0) / 1024.0 / 12.0);
  if (rootHz <= 0.0) {
    wave_cache_release(cache, voice->pcm);
    voice->pcm = nullptr;
    return false;
  }
  voice->increment = (keyHz / rootHz) * (kXpNativeRate / outputRate);
  /* THE PLAYBACK CEILING: a wave is never read faster than four times its
     own rate, two octaves above the root its zone was recorded at,
     whatever asks for more. MEASURED on three zones of the `Sine`, each
     against the root and fine tune its element record gives: M-005's
     keys 120 and 121 both sound 7998 Hz, four times the top zone's
     1999.4 Hz root; and in TASK-364's `porta_cap_rate` (`P-xxxx`) a
     glide down to key 48 holds at key 70.04 until its own law carries it
     below, on the zone whose root stands at 46.04, and one down to key
     72 holds at 94.43 on the zone rooted at 70.41 - both +24.00 keys.
     This is `H-012`, the mechanism M-005 left open. NOT MEASURED: where
     in the pitch sum the ceiling bites relative to FXM, which is applied
     after it here, and whether a reversed wave has the same one. */
  voice->step_ceiling = 4.0 * (kXpNativeRate / outputRate);
  voice->bend_ratio = 1.0;
  /* A glide moves the key, so the tone's pitch key follow scales it as it
     scales the key: at 100 % a key is 100 cents. NOT MEASURED at any other
     key follow - every portamento take is on the bench Sine at 100 %. */
  voice->porta_kf = key_follow(rom, fields->pitchKeyFollow, tone, 1.0);
  voice->porta_ratio = voice->porta_end_ratio = 1.0;
  voice->porta_cents = voice->porta_end_cents = 0.0;
  voice->porta_factor = 1.0;
  voice->porta_cents_step = 0.0;
  voice->porta_left = 0u;

  /* Amplitude. MEASURED (`M-029`): curve 0 fits `40*log10(v/127)` - the
     same square law the level fields use - to a worst 1.35 dB, and curve 0
     is what the bench selects. The other six are seven distinct measured
     laws and an implementation needs all seven; only ten points per curve
     exist in `M-029` and they are not transcribed into this project's data
     yet, so a tone selecting one of them is rendered on curve 0 and is
     WRONG BY UP TO 36 dB at velocity 64 (curve 2 reads -48.4 dB there
     against curve 0's -11.8). */
  double velocityGain = velocity_curve_gain(
    field_or(fields, fields->ampVelocityCurve, tone, 0u),
    sensed_velocity(velocity,
                    (int)(int8_t)(uint8_t)field_or(
                      fields, fields->ampVelocitySens, tone, 50u)));
  voice->tone_level_gain = square_law_gain(tone[fields->level]) *
    bias_gain(fields, tone, key);
  voice->outer_level_gain = square_law_gain(controls->patch_level) *
    square_law_gain(controls->part_level);
  voice->gain_levels = voice->tone_level_gain * voice->outer_level_gain;
  voice->velocity_gain = velocityGain;
  voice->gain_velocity = velocityGain;
  /* MEASURED (`P-xxxx`, `basic/wave_number_map_g2` and `_g3`: 142 waves,
     one note each at key 60 against our render of the same files): the
     element record's first byte is a level, read through the same square
     law as the level fields. Against the byte-127 elements, 11 notes at
     117 read -1.49 dB (the law: -1.42), 4 at 112 -2.45 (-2.18), 5 at 120
     -0.74 (-0.98), 3 at 107 -3.71 (-2.98); the few below 107 scatter and
     do not pin the law there. */
  voice->gain_wave = wave_gain(field_or(fields, fields->waveGain, tone, 1u)) *
    square_law_gain(element.attenuation);
  voice->gain_mix =
    profile->voiceMixScale > 0.0 ? profile->voiceMixScale : 1.0;
  voice->fade_gain = velocity_fade_gain(fields, tone, velocity);
  voice->gain_fade = voice->fade_gain;
  jv1080_voice_set_volume(voice, controls->volume);

  /* Pan: the tone's and the patch's index one table and sum as offsets from
     centre (`M-002`, `M-048`). */
  int panOffset = (int)tone[fields->pan] - 64 +
    ((int)controls->patch_pan - 64) + ((int)controls->part_pan - 64) +
    note_pan_offset(fields, tone, soundedKey, controls);
  if (panOffset < -64)
    panOffset = -64;
  if (panOffset > 63)
    panOffset = 63;
  /* The difference is the right channel's level minus the left's, so the
     ratio multiplies the RIGHT gain. Applying it to the left instead put
     every pan on the wrong side, which is audible on any part the song
     places off centre and was confirmed on the device: a tone pan of 127
     with the part centred measures the right channel 56 dB above the left
     on hardware, where this rendered it 62 dB BELOW. */
  voice->pan_offset = panOffset;
  double difference = pan_difference_db(panOffset);
  double ratio = std::pow(10.0, difference / 20.0);
  double left = std::sqrt(1.0 / (1.0 + ratio * ratio));
  voice->gain_left = left;
  voice->gain_right = ratio * left;
  voice->pan_target_left = voice->gain_left;
  voice->pan_target_right = voice->gain_right;
  voice->pan_moving_left = false;
  voice->pan_moving_right = false;
  voice->pan_started = false;
  voice->pan_period = (size_t)std::lround(outputRate * kPanSlewTickSeconds);
  if (!voice->pan_period)
    voice->pan_period = 1u;
  voice->pan_countdown = 0u;

  /* The envelope's three level fields plus its implicit final zero. */
  for (unsigned i = 0; i < 3u; ++i) {
    voice->level_units[i] = (double)tone[fields->ampLevel1 + i];
    voice->level[i] = amp_env_units_amplitude(voice->level_units[i]);
  }
  voice->level_units[3] = 0.0;
  voice->level[3] = 0.0;
  voice->time[0] = amp_env_attack_seconds(tone[fields->ampTime1]) *
    amp_env_velocity_time_scale(
      field_or(fields, fields->ampEnvVelTime1, tone, 7u), velocity);
  /* MEASURED (`M-066`): time key follow scales times 2-4 by a factor of
     two per octave of the key about key 60, and leaves the attack alone -
     the same law the filter and pitch envelopes take (`M-069`). The key is
     the sounded one, as theirs is; whether the patch's octave shift counts
     toward it is not measured - `M-066`'s takes have none, and the factory
     patches that carry one do not separate the two readings. */
  double ampTimeKf = time_key_follow(rom, fields->ampEnvTimeKeyFollow, tone);
  for (unsigned i = 1; i < 4u; ++i)
    voice->time[i] =
      amp_env_fall_seconds_per_20db(tone[fields->ampTime1 + i]) *
      time_key_follow_scale(ampTimeKf, soundedKey);
  /* NO-SUSTAIN, the rhythm note's envelope mode 0 and 619 of the 640
     factory drum keys: the note-off does not cut the first three segments
     short, and waits for them. Read off the six factory-kit takes
     (`rhythm/kit_pr_*`, `kit_gm_1`: 64 keys each, 350 ms gates), against
     three other readings of the mode:

       level 360-430 ms after the onset, mean |ours - hardware| over keys
                          PR-A1  PR-A2  PR-B1  PR-C1  PR-C2  GM1
       release at note-off 21.7   21.6   21.4   10.2   11.6  10.8
       deferred (this)     10.6    9.1    8.1    8.0   10.5  10.8

     Releasing at the end of time 3 whatever the key is doing runs 9 to 15
     dB low before the note-off on PR-C and GM, whose keys sustain at level
     3 while held, and never releasing leaves them ringing. `M-004`'s one
     record - mode 0 with times 1 to 3 at zero, silent on hardware - is NOT
     reproduced by this law and stays unexplained; its own bisection calls
     the result context-dependent. */
  voice->one_shot = fields->envelopeMode != XP_VOICE_FIELD_NONE &&
    tone[fields->envelopeMode] == 0u;
  voice->segment = 0u;
  voice->envelope = 0.0;
  voice->segment_start = 0.0;
  voice->segment_remaining = voice->time[0];
  voice->sample_period = 1.0 / outputRate;
  voice->segment_total = voice->segment_remaining;

  voice->filter_type = (int)tone[fields->filterType];
  voice->output_rate = outputRate;
  /* MEASURED (`M-077`): cutoff key follow scales the corner by
     `2^(kf * (key - 60)/12)`, which in the cutoff law's own units - ten per
     octave (`M-012`) - is an offset to the parameter, so the clamp below
     applies to the sum. */
  voice->cutoff_base = (double)tone[fields->cutoff] +
    10.0 * key_follow(rom, fields->cutoffKeyFollow, tone, 0.0) *
      ((double)soundedKey - kKeyFollowPivot) / 12.0;
  voice->resonance_base = tone[fields->resonance];
  voice->resonance_value = voice->resonance_base;
  voice->resonance_q = tvf_q(voice->resonance_base);
  voice->resonance_q_base = voice->resonance_q;
  voice->matrix_cutoff = 0.0;
  voice->matrix_pan = 0.0;
  voice->matrix_pitch_ratio = 1.0;

  /* The filter envelope. It moves the cutoff PARAMETER, so its whole
     sweep is worked out in cutoff units here and the corner law is applied
     to the sum once per control block. */
  voice->cutoff_offset = jv1080_filter_env_offset(fields, tone, velocity);
  if (voice->cutoff_offset != 0.0) {
    double timeKf = time_key_follow(rom, fields->filterEnvTimeKeyFollow, tone);
    unsigned velT1 = field_or(fields, fields->filterEnvVelTime1, tone, 7u);
    unsigned velT4 = field_or(fields, fields->filterEnvVelTime4, tone, 7u);
    for (unsigned i = 0; i < 4u; ++i) {
      voice->fenv_level[i] =
        (double)tone[fields->filterEnvLevel1 + i] / 127.0;
      voice->fenv_time[i] =
        filter_env_segment_seconds(tone[fields->filterEnvTime1 + i]);
      /* `M-066` measured on the amplitude envelope that time key follow
         does not scale the attack; the filter envelope's own attack was
         not measured separately and follows that rule here. */
      if (i)
        voice->fenv_time[i] *= time_key_follow_scale(timeKf, soundedKey);
    }
    voice->fenv_time[0] *= velocity_time_scale(velT1, velocity);
    voice->fenv_time[3] *= velocity_time_scale(velT4, velocity);
    /* The envelope starts closed - at the record's own cutoff - and its
       first segment is the move to level 1. */
    voice->fenv_value = 0.0;
    filter_env_enter(voice, 0u);
    /* `M-074`: a modulator re-evaluated once per block of a millisecond or
       less is indistinguishable from the machine on the one observable
       that can see it. */
    voice->control_period = (size_t)(outputRate / 1000.0);
    if (!voice->control_period)
      voice->control_period = 1u;
    voice->control_countdown = 0u;
  }

  /* The two LFOs. A record with no depth anywhere leaves lfo_active false
     and takes none of the paths below. */
  voice->lfo_active = false;
  voice->lfo_pitch_ratio = 1.0;
  voice->lfo_gain = 1.0;
  voice->lfo_cutoff = 0.0;
  if (fields->lfoFirst[0] != XP_VOICE_FIELD_NONE) {
    static const double kOffsets[5] = { -1.0, -0.5, 0.0, 0.5, 1.0 };
    for (unsigned i = 0; i < 2u; ++i) {
      const uint8_t *f = tone + fields->lfoFirst[i];
      struct XpJv1080Lfo *lfo = voice->lfo + i;
      lfo->form = f[0] & 7u;
      /* EXT SYNC (f[7]) follows the tempo in force: the performance's
         Default tempo, not the program-changed patch's own - a PR-B 055
         chord selected by program change inside a 120 BPM performance
         reads 7.95 Hz from a rate-12 synced LFO, 6.04 pulses at 120 BPM
         where the patch's 86 BPM would give a non-integer 4.33
         (`M-142`). Clock
         source MIDI (`M-103`) is not modelled: this engine keeps no
         incoming clock. */
      lfo->frequency = f[7] ? lfo_sync_hz(f[2], controls->tempo_bpm)
                            : lfo_rate_hz(f[2]);
      lfo->offset = kOffsets[f[3] > 4u ? 2u : f[3]];
      lfo->delay = lfo_time_seconds(kLfoDelay, 7u, f[4]);
      lfo->fade_mode = f[5] & 3u;
      lfo->fade = lfo_time_seconds(kLfoFade, 7u, f[6]);
      /* Key trigger starts the cycle at zero (`M-071`); off, the LFO runs
         free, taken here as one clock the whole engine shares - whether
         the machine keeps one per part or per voice is not measured. */
      double cycles = f[1] ? 0.0 : controls->clock_seconds * lfo->frequency;
      lfo->phase = cycles - std::floor(cycles);
      lfo->seed = controls->lfo_seed * 2654435761u + i * 40503u + 1u;
      lfo->held = (double)(lfo_random(&lfo->seed) >> 8) / 8388608.0 - 1.0;
      lfo->since_on = 0.0;
      lfo->since_off = -1.0;
      double pitch = (double)(int8_t)tone[fields->pitchLfoDepth + i];
      double filter = (double)(int8_t)tone[fields->filterLfoDepth + i];
      double amp = (double)(int8_t)tone[fields->ampLfoDepth + i];
      double pan = (double)(int8_t)tone[fields->panLfoDepth + i];
      voice->lfo_pitch_cents[i] = signed_law(pitch, pitch_depth_cents);
      voice->lfo_cutoff_units[i] =
        voice->filter_type ? 10.0 * signed_law(filter, filter_depth_octaves)
                           : 0.0;
      voice->lfo_amp_db[i] = signed_law(amp, amp_depth_db);
      voice->lfo_pan_units[i] = signed_law(pan, pan_depth_units);
      if (voice->lfo_pitch_cents[i] != 0.0 ||
          voice->lfo_cutoff_units[i] != 0.0 || voice->lfo_amp_db[i] != 0.0 ||
          voice->lfo_pan_units[i] != 0.0)
        voice->lfo_active = true;
    }
    voice->lfo_period = (size_t)(outputRate / 1000.0);
    if (!voice->lfo_period)
      voice->lfo_period = 1u;
    voice->lfo_countdown = 0u;
  }
  if (fields->lfoFirst[0] != XP_VOICE_FIELD_NONE)
    for (unsigned i = 0; i < 2u; ++i) {
      voice->lfo_base_cents[i] = voice->lfo_pitch_cents[i];
      voice->lfo_base_frequency[i] = voice->lfo[i].frequency;
      voice->lfo_base_cutoff_units[i] = voice->lfo_cutoff_units[i];
    }

  /* FXM. MEASURED (`P-xxxx`, `structure/fxm_off`, `fxm_color_c0..c3`,
     `fxm_depth_sweep`: a `Sine` at key 60). The switch on puts sidebands
     at exactly f +- n * 250 / (colour + 1) Hz - 250, 125, 83.3 and 62.5 Hz
     for colours 0 to 3 - odd orders strong and even ones some 45 dB under,
     so the rate is switched in a square of that frequency. Read as an
     instantaneous frequency on colour 3, the note holds two rates in turn,
     205 and 316.5 Hz, whose arithmetic mean is the unmodulated 261.6: the
     pair straddles the pitch, 2 / (1 + k) and 2 k / (1 + k). Fitting k to
     the upper first sideband on the four colour takes, all at depth 8,
     gives 2^(-g/16) with g 9.20, 9.25, 8.80 and 9.20 - the 2^(-(d + 1)/16)
     the sibling engine reads off its own ROM, with depth 0 still
     modulating; the depth sweep's shorter notes read g about d, a spread
     the line fit's window has not been separated from. Against our own
     render of the same files through the same analysis, colours 1 and 3
     agree within 0.3 dB and colour 2 within 1 dB; at colour 0 our upper
     first sideband runs 1.1 to 1.3 dB over the machine's at every depth,
     and the machine carries a line at f - 250 Hz (11.6 Hz here) some 6 dB
     over ours, present on every FXM take and absent from `fxm_off`. Our
     output's DC blocker takes part of that line; the rest, and the colour
     0 excess, are not resolved. Which rate comes first after note-on is
     not measured. With the switch off the depth does nothing. */
  voice->fxm_active = false;
  voice->fxm_clock = 0.0;
  if (fields->fxmSwitch != XP_VOICE_FIELD_NONE && tone[fields->fxmSwitch]) {
    unsigned colour = field_or(fields, fields->fxmColor, tone, 0u) & 3u;
    unsigned depth = field_or(fields, fields->fxmDepth, tone, 0u) & 15u;
    double k = std::pow(2.0, -(double)(depth + 1u) / 16.0);
    voice->fxm_ratio[0] = 2.0 / (1.0 + k);
    voice->fxm_ratio[1] = 2.0 * k / (1.0 + k);
    voice->fxm_half = 0.5 * (double)(colour + 1u) / 250.0;
    voice->fxm_active = true;
  }

  /* The pitch envelope. */
  voice->penv_active = false;
  voice->penv_ratio = 1.0;
  if (fields->pitchEnvDepth != XP_VOICE_FIELD_NONE) {
    double depth = (double)(int8_t)tone[fields->pitchEnvDepth];
    double reach = (double)sensed_velocity(velocity,
      (int)(int8_t)(uint8_t)field_or(fields, fields->pitchEnvVelSens, tone, 0u));
    reach = reach <= 1.0 ? 0.0 : (reach - 1.0) / 126.0;
    double timeKf = time_key_follow(rom, fields->pitchEnvTimeKeyFollow, tone);
    bool moves = false;
    for (unsigned i = 0; i < 4u; ++i) {
      voice->penv_level[i] = depth * 100.0 * reach *
        (double)(int8_t)tone[fields->pitchEnvLevel1 + i] / 63.0;
      voice->penv_time[i] = pitch_env_seconds(tone[fields->pitchEnvTime1 + i]);
      if (i)
        voice->penv_time[i] *= time_key_follow_scale(timeKf, soundedKey);
      if (voice->penv_level[i] != 0.0)
        moves = true;
    }
    voice->penv_time[0] *= velocity_time_scale(
      field_or(fields, fields->pitchEnvVelTime1, tone, 7u), velocity);
    voice->penv_time[3] *= velocity_time_scale(
      field_or(fields, fields->pitchEnvVelTime4, tone, 7u), velocity);
    if (moves) {
      voice->penv_active = true;
      voice->penv_value = 0.0;
      pitch_env_enter(voice, 0u);
      voice->penv_period = (size_t)(outputRate / 1000.0);
      if (!voice->penv_period)
        voice->penv_period = 1u;
      voice->penv_countdown = 0u;
    }
  }

  /* The controller matrix, at the sources as they stand at note-on. */
  voice->matrix_used = false;
  if (fields->matrixFirst != XP_VOICE_FIELD_NONE)
    for (unsigned s = 0; s < 12u; ++s) {
      voice->matrix_dest[s] = tone[fields->matrixFirst + 2u * s];
      voice->matrix_depth[s] =
        (double)(int8_t)tone[fields->matrixFirst + 2u * s + 1u];
      if (voice->matrix_dest[s] && voice->matrix_depth[s] != 0.0)
        voice->matrix_used = true;
    }
  jv1080_voice_set_matrix(voice, controls->matrix_source);

  if (voice->filter_type)
    set_filter(voice, outputRate);

  voice->active = true;
  return true;
}

/* Time 4 names a rate, so the release's duration is how far the envelope
   has to travel at it. Sixty dB is taken as silent: the level table's own
   floor is the machine's noise floor and no field can ask for less. */
/* MEASURED (`closeout/cc7_residual`): CC7 moves a note that is already
   sounding - one note held through the whole take while CC7 steps 127, 0,
   1, 2, 3, 4, 6, 8 and back, each step landing on the law. Whether the
   machine eases the step rather than jumping is not measured; here it
   jumps. */
void jv1080_voice_set_volume(struct XpJv1080Voice *voice, unsigned volume)
{
  if (!voice)
    return;
  voice->volume = volume;
  voice->static_gain_unwaved = voice->gain_levels * cc7_gain(volume) *
    voice->gain_velocity * voice->gain_fade * voice->gain_mix;
  voice->static_gain = voice->gain_levels * cc7_gain(volume) *
    voice->gain_velocity * voice->gain_fade * voice->gain_wave * voice->gain_mix;
  voice->tone_gain = (voice->outer_level_gain > 0.0
                        ? voice->gain_levels / voice->outer_level_gain : 0.0) *
    voice->gain_velocity * voice->gain_fade * voice->gain_wave;
}

/* THE CONTROLLER MATRIX. Each slot's effective depth is its depth times its
   controller's source, 0..1 - linear, MEASURED on LEV at four source values
   and on CUT at two (`M-116`). Slots aiming at one destination are summed;
   that summing is not measured. The laws, all `M-116`, at tone level 127
   unless named:

     PCH  0.31 * d^2 cents, signed. Three points, depths 8, 20, 40, fit to
          4 %; above 40 the law is extrapolated.
     LEV  adds d/63 of full-scale amplitude to the tone level's own square
          law, the sum clamped to 0..1 - within 0.2 dB at tone levels 32, 64,
          96 and 127, both signs, at velocity sensitivity 0. Velocity
          scales the tone level BEFORE the sum and is not applied again
          after it (`M-159`, TASK-421: TASK-375's factory sweep, key 60,
          velocity 100, CC11 127 - on the 227 slots where the two orders
          differ, binned by predicted difference, level against the
          untouched slots reads -4.67 to -0.74 dB with velocity after the
          sum and -0.51 to +0.06 with it before). Only velocity 100 is
          measured this way. Patch and part level scale after the sum.
          The velocity cross fade (velocity_fade_gain) scales the tone
          term before the sum too. MEASURED (`M-169`, TASK-428) on a
          dedicated hardware capture, isolating the order directly: one
          tone ranged 65-127 with fade depth 64 against a fixed reference
          tone, matrix LEV +20 via CC1, at sensitivity 0 and +50, read at
          nine velocities down to 9. With the fade before the sum, the
          predicted L-R level tracks the hardware capture within about
          0.4 dB at every depth and both sensitivities; fading it after
          the sum instead diverges with depth, reaching 28-42 dB off at
          the deepest points measured. The factory sweep first suggested
          this order (TASK-375's own key-60/velocity-100/CC11-127 sweep,
          `M-159`, TASK-421, confirmed the same order for the velocity
          term itself); the dedicated capture confirms it directly for
          the fade term too.
     CUT  2.3 cutoff units per step, from depths -16 to +16 read against the
          cutoff field itself, +-2 units.
     RES  about 2 resonance units per step: APPROXIMATE, the four readings
          scatter +-14 units and none matches a resonance-field spectrum
          well, so the matrix may not act on the field at all.
     PL1  adds to the tone's own pitch LFO depth in cents, through the same
          depth law. The matrix's own readings run 3-9 % above that law at
          the same depth; that gap is not recovered and not fitted.
     L1R  0.198 Hz per step, linear in hertz rather than in rate units, and
          clamped at 0 Hz - three points 5, 20, 63 at rate 64.
     PAN  2 pan-table units of distance per step, both signs (`M-155`, the
          corpus `routing/matrix_dest_all_{pos,neg}` takes: at effective
          depths 8.9, 17.9 and 26.8 the channel difference reads 18.6,
          35.3, 53.3 units right and 17.8, 34.8, 51.9 left through the pan
          table, 1.94-2.08 per step; 35.7 and above are hard). It adds to
          the voice's own pan and the tone's pan LFO, and the sum is clamped
          to the table once (`M-155`, `pan_clamp`: tone pan 64, PAN +63 and
          a pan LFO of +63 on a square stay hard right through both halves
          at CC1 127, and at CC1 64 the bottom half lands on centre, 0.0 dB).
          A move then slews rather than jumps (see set_pan).
     FL1  2.3 cutoff units per step, LINEAR, added to the tone's own filter
          LFO swing (`M-155`, `fl_law`/`fl_sum`: White Noise through LPF 86
          resonance 80 under a key-triggered TRI at rate 64, the resonant
          peak read at the triangle's tops and bottoms). The corner moves
          0.41, 0.80, 1.61, 2.48 and 3.18 octaves at depths 2, 4, 8, 12 and
          16, -8 the mirror of +8; read back through tvf_natural_hz, whose
          static peak here lands within 1 % of the machine's, that is
          2.20-2.36 units per step over twelve unclipped half-cycles, mean
          2.29 - CUT's own 2.3 (`M-116`). Tone depths 16 and 32 with the
          matrix at +-8 land on the sum of the two swings. It is not the
          tone depth's own law (`M-114`), 0.31 octave at 16. Above 16 each
          half of the swing clips against the resonant peak's ceiling or
          the analysis floor, so the line is carried on there, not
          measured.
     FL2  the same on the second LFO, measured at 8 and +-16.

   PL2 and L2R are taken as PL1 and L1R on the second LFO, which is not
   measured. MIX, CHO and REV (the sends), AL1/AL2 and pL1/pL2 are NOT
   IMPLEMENTED: a slot routed to one does nothing. */
const uint8_t kMatrixPch = 1u, kMatrixCut = 2u, kMatrixRes = 3u,
  kMatrixLev = 4u, kMatrixPan = 5u, kMatrixPl1 = 9u, kMatrixPl2 = 10u,
  kMatrixFl1 = 11u, kMatrixFl2 = 12u, kMatrixL1r = 17u, kMatrixL2r = 18u;
const double kMatrixPanUnitsPerStep = 2.0;
const double kMatrixFilterLfoUnitsPerStep = 2.3;

void jv1080_voice_set_matrix(struct XpJv1080Voice *voice,
                              const double source[3])
{
  if (!voice || !source || !voice->matrix_used)
    return;
  double cents = 0.0, level = 0.0, cutoff = 0.0, resonance = 0.0, pan = 0.0;
  double lfoCents[2] = { 0.0, 0.0 }, lfoHz[2] = { 0.0, 0.0 };
  double lfoCutoff[2] = { 0.0, 0.0 };
  bool moveLevel = false, moveResonance = false, movePan = false;
  bool moveLfoCutoff = false;
  for (unsigned s = 0; s < 12u; ++s) {
    double d = voice->matrix_depth[s] * source[s / 4u];
    switch (voice->matrix_dest[s]) {
    case kMatrixPch: cents += (d < 0.0 ? -0.31 : 0.31) * d * d; break;
    case kMatrixLev: level += d / 63.0; moveLevel = true; break;
    case kMatrixCut: cutoff += 2.3 * d; break;
    case kMatrixRes: resonance += 2.0 * d; moveResonance = true; break;
    case kMatrixPan:
      pan += kMatrixPanUnitsPerStep * d;
      movePan = true;
      break;
    case kMatrixFl1:
    case kMatrixFl2:
      lfoCutoff[voice->matrix_dest[s] - kMatrixFl1] +=
        kMatrixFilterLfoUnitsPerStep * d;
      moveLfoCutoff = true;
      break;
    case kMatrixPl1:
    case kMatrixPl2:
      lfoCents[voice->matrix_dest[s] - kMatrixPl1] +=
        signed_law(d, pitch_depth_cents);
      break;
    case kMatrixL1r:
    case kMatrixL2r:
      lfoHz[voice->matrix_dest[s] - kMatrixL1r] += 0.198 * d;
      break;
    default: break;
    }
  }
  voice->matrix_pitch_ratio = std::pow(2.0, cents / 1200.0);
  double tone = voice->tone_level_gain;
  voice->gain_velocity = voice->velocity_gain;
  voice->gain_fade = voice->fade_gain;
  if (moveLevel) {
    tone = tone * voice->velocity_gain * voice->fade_gain + level;
    tone = tone < 0.0 ? 0.0 : (tone > 1.0 ? 1.0 : tone);
    voice->gain_velocity = 1.0;
    voice->gain_fade = 1.0;
  }
  voice->gain_levels = tone * voice->outer_level_gain;
  jv1080_voice_set_volume(voice, voice->volume);
  for (unsigned i = 0; i < 2u; ++i) {
    voice->lfo_pitch_cents[i] = voice->lfo_base_cents[i] + lfoCents[i];
    double hz = voice->lfo_base_frequency[i] + lfoHz[i];
    voice->lfo[i].frequency = hz < 0.0 ? 0.0 : hz;
    if (voice->lfo_pitch_cents[i] != 0.0)
      voice->lfo_active = true;
    if (moveLfoCutoff && voice->filter_type) {
      voice->lfo_cutoff_units[i] =
        voice->lfo_base_cutoff_units[i] + lfoCutoff[i];
      if (voice->lfo_cutoff_units[i] != 0.0)
        voice->lfo_active = true;
    }
  }
  if (movePan) {
    voice->matrix_pan = pan;
    set_pan(voice, (double)voice->pan_offset + pan);
  }
  if (voice->filter_type) {
    voice->matrix_cutoff = cutoff;
    if (moveResonance) {
      double r = (double)voice->resonance_base + resonance;
      r = r < 0.0 ? 0.0 : (r > 127.0 ? 127.0 : r);
      voice->resonance_value = (unsigned)std::lround(r);
      voice->resonance_q = tvf_q(voice->resonance_value);
    } else {
      voice->resonance_value = voice->resonance_base;
      voice->resonance_q = voice->resonance_q_base;
    }
    set_filter(voice, voice->output_rate);
  }
}

void jv1080_voice_glide(struct XpJv1080Voice *voice, double fromCents,
                        double toCents, double centsPerSecond)
{
  voice->porta_end_cents = toCents;
  voice->porta_end_ratio =
    std::pow(2.0, toCents * voice->porta_kf / 1200.0);
  double distance = std::fabs(toCents - fromCents);
  double samples = centsPerSecond > 0.0
    ? distance / centsPerSecond * voice->output_rate : 0.0;
  if (!(samples >= 1.0)) {
    voice->porta_left = 0u;
    voice->porta_ratio = voice->porta_end_ratio;
    voice->porta_cents = toCents;
    return;
  }
  voice->porta_left = (size_t)std::ceil(samples);
  voice->porta_cents = fromCents;
  voice->porta_ratio = std::pow(2.0, fromCents * voice->porta_kf / 1200.0);
  voice->porta_cents_step = (toCents - fromCents) / samples;
  voice->porta_factor =
    std::pow(2.0, voice->porta_cents_step * voice->porta_kf / 1200.0);
}

double jv1080_voice_glide_cents(const struct XpJv1080Voice *voice)
{
  return voice->porta_cents;
}

void jv1080_voice_note_off(struct XpJv1080Voice *voice)
{
  if (voice && voice->one_shot && voice->segment < 4u && !voice->releasing) {
    voice->pending_release = true;
    return;
  }
  jv1080_voice_release(voice);
}

void jv1080_voice_release(struct XpJv1080Voice *voice)
{
  if (!voice || !voice->active || voice->releasing)
    return;
  voice->releasing = true;
  voice->lfo[0].since_off = 0.0;
  voice->lfo[1].since_off = 0.0;
  if (voice->penv_active)
    pitch_env_enter(voice, 3u);
  voice->segment = 3u;
  voice->segment_start = amp_env_amplitude_units(voice->envelope);
  voice->segment_total = amp_env_segment_seconds(voice->time[3]);
  voice->segment_remaining = voice->segment_total;
  voice->envelope_ratio_valid = false;
  /* The filter envelope releases on the same note-off, from wherever it
     had reached, to its own level 4. */
  if (voice->cutoff_offset != 0.0)
    filter_env_enter(voice, 3u);
}

namespace {

/* One sample of a voice, in the stages the structure types route between:
   the control blocks, the wave generator, the amplitude envelope, the TVA's
   gain, the TVF and the read head's advance. jv1080_voice_render runs them
   in this order; a structured pair runs the two voices' stages in its own. */
void voice_controls(struct XpJv1080Voice *voice, bool sweeping, bool lfoFilter)
{
  /* The LFOs, once per control block as the filter envelope is
     (`M-074`). */
  if (voice->lfo_active) {
    if (!voice->lfo_countdown) {
      lfo_update(voice, (double)voice->lfo_period * voice->sample_period);
      if (lfoFilter && !sweeping)
        set_filter(voice, voice->output_rate);
      voice->lfo_countdown = voice->lfo_period;
    }
    --voice->lfo_countdown;
  }
  /* The pitch envelope, once per control block as the others are. */
  if (voice->penv_active) {
    if (!voice->penv_countdown) {
      pitch_env_advance(voice, (double)voice->penv_period *
                                 voice->sample_period);
      voice->penv_ratio = std::pow(2.0, voice->penv_value / 1200.0);
      voice->penv_countdown = voice->penv_period;
    }
    --voice->penv_countdown;
  }
  /* The filter envelope, once per control block rather than per sample
     (`M-074`). A voice whose envelope cannot move the corner - no filter
     or no depth - never enters here and its section is solved once at
     note-on, as it was before this envelope existed. */
  if (sweeping) {
    if (!voice->control_countdown) {
      filter_env_advance(voice, (double)voice->control_period *
                                  voice->sample_period);
      set_filter(voice, voice->output_rate);
      voice->control_countdown = voice->control_period;
    }
    --voice->control_countdown;
  }
  /* The pan, after the LFOs so that the first tick snaps to a pan their
     first update has already moved. */
  if (!voice->pan_countdown) {
    pan_tick(voice);
    voice->pan_countdown = voice->pan_period;
  }
  --voice->pan_countdown;
}

bool voice_wave(struct XpJv1080Voice *voice, double *out)
{
  /* THE INTERPOLATOR IS A FOUR-POINT CUBIC B-SPLINE, the kernel this
     chip was measured to use - the same one `oscillator.cc` gives the
     sibling engine, since it is the same Roland part.

     This read was two-point linear, on `M-087`. THAT MEASUREMENT IS
     WITHDRAWN ON THE KERNEL by `M-107` and `M-108`, and the reason is
     instructive rather than a correction of arithmetic: M-087 ranked
     four candidates by time-domain correlation, all four of them
     INTERPOLATING kernels and therefore the identity at fraction 0, so
     none of them could represent a chip that smooths when the fraction
     is zero and the ranking could not detect one. Linear won by being
     the least sharp of four candidates all sharper than the chip. Read
     as a RESPONSE instead, over 42 dB of span, a cubic B-spline fits to
     0.63 and 0.59 dB rms on the two waves against two-point linear's
     5.48 and 4.23; and a dedicated unity-ladder capture measures the
     fraction-0 weights directly as `[w, 1-2w, w]` with
     w = 0.1614 +- 0.0088, against 1/6 for the cubic (0.6 sigma) and 0
     for every interpolating kernel (18.3 sigma). The chip is NOT
     transparent at its own root key: it is 8.4 dB down at theta 0.39
     where a linear read would be flat.

     What that costs in the mix is the top octave. Reading linearly left
     the interpolation's images unsmoothed, and against a hardware take
     of the first factory song this engine carried 15.2 % of its energy
     between 8 and 16 kHz where the machine carries 1.3 %. */
  double frac;
  double v0, v1, vBack, vFwd;
  if (voice->in_cycle) {
    /* Past the first turn, `position` is a position in the ping-pong
       cycle, not an index into `pcm`. Every one of the four taps goes
       through the same mapping, so the turn needs no special case: the
       interpolator reads across it exactly as it reads anywhere else. */
    long long c0 = (long long)voice->position;
    frac = voice->position - (double)c0;
    struct PingPongCycle c;
    if (ping_pong_cycle(voice, &c)) {
      v0 = cycle_value(&c, cycle_wrap(c0, c.cycle));
      v1 = cycle_value(&c, cycle_wrap(c0 + 1, c.cycle));
      vBack = cycle_value(&c, cycle_wrap(c0 - 1, c.cycle));
      vFwd = cycle_value(&c, cycle_wrap(c0 + 2, c.cycle));
    } else {
      v0 = v1 = vBack = vFwd = 0.0;
    }
  } else {
  size_t i0 = (size_t)voice->position;
  /* The loop's last sample is a valid read head position - its partner
     for the interpolation is the loop's first sample - so only a head
     genuinely past it has run off the end. THE LOOP INCLUDES THAT LAST
     SAMPLE: reading straight on past it instead costs the loop a sample,
     a pitch error of one part in the loop's length, which is nothing on
     a long loop and 73.6 measured cents on the internal `Sine` wave's
     24-sample top zone. */
  bool atLoopEnd = voice->looping && i0 == voice->loop_last;
  /* A reversed element starts its head on the last sample and walks down,
     so the far end is where it begins, not where it has played out; its
     own end is voice_advance's, and wave_tap stands in for the tap past
     the last sample. */
  if (!voice->reverse && !atLoopEnd && i0 + 1u >= voice->pcm_count) {
    if (!voice->looping) {
      voice->active = false;   /* the element is played out */
      return false;
    }
    i0 = voice->loop_first;
    voice->position = (double)i0;
  }
  frac = voice->position - (double)i0;
  /* The uniform cubic B-spline basis, with `rest` = 1 - fraction:
       index - 1   rest^3 / 6
       index       2/3 - fraction^2 + fraction^3 / 2
       index + 1   2/3 - rest^2     + rest^3 / 2
       index + 2   fraction^3 / 6
     which is [1/6, 2/3, 1/6, 0] at fraction 0 - a smoother, not an
     identity - and sums to one at every fraction. */
  v0 = (double)voice->pcm[i0];
  /* Clear of both ends of the data and of the loop's seam, every tap is
     the plain sample wave_tap would read there - the common case, taken
     without its wrap and turn tests. */
  const bool seam = voice->loop_last >= voice->loop_first &&
    (voice->ping_pong || voice->looping);
  if (i0 >= 1u && i0 + 2u < voice->pcm_count &&
      (!seam || (i0 + 2u <= voice->loop_last &&
                 (voice->ping_pong || i0 != voice->loop_first)))) {
    v1 = (double)voice->pcm[i0 + 1u];
    vBack = (double)voice->pcm[i0 - 1u];
    vFwd = (double)voice->pcm[i0 + 2u];
  } else {
    v1 = wave_tap(voice, i0, 1, v0);
    vBack = wave_tap(voice, i0, -1, v0);
    vFwd = wave_tap(voice, i0, 2, v1);
  }
  }
  double rest = 1.0 - frac;
  double sample = (rest * rest * rest / 6.0 * vBack +
                    (2.0 / 3.0 - frac * frac * (1.0 - frac * 0.5)) * v0 +
                    (2.0 / 3.0 - rest * rest * (1.0 - rest * 0.5)) * v1 +
                    frac * frac * frac / 6.0 * vFwd) / 8388608.0;
  *out = sample;
  return true;
}

bool voice_envelope(struct XpJv1080Voice *voice)
{
  /* The envelope, one segment at a time. The attack follows its measured
     shape over its measured duration; every later segment moves linearly
     in level units over its one duration (see amp_env_segment_seconds).
     Every segment that has run out is resolved before this sample's
     value, so a chain of zero-length ones takes no time at all. Neither
     is the chip's own segment stepper - that is internal (`M-011`'s own
     caveat). */
  if (voice->segment < 4u) {
    while (voice->segment < 4u && voice->segment_remaining <= 0.0) {
      voice->envelope = voice->level[voice->segment];
      if (voice->releasing) {
        voice->active = false;
        break;
      }
      if (voice->segment < 2u) {
        ++voice->segment;
        voice->segment_start = voice->level_units[voice->segment - 1u];
        voice->segment_total =
          amp_env_segment_seconds(voice->time[voice->segment]);
        voice->segment_remaining = voice->segment_total;
        voice->envelope_ratio_valid = false;
      } else {
        voice->segment = 4u;   /* holding the sustain level */
        if (voice->pending_release || voice->release_at_sustain) {
          jv1080_voice_release(voice);
        } else if (voice->one_shot && voice->envelope < 1e-5) {
          /* Holding at a level-3 of zero, with nothing left to come. */
          voice->active = false;
          break;
        }
      }
    }
    if (!voice->active)
      return false;
    if (voice->segment < 4u) {
      double done = voice->segment_total > 0.0
        ? 1.0 - voice->segment_remaining / voice->segment_total : 1.0;
      if (!voice->segment) {
        voice->envelope = voice->level[0] * amp_env_attack_shape(done);
      } else {
        double units = voice->segment_start +
          (voice->level_units[voice->segment] - voice->segment_start) * done;
        if (units != voice->envelope_units) {
#ifdef EMUSC_LEGACY_DSP_FAST
          voice->envelope_units_amplitude = amp_env_units_amplitude_carried(
            voice, units, voice->segment_total > 0.0
              ? (voice->level_units[voice->segment] - voice->segment_start) *
                  voice->sample_period / voice->segment_total
              : 0.0);
#else
          voice->envelope_units_amplitude = amp_env_units_amplitude(units);
#endif
          voice->envelope_units = units;
        }
        voice->envelope = voice->envelope_units_amplitude;
      }
      voice->segment_remaining -= voice->sample_period;
    }
  }
  return true;
}

double voice_tva(const struct XpJv1080Voice *voice, double sample)
{
  double value = sample * voice->envelope * voice->static_gain;
  if (voice->lfo_active)
    value *= voice->lfo_gain;

  return value;
}

double voice_tvf(struct XpJv1080Voice *voice, double value)
{
  if (voice->filter_type) {
    const double k = voice->svf_k;
    const double h1 = voice->svf_h1, h2 = voice->svf_h2, h3 = voice->svf_h3;
    const double v3 = value - voice->s2;
    const double v1 = h1 * voice->s1 + h2 * v3;             /* band-pass */
    const double v2 = voice->s2 + h2 * voice->s1 + h3 * v3; /* low-pass */
    voice->s1 = 2.0 * v1 - voice->s1;
    voice->s2 = 2.0 * v2 - voice->s2;
    value = voice->m_hp * (value - k * v1 - v2) + voice->m_bp * v1 +
      voice->m_lp * v2;
  }

  return value;
}

double fxm_step(struct XpJv1080Voice *voice)
{
  if (!voice->fxm_active)
    return 1.0;
  double ratio = voice->fxm_ratio[voice->fxm_clock < voice->fxm_half ? 0 : 1];
  voice->fxm_clock += voice->sample_period;
  if (voice->fxm_clock >= 2.0 * voice->fxm_half)
    voice->fxm_clock -= 2.0 * voice->fxm_half;
  return ratio;
}

bool voice_advance(struct XpJv1080Voice *voice)
{
  const double fxm = fxm_step(voice);
  if (voice->porta_left) {
    if (--voice->porta_left) {
      voice->porta_ratio *= voice->porta_factor;
      voice->porta_cents += voice->porta_cents_step;
    } else {
      voice->porta_ratio = voice->porta_end_ratio;
      voice->porta_cents = voice->porta_end_cents;
    }
  }
  if (voice->reverse) {
    double step = voice->increment * voice->bend_ratio *
      voice->matrix_pitch_ratio * voice->penv_ratio * voice->porta_ratio;
    if (voice->lfo_active)
      step *= voice->lfo_pitch_ratio;
    if (step > voice->step_ceiling)
      step = voice->step_ceiling;
    step *= fxm;
    voice->position -= step;
    if (voice->position < 1.0) {
      voice->active = false;
      return false;
    }
  } else {
    double step = voice->increment * voice->bend_ratio *
      voice->matrix_pitch_ratio * voice->penv_ratio * voice->porta_ratio;
    if (voice->lfo_active)
      step *= voice->lfo_pitch_ratio;
    if (step > voice->step_ceiling)
      step = voice->step_ceiling;
    step *= fxm;
    voice->position += step;
    if (voice->in_cycle) {
      /* Wrapped by the cycle, not by the loop's length: a full cycle is
         the reflected descending pass and the forward ascending one, so
         it is twice the loop. */
      double cycle =
        2.0 * (double)(voice->loop_last - voice->loop_first + 1u);
      while (voice->position >= cycle)
        voice->position -= cycle;
    } else if (voice->position >= (double)voice->loop_last + 1.0) {
      if (voice->ping_pong) {
        /* The head has read loop_last and turned. Cycle position 0 is the
           first reflected sample, at loop_last-1. */
        voice->in_cycle = true;
        voice->position -= (double)voice->loop_last + 1.0;
      } else if (voice->looping)
        voice->position -=
          (double)(voice->loop_last - voice->loop_first + 1u);
      else if (voice->position + 1.0 >= (double)voice->pcm_count) {
        voice->active = false;
        return false;
      }
    }
  }
  return true;
}

}  // namespace

void jv1080_voice_pair(struct XpJv1080Voice *first, struct XpJv1080Voice *second,
                       unsigned type, unsigned booster)
{
  if (!first || !second)
    return;
  second->partner = first;
  second->structure = type;
  second->booster = booster > 3u ? 3u : booster;
}

namespace {

/* THE STRUCTURE TYPES, as the owner's manual draws them (pp. 43-44) except
   for 3 and 4, which are as measured (`M-143`, below): W is a tone's wave
   generator, F its TVF, A its TVA, R the ring modulator, B the booster.

     2   A2 F2 F1 (A1 W1 + W2)
     3   A2 F2 B(F1 (A1 W1 + W2))
     4   A2 F2 F1 B(A1 W1 + W2)
     5   A2 F2 F1 R(A1 W1, W2)
     6   A2 F2 (F1 R(A1 W1, W2) + W2)
     7   A2 F2 R(A1 F1 W1, W2)
     8   A2 F2 (R(A1 F1 W1, W2) + W2)
     9   A2 R(A1 F1 W1, F2 W2)
     10  A2 (R(A1 F1 W1, F2 W2) + F2 W2)

   MEASURED (`M-038`, `P-xxxx`, `structure/structure_1_2_s0..s9`,
   `structure_3_4_s2`, `booster_1_2_b0..b3`: tone 1 `Synth Saw 2`, tone 2 a
   `Sine` a fifth up, both at level 110, filters off; tones 3 and 4 answer
   their own structure exactly as 1 and 2 do theirs). Type 2's saw falls
   2.6 dB against type 1's - (110/127)^2, the saw through both TVAs - with
   the sine unchanged.

   The ring types keep the sine at -62.7 dB and the saw's own lines under
   -80, and their sidebands reproduce the saw's harmonic balance (f2 + 2 f1
   against f2 + f1: -7.2 dB, the saw's own h2 against h1), so R is a
   product. Types 6, 8 and 10 restore the sine at -35.2; 5, 7 and 9 do not.
   The f2 +- f1 sidebands stand 26.5 dB (geometric mean) above the sum, in
   dB, of type 1's saw fundamental and sine. A product's scale moves with
   the level gauge, taken here as the 512-patch sweep's median: our renders
   4.6 dB over the takes. Against this take alone our saw runs 6.7 dB over
   and the sine 4.4, so the scale is good to about 2 dB. The two sidebands
   part the wrong way, f2 + f1 1.4 dB over f2 - f1 against 1.4 dB under on
   the machine; that is not resolved.

   THE BOOSTER (`M-143`, `P-xxxx`: a sine on tone 2 of a type 3 or 4 pair,
   tone 1 at level 1, drive set by wave gain and booster together, filters
   on and off; confirming `M-134`'s reading of the `booster_1_2` takes) is a
   gain of exactly 2^boost - 1, 2, 4, 8, read two independent ways at boosts
   1 and 2 to 0.01 dB - into a symmetric hard clip (h2 absent at -74 dB)
   shared by both tones: two sines through it intermodulate. Its ceiling
   stands 0.2 dB above the peak of a wave-gain-0 `Sine`, so wave gain alone
   reaches it at boost 0, and that one ceiling places h1 at all six drives
   within 0.03 dB. h3 and h5 run a steady ~0.6 dB under a hard clip's, not
   resolved. Tone 2's wave gain drives the clip; its level scales the
   output after it (the level's square law holds at two depths of clip);
   the rest of A2, envelope and velocity, is taken to follow it there, not
   measured. Type 3 filters tone 1's TVF before the clip
   and tone 2's after it; type 4 puts both after it, and with the filters
   off the two are one stage. Tone 2 passes tone 1's TVF in both.

   The ceiling below is that sine's peak here, 0.4849 (the probe read at
   wave gain +12: 1.9303 over 3.9811), and 0.2 dB over it. Rendered against
   the capture's own takes, every slot of all four files - both types,
   boosts 0-3, drives -6 to +30 dB, each filter on and off, pair 3&4 - lies
   a steady 1.0-1.7 dB under the machine on h1, the same as its type-1
   control slots, so the offset is the sine's own level, not the booster.

   WHICH FILTER EACH SIGNAL PASSES THROUGH - all that separates 5 from 7
   from 9, and 6 from 8 from 10 - rests on the diagrams alone: the takes
   run with the filters off and cannot see it. */
const double kRingScale = 1.87;
const double kBoostGain[4] = {1.0, 2.0, 4.0, 8.0};
const double kBoostCeiling = 0.496;

double boost(double x, unsigned setting)
{
  double y = kBoostGain[setting] * x;
  return y > kBoostCeiling ? kBoostCeiling
    : (y < -kBoostCeiling ? -kBoostCeiling : y);
}

bool render_pair(struct XpJv1080Voice *v2, float *l, float *r, size_t frames,
                 float *unpanned)
{
  struct XpJv1080Voice *v1 = v2->partner;
  const bool sweep1 = v1->filter_type && v1->cutoff_offset != 0.0;
  const bool lfoF1 = v1->filter_type &&
    (v1->lfo_cutoff_units[0] != 0.0 || v1->lfo_cutoff_units[1] != 0.0);
  const bool sweep2 = v2->filter_type && v2->cutoff_offset != 0.0;
  const bool lfoF2 = v2->filter_type &&
    (v2->lfo_cutoff_units[0] != 0.0 || v2->lfo_cutoff_units[1] != 0.0);
  for (size_t n = 0; n < frames; ++n) {
    double w1 = 0.0, g1 = 0.0;
    voice_controls(v1, sweep1, lfoF1);
    if (!v1->wave_done && !v1->envelope_done) {
      if (!voice_wave(v1, &w1)) {
        v1->wave_done = true;
        w1 = 0.0;
      } else if (!voice_envelope(v1)) {
        v1->envelope_done = true;
        w1 = 0.0;
      } else {
        g1 = v1->envelope * v1->tone_gain;
        if (v1->lfo_active)
          g1 *= v1->lfo_gain;
      }
      v1->active = true;
    }
    voice_controls(v2, sweep2, lfoF2);
    double w2 = 0.0;
    if (!v2->wave_done && !voice_wave(v2, &w2)) {
      v2->wave_done = true;
      v2->active = true;
      w2 = 0.0;
    }
    if (!voice_envelope(v2))
      break;
    double x;
    bool boosted = false;
    switch (v2->structure) {
    case 2: x = voice_tvf(v2, voice_tvf(v1, g1 * w1 + w2)); break;
    case 3:
    case 4: {
      /* Tone 2's wave gain goes in before the clip, the rest of its TVA
         after. */
      double in = g1 * w1 + v2->gain_wave * w2;
      x = v2->structure == 3u
        ? voice_tvf(v2, boost(voice_tvf(v1, in), v2->booster))
        : voice_tvf(v2, voice_tvf(v1, boost(in, v2->booster)));
      boosted = true;
      break;
    }
    case 5: x = voice_tvf(v2, voice_tvf(v1, kRingScale * g1 * w1 * w2)); break;
    case 6: x = voice_tvf(v2, voice_tvf(v1, kRingScale * g1 * w1 * w2) + w2); break;
    case 7: x = voice_tvf(v2, kRingScale * g1 * voice_tvf(v1, w1) * w2); break;
    case 8: x = voice_tvf(v2, kRingScale * g1 * voice_tvf(v1, w1) * w2 + w2); break;
    case 9: x = kRingScale * g1 * voice_tvf(v1, w1) * voice_tvf(v2, w2); break;
    default: {
      double f2 = voice_tvf(v2, w2);
      x = kRingScale * g1 * voice_tvf(v1, w1) * f2 + f2;
      break;
    }
    }
    double value;
    if (boosted) {
      value = x * v2->envelope * v2->static_gain_unwaved;
      if (v2->lfo_active)
        value *= v2->lfo_gain;
    } else {
      value = voice_tva(v2, x);
    }
    l[n] += (float)(value * v2->gain_left);
    r[n] += (float)(value * v2->gain_right);
    if (unpanned)
      unpanned[n] += (float)value;
    if (!v1->wave_done && !v1->envelope_done && !voice_advance(v1)) {
      v1->wave_done = true;
      v1->active = true;
    }
    if (!v2->wave_done && !voice_advance(v2)) {
      v2->wave_done = true;
      v2->active = true;
    }
  }
  return v2->active;
}

}  // namespace

bool jv1080_voice_render(struct XpJv1080Voice *voice, float *l, float *r,
                          size_t frames, float *unpanned)
{
  if (!voice || !voice->active || !voice->pcm || !l || !r)
    return false;
  if (voice->partner && voice->structure >= 2u)
    return render_pair(voice, l, r, frames, unpanned);

  const bool sweeping = voice->filter_type && voice->cutoff_offset != 0.0;
  const bool lfoFilter = voice->filter_type &&
    (voice->lfo_cutoff_units[0] != 0.0 || voice->lfo_cutoff_units[1] != 0.0);

  for (size_t n = 0; n < frames; ++n) {
    voice_controls(voice, sweeping, lfoFilter);
    double sample;
    if (!voice_wave(voice, &sample))
      break;
    if (!voice_envelope(voice))
      break;
    double value = voice_tvf(voice, voice_tva(voice, sample));
    l[n] += (float)(value * voice->gain_left);
    r[n] += (float)(value * voice->gain_right);
    if (unpanned)
      unpanned[n] += (float)value;
    if (!voice_advance(voice))
      break;
  }
  return voice->active;
}

}}  // namespace EmuSC::Xp
