/* SPDX-License-Identifier: CC0-1.0 */
/* See output.h for what this stage is and why it is not part of any
   ROM-derived law. */
#include "output.h"

#include "common/constants.h"
#include "../common/dsp_kernels.h"

#include <cmath>
#include <cstring>

namespace EmuSC { namespace Xp {

/* The response list is EMPTY, and that is the finding.

   A high shelf stood here: 6692.7 Hz, +8.986 dB, Q 0.910, the
   least-squares inverse of the whole-set median band deficit. It was
   standing in for high frequency the engine lost in its own TVF, whose
   trapezoidal integrators are a bilinear transform and left a double
   zero at Nyquist, so the two poles the ROM asks for rolled off like
   three. With the TVF realised in the topology the chip's limit table
   names (tvf.cc, XpDeviceProfile::limitTable) there is nothing left for the
   shelf to stand in for.

   Measured, 63 single notes against the archive recordings, median band
   error of ours minus the hardware, level-matched per tone:

     trapezoidal, no shelf   median MAD 2.94 dB   median tilt -1.56 dB/oct
     trapezoidal + shelf           1.81                  -0.36
     forward Euler + shelf         2.73                  +1.23
     forward Euler, no shelf       1.77                  +0.25

   The structure change on its own does what the fitted shelf did and a
   little more, and the two together over-correct. Per band, with no
   shelf, the median error is now within 0.7 dB of flat from 111 Hz to
   5.9 kHz and runs +1.3 dB at 7.2 kHz, +2.6 at 8.9 and +4.1 at 11.0.

   Nothing is fitted to that remainder, on purpose. Split by the tone's
   own cutoff it is not one response: tones cut off below 3.2 kHz are
   flat to 0.7 dB across the whole band, while tones cut off above
   3.2 kHz carry all of it. A fixed output section cannot be that shape,
   and fitting one to the median would darken the tones that are already
   right - which is exactly what the shelf it replaces did in the other
   direction, brightening Bagpipe from 0.44 to 1.59 dB MAD and Oboe from
   0.61 to 1.53. The remainder is the open question in TASK-193, not an
   output response.

   The stage stays, with the DC blocker in it, because it is the one
   labelled place for this class of thing. C has no empty array, so one
   zeroed entry stands in the list and the count is zero. */
extern "C" const struct xp_output_section XP_OUTPUT_RESPONSE[1] = {
  { XP_OUTPUT_PEAKING, 0.0f, 0.0f, 0.0f },
};
extern "C" const unsigned XP_OUTPUT_RESPONSE_SECTIONS = 0;

/* THE CONVERTER'S HOLD, and it is measured rather than assumed.

   The SC-88's parts list gives a PCM69AU-1/T2 and no oversampling filter
   in front of it, so the DAC should be a plain zero-order hold at the
   chip's 32 kHz - a sin(x)/x droop of -0.48 dB at 5.9 kHz rising to
   -2.72 at 13.5 and -3.92 at Nyquist. Our engine runs the whole voice
   path at the host rate and emits samples, not steps, so it carried none
   of it.

   The measurement that establishes the hold, and it does not use the
   engine at all. A digital filter clocked at 32 kHz has
   |H(32000 - f)| = |H(f)|, so everything digital in the machine attenuates
   a DAC image exactly as much as the baseband component that produced it
   and cancels in their ratio. The hold and the analog board do not. The
   archive recordings carry the images: correlating each recording's
   spectrum above 16.3 kHz against its own baseband mirrored about a
   candidate centre, swept from 29 to 35 kHz, peaks at 32.000 kHz and
   nowhere else - 0.92 on Bagpipe, 0.91 on Harpsichord, 0.84 on Seashore,
   0.81 on Accordion, 0.79 on Applause, and near zero or negative at
   every other centre tried.

   Their ratio, ten recordings, image at 32000-f over baseband at f, f
   from 11.0 to 15.8 kHz:

     f base kHz      11.0  12.0  13.0  14.0  15.0  15.8
     measured dB     -7.3  -5.7  -4.3  -2.9  -1.4  -0.1
     sin(x)/x dB     -5.6  -4.4  -3.3  -2.2  -1.1  -0.3
     residual        -1.7  -1.3  -1.0  -0.7  -0.3  +0.2

   So the hold is there, at 32 kHz, to within a dB and a half over a ten
   kilohertz span - and the residual is the whole analog path, device plus
   whatever recorded these, which is down only 1.7 dB from 11 kHz to
   21 kHz. That is the reason the response list above this one is still
   empty: an output filter steep enough to matter in the audio band would
   have crushed these images, and it did not.

   Whole board, 63 single notes against the archive recordings, wet at
   CC91 24: median MAD 1.5 -> 1.2 dB, 57 of 63 within 3 dB unchanged, 2
   past 6 dB unchanged, tilted bright 9 -> 7.

   [MEASURED]. It is the converter, not the chip, so it is here and not in
   the engine. Delete it the day libEmuSC's SC-88 path emits at 32 kHz and
   reconstructs properly, because then it is already in the signal. */
/* @provenance class=DOCUMENTED devices=SC-88 ref=SC-88-service-manual:p15
   THE ANALOG BOARD, read off the schematic and not fitted to anything.

   Roland's SC-88 Service Manual (Jun. 1994), page 15, CIRCUIT DIAGRAM
   (ANALOG, SWITCH, TRANS, PHONES HOLDER), "SC-88 ANALOG & POWER SUPPLY
   BOARD". The whole path from the converter to the OUTPUT jacks, in
   order, with the parts as the sheet names them:

     IC111  PCM69AU, current out per channel, LOUT/LCOM and ROUT/RCOM
     IC110  NJM4570, I/V, feedback R146 4k7 (M472) || C152 100p (101P)
            with the same network R150 || C155 on the LCOM leg
     C147   47u/16 into R144 100k (M104)
     IC109  NJM4570 inverting, R142 12k (M123) in, R140 22k (M223) ||
            C144 100p (101P) feedback
     R138   100R (M101) with C141 680p (681PR) to AGND
     C139   47u/16 into R136 100k (M104), out over CN101 to the VR board
     VR501  RK097121, the volume pot, and nothing else on that board
     IC108  NJM4570 inverting, R133 12k (M123) in, R127 12k (M123) ||
            C131 120p (121P) feedback
     IC102  M5218 inverting, R119 12k in, R113 12k feedback
     Q101   2SK363 source follower
     C122   47u/16 into R106 100k (M104)
     R101   1k8 (M182) with C113 1000p (102PR) to AGND
     L102   391CA three-terminal EMI filter, then JK101B

   The sheet is a 400 dpi bilevel scan and its 6 and 8 glyphs are one
   stroke apart, so the designators above are as good as the scan gets and
   page 11's block diagram numbers the same amplifiers differently again.
   The values are what this file uses and they are legible: the closest
   call is R127, read M123, and 12k against 22k moves its pole between
   110 and 60 kHz, which is 0.03 dB at 16 kHz either way.

   Five poles, every one of them decades above the audio band: 338.6 kHz,
   72.3 kHz, 2.34 MHz, 110.5 kHz, 88.4 kHz. Together they are -0.06 dB at
   10 kHz and -0.33 dB at 16 kHz relative to 8 kHz. THERE IS NO
   RECONSTRUCTION FILTER on this board - the SC-88 puts a 32 kHz
   zero-order hold through an amplifier chain that is flat past 70 kHz and
   out of the jack. That is also why the DAC images survive into the
   recordings at all, and it is what makes the hold above measurable.

   The SC-88 Pro's board (Roland SC-88 Pro Service Manual, Nov. 1996,
   page 14) does carry one - 6k8/2700p, 6k8/2200p, 6k8 into 13k6 || 390p
   around IC104, third order, -5.1 dB at 16 kHz re 8 kHz. That is a
   different machine two years later. It is named here because the
   SC-88's own renders run about 5 dB bright at 16 kHz against the seven
   demo-song recordings, which is what the Pro's filter is worth there to
   within half a decibel, and the one thing that must not happen is for
   it to be borrowed on that resemblance. Most of that 5 dB is the
   recordings: the SCVA oracle, put through the identical path, is itself
   +4.4 dB at 16 kHz and +21 dB at 20 kHz against them, and the images in
   those same recordings sit 18 to 27 dB below an unfiltered hold from
   17 to 20 kHz while the single-note archive's sit within 1 dB of it.

   DOCUMENTED, a tier above MEASURED: component values read off
   Roland's published service manual, not recovered from audio.
   The poles are folded into the hold FIR's design target below rather
   than run as biquads: all five sit above the engine's own Nyquist,
   where a bilinear transform has no pole to place, and the FIR is
   designed by frequency sampling, which does not care. */
extern "C" const struct xp_output_rc
  XP_OUTPUT_ANALOG[XP_OUTPUT_ANALOG_SECTIONS] = {
  { 4.7e3, 100e-12 },   /* IC110 R146 || C152 */
  { 22.0e3, 100e-12 },  /* IC109 R140 || C144 */
  { 100.0, 680e-12 },   /* R138 + C141        */
  { 12.0e3, 120e-12 },  /* IC108 R127 || C131 */
  { 1.8e3, 1000e-12 },  /* R101 + C113        */
};

namespace {

/* |H(f)| of the five one-pole sections above, at DC gain one. */
double analogMag(double f)
{
  const double pi = 3.14159265358979323846;
  double mag = 1.0;
  for (unsigned i = 0; i < XP_OUTPUT_ANALOG_SECTIONS; ++i) {
    double fc = 1.0 / (2.0 * pi * XP_OUTPUT_ANALOG[i].r_ohm *
                       XP_OUTPUT_ANALOG[i].c_farad);
    double r = f / fc;
    mag /= std::sqrt(1.0 + r * r);
  }
  return mag;
}

double besselI0(double x)
{
  return bessel_i0(x);
}

/* The zero-phase filter whose magnitude is |sin(pi f / 32000) /
   (pi f / 32000)| times the analog board's own response over the whole
   output band, by frequency sampling, then a Kaiser window so a 31-tap
   truncation does not ripple. */
void designHold(struct xp_output *out, double rate)
{
  const double pi = 3.14159265358979323846;
  const int half = XP_OUTPUT_HOLD_TAPS / 2;
  const int steps = 4096;
  const double beta = 7.0;
  const double i0beta = besselI0(beta);
  double h[XP_OUTPUT_HOLD_TAPS];
  double sum = 0.0;

  for (int k = -half; k <= half; ++k) {
    double acc = 0.0;
    for (int j = 0; j <= steps; ++j) {
      double f = 0.5 * rate * (double)j / (double)steps;
      double x = pi * f / kXpNativeRate;
      double mag = (x > 1e-12) ? std::fabs(std::sin(x) / x) : 1.0;
      double w = (j == 0 || j == steps) ? 0.5 : 1.0;
      mag *= analogMag(f);
      acc += w * mag * std::cos(2.0 * pi * f * (double)k / rate);
    }
    acc *= (0.5 * rate / (double)steps) * 2.0 / rate;
    double wn = (double)k / (double)half;
    double win = besselI0(beta * std::sqrt(std::fmax(0.0, 1.0 - wn * wn))) /
      i0beta;
    h[k + half] = acc * win;
    sum += h[k + half];
  }
  for (int k = 0; k < XP_OUTPUT_HOLD_TAPS; ++k)
    out->hold[k] = (float)(h[k] / sum);
  out->hold_taps = XP_OUTPUT_HOLD_TAPS;
  out->hold_pos = 0;
}

/* Audio EQ Cookbook forms, normalised by a0. */
void design(struct xp_output_biquad *bq, const struct xp_output_section *s,
            double rate)
{
  double a = std::pow(10.0, s->gain_db / 40.0);
  double w0 = 2.0 * 3.14159265358979323846 * s->frequency / rate;

  std::memset(bq, 0, sizeof *bq);
  if (!(w0 > 0.0) || w0 >= 3.14159265358979323846) {
    bq->b0 = 1.0f;
    return;
  }
  double cw = std::cos(w0);
  double sw = std::sin(w0);
  double alpha = sw / (2.0 * (s->q > 0.0f ? s->q : 0.7071));

  double b0, b1, b2, a0, a1, a2;
  switch (s->type) {
  case XP_OUTPUT_HIGH_SHELF: {
    double sq = 2.0 * std::sqrt(a) * alpha;
    b0 = a * ((a + 1.0) + (a - 1.0) * cw + sq);
    b1 = -2.0 * a * ((a - 1.0) + (a + 1.0) * cw);
    b2 = a * ((a + 1.0) + (a - 1.0) * cw - sq);
    a0 = (a + 1.0) - (a - 1.0) * cw + sq;
    a1 = 2.0 * ((a - 1.0) - (a + 1.0) * cw);
    a2 = (a + 1.0) - (a - 1.0) * cw - sq;
    break;
  }
  case XP_OUTPUT_LOW_SHELF: {
    double sq = 2.0 * std::sqrt(a) * alpha;
    b0 = a * ((a + 1.0) - (a - 1.0) * cw + sq);
    b1 = 2.0 * a * ((a - 1.0) - (a + 1.0) * cw);
    b2 = a * ((a + 1.0) - (a - 1.0) * cw - sq);
    a0 = (a + 1.0) + (a - 1.0) * cw + sq;
    a1 = -2.0 * ((a - 1.0) + (a + 1.0) * cw);
    a2 = (a + 1.0) + (a - 1.0) * cw - sq;
    break;
  }
  default:
    b0 = 1.0 + alpha * a;
    b1 = -2.0 * cw;
    b2 = 1.0 - alpha * a;
    a0 = 1.0 + alpha / a;
    a1 = -2.0 * cw;
    a2 = 1.0 - alpha / a;
    break;
  }
  bq->b0 = (float)(b0 / a0);
  bq->b1 = (float)(b1 / a0);
  bq->b2 = (float)(b2 / a0);
  bq->a1 = (float)(a1 / a0);
  bq->a2 = (float)(a2 / a0);
}

}  // namespace

void output_init(struct xp_output *out, double rate)
{
  if (!out)
    return;
  std::memset(out, 0, sizeof *out);
  if (!(rate > 0.0))
    rate = 32000.0;
  out->sections = XP_OUTPUT_RESPONSE_SECTIONS;
  if (out->sections > XP_OUTPUT_MAX_SECTIONS)
    out->sections = XP_OUTPUT_MAX_SECTIONS;
  for (unsigned i = 0; i < out->sections; ++i)
    design(&out->section[i], &XP_OUTPUT_RESPONSE[i], rate);
  designHold(out, rate);
  /* A 10 Hz single-pole blocker: 0.03 dB at 111 Hz and unity everywhere
     the audit measures, so it is not what removes the high frequency
     above - but it is not cited to any ROM either, and libEmuSC's SC-55
     path has no output blocker at all, so it is declared here with the
     rest of the unverified output behaviour rather than inline. */
  out->dc_pole = (float)(1.0 - 2.0 * 3.14159265358979323846 * 10.0 / rate);
  out->enabled = true;
}

void output_reset(struct xp_output *out)
{
  if (!out)
    return;
  for (unsigned i = 0; i < XP_OUTPUT_MAX_SECTIONS; ++i) {
    std::memset(out->section[i].x1, 0, sizeof out->section[i].x1);
    std::memset(out->section[i].x2, 0, sizeof out->section[i].x2);
    std::memset(out->section[i].y1, 0, sizeof out->section[i].y1);
    std::memset(out->section[i].y2, 0, sizeof out->section[i].y2);
  }
  out->dc_x[0] = out->dc_x[1] = 0.0f;
  out->dc_y[0] = out->dc_y[1] = 0.0f;
  std::memset(out->hold_z, 0, sizeof out->hold_z);
  out->hold_pos = 0;
}

void output_process(struct xp_output *out, float *stereo, size_t frames)
{
  if (!out || !out->enabled || !stereo)
    return;
  for (size_t k = 0; k < frames; ++k)
    for (unsigned ch = 0; ch < 2; ++ch) {
      float x = stereo[k * 2 + ch];
      float y;
      for (unsigned i = 0; i < out->sections; ++i) {
        struct xp_output_biquad *b = &out->section[i];
        y = b->b0 * x + b->b1 * b->x1[ch] + b->b2 * b->x2[ch] -
          b->a1 * b->y1[ch] - b->a2 * b->y2[ch];
        b->x2[ch] = b->x1[ch];
        b->x1[ch] = x;
        b->y2[ch] = b->y1[ch];
        b->y1[ch] = y;
        x = y;
      }
      y = x - out->dc_x[ch] + out->dc_pole * out->dc_y[ch];
      out->dc_x[ch] = x;
      out->dc_y[ch] = y;
      stereo[k * 2 + ch] = y;
    }
  /* Tap t reads the sample t frames back, so the ring index walks down from
     the newest one and wraps exactly once per pass. A modulo expresses that
     walk in one expression, but costs a hardware divide per tap per channel:
     63 per frame at 31 taps, 2.8 million a second at 44.1 kHz, on the path
     every rendered frame passes through. The decrement below is the same
     sequence without them.

     Tap order, channel order and accumulation order are what this filter's
     output is; none of the three may be reordered for speed. */
  const unsigned taps = out->hold_taps;
  const unsigned last = taps - 1;
  /* Read through locals: `stereo` is a float pointer too, so the compiler
     would otherwise reload the taps and the ring after every store to it. */
  const float *const hold = out->hold;
  for (size_t k = 0; k < frames; ++k) {
    unsigned pos = out->hold_pos;
    for (unsigned ch = 0; ch < 2; ++ch) {
      float *const z = out->hold_z[ch];
      float acc = 0.0f;
      unsigned idx = pos;
      z[pos] = stereo[k * 2 + ch];
      for (unsigned t = 0; t < taps; ++t) {
        acc += hold[t] * z[idx];
        idx = idx ? idx - 1 : last;
      }
      stereo[k * 2 + ch] = acc;
    }
    out->hold_pos = (pos == last) ? 0 : pos + 1;
  }
}

}}  // namespace EmuSC::Xp
