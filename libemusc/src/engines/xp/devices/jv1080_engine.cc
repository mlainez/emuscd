/* SPDX-License-Identifier: CC0-1.0 */
/*
 *  Roland JV-1080 voice engine for the XP engine (engines/xp/).
 *
 *  What the shared device layer calls through XpDeviceProfile::voiceEngine:
 *  sixteen parts, a voice pool over this device's own behavioural voice
 *  model (jv1080_voice.cc), and the parameter writes that fill a part in.
 *
 *  THIS IS NOT A FIRMWARE PORT. The sibling engine in engine.h/renderer.h
 *  is one - the SC-88's firmware is dumped, so its voice state is that
 *  chip's own register and RAM layout and its scheduler is that firmware's
 *  control period. This device's synthesis engine is in the SH7034's
 *  undumped 64 KB internal mask ROM. What is exact here is everything read
 *  out of the external ROM: the parameter address map, the bit-packed
 *  records, the bank selection and the wave chain. What the voices do with
 *  those values is measured behaviour, and jv1080_voice.cc carries each
 *  law's own measurement id and says where the measurement runs out.
 *
 *  Allocation steals the oldest voice among the parts holding more voices
 *  than their Voice Reserve, a part within its reserve being exempt
 *  (`M-031`, `M-072`; take_voice). The reserves are the performance
 *  common's own bytes, so they bind only once a song writes them; until
 *  then every reserve is 0 and the steal is plain oldest-first.
 */
#include "jv1080.h"
#include "jv1080_resample.h"
#include "../wave_cache.h"
#include "../reverb.h"
#include "../chorus.h"
#include "../drive.h"
#include "../stereo_eq.h"
#include "../spectrum.h"
#include "../enhancer.h"
#include "../phaser.h"
#include "../rotary.h"
#include "../dynamics.h"
#include "../efx.h"
#include "../common/constants.h"
#include "../common/worker_pool.h"

#include "../packed_rom.h"
#include "../rom.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <new>

namespace EmuSC { namespace Xp {

namespace {

/* The device's own part count, and the pool the shared engine ceiling
   allows. Sixteen parts is this machine's, measured alongside its
   polyphony (`M-031`, `M-072`); the pool is capped by the engine's own
   slot ceiling so the two cannot disagree. */
/* The performance common block, and the fields of it this engine reads:
   the reverb block at 0x28..0x2C - type, level, time, HF damp, and a
   feedback this engine does not plumb. */
const unsigned kPerfCommonFields = 66u;
const unsigned kReverbTypeField = 0x28u;
const unsigned kReverbLevelField = 0x29u;
const unsigned kReverbTimeField = 0x2au;
const unsigned kReverbDampField = 0x2bu;
const unsigned kReverbFeedbackField = 0x2cu;
/* The sixteen Voice Reserve bytes, one per part, 0-64 each
   (`04_protocol/sysex.md` `01 00 00 30-3F`). */
const unsigned kVoiceReserveField = 0x30u;
/* Default tempo, `01 00 00 2D-2E`: one byte assembled from two nibbles,
   20-250 BPM; a value outside that range is discarded and the tempo in
   force stands (`M-098`). */
const unsigned kDefaultTempoField = 0x2du;
/* REVERB TYPES 6 AND 7 ARE NOT TANKS. For them Reverb:Time is the DELAY
   LENGTH rather than the decay, and it is patched straight into nine PRAM
   ERAM address fields as `112*v + 0x2016` - the eight output taps plus
   instruction 176 (`08_effects/reverb.md`, `FW-EXACT`). PAN-DLY splits the
   nine into a five-tap left list and a four-tap right list and patches each
   with its own value. The character record's own network is nearly empty on
   these two, which is why reading it as a tank finds nothing to play.

   The two values are MEASURED off `effects/reverb_type_pandly`: at Time 96
   the repeat lags 10769 samples on the left and 5393 on the right, against
   112*96 and 56*96, both long by exactly +17 - the same constant on both
   sides, the block's own input-path latency. `0x2016` is the line's base,
   so the delay is the patched address less that base.

   The tap gains are the program's own: the five left taps carry +0.7505,
   +1, +1, +1 and +0.0006 and the four right ones +1 each, so the right
   return sits 0.56 dB above the left.

   FEEDBACK IS THE LOOP GAIN ON THESE TWO TYPES, and it writes the same two
   CRAM slots - 165 and 181 - that Reverb:Time writes on types 0..5. One
   coefficient under two names: `48*v` as a time, `64*v` as a feedback, both
   against 8192 (`08_effects/reverb.md`, FW-EXACT). On types 0..5 the
   feedback parameter is NOT APPLIED AT ALL, which is why `M-045` could only
   measure 3 dB across its whole range.

   THE TWO TYPES RETURN THEIR LINE DIFFERENTLY, which is measured rather than
   reasoned. Type 7 at feedback 0 still repeats at full level - +3.58 dB
   against the click - while type 6 at feedback 0 is silent for six seconds.
   On type 6 the return is scaled by the loop gain: at feedback 64 the gain
   is 64*64/8192 = 0.5 and the first repeat measures -6 dB, at 127 it is
   0.992 and the repeats sustain past two and a half seconds without
   decaying. So type 6's return carries the loop gain and type 7's does not.
   Why the two differ that way is not established; that they do is. */
const unsigned kReverbTypeDelay = 6u;
const unsigned kReverbTypePanningDelay = 7u;
const double kReverbDelaySlopeLeft = 112.0;
const double kReverbDelaySlopeRight = 56.0;
const double kReverbDelayLatency = 17.0;
const double kReverbDelayTapSumLeft = 3.7511;
const double kReverbDelayTapSumRight = 4.0;
const double kReverbDelayFeedbackSlope = 64.0;
const unsigned kChorusLevelField = 0x22u;
const unsigned kChorusRateField = 0x23u;
const unsigned kChorusDepthField = 0x24u;
const unsigned kChorusPreDelayField = 0x25u;
/* The insert effect's output block. The assign decides whether the effect's
   own chorus and reverb sends reach anything at all: they are written as
   `table[v] & mask`, and the mask is all ones only for MIX
   (`08_effects/routing.md`, FW-EXACT). */
/* The performance common's own effect block: the source selector, the type
   and its twelve parameters, then the four output bytes. A PATCH's copy of
   the same block sits one byte lower throughout, because a patch has no
   source selector to carry (`05_data_model/effect_schema.md`). */
/* Where a voice's audio leaves the chip. The part's own assign decides for
   the whole part unless it reads PATCH, which hands the decision to the
   record the voice came from - a tone, or a rhythm note (`M-006`,
   `M-019`). A record has no PATCH value of its own; only a part does. */
const unsigned kOutputMix = 0u;
const unsigned kOutputEfx = 1u;
const unsigned kOutputOne = 2u;
const unsigned kOutputTwo = 3u;

const unsigned kEfxSourceField = 0x0cu;
const unsigned kEfxTypeField = 0x0du;
const unsigned kEfxFirstParameterField = 0x0eu;
const unsigned kEfxParameters = 12u;
const unsigned kEfxPatchBlockShift = 1u;

/* THE PURE-DELAY FAMILY, so far type 17 (0-based 16), STEREO-DELAY.

   THE STORED BYTE ORDER IS MEASURED AND IS NOT THE DISPLAY ORDER
   (`M-091`, `M-092`, per-parameter maxima read off the machine):

     p1 Mode(1)  p2 DelayL(126)  p3 DelayR(126)  p4 PhaseL(1)  p5 PhaseR(1)
     p6 Fbk(98)  p7 HFDamp(17)   p8 LowGain(30)  p9 HiGain(30)
     p10 Balance(100)  p11 Level(127)

   Delay L and R are independently named by an isolated audio probe rather
   than by the label page, which this family proves unreliable - EFX 19's
   labels start at parameter 1 where EFX 17's start at 2 (`M-065`,
   `M-104`: "STEREO-DELAY's parameter 3 is the right delay, so 2 is the
   left").

   Each maximum pins its table: DelayL/R max 126 against `0x038FC8`'s 127
   entries, HF Damp max 17 against `0x039700`'s 18 rows, Balance max 100
   against the 101-step D100:0W..D0:100W display, Level max 127 against
   `0x03856C`. The delay table is raw SAMPLE COUNTS at 32 kHz, measured
   1.000 at all eight values over 0.4 to 60 ms plus 140 ms (`M-067`,
   `M-065`).

   FEEDBACK IS BIPOLAR AND ITS RAW ZERO IS 49, NOT 0 (`M-101`): 99 raw
   steps of 2 % span -98 to +98 %, so the gain is (raw - 49) / 50 and
   writing 0 asks for maximum NEGATIVE feedback.

   THE FEEDBACK CROSSES THE TWO LINES. Its cepstrum on noise carries an
   even-only series - 2T, 4T, 6T, 8T - with the single echo at T present
   but weak, which is a loop crossing the two delay lines rather than each
   feeding itself, and both output channels read the same so it is not a
   ping-pong between them (`M-067`, confirmed independently by `M-091`'s
   Mode 0 = CROSS / 1 = NORMAL range fingerprint).

   LOW GAIN AND HI GAIN are the shared output shelves, read from the ROM's
   own shelf tables - see THE OUTPUT SHELVES below. */
const unsigned kEfxTypeStereoDelay = 16u;
/* THE NONLINEAR DRIVE FAMILY, types 2 OVERDRIVE and 3 DISTORTION (0-based
   1 and 2): one program, and the only difference between the two is the
   value row the firmware's stubs pass to `0x0A0020EC` - 0 and 1 - which is
   the whole of what `efx_drive_type` below hands on. Everything else is
   engines/xp/drive.h. */
/* STEREO-EQ, type 1 (0-based 0): engines/xp/stereo_eq.h. */
const unsigned kEfxTypeStereoEq = 0u;
/* SPECTRUM, type 5, and ENHANCER, type 6 (0-based 4 and 5):
   engines/xp/spectrum.h and engines/xp/enhancer.h. */
const unsigned kEfxTypeSpectrum = 4u;
const unsigned kEfxTypeEnhancer = 5u;
/* PHASER, type 4 (0-based 3): engines/xp/phaser.h. */
const unsigned kEfxTypePhaser = 3u;

/* ROTARY, type 8 (0-based 7): engines/xp/rotary.h. */
const unsigned kEfxTypeRotary = 7u;
/* COMPRESSOR and LIMITER, types 9 and 10 (0-based 8 and 9):
   engines/xp/dynamics.h. */
const unsigned kEfxTypeCompressor = 8u;
const unsigned kEfxTypeLimiter = 9u;
const unsigned kEfxTypeOverdrive = 1u;
const unsigned kEfxTypeDistortion = 2u;
const unsigned kEfxDriveParameters = 6u;
bool efx_drive_type(unsigned type, unsigned *row)
{
  if (type != kEfxTypeOverdrive && type != kEfxTypeDistortion)
    return false;
  if (row)
    *row = type - kEfxTypeOverdrive;
  return true;
}
/* TRIPLE-TAP-DELAY (display 19). One delay line read at three taps, which is
   what its program carries: slot 8 has three ERAM reads and one write.

   ITS STORED BYTE ORDER IS THE LABEL PAGE'S, which is measured rather than
   assumed - an isolated audio probe puts its delay times at p1, p2 and p3
   (`M-065`), and correlating the tick template at the expected 850 ms
   against the 400 ms baseline separates those three from the other nine by
   an order of magnitude. Type 17 is the family's exception, having eleven
   labels for twelve slots and so starting at p2; 19, 20 and 21 do not.

     p1 Delay C  p2 Delay L  p3 Delay R  p4 Fbk
     p5 Level C  p6 Level L  p7 Level R  p8 HF Damp
     p9 Low Gain p10 Hi Gain p11 Balance p12 Level

   The delays read `0x0390C6`, whose entries are raw sample counts at
   32 kHz and which measures 1.0000 at all eight values over 200 to 1000 ms
   (`M-067`). Low Gain and Hi Gain are the output shelves (below). */
const unsigned kEfxTypeTripleTap = 18u;
/* TIME-CONTROL-DELAY (display 21). One delay line, one tap - slot 28 carries
   a single ERAM write and no static read, its tap being patched at runtime.

   p1 IS THE DELAY AND p2 IS THE ACCEL, measured from the corpus's own
   addressing rather than from the page: the four `closeout/efx21_accel_*`
   stimuli sweep the delay by writing efx_p_1, and the only byte differing
   between the `a000` and `a127` files of a pair is efx_p_2. `M-068` derived
   its constant by watching the delay glide in those takes, which is only
   possible if the swept byte is the delay.

   p6 LOW GAIN AND p7 HI GAIN ARE READ OFF THE UPDATER `0x0A0051F0`, which
   hands bytes 5 and 6 to the shelf writers (FW-EXACT, THE OUTPUT SHELVES
   below). THE OTHER POSITIONS ARE THE LABEL PAGE'S AND ARE NOT
   INDIVIDUALLY MEASURED - p3 Feedback, p4 Pan, p5 HF Damp, p8 Balance,
   p9 Level. The page is trusted here because its first two entries and
   the two gains are exactly where measurement and the updater put them,
   but it carries nine labels for twelve slots, which is the same
   spare-slot condition that makes type 17's page start at p2. Said
   plainly rather than presented as measured.

   THE DELAY IS THE SETTLED VALUE ONLY. Accel makes the delay time a TARGET
   rather than a setting: the line glides to it over seconds, reading 680
   then 476 then 389 then 386 ms across successive windows of one take
   (`M-068`). WHAT IS NOT MODELLED HERE IS THAT GLIDE - the rate for a given
   Accel value is not recovered, so this jumps to the settled value where
   the machine slides to it. A transient, and a real difference for the
   seconds it lasts.

   THE 0.966 IS MEASURED AND UNEXPLAINED. Where every other delay table on
   this device reads as raw samples at 32 kHz, `0x0391AE` settles at a
   CONSTANT 0.966 of its entry across a 4.75x span, approached from above
   and from below to within 0.1 ms (`M-068`). It is applied here as the
   measured constant it is and NOT folded into a general scale, which is
   TASK-342.02's own AC#4. */
const unsigned kEfxTypeTimeControl = 20u;
/* The second channel's LFO offset, in cycles, against p6.

   MEASURED, seven points, and it is a table rather than a formula. Read at
   full wet by cross-correlating the two channels' frequency deviations,
   which at full wet ARE the two delay trajectories; every point peaks at a
   correlation of +1.000.

     p6      0     15     30     45     60     75     90
     cycles  0  .0808  .1610  .2418  .3221  .4028  .4999
     degrees 0  29.08  57.97  87.05 115.94 145.02 179.95

   p6 = 0 puts the two channels exactly together, which is what says the
   parameter is the phase at all. The six points to 75 lie on one line at
   **1.9335 degrees a step**, not the 2.0 the panel implies by accepting
   0..90 and displaying 0..180 - and the top of the field then breaks that
   line, landing on 180.0 where the line would put it at 174.0.

   That break is not the instrument: a render built with a flat 1.9346
   degrees a step reads back 174.10 at p6 = 90 through the same code, so
   six degrees is well within reach.

   WHERE THE LINE TURNS, BETWEEN 75 AND 90, IS NOT RESOLVED - the sweep
   steps by 15. So the values between the measured points are drawn as
   straight lines between them and nothing is extrapolated. */
const unsigned kEfxChorusPhasePoints = 7u;
const unsigned kEfxChorusPhaseSpacing = 15u;
const double kEfxChorusPhaseCycles[kEfxChorusPhasePoints] =
  { 0.0, 0.0808, 0.1610, 0.2418, 0.3221, 0.4028, 0.4999 };

double efx_chorus_phase(unsigned v)
{
  unsigned top = kEfxChorusPhaseSpacing * (kEfxChorusPhasePoints - 1u);
  if (v >= top)
    return kEfxChorusPhaseCycles[kEfxChorusPhasePoints - 1u];
  unsigned i = v / kEfxChorusPhaseSpacing;
  double f = (double)(v - i * kEfxChorusPhaseSpacing) /
    (double)kEfxChorusPhaseSpacing;
  return kEfxChorusPhaseCycles[i] +
    f * (kEfxChorusPhaseCycles[i + 1u] - kEfxChorusPhaseCycles[i]);
}
const unsigned kEfxAccelDelayTable = 12u;   /* 0x0391AE */
const double kEfxTimeControlScale = 0.966;
const unsigned kEfxLongDelayTable = 11u;   /* 0x0390C6 */
const double kEfxFeedbackZero = 49.0;
const double kEfxFeedbackStep = 50.0;
const unsigned kEfxDelayTable = 10u;      /* XP_EFX_TABLE_DELAY */
const unsigned kEfxBalanceTable = 14u;
const unsigned kEfxDampTable = 15u;
const unsigned kEfxLevelTableIndex = 0u;
/* --- THE INSERT REVERBS, types 24 and 25 ---------------------------

   Not the system reverb: their own entry points, and `M-111` measured
   their Time law separately rather than assuming it.

   THEY DO NOT FIT THE SHARED REVERB STRUCT, which is why they have their
   own. Read through the type table at `0x044EBC`, REVERB's program makes
   46 ERAM accesses - 24 writes and 22 reads - and GATE-REVERB's 46, 16 and
   30, against `XP_REVERB_BUFFERS` of twelve and `XP_REVERB_TAPS` of eight.

   THE LENGTHS ARE THE MACHINE'S OWN. An ERAM address is sixteen bits, so
   the space is 0..0xFFFF and it WRAPS: twenty-four of GATE-REVERB's thirty
   reads sit below every one of its writes, which is a wrapped line and not
   a decode error. A tap is `(read - write) mod 2^16` from the nearest
   write in that circular space, giving REVERB twenty-two lengths from 0.03
   to 71.72 ms and GATE-REVERB thirty from 0.03 to 681.47 ms.

   THAT DIRECTION IS A CHOICE. `(write - read)` is equally decodable and
   gives REVERB only twelve distinct lengths and GATE-REVERB a ceiling of
   341.75 ms. Three things pick this one: every read yields its own length,
   22 of 22 and 30 of 30; GATE-REVERB's late cluster then lands at 428 to
   681 ms, where its own gate runs to 500; and REVERB's spread becomes a
   reverb's rather than one long line.

   WHICH LINE FEEDS WHICH IS NOT RECOVERED. The addresses are measured; the
   routing is inside the DSP program, which this engine does not interpret.
   So the arrangement below IS MINE. What it reproduces is the modal
   structure those lengths give and the decay `M-111` measured; it is not a
   claim about the topology.

   THE ARRANGEMENT, AND WHY IT IS THIS ONE. A first attempt put every tap
   on ONE loop and solved a single gain for the target decay. That cannot
   work and the control said so - 0.34 times the law at Time 0 and 1.17 at
   64 - because `g = 10^(-3L/(RT60*fs))` is exact for a comb of ONE length
   and a loop carrying twenty-two lengths has no single decay. Each tap is
   therefore its own COMB with its own gain from that same relation, so
   each decays at the target by construction and so does their sum.
   GATE-REVERB needs none of it: its tail is cut by the gate rather than
   decayed, so it keeps one shared line read at many points. */
const unsigned kEfxTypeReverb = 23u;
const unsigned kEfxTypeGateReverb = 24u;
const unsigned kXpEramSpace = 65536u;
const unsigned kInsertReverbMaxTaps = 48u;

struct InsertReverbSpec {
  unsigned type;
  unsigned preDelay, time, damp, balance, level;
  int gate;                     /* -1 where the type has no gate */
  double rt60Base, rt60Steps;   /* `M-111`: RT60 = base * 2^(v/steps) */
};
const struct InsertReverbSpec kInsertReverbSpecs[] = {
  /* 24 REVERB: p1 Type, p2 Pre-delay, p3 Time, p4 HF damp, p5/p6 gains,
     p7 Balance, p8 Level - all from the ceilings `M-092` measured. */
  { 23u, 1u, 2u, 3u, 6u, 7u, -1, 0.583, 39.8 },
  /* 25 GATE-REVERB IS NOT HERE, AND ITS CODE PATH BELOW IS KEPT ONLY
     BECAUSE IT DOCUMENTS THE ATTEMPT. It was written alongside REVERB and
     failed its own control while REVERB passed: against the machine's
     122.0, 178.2, 271.5, 376.5, 479.0 and 569.0 ms of gate at values 10 to
     99, it gave 33.0, 88.8, 88.8, 88.8, 88.8 and 511.5 - saturating at its
     own longest tap of 84.4 ms for four settings in a row, which is the
     gate never closing rather than closing late. The gate's LAW is
     measured, 5.022 ms a step against the manual's 5.000, so what is
     missing is the trigger and not the number. Claiming the type on that
     would be claiming a gate that does not gate. */
};

const struct InsertReverbSpec *insert_reverb_spec(unsigned type)
{
  for (unsigned i = 0;
       i < sizeof kInsertReverbSpecs / sizeof *kInsertReverbSpecs; ++i)
    if (kInsertReverbSpecs[i].type == type)
      return kInsertReverbSpecs + i;
  return NULL;
}

const unsigned kEfxLfoRateTable = 5u;      /* XP_EFX_TABLE_LFO_RATE */
const unsigned kEfxPreDelayTable = 9u;     /* XP_EFX_TABLE_PRE_DELAY */

/* The modulated-delay types this engine renders, and where each one keeps
   its parameters. Every slot here is MEASURED on that type's own take, not
   read off a label page this repository has caught wrong on this family.

   `sweepMs` is the peak-to-peak delay sweep at depth 127, measured wet-only
   with the feedback at its own zero. The two agree to 0.5 %, which is the
   spread of the reading rather than a difference between the types.

   The other four of the family are absent, each for a reason its own take
   gave: HEXA-CHORUS and SPACE-D put more than one tap in each channel, so
   the deviation reading that settles a modulator does not apply to them;
   TREMOLO-CHORUS modulates amplitude as well, which corrupts a frequency
   reading; and STEP-FLANGER has two rates and a staircase modulator.
   MODULATION-DELAY is absent for a different reason - everything about its
   LFO is measured and matches these two exactly, but its delay times are
   not, and a delay whose time is guessed is not the effect. */
/* The shape the delay is swept with, named by the DEVIATION it produces,
   because that is what is measured. A triangle delay makes the carrier's
   frequency deviation a SQUARE; a parabolic one makes it a SAWTOOTH. */
#define kEfxModTriangle 0u
#define kEfxModParabola 1u

struct EfxModSpec {
  unsigned type;
  unsigned shape;
  unsigned nominal[2];          /* the still delay each channel sweeps from */
  unsigned nominalTable;        /* which conversion table those slots read */
  unsigned rate, depth, balance, level;
  int phase;                    /* -1 where the channels share one LFO */
  bool invertRight;             /* the right channel is the left, negated */
  int feedback;                 /* -1 where none is identified */
  int damp;                     /* -1 where the type has none */
  double sweepMs;
  /* A second modulator on the AMPLITUDE, where the type carries one.
     -1 on both where it does not. */
  int tremRate, tremDepth;
  /* Several voices on one line, evenly spread and unevenly weighted.
     `spread` names the parameter that sets the gap and `spreadMs` what a
     step of it is worth. `voices` is 1 and `spread` -1 for a type with
     one, and `weight` is then unused. */
  unsigned voices;
  int spread;
  double spreadMs;
  double weight[4];
};
const struct EfxModSpec kEfxModSpecs[] = {
  /* 14 STEREO-CHORUS: one pre-delay for both channels; p7 has a ceiling of
     127 and a factory 0 and is not identified, so no feedback is applied. */
  { 13u, kEfxModTriangle, { 2u, 2u }, kEfxPreDelayTable,
    3u, 4u, 9u, 10u, 5, false, -1, -1, 12.38, -1, -1, 1u, -1, 0.0, { 1.0, 0.0, 0.0, 0.0 } },
  /* 15 STEREO-FLANGER: the same layout with p7 a bipolar feedback - and a
     DIFFERENT modulator, which took three attempts to pin down. Its
     deviation's harmonic ratios at the rates where they resolve read
     h2 0.462, h3 0.344, h4 0.261, h5 0.191 against an ideal sawtooth's
     0.500, 0.333, 0.250, 0.200; the chorus at the same settings reads
     0.006, 0.343, 0.002, 0.191 against an ideal square's 0, 0.333, 0,
     0.200. Every harmonic present, not just the odd ones, is what
     separates the two, and peak/rms cannot - a triangle and a sawtooth
     both give root three.

     Two wrong explanations were tried and measured away first: the filter
     pair, which does nothing to the deviation on either type (driven off,
     type 15 still reads 1.709), and clipping against the delay floor,
     which would have shown as a dependence on the pre-delay and does not
     - 10, 32, 64 and 96 give the same harmonics to three decimals. That
     last one also settles the direction: a sweep this wide that never
     clips at a 1.00 ms nominal is one-sided UPWARD, as the chorus's is. */
  { 14u, kEfxModParabola, { 2u, 2u }, kEfxPreDelayTable,
    3u, 4u, 9u, 10u, 5, false,  6, -1, 12.32, -1, -1, 1u, -1, 0.0, { 1.0, 0.0, 0.0, 0.0 } },
  /* 18 MODULATION-DELAY: a stereo delay with the same LFO on top. Its two
     delay slots read `0x038FC8` at `ms = table / 32`, measured with the
     depth at zero so the delay stands still - ratio 1.0000 at values 16,
     32, 64, 96, 112 and 126, which is 1.59 ms to 500.01 ms, a 314x range.
     `0x038EC8` predicts 46, 74 and 101 ms where the machine gives 100, 260
     and 500, so it is not that table. */
  { 17u, kEfxModTriangle, { 1u, 2u }, kEfxDelayTable,
    5u, 6u, 10u, 11u, 7, false,  3,  4, 49.55, -1, -1, 1u, -1, 0.0, { 1.0, 0.0, 0.0, 0.0 } },
  /* 12 TREMOLO-CHORUS: a chorus and an amplitude modulator, each with its
     own rate and depth, separated by driving one with the other at zero.

     p2 is the chorus rate and p4 the tremolo rate, both on the family law:
     p2 at 24 and 72 counts 1.2493 and 3.6487 Hz against 1.2493 and 3.6488,
     and p4 at its factory 76 counts 3.845 against 3.849.

     THE TREMOLO'S MODULATOR IS A TRIANGLE, measured on its envelope's
     harmonics at six depths: h2 0.008, h3 0.111, h5 0.040 against an ideal
     triangle's 0, 0.111, 0.040, identical to three decimals at every one.
     Its depth is the same level curve the rest of the family uses: read as
     an envelope falling from 1 to 1-m, the six values give m normalised at
     0.142, 0.266, 0.418, 0.585, 0.776, 1.000 against the curve's 0.121,
     0.258, 0.418, 0.590, 0.783, 1.000.

     The chorus sweep tops out at 12.23 ms, alongside 12.38 and 12.32 on
     the other two that have a pre-delay. Its SHAPE matches the level curve
     only to 0.019 where the other three match to 0.0005, so that this type
     reads the same table is consistent rather than established.

     p1 is not a rate - driven to 72 it moves the deviation 0.037 % against
     p2's 9.372 - and it is wired as the pre-delay because that is the
     family's shape and it ships at 0, where the table's own entry is one
     sample. If it is something else, nothing here depends on it. */
  { 11u, kEfxModTriangle, { 0u, 0u }, kEfxPreDelayTable,
    1u, 2u, 6u, 7u, 5, false, -1, -1, 12.23, 3, 4, 1u, -1, 0.0, { 1.0, 0.0, 0.0, 0.0 } },
  /* 13 SPACE-D: ONE delay in both channels, with the right one NEGATED.

     Measured on the cross-correlation between the channels, which needs no
     inference: its peak sits at lag 0.0000 ms and its sign is negative at
     pre-delays of 3.19, 14.00 and 46.00 ms, at full depth, and at all
     three settings of p4 - the only parameter this type accepts a ceiling
     of 90 on, and therefore the family's phase slot, which moves the lag
     not at all. So the two channels share one delay line and p4 is not an
     inter-channel phase here, which is why this spec carries none.

     Its sweep tops out at 3.04 ms - a quarter of the chorus family's
     twelve, which is why every earlier instrument read this type as
     static - following the level curve at 0.001, 0.191, 0.398, 0.687,
     1.000 against 0.000, 0.191, 0.418, 0.689, 1.000. Its modulator is a
     triangle, read by folding the delay trajectory itself rather than
     through a carrier: one cycle descends linearly, turns, and ascends
     linearly, with no bulge at the ends and no snap back.

     p1 is the pre-delay, measured exactly: 3.19, 14.00, 46.00 and 100.00
     ms at values 32, 64, 96 and 125 against `0x038EC8`'s own table/32. */
  { 12u, kEfxModTriangle, { 0u, 0u }, kEfxPreDelayTable,
    1u, 2u, 6u, 7u, -1, true, -1, -1, 3.04, -1, -1, 1u, -1, 0.0, { 1.0, 0.0, 0.0, 0.0 } },
  /* 11 HEXA-CHORUS: THREE voices on one line, evenly spread and tapered.

     Three, not six, and that is measured rather than read off the name.
     With p4 at 20 the taps sit 40.4 ms apart - far enough that the
     inter-tap differences cannot be confused with them - and the
     autocorrelation at the first six tap positions reads 0.3224, 0.2634,
     0.2121, 0.0002, 0.0002, 0.0002. The last three are the noise floor:
     there is nothing there. p4 at 15 gives the same picture.

     THE TAPER is those first three normalised: 1.000, 0.817, 0.658. Taken
     at p4 = 20 because its taps are the best separated; p4 = 15 agrees on
     the second at 0.824 and gives 0.542 on the third, so the third is
     measured to about a tenth and not better. Equal levels are ruled out
     by a stronger argument than the numbers: with them the inter-tap
     correlations are as large as the dry-to-tap ones, and the machine's
     are half the size.

     p4 SETS THE SPREAD, LINEARLY: 0, 5, 10, 15 and 20 give 0, 10.1, 20.2,
     30.1 and 40.6 ms, about 2.02 ms a step, consistent with 64 samples at
     32 kHz to within one percent. At p4 = 0 all three coincide.

     Sweep 11.96 ms at depth 127 following the level curve at 0.000, 0.190,
     0.429, 0.685, 1.000 against 0.000, 0.191, 0.418, 0.689, 1.000;
     modulator a triangle, folded out of the tracked trajectory.

     NOT MODELLED: p6 moves the spacing not at all, which fits a PAN
     deviation a mono autocorrelation cannot see, so all three voices are
     summed to both channels and this renders narrower than the machine.
     p5 does nothing with the depth at zero and is presumably a depth
     deviation; the three sweep together here. */
  { 10u, kEfxModTriangle, { 0u, 0u }, kEfxPreDelayTable,
    1u, 2u, 6u, 7u, -1, false, -1, -1, 11.96, -1, -1,
    3u, 3, 2.02, { 1.000, 0.817, 0.658, 0.0 } },
};
const unsigned kEfxModSpecCount =
  (unsigned)(sizeof kEfxModSpecs / sizeof *kEfxModSpecs);
const struct EfxModSpec *efx_mod_spec(unsigned type)
{
  for (unsigned i = 0; i < kEfxModSpecCount; ++i)
    if (kEfxModSpecs[i].type == type)
      return kEfxModSpecs + i;
  return NULL;
}

/* THE OUTPUT SHELVES: LowGain and HiGain on the types that carry them.

   ONE PAIR OF WRITERS SERVES THEM ALL. Every updater below hands its two
   gain bytes to the shelf writers `0x0A0022CE` (low) and `0x0A002372`
   (high) - the same two STEREO-EQ and ENHANCER use - once per channel,
   and sets RAM 0x0901F874 to 1 before the low call and 0 before the high
   one, so the corners are fixed at 400 Hz (0x039802) and 4 kHz (0x0398BC)
   on every one of them (FW-EXACT, stereo_eq.h `stereo_eq_shelf_set`). The
   byte positions are the updaters' own (FW-EXACT), and each agrees with
   the 30-step ceilings `M-092` measured at those positions:

     type  updater      low/high  CRAM (first; second call)
     13    `0x0A003C28` p5 / p6   53..58; 68..73    SPACE-D
     14/15 `0x0A003E5C` p8 / p9   72..77; 86..91    STEREO-CHORUS/FLANGER
     17    `0x0A00452C` p8 / p9   25..30; 52..57    STEREO-DELAY
     18    `0x0A0047F0` p9 / p10  59..64; 73..78    MODULATION-DELAY
     19    `0x0A004AE4` p9 / p10  32..37; 47..52    TRIPLE-TAP-DELAY
     21    `0x0A0051F0` p6 / p7   39..44; 52..57    TIME-CONTROL-DELAY
     24    `0x0A005F00` p5 / p6   91..96; 81..86    REVERB

   (display numbers). The writers also serve five types this engine does
   not render - COMPRESSOR p5/p6, LIMITER p6/p7, STEP-FLANGER p7/p8,
   FBK-PITCH-SHIFTER p7/p8, GATE-REVERB p4/p5 - and ENHANCER, which builds
   its own (enhancer.h). HEXA-CHORUS, TREMOLO-CHORUS, QUADRUPLE-TAP-DELAY
   and the compounds call neither writer and have no shelves.

   WHERE THEY SIT. In each of these programs the shelf sections are the
   last filter stage of each channel, after every delay-memory access, so
   they are outside any feedback loop (FW-STRUCT: the coefficient index is
   the instruction index; the opcodes are not decoded, `U-R5-02`). That
   they shape the WHOLE return, dry side of the balance included, is
   MEASURED: STEREO-DELAY ships with Balance 0, which is no wet at all
   (`0x0394A0` row 0 is 0 wet, 1 dry, and the take's tick has no echo),
   and its `efx_probe` noise burst still carries the factory shelves
   (LowGain +4, HiGain +2 dB) - against the STEREO-EQ take as the
   known-good reference through the same path, the residual matches the
   two shelves to 0.04 dB mean and 0.19 dB worst over 60 Hz to 13 kHz,
   where leaving them out misses by 0.82 dB mean. So they are run here
   on the balanced, levelled return, ahead of the sends as the program's
   own output stage is.

   A DEDICATED CAPTURE (each type's own noise burst, both shelves driven
   to their extremes, dry-only and factory-balance) confirms all eight
   directly rather than by analogy to STEREO-DELAY: every dry-isolated
   response matches the model to 0.03-0.10 dB mean, worst case 0.35 dB,
   over 60 Hz-13 kHz - including REVERB (0.04/0.05 dB), whose own
   factory-balance `efx_probe` reading (a different, wet-dominated take)
   had earlier looked worse with the shelves than without; this isolated
   capture resolves that in the shelves' favour. SPACE-D's
   factory-balance figure is the one outlier (0.63 dB mean, 2.73 dB
   worst) despite a clean dry reading (0.04/0.05 dB) - some interaction
   with its own modulation at factory balance, not chased further. P-xxxx. */
const uint8_t kEfxShelfFlat = 15u;
struct EfxShelfSpec {
  unsigned type;                /* 0-based */
  unsigned low, high;           /* 0-based parameter bytes */
};
const struct EfxShelfSpec kEfxShelfSpecs[] = {
  { 12u, 4u, 5u }, { 13u, 7u, 8u }, { 14u, 7u, 8u }, { 16u, 7u, 8u },
  { 17u, 8u, 9u }, { 18u, 8u, 9u }, { 20u, 5u, 6u }, { 23u, 4u, 5u },
};
const struct EfxShelfSpec *efx_shelf_spec(unsigned type)
{
  for (unsigned i = 0;
       i < sizeof kEfxShelfSpecs / sizeof *kEfxShelfSpecs; ++i)
    if (kEfxShelfSpecs[i].type == type)
      return kEfxShelfSpecs + i;
  return NULL;
}


const unsigned kEfxOutputAssignField = 0x1au;
const unsigned kEfxOutputLevelField = 0x1bu;
const unsigned kEfxChorusSendField = 0x1cu;
const unsigned kEfxReverbSendField = 0x1du;

const unsigned kChorusFeedbackField = 0x26u;
const unsigned kChorusOutputField = 0x27u;
/* MIX / REVERB / MIX+REV, the machine's own order (`02_rom/strings.md`
   `0x057109`). */
const unsigned kChorusOutMix = 0u;
const unsigned kChorusOutReverb = 1u;

const unsigned kParts = 16u;
/* The part the patch-mode temporary area `03 00 bb xx` writes, and so the
   one a power-on patch selection loads. This engine's arrangement, not the
   firmware's own buffer index. */
const unsigned kPatchModePart = 0u;
const unsigned kMaxVoices = XP_ENGINE_SLOT_COUNT;

/* Which block of a temporary-patch address names what. The tone blocks are
   two apart because the machine's own address map puts them there, and the
   rhythm set lives at part index nine. */
const unsigned kBlockPatchCommon = 0x00u;
const unsigned kBlockFirstTone = 0x10u;
const unsigned kBlockToneStride = 0x02u;

/* A block addresses 128 parameters - the offset byte is seven bits - which
   is why a 130-parameter tone needs two of them. */
const unsigned kBlockParameters = 0x80u;

/* The performance-part block is twenty decoded bytes; the engine reads four
   of them by role and keeps the rest so a write is not discarded. */
const unsigned kPartFields = 20u;

/* The rhythm set: one common block and one record per key it holds. */
const unsigned kRhythmCommonFields = 13u;
const unsigned kRhythmNoteFields = 59u;
const unsigned kRhythmKeys = 64u;

struct Part {
  uint8_t common[XP_JV1080_PATCH_COMMON_FIELDS];
  uint8_t tone[XP_JV1080_TONES_PER_PATCH][XP_JV1080_TONE_FIELDS];
  uint8_t part[kPartFields];
  /* The part's CC0/CC32 latch, resolved to a packed bank only when a
     program change arrives - which is the order the device resolves them
     in, since either may come first. Each byte is written on its own, so a
     CC0 alone leaves the old LSB in place. Seeded from the part record
     whenever that names a new patch. */
  uint8_t bank_msb;
  uint8_t bank_lsb;
  /* MEASURED (`M-081`): CC7 indexes the same square law with a floor -
     values 0, 1 and 2 all give what the law gives for 1.15. Held per part
     because it is received per channel. */
  uint8_t volume;
  /* The performance controllers as received: the bender re-centred to
     -8192..8191, modulation, channel aftertouch, and the hold pedal.
     `cc` holds every controller's last value, which is what a matrix
     source that names a controller reads. */
  int bend;
  uint8_t modulation;
  uint8_t pressure;
  uint8_t cc[128];
  bool hold;
  /* The sign the part's next note throws its alternate pan with: flipped
     by every note-on, reset to +1 by a program change. */
  int alternate_next;
  /* When each key last had a note-on on this part, as the engine frame
     plus one; zero for never. What a KEY-INTERVAL tone delay times. */
  uint64_t key_on_frame[128];
  /* The RPN latch, 127/127 when none is selected, and the bend range RPN
     0/0 set, -1 while it has set none. */
  uint8_t rpn_msb;
  uint8_t rpn_lsb;
  int rpn_bend;
  /* RPN 0/2 and 0/1: coarse tune in semitones and fine tune in cents. */
  int rpn_coarse;
  double rpn_fine;
  /* The keys held down on this part, by their note-on velocity, zero for
     up - the hold pedal does not keep a key here. A SOLO part returns to
     one of them, and portamento's LEGATO mode asks whether any is. */
  uint8_t held_velocity[128];
  /* The key a SOLO part is sounding, -1 for none. */
  int solo_key;
  /* Portamento's part state as the part's next note sees it: the key of
     the last note-on it took in, -1 for none yet, and the glide that note
     started, in key cents (a key is 100), from `porta_frame` for
     `porta_frames` frames. A note-on is taken in kPortaSeenFrames after it
     arrives; until then it waits in `porta_pending`. */
  int porta_key;
  double porta_from;
  double porta_to;
  uint64_t porta_frame;
  double porta_frames;
  struct PortaNote {
    uint64_t frame;
    int key;
    double from, to, frames;
  } porta_pending[8];
  unsigned porta_pending_count;
};

struct Voice {
  struct XpJv1080Voice voice;
  /* Where voice.pcm came from, and where free_voice gives it back. */
  struct xp_wave_cache *wave_cache;
  uint64_t serial;
  uint8_t part;
  uint8_t key;
  /* The key the voice was started on. `key` is the key its note-off
     matches, which a SOLO legato note moves on to without a new voice. */
  uint8_t start_key;
  /* Nonzero on a rhythm voice, and the group it belongs to: striking
     another key of the same group stops this one, which is what a hi-hat
     pair does. */
  uint8_t mute_group;
  /* This voice's share of each send bus, from its part at note-on. */
  float reverb_send;
  float chorus_send;
  /* MIX, EFX, OUTPUT1 or OUTPUT2, resolved at note-on. */
  uint8_t destination;
  bool allocated;
  bool key_down;
  /* The tone delay. `wait` is the frames still to pass before the voice
     sounds, and `release_in` the frames to a release its note-off
     postponed, zero for none. `delay` is the record's own delay in
     frames. */
  uint8_t delay_mode;
  size_t delay;
  size_t wait;
  size_t release_in;
  /* The bend range this voice answers, in cents each way, and the hold
     pedal: whether its record answers it, and a note-off it is holding. */
  double bend_up;
  double bend_down;
  bool holdable;
  bool sustained;
  /* A KEY-OFF-DECAY voice runs from its note-on unheard until its release
     begins. */
  bool muted;
  /* A structured tone pair: `pair` is the other voice, and the first
     tone's voice, `pair_feed`, is rendered by the second's. */
  struct Voice *pair;
  bool pair_feed;
};

/* MEASURED (`M-014`): linear in the 14-bit value, scaled by the range on
   its own side, the full down deflection reaching the whole range. */
double bend_ratio(int bend, double up, double down)
{
  double cents = bend >= 0 ? (double)bend * up / 8192.0
                           : (double)bend * down / 8192.0;
  return cents == 0.0 ? 1.0 : std::pow(2.0, cents / 1200.0);
}

/* THE TONE DELAY, by the modes of the panel's own list at PRG `0x0573B7`:
   NORMAL, HOLD, PLAY-MATE, CLOCK-SYNC, TAP-SYNC, KEY-OFF-NORMAL,
   KEY-OFF-DECAY. */
const uint8_t kDelayNormal = 0u;
const uint8_t kDelayHold = 1u;
const uint8_t kDelayKeyInterval = 2u;
const uint8_t kDelayKeyOffNormal = 5u;
const uint8_t kDelayKeyOffDecay = 6u;
const size_t kWaitForKeyOff = SIZE_MAX;

/* MEASURED (`M-016`), key 60, one onset per value: the delay time field
   against the onset after the note-on, the ~5 ms every value shows at 0
   taken off. Linear at about 10 ms a step to 96, then a coarse tail. The
   law between these points is not recovered and is read here as straight
   lines between them. */
struct DelayPoint {
  uint8_t value;
  double ms;
};
const struct DelayPoint kDelayPoints[] = {
  { 0u, 0.0 },     { 16u, 170.0 },  { 32u, 320.0 },
  { 48u, 480.0 },  { 64u, 640.0 },  { 80u, 805.0 },
  { 96u, 955.0 },  { 112u, 2210.0 }, { 127u, 5010.0 },
};

double tone_delay_ms(unsigned value)
{
  const size_t count = sizeof kDelayPoints / sizeof kDelayPoints[0];
  if (value >= kDelayPoints[count - 1].value)
    return kDelayPoints[count - 1].ms;
  for (size_t i = 1; i < count; ++i)
    if (value <= kDelayPoints[i].value) {
      const DelayPoint &a = kDelayPoints[i - 1], &b = kDelayPoints[i];
      return a.ms + (b.ms - a.ms) * (double)(value - a.value) /
                      (double)(b.value - a.value);
    }
  return 0.0;
}

/* THE OUTPUT IS AC-COUPLED, AS EVERY ANALOGUE AUDIO OUTPUT IS.
 *
 *   The sample decoder integrates differences, so an element whose deltas do
 *   not sum to zero leaves a standing offset and a sustained note holds it.
 *   Measured on this device's own first factory song: without this the render
 *   carries 7.8 % of its energy below 20 Hz against the hardware take's
 *   0.1 %, and a per-100 ms offset six times the hardware's. One pole at
 *   10 Hz, which is 0.3 dB down by 40 Hz - inaudible, and it is what takes
 *   the offset out of the headroom.
 */
const double kDcBlockerHz = 10.0;

struct DcBlocker {
  double x1, y1, coefficient;
};

void dc_blocker_init(struct DcBlocker *dc, double rate)
{
  dc->x1 = 0.0;
  dc->y1 = 0.0;
  /* One-pole highpass, the usual difference form: the pole sits at
     exp(-2*pi*fc/fs), which at 10 Hz and any rate this renders at is a hair
     inside one. */
  dc->coefficient = std::exp(-2.0 * 3.14159265358979323846 *
                              kDcBlockerHz / rate);
}

float dc_blocker_step(struct DcBlocker *dc, double x)
{
  double y = x - dc->x1 + dc->coefficient * dc->y1;
  dc->x1 = x;
  dc->y1 = y;
  return (float)y;
}

/* THE ANALOGUE OUTPUT STAGE LIFTS THE BASS: A FIRST-ORDER LOW SHELF.
 *
 *   MEASURED (`P-xxxx`), no new capture: hardware against our render of the
 *   identical files. On `key_scaling/key_sweep_saw2`, 1492 harmonic lines
 *   over keys 0-127, the hardware-minus-ours level of each line splits into
 *   a term in the note and a term in the line's own frequency: frequency
 *   alone leaves 0.18 dB rms, the note alone 0.88, and the note term is flat
 *   to +-0.2 dB over the whole keyboard. `key_sweep_piano` gives the same
 *   frequency term. So this is a response in OUTPUT FREQUENCY and not a gain
 *   on the note - which is why a single sine reads it against its sounded
 *   pitch, whether the pitch comes from the key (`key_sweep_sine`, every key
 *   on its own element), coarse tune (`pitch/coarse_octaves`,
 *   `coarse_semitones`), part coarse tune (`part_coarse`) or a key follow of
 *   -100 % (`pitch_kf_i00`): relative to 261.6 Hz the sine is +4.9 dB at
 *   16 Hz, +3.8 at 33, +2.6 at 65, +1.0 at 131, -0.4 at 523 and -0.55 from
 *   2 kHz up, identical in all five to 0.1 dB.
 *
 *   The capture chain is not the source: the same interface recorded a
 *   JV-880 within 0.4 dB of an independent archival chain from 40 Hz to
 *   6.4 kHz (JV-880 `M-109`).
 *
 *   The shape is a single pole and a single zero, 6 dB per octave between
 *   two plateaus, and it is realised as one: a least-squares fit over
 *   20 Hz-6 kHz puts the zero at 108.0 Hz and the pole at 58.1 Hz (5.39 dB
 *   of lift), leaving 0.14 dB rms (0.55 worst) on the saw's frequency term
 *   and 0.09 dB rms (0.28 worst) on the sine sweep. The two sets alone give
 *   100.8/51.4 and 112.3/61.9 Hz. Both corners are FITTED, not read off a
 *   component: which part of the board makes this shelf is not traced.
 *
 *   The stage is unity at 261.6 Hz, key 60's pitch, where this engine's
 *   level laws were measured, so none of them moves. That is a convention:
 *   the recordings fix the shape, not the board's absolute gain. It runs on
 *   the finished mix, effects included, because that is where an analogue
 *   board is; the dry takes above cannot show that the returns pass through
 *   it. Above 6 kHz the saw reads a further 0.3-1.5 dB of loss by 12.6 kHz,
 *   which this stage does not carry.
 */
const double kOutputShelfZeroHz = 108.0;
const double kOutputShelfPoleHz = 58.1;
const double kOutputShelfUnityHz = 261.6;

struct OutputShelf {
  double b0, b1, a1;
  double x1[2], y1[2];
  bool active;
};

void output_shelf_init(struct OutputShelf *sh, double rate)
{
  const double pi = 3.14159265358979323846;
  sh->x1[0] = sh->x1[1] = sh->y1[0] = sh->y1[1] = 0.0;
  sh->active = !std::getenv("EMUSC_NO_ANALOG_STAGE");
  /* H(s) = (s + wz) / (s + wp) through the bilinear transform, the corners
     prewarped so they land where they were measured. */
  const double k = 2.0 * rate;
  const double wz = k * std::tan(pi * kOutputShelfZeroHz / rate);
  const double wp = k * std::tan(pi * kOutputShelfPoleHz / rate);
  double b0 = (k + wz) / (k + wp), b1 = (wz - k) / (k + wp);
  sh->a1 = (wp - k) / (k + wp);
  const double w = 2.0 * pi * kOutputShelfUnityHz / rate;
  const double nr = b0 + b1 * std::cos(w), ni = -b1 * std::sin(w);
  const double dr = 1.0 + sh->a1 * std::cos(w), di = -sh->a1 * std::sin(w);
  const double g = std::sqrt((dr * dr + di * di) / (nr * nr + ni * ni));
  sh->b0 = b0 * g;
  sh->b1 = b1 * g;
}

void output_shelf_run(struct OutputShelf *sh, float *stereo, size_t frames)
{
  if (!sh->active)
    return;
  for (size_t k = 0; k < frames; ++k)
    for (unsigned c = 0; c < 2u; ++c) {
      const double x = stereo[k * 2u + c];
      const double y = sh->b0 * x + sh->b1 * sh->x1[c] - sh->a1 * sh->y1[c];
      sh->x1[c] = x;
      sh->y1[c] = y;
      stereo[k * 2u + c] = (float)y;
    }
}

struct Rhythm {
  uint8_t common[kRhythmCommonFields];
  uint8_t note[kRhythmKeys][kRhythmNoteFields];
};

struct Engine {
  struct xp_rom rom;
  const uint8_t *banks[XP_WAVE_BANK_COUNT];
  size_t bank_sizes[XP_WAVE_BANK_COUNT];
  /* Decoded element PCM, shared by every voice reading the same span. */
  struct xp_wave_cache wave_cache;
  /* The rate everything below runs at: the machine's own 32 kHz, whatever
     the host asks for. `resampler` carries it to the host's rate; at a
     32 kHz host there is none and the engine's frames are the host's. */
  double output_rate;
  bool resampling;
  struct JvResampler resampler;
  unsigned max_voices;
  uint64_t serial;
  /* Frames rendered since the engine was created: the clock a
     free-running LFO takes its phase from. */
  uint64_t frames;
  struct Part parts[kParts];
  /* GM mode: entered by GM System On, left only by a reset. */
  bool gm_mode;
  /* System Control Source 1 and 2 (System common `00 12`, `00 13`): 0-95
     a controller number, 96 BENDER, 97 AFTERTOUCH. */
  uint8_t sys_ctrl[2];
  /* System common `00 0B`, Patch Remain: whether a program change leaves
     the part's sounding voices alone. */
  bool patch_remain;
  struct Rhythm rhythm;
  struct Voice voices[kMaxVoices];
  struct DcBlocker dc_left;
  struct DcBlocker dc_right;
  struct OutputShelf output_shelf;
  /* The performance common block, which carries this device's reverb
     parameters, and the reverb itself. */
  uint8_t common[kPerfCommonFields];
  /* The tempo in force, which EXT SYNC LFOs count against. What sets it in
     patch mode (the patch's own Default tempo, presumably) is not modelled:
     only a performance common write moves it. */
  double tempo_bpm;
  struct xp_reverb reverb;
  bool reverb_ready;
  uint8_t reverb_character;
  struct xp_chorus chorus;
  bool chorus_ready;
  /* The panning delay, which replaces the tank on reverb type 7. */
  float *delay_buf;
  size_t delay_len;
  size_t delay_pos;
  double delay_left;
  double delay_right;
  float delay_gain_left;
  float delay_gain_right;
  float delay_feedback;
  bool delay_ready;
  /* The insert effect's output block, as coefficients. No effect renders
     yet, so nothing reads these but the parameter path that forms them -
     which is the point: the routing rule is testable before any algorithm
     exists. */
  float efx_output_level;
  float efx_chorus_send;
  float efx_reverb_send;
  /* The effect block in force: its type and twelve parameters, fetched
     from whichever patch the source selector names. `efx_dirty` is the
     change detection the firmware's own applied-cache stands for - set
     when anything in the block moves, and the type moving clears the
     parameters with it, since a parameter means something different under
     a different effect. */
  uint8_t efx_type;
  uint8_t efx_parameter[kEfxParameters];
  bool efx_dirty;
  /* The pure-delay family's two lines, and what the parameters made of
     them. `efx_ready` is false for every type this engine does not yet
     render, and then nothing is routed anywhere. */
  float *efx_buf[2];
  size_t efx_len;
  size_t efx_pos;
  double efx_delay[2];
  float efx_feedback;
  bool efx_cross;
  float efx_phase[2];
  float efx_damp;
  float efx_damp_state[2];
  /* The tap-delay family's taps: one line read at up to four points, each
     with its own level and its own place in the image. */
  double efx_tap_delay[4];
  float efx_tap_left[4];
  float efx_tap_right[4];
  unsigned efx_taps;
  /* The modulated-delay family's LFO: a phase in cycles, its per-sample
     step, the second channel's offset, and the sweep the depth allows. */
  double efx_lfo_phase;
  double efx_lfo_step;
  double efx_lfo_offset;
  double efx_sweep;
  double efx_nominal[2];
  unsigned efx_shape;
  double efx_trem_phase, efx_trem_step;
  float efx_trem_depth;
  bool efx_invert_right;
  unsigned efx_voices;
  double efx_spread;
  float efx_weight[4];
  /* The insert reverbs' own network - see the block above for why it is
     not the shared one. REVERB uses the comb bank; GATE-REVERB the one
     shared line, its tail being gated rather than decayed. */
  unsigned rv_taps;
  float *rv_comb[kInsertReverbMaxTaps];
  size_t rv_comb_len[kInsertReverbMaxTaps];
  size_t rv_comb_pos[kInsertReverbMaxTaps];
  float rv_comb_g[kInsertReverbMaxTaps];
  float rv_comb_damp[kInsertReverbMaxTaps];
  float *rv_buf;
  size_t rv_len, rv_pos;
  size_t rv_lag[kInsertReverbMaxTaps];
  float *rv_pre_buf;
  size_t rv_pre_len, rv_pre_pos, rv_pre;
  float rv_damp;
  size_t rv_gate, rv_since;
  float rv_env;
  bool rv_gated;
  struct xp_drive efx_drive;
  struct xp_stereo_eq efx_eq;
  struct xp_spectrum efx_spectrum;
  struct xp_enhancer efx_enhancer;
  struct xp_phaser efx_phaser;
  struct xp_rotary efx_rotary;
  struct xp_dynamics efx_dynamics;
  /* The output shelves (THE OUTPUT SHELVES), low then high, and the gains
     they were built from - a gain past 30 is discarded and the previous
     one kept, as on every other byte. `efx_shelf_type` is the type they
     were built for; a type change clears their state. */
  struct xp_stereo_eq_shelf efx_shelf[2];
  uint8_t efx_shelf_gain[2];
  uint8_t efx_shelf_type;
  bool efx_shelf_on;
  float efx_wet;
  float efx_dry;
  float efx_level;
  bool efx_ready;
  /* Where jv_render_native renders its voices, or null to render them on
     the calling thread; not owned. */
  WorkerPool *pool;
  /* Owned; allocated by the first engine_set_worker_pool_jv that sets a
     pool. */
  struct VoiceBatch *batch;
  /* Decodes the waves a newly loaded patch or kit can play into the cache
     before its notes ask for them, off the render path. Owned. */
  BackgroundWorker *prefetch;
  /* Parts whose records changed since their waves were last queued, bit
     kParts for the rhythm set. */
  uint32_t prefetch_parts;
};

void voice_batch_free(struct VoiceBatch *batch);

/* THE PART'S REVERB SEND IS THE LIVE ONE, not the tone's (`M-039`,
   `M-052`), so a voice carries its part's send from note-on. */
void free_voice(struct Voice *voice)
{
  /* A pair goes as one, whichever half is freed. Only a steal frees the
     first tone alone, and a second tone left behind would fall back to
     the one-tone path mid-note with its TVF state built on the pair's
     signal: in type 9 that filter runs ahead of the TVA, so its state is
     at the unattenuated wave's scale, and at high resonance it rings out
     at full scale (`1080 rave` at 92.03 s). What the machine steals when
     it takes one tone of a pair is NOT MEASURED. */
  if (struct Voice *other = voice->pair) {
    voice->pair = nullptr;
    other->pair = nullptr;
    free_voice(other);
  }
  voice->pair_feed = false;
  wave_cache_release(voice->wave_cache, voice->voice.pcm);
  voice->voice.pcm = nullptr;
  voice->wave_cache = nullptr;
  voice->allocated = false;
  voice->key_down = false;
  voice->delay_mode = 0u;
  voice->delay = 0u;
  voice->wait = 0u;
  voice->release_in = 0u;
  voice->bend_up = 0.0;
  voice->bend_down = 0.0;
  voice->holdable = false;
  voice->sustained = false;
  voice->muted = false;
  std::memset(&voice->voice, 0, sizeof voice->voice);
}

/* Which packedBankSelect entry a latched pair names, or the table's count
   when it names no group at all. An entry whose bank is
   XP_PACKED_BANK_NONE is a group that exists and is not held. */
unsigned select_by_pair(const struct XpDeviceProfile *profile, uint8_t msb,
                        uint8_t lsb)
{
  for (unsigned i = 0; i < profile->packedBankSelectCount; ++i)
    if (profile->packedBankSelect[i].msb == msb &&
        profile->packedBankSelect[i].lsb == lsb)
      return i;
  return profile->packedBankSelectCount;
}

/* Which packedBankSelect entry the part record's own group names, or the
   table's count when it names none this implementation lists. Only type 0
   is resolved: types 1 and 2 match installed cards and boards, and none
   is installed. */
unsigned select_by_record(const struct xp_rom *rom, const struct Part *part)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  unsigned none = profile->packedBankSelectCount;
  if (profile->partFieldPatchGroupType == XP_VOICE_FIELD_NONE ||
      !profile->partGroupIdTable ||
      part->part[profile->partFieldPatchGroupType] != 0u)
    return none;
  unsigned id = part->part[profile->partFieldPatchGroupId];
  if (id >= profile->partGroupIdCount ||
      profile->partGroupIdTable + id >= rom->size)
    return none;
  uint8_t group = rom->bytes[profile->partGroupIdTable + id];
  for (unsigned i = 0; i < none; ++i)
    if (profile->packedBankSelect[i].group == group)
      return i;
  return none;
}

/* Point the part's latch at the group its record names, which is what the
   device does when a performance loads (`0x0A018E1C`) and when a part's
   patch selection is written (`0x0A0140FA`): both pass the record's group
   through the reverse map `0x0A014EBE`. A record naming no listed group
   leaves the latch alone. */
void seed_latch(const struct xp_rom *rom, struct Part *part)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  unsigned i = select_by_record(rom, part);
  if (i == profile->packedBankSelectCount)
    return;
  part->bank_msb = profile->packedBankSelect[i].msb;
  part->bank_lsb = profile->packedBankSelect[i].lsb;
}

/* A part's power-on patch: silent, centred, at full level. Every tone
   switch is zero, so a part nothing has written to sounds nothing rather
   than sounding whatever an all-zero tone record would resolve to. */
void reset_part(const struct xp_rom *rom, struct Part *part, unsigned index)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  std::memset(part, 0, sizeof *part);
  part->common[profile->patchFieldLevel] = 127u;
  part->common[profile->patchFieldPan] = 64u;
  part->part[profile->partFieldLevel] = 127u;
  part->part[profile->partFieldPan] = 64u;
  /* Until a performance says otherwise a part receives on the channel of
     its own number, which is the state a caller with no performance to
     load - a bank select and a program change - needs. */
  part->part[profile->partFieldReceiveChannel] = (uint8_t)index;
  /* An all-zero record names type 0 id 0, which the device's own table
     resolves to USER, so that is where the latch starts. */
  part->bank_msb = 0xffu;
  part->bank_lsb = 0xffu;
  seed_latch(rom, part);
  part->volume = 127u;
  part->rpn_msb = 0x7fu;
  part->rpn_lsb = 0x7fu;
  part->rpn_bend = -1;
  part->alternate_next = 1;
  /* Pan centred and volume full before either is received. CC11 starts
     at 0 as a matrix source, read off the PRG and MEASURED (`M-160`,
     TASK-424): the per-part reset `0x0A0137C0`, which machine init runs,
     writes 0 to both bytes the matrix reads CC11 through - EXPRESSION's
     `0x09001391 + p` and SYS-CTRL2's `0x09000A67 + p` when SYS-CTRL2 is
     11 - and 127 only to `0x090013A2 + p`, which no matrix source reads.
     On the device, a part reset by CC121, by a change of its receive
     channel or by its receive switch going off and on reads 0.0 on both
     sources, against a calibration of CC11 0 to 127 in the same take. */
  part->cc[7] = 127u;
  part->cc[10] = 64u;
  part->cc[11] = 0u;
  part->solo_key = -1;
  part->porta_key = -1;
}

/* A controller's value as a source, 0..1. CC7 is read from the part's
   volume, which a GM reset sets without a CC7 arriving. */
double controller_value(const struct Part &p, unsigned cc)
{
  return (double)(cc == 7u ? p.volume : p.cc[cc]) / 127.0;
}

/* One matrix source, 0..1. Patch Control Source is the ROM's list at PRG
   `0x057280`: OFF, SYS-CTRL1, SYS-CTRL2, MODULATION, BREATH, FOOT, VOLUME,
   PAN, EXPRESSION, BENDER, AFTERTOUCH, LFO1, LFO2, VELOCITY, KEYFOLLOW,
   PLAY-MATE. MEASURED (`M-116`): controller 1 answers CC1 alone, and with
   the System sources at their readback values 97 and 11, SYS-CTRL1
   answers channel aftertouch and SYS-CTRL2 CC11; MODULATION answers CC1.
   BREATH, FOOT, VOLUME, PAN and EXPRESSION read CC2, 4, 7, 10 and 11 by the
   ROM's names, not measured through the matrix. BENDER, LFO1, LFO2,
   VELOCITY, KEYFOLLOW and PLAY-MATE are NOT IMPLEMENTED and read zero - a
   bipolar source's scaling is not measured. */
double matrix_source(const struct Engine *engine, const struct Part &p,
                     unsigned controller)
{
  static const int kNamed[16] = { -1, -1, -1, 1, 2, 4, 7, 10, 11,
                                  -1, -2, -1, -1, -1, -1, -1 };
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  if (!controller)
    return controller_value(p, 1u);
  uint16_t field = controller == 1u ? profile->patchFieldControlSource2
                                    : profile->patchFieldControlSource3;
  if (field == XP_VOICE_FIELD_NONE)
    return 0.0;
  unsigned index = p.common[field];
  if (index == 1u || index == 2u) {
    unsigned sys = engine->sys_ctrl[index - 1u];
    if (sys < 96u)
      return controller_value(p, sys);
    return sys == 97u ? (double)p.pressure / 127.0 : 0.0;
  }
  if (index >= 16u)
    return 0.0;
  int named = kNamed[index];
  if (named == -2)
    return (double)p.pressure / 127.0;
  return named < 0 ? 0.0 : controller_value(p, (unsigned)named);
}

void matrix_sources(const struct Engine *engine, const struct Part &p,
                    double out[3])
{
  for (unsigned c = 0; c < 3u; ++c)
    out[c] = matrix_source(engine, p, c);
}

/* A free voice if there is one; otherwise the voice stolen for the new
   note, freed. Returns null only when the pool is empty, which cannot
   happen with max_voices >= 1.

   THE VICTIM IS THE OLDEST VOICE AMONG THE PARTS OVER THEIR RESERVE.
   Measured (`M-031`: oldest-first; `M-072`: the reserve decides the
   victim). The two passes are the victim scan at PRG `0x0A0014EE`
   (FW-EXACT): pass 1 skips every voice whose part satisfies
   `count - reserve <= threshold[part]`, pass 2 runs only when pass 1 found
   nothing and ignores the reserve.

   The threshold array (`0x09013A3F + part`) has no recovered value and is
   taken as 0. `M-072` bounds it to 0..7: at reserves 0/32 the part holding
   exactly its 32 kept every voice (so not negative), and at 32/0 the part
   holding 40 against 32 lost voices (so below 8). So a part holding no
   more voices than its reserve is exempt, which is the measured outcome.

   What `count` counts is the ROM's per-part counter `0x09000A34`, whose
   increment and decrement sit in the undumped internal ROM. Here it is the
   part's allocated voices, releasing ones included, since those hold a
   slot. With every reserve 0 - no performance common written - pass 1
   admits every voice and the choice is plain oldest-first. */
struct Voice *take_voice(struct Engine *engine)
{
  unsigned count[kParts] = {};
  for (unsigned i = 0; i < engine->max_voices; ++i) {
    struct Voice *voice = engine->voices + i;
    if (!voice->allocated)
      return voice;
    if (voice->part < kParts)
      ++count[voice->part];
  }
  struct Voice *oldest = nullptr;
  for (unsigned i = 0; i < engine->max_voices; ++i) {
    struct Voice *voice = engine->voices + i;
    unsigned p = voice->part;
    if (p < kParts &&
        count[p] <= (unsigned)engine->common[kVoiceReserveField + p])
      continue;
    if (!oldest || voice->serial < oldest->serial)
      oldest = voice;
  }
  if (!oldest)
    for (unsigned i = 0; i < engine->max_voices; ++i) {
      struct Voice *voice = engine->voices + i;
      if (!oldest || voice->serial < oldest->serial)
        oldest = voice;
    }
  if (oldest)
    free_voice(oldest);
  return oldest;
}

/* THE JV-1080'S OWN REVERB PARAMETERS, against the shared reverb's fields.
   Each one is this device's, read from its own tables:

   TYPE picks the character record; this device has eight.

   LEVEL indexes the CRAM-destined level table, 128 monotonic words against
   8192 = unity. That is not the SC-88's linear 4*p: `M-045` measures tails
   of -99.8, -65.0 and -57.0 dBFS at levels 0, 64 and 127, and the table's
   own 0, 3424 and 8191 give 7.6 dB between the last two where a linear law
   gives 6.0.

   TIME reaches the tank's per-pass loop gain through a coefficient rather
   than the SC-88's register - `48 * v / 8192`, capped at 0.7441 - which
   `reverb.cc`'s own note on the decay register already records as this
   device's form of the same quantity, applied once per tank half.

   HF DAMP is `M-064`, exact to the unit: 18 rows of (a, 0x1FFF - a) where
   the one-pole is `y = a*x + (1-a)*y'` with its -3 dB corner at the row's
   own frequency, so the runtime's pole is 1 - a. Row 17 is BYPASS, and it
   only reads as one under the shift-field law - `0x5000` decodes to unity
   there and to 2.5 without it, which is a third witness for that half of
   `U-R5-01` on top of the 33 coefficient images.

   FEEDBACK is not plumbed: `M-045` measures only 3 dB across its whole
   range (-46.8, -46.2, -43.8 dBFS at 0, 64, 127) and what it does to the
   network is not recovered. */
void reverb_apply_params(struct Engine *engine)
{
  if (!engine->reverb_ready)
    return;
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  struct xp_reverb *rv = &engine->reverb;
  const uint8_t *common = engine->common;
  unsigned level = common[kReverbLevelField];
  unsigned time = common[kReverbTimeField];
  unsigned damp = common[kReverbDampField];

  if (profile->reverbLevelTable && level < 128u) {
    uint32_t a = profile->reverbLevelTable + 2u * level;
    if (a + 2u <= engine->rom.size)
      rv->level = (float)(((unsigned)engine->rom.bytes[a] << 8 |
                            engine->rom.bytes[a + 1u]) / 8192.0);
  }
  double g = 48.0 * (double)(time > 127u ? 127u : time) / 8192.0;
  if (g > 0.7441)
    g = 0.7441;
  if (g < 0.001)
    g = 0.001;
  unsigned samples[2] = { 0u, 0u };
  for (unsigned h = 0; h < 2; ++h)
    for (unsigned i = 0; i < XP_REVERB_HALF_BUFFERS; ++i) {
      unsigned b = XP_REVERB_HALF_BUFFERS * (h + 1u) + i;
      samples[h] += rv->far[b] - rv->head[b];
    }
  for (unsigned h = 0; h < 2; ++h)
    rv->decay[h] = (float)g;
  double seconds = rv->output_rate > 0.0
    ? (double)(samples[0] + samples[1]) / rv->output_rate : 0.0;
  rv->target_t60 = 3.0 * seconds / (2.0 * std::log10(1.0 / g));

  float pole = 0.0f;
  if (profile->reverbDampTable && damp < 18u) {
    uint32_t a = profile->reverbDampTable + 4u * damp;
    if (a + 2u <= engine->rom.size) {
      uint16_t raw = (uint16_t)((unsigned)engine->rom.bytes[a] << 8 |
                                 engine->rom.bytes[a + 1u]);
      /* The same coefficient law the rest of this engine reads CRAM with,
         credited where it is declared: a sign-extended 14-bit mantissa
         scaled by the two-bit field above it, against 8192 = unity. */
      int mantissa = raw & 0x3fff;
      if (mantissa & 0x2000)
        mantissa -= 0x4000;
      double coeff = (double)mantissa *
        (double)(1u << kXpCoefficientShift[raw >> 14]) / 8192.0;
      if (coeff > 0.0 && coeff < 1.0)
        pole = (float)(1.0 - coeff);
    }
  }
  rv->damp[0] = pole;
  rv->damp[1] = pole;
  /* The return path. This device has no per-character trim word - its
     return is the 9-bit level register the level table feeds - so the
     character returns unattenuated and `level` carries it. */
  rv->trim = (float)rv->character.return_trim * 16.0f / 512.0f;
  rv->wet_gain_left =
    rv->level * rv->trim / std::sqrt((float)XP_REVERB_TAPS);
  rv->wet_gain_right = rv->wet_gain_left;
}

/* A monotonic u16 table against 8192 = unity, which is how this device
   scales every level-like quantity. */
double table_unit(const struct Engine *engine, uint32_t base, unsigned v)
{
  if (!base || v > 127u)
    return 0.0;
  uint32_t a = base + 2u * v;
  if (a + 2u > engine->rom.size)
    return 0.0;
  return (double)(((unsigned)engine->rom.bytes[a] << 8) |
                   engine->rom.bytes[a + 1u]) / 8192.0;
}

/* THE JV-1080'S OWN CHORUS LAWS, each measured; the reasoning and the
   numbers are in jv1080.cc beside the tables they read. */
void chorus_refresh(struct Engine *engine)
{
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  if (!profile->chorusRateTable)
    return;
  if (!engine->chorus_ready) {
    engine->chorus_ready =
      chorus_init(&engine->chorus, engine->output_rate, profile);
    if (!engine->chorus_ready)
      return;
  }
  const uint8_t *common = engine->common;
  double scale = engine->output_rate / kXpNativeRate;
  /* the pre-delay IS the nominal delay, in samples at the wave rate */
  double delay = 0.0;
  {
    unsigned p = common[kChorusPreDelayField];
    if (p > 127u) p = 127u;
    uint32_t a = profile->chorusPreDelayTable + 2u * p;
    if (a + 2u <= engine->rom.size)
      delay = (double)(((unsigned)engine->rom.bytes[a] << 8) |
                        engine->rom.bytes[a + 1u]) * scale;
  }
  /* the sweep, peak to peak, from the depth table's shape */
  double top = table_unit(engine, profile->chorusDepthTable, 127u);
  double shape = top > 0.0
    ? table_unit(engine, profile->chorusDepthTable,
                  common[kChorusDepthField]) / top
    : 0.0;
  double depth = profile->chorusDepthMaxMs * shape *
    engine->output_rate / 1000.0;
  /* the modulation, as a phase step per output sample */
  double hz = 0.0;
  {
    unsigned p = common[kChorusRateField];
    if (p > 127u) p = 127u;
    uint32_t a = profile->chorusRateTable + 2u * p;
    if (a + 2u <= engine->rom.size && profile->chorusRateAccumulator > 0.0)
      hz = (double)(((unsigned)engine->rom.bytes[a] << 8) |
                     engine->rom.bytes[a + 1u]) /
        profile->chorusRateAccumulator / kXpControlPeriodSeconds;
  }
  float level = (float)table_unit(engine, profile->chorusLevelTable,
                                   common[kChorusLevelField]);
  /* Feedback is the level table read UNSHIFTED into CRAM word 225, so 127
     is 8191/8192 (`08_effects/chorus.md`, FW-EXACT). */
  float feedback = (float)table_unit(engine, profile->chorusLevelTable,
                                      common[kChorusFeedbackField]);
  chorus_set_runtime(&engine->chorus, delay, depth,
                      hz / engine->output_rate, feedback, level);
}

/* One tap of the delay line, read back a fractional number of samples. */
float delay_tap(const struct Engine *engine, double back)
{
  if (back < 1.0)
    back = 1.0;
  if (back > (double)(engine->delay_len - 2u))
    back = (double)(engine->delay_len - 2u);
  double read = (double)engine->delay_pos - back;
  while (read < 0.0)
    read += (double)engine->delay_len;
  size_t i0 = (size_t)read;
  double frac = read - (double)i0;
  size_t i1 = i0 + 1u >= engine->delay_len ? 0u : i0 + 1u;
  return (float)((1.0 - frac) * engine->delay_buf[i0] +
                  frac * engine->delay_buf[i1]);
}

void delay_process(struct Engine *engine, const float *send, float *stereo,
                    size_t frames)
{
  for (size_t k = 0; k < frames; ++k) {
    float l = delay_tap(engine, engine->delay_left);
    float r = delay_tap(engine, engine->delay_right);
    /* The line is written with the input plus what the loop returns, which
       is what makes the repeats repeat. */
    engine->delay_buf[engine->delay_pos] =
      send[k] + engine->delay_feedback * 0.5f * (l + r);
    if (++engine->delay_pos >= engine->delay_len)
      engine->delay_pos = 0;
    stereo[k * 2u] += engine->delay_gain_left * l;
    stereo[k * 2u + 1u] += engine->delay_gain_right * r;
  }
}

/* The panning delay's own refresh: two taps on one line, from the same
   Time parameter the tank reads as a decay. */
void delay_refresh(struct Engine *engine, bool panning)
{
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  double scale = engine->output_rate / kXpNativeRate;
  unsigned v = engine->common[kReverbTimeField];
  if (v > 127u)
    v = 127u;
  unsigned fb = engine->common[kReverbFeedbackField];
  if (fb > 127u)
    fb = 127u;
  if (!engine->delay_buf) {
    double longest = (kReverbDelaySlopeLeft * 127.0 + kReverbDelayLatency) *
      scale + 4.0;
    engine->delay_len = (size_t)longest + 4u;
    engine->delay_buf = (float *)std::calloc(engine->delay_len,
                                              sizeof *engine->delay_buf);
    if (!engine->delay_buf) {
      engine->delay_len = 0;
      return;
    }
    engine->delay_pos = 0;
  }
  engine->delay_left =
    (kReverbDelaySlopeLeft * (double)v + kReverbDelayLatency) * scale;
  /* DELAY puts all nine taps on one address; PAN-DLY splits them, and the
     right list gets half the left's delay. */
  engine->delay_right = panning
    ? (kReverbDelaySlopeRight * (double)v + kReverbDelayLatency) * scale
    : engine->delay_left;
  engine->delay_feedback =
    (float)(kReverbDelayFeedbackSlope * (double)fb / 8192.0);
  /* The same return the tank uses - this device's level table against a
     512 full scale - carrying each side's own tap sum, and normalised the
     way the tank's eight taps are. */
  double level = table_unit(engine, profile->reverbLevelTable,
                             engine->common[kReverbLevelField]);
  double norm = std::sqrt((double)XP_REVERB_TAPS);
  /* Type 6 carries the loop gain into its return and type 7 does not - the
     measurement above, not a choice made here. */
  double ret = panning ? 1.0 : (double)engine->delay_feedback;
  engine->delay_gain_left =
    (float)(level * ret * kReverbDelayTapSumLeft / norm);
  engine->delay_gain_right =
    (float)(level * ret * kReverbDelayTapSumRight / norm);
  engine->delay_ready = true;
}

/* The effect block in force: the type and its twelve parameters, from
   whichever patch the source selector names. The output bytes follow the
   same selector and are read by `efx_refresh`. */
void efx_block_refresh(struct Engine *engine)
{
  bool performance = true;
  unsigned image = 0;
  if (!efx_resolve_source(engine->common[kEfxSourceField], &performance,
                           &image))
    return;                      /* past the selector's range: leave it */
  const uint8_t *block;
  unsigned first;
  if (performance) {
    block = engine->common;
    first = kEfxTypeField;
  } else {
    if (image >= kParts)
      return;
    block = engine->parts[image].common;
    first = kEfxTypeField - kEfxPatchBlockShift;
    if (first + kEfxParameters >= XP_JV1080_PATCH_COMMON_FIELDS)
      return;
  }
  uint8_t type = block[first];
  if (type != engine->efx_type) {
    /* A parameter means something different under a different effect, so
       the type carries the parameters away with it. */
    engine->efx_type = type;
    std::memset(engine->efx_parameter, 0, sizeof engine->efx_parameter);
    engine->efx_dirty = true;
  }
  for (unsigned i = 0; i < kEfxParameters; ++i) {
    uint8_t v = block[first + 1u + i];
    if (v != engine->efx_parameter[i]) {
      engine->efx_parameter[i] = v;
      engine->efx_dirty = true;
    }
  }
}

/* One table entry as a fraction of unity, 0 where the table has no such
   row - the same 8192 unity the rest of this device's coefficients use. */
double efx_unit(const struct Engine *engine, unsigned table, unsigned index,
                 unsigned column)
{
  uint16_t v = 0;
  if (!efx_table_value(&engine->rom, table, index, column, &v))
    return 0.0;
  return (double)v / 8192.0;
}

/* Both delay lines, at least `need` samples long.

   The three earlier types each allocated once and only when the pointer
   was null, which silently kept a SHORTER line when the patch changed to a
   type that needs a longer one - STEREO-DELAY tops out at 16000 samples
   and TRIPLE-TAP at 32000, and `efx_tap` clamps rather than complains, so
   the taps would have come back quietly wrong. Growing is free here: a
   type change discards the line's contents anyway. */
bool efx_buf_ensure(struct Engine *engine, size_t need)
{
  if (engine->efx_buf[0] && engine->efx_buf[1] && engine->efx_len >= need)
    return true;
  for (unsigned c = 0; c < 2u; ++c) {
    std::free(engine->efx_buf[c]);
    engine->efx_buf[c] =
      (float *)std::calloc(need, sizeof **engine->efx_buf);
  }
  engine->efx_len = need;
  engine->efx_pos = 0;
  engine->efx_damp_state[0] = 0.0f;
  engine->efx_damp_state[1] = 0.0f;
  return engine->efx_buf[0] && engine->efx_buf[1];
}

/* STEREO-DELAY. Two lines, each fed by the input plus the feedback from
   the other - or from itself where Mode says NORMAL. */
void efx_stereo_delay_refresh(struct Engine *engine)
{
  const uint8_t *p = engine->efx_parameter;
  double scale = engine->output_rate / kXpNativeRate;
  {
    unsigned n = 0;
    efx_table_shape(&engine->rom, kEfxDelayTable, &n, NULL);
    uint16_t top = 0;
    if (n)
      efx_table_value(&engine->rom, kEfxDelayTable, n - 1u, 0, &top);
    if (!efx_buf_ensure(engine, (size_t)((double)top * scale) + 8u))
      return;
  }
  for (unsigned c = 0; c < 2u; ++c) {
    uint16_t samples = 0;
    efx_table_value(&engine->rom, kEfxDelayTable, p[1u + c], 0, &samples);
    engine->efx_delay[c] = (double)samples * scale;
    /* Phase: NORMAL or INVERT, one bit each. */
    engine->efx_phase[c] = p[3u + c] ? -1.0f : 1.0f;
  }
  /* Bipolar, raw zero 49, 2 % a step. */
  engine->efx_feedback =
    (float)(((double)p[5] - kEfxFeedbackZero) / kEfxFeedbackStep);
  engine->efx_cross = p[0] == 0u;
  /* The damping one-pole's pole, the same 18-row table the reverb reads;
     its last row is a bypass rather than a pair. */
  {
    unsigned row = p[6];
    unsigned rows = 0;
    efx_table_shape(&engine->rom, kEfxDampTable, &rows, NULL);
    double a = row + 1u < rows ? efx_unit(engine, kEfxDampTable, row, 0)
                                : 0.0;
    engine->efx_damp = (float)(a > 0.0 && a < 1.0 ? 1.0 - a : 0.0);
  }
  engine->efx_wet = (float)efx_unit(engine, kEfxBalanceTable, p[9], 0);
  engine->efx_dry = (float)efx_unit(engine, kEfxBalanceTable, p[9], 1);
  engine->efx_level = (float)efx_unit(engine, kEfxLevelTableIndex, p[10], 0);
  engine->efx_ready = engine->efx_buf[0] && engine->efx_buf[1];
}

float efx_tap(const struct Engine *engine, unsigned channel, double back)
{
  if (back < 1.0)
    back = 1.0;
  if (back > (double)(engine->efx_len - 2u))
    back = (double)(engine->efx_len - 2u);
  double read = (double)engine->efx_pos - back;
  while (read < 0.0)
    read += (double)engine->efx_len;
  size_t i0 = (size_t)read;
  double frac = read - (double)i0;
  size_t i1 = i0 + 1u >= engine->efx_len ? 0u : i0 + 1u;
  const float *b = engine->efx_buf[channel];
  return (float)((1.0 - frac) * b[i0] + frac * b[i1]);
}

/* The insert, in place: `stereo` holds the dry the effect was fed, and is
   replaced by the balance of that dry against the effect's own output. */
void efx_process(struct Engine *engine, const float *inL, const float *inR,
                  float *wetL, float *wetR, size_t frames)
{
  for (size_t k = 0; k < frames; ++k) {
    float l = efx_tap(engine, 0, engine->efx_delay[0]);
    float r = efx_tap(engine, 1, engine->efx_delay[1]);
    /* the damping one-pole, on the way round the loop */
    for (unsigned c = 0; c < 2u; ++c) {
      float x = c ? r : l;
      engine->efx_damp_state[c] = x * (1.0f - engine->efx_damp) +
        engine->efx_damp_state[c] * engine->efx_damp;
    }
    float dl = engine->efx_damp_state[0];
    float dr = engine->efx_damp_state[1];
    /* CROSS is the measured default shape: each line is fed by the OTHER
       one's output, which is what puts the cepstrum's series on the even
       multiples of the delay. */
    engine->efx_buf[0][engine->efx_pos] =
      inL[k] + engine->efx_feedback * (engine->efx_cross ? dr : dl);
    engine->efx_buf[1][engine->efx_pos] =
      inR[k] + engine->efx_feedback * (engine->efx_cross ? dl : dr);
    if (++engine->efx_pos >= engine->efx_len)
      engine->efx_pos = 0;
    wetL[k] = engine->efx_phase[0] * l;
    wetR[k] = engine->efx_phase[1] * r;
  }
}

/* TRIPLE-TAP: one line, three taps. The centre tap is shared between the
   two sides and the other two take one side each, which is what the labels
   name them - a tap called L is on the left. */
void efx_triple_tap_refresh(struct Engine *engine)
{
  const uint8_t *p = engine->efx_parameter;
  double scale = engine->output_rate / kXpNativeRate;
  {
    unsigned n = 0;
    efx_table_shape(&engine->rom, kEfxLongDelayTable, &n, NULL);
    uint16_t top = 0;
    if (n)
      efx_table_value(&engine->rom, kEfxLongDelayTable, n - 1u, 0, &top);
    if (!efx_buf_ensure(engine, (size_t)((double)top * scale) + 8u))
      return;
  }
  engine->efx_taps = 3u;
  /* centre, left, right - the label order, which for this type IS the
     stored order */
  static const float kLeft[3] = { 0.5f, 1.0f, 0.0f };
  static const float kRight[3] = { 0.5f, 0.0f, 1.0f };
  for (unsigned t = 0; t < 3u; ++t) {
    uint16_t samples = 0;
    efx_table_value(&engine->rom, kEfxLongDelayTable, p[t], 0, &samples);
    engine->efx_tap_delay[t] = (double)samples * scale;
    double lvl = efx_unit(engine, kEfxLevelTableIndex, p[4u + t], 0);
    engine->efx_tap_left[t] = (float)(lvl * kLeft[t]);
    engine->efx_tap_right[t] = (float)(lvl * kRight[t]);
  }
  engine->efx_feedback =
    (float)(((double)p[3] - kEfxFeedbackZero) / kEfxFeedbackStep);
  {
    unsigned row = p[7];
    unsigned rows = 0;
    efx_table_shape(&engine->rom, kEfxDampTable, &rows, NULL);
    double a = row + 1u < rows ? efx_unit(engine, kEfxDampTable, row, 0) : 0.0;
    engine->efx_damp = (float)(a > 0.0 && a < 1.0 ? 1.0 - a : 0.0);
  }
  engine->efx_wet = (float)efx_unit(engine, kEfxBalanceTable, p[10], 0);
  engine->efx_dry = (float)efx_unit(engine, kEfxBalanceTable, p[10], 1);
  engine->efx_level = (float)efx_unit(engine, kEfxLevelTableIndex, p[11], 0);
  engine->efx_ready = engine->efx_buf[0] && engine->efx_buf[1];
}

void efx_tap_process(struct Engine *engine, const float *inL,
                      const float *inR, float *wetL, float *wetR,
                      size_t frames)
{
  for (size_t k = 0; k < frames; ++k) {
    float l = 0.0f, r = 0.0f, fb = 0.0f;
    for (unsigned t = 0; t < engine->efx_taps; ++t) {
      float v = efx_tap(engine, 0, engine->efx_tap_delay[t]);
      l += engine->efx_tap_left[t] * v;
      r += engine->efx_tap_right[t] * v;
      fb += v;
    }
    fb /= (float)engine->efx_taps;
    engine->efx_damp_state[0] = fb * (1.0f - engine->efx_damp) +
      engine->efx_damp_state[0] * engine->efx_damp;
    float in = 0.5f * (inL[k] + inR[k]);
    engine->efx_buf[0][engine->efx_pos] =
      in + engine->efx_feedback * engine->efx_damp_state[0];
    if (++engine->efx_pos >= engine->efx_len)
      engine->efx_pos = 0;
    wetL[k] = l;
    wetR[k] = r;
  }
}

/* TIME-CONTROL: one line, one tap, settled value only. */
void efx_time_control_refresh(struct Engine *engine)
{
  const uint8_t *p = engine->efx_parameter;
  double scale = engine->output_rate / kXpNativeRate;
  {
    unsigned n = 0;
    efx_table_shape(&engine->rom, kEfxAccelDelayTable, &n, NULL);
    uint16_t top = 0;
    if (n)
      efx_table_value(&engine->rom, kEfxAccelDelayTable, n - 1u, 0, &top);
    if (!efx_buf_ensure(engine, (size_t)((double)top * scale) + 8u))
      return;
  }
  engine->efx_taps = 1u;
  uint16_t samples = 0;
  efx_table_value(&engine->rom, kEfxAccelDelayTable, p[0], 0, &samples);
  engine->efx_tap_delay[0] =
    (double)samples * kEfxTimeControlScale * scale;
  /* Pan places the single tap in the image; centre is the field's middle. */
  double pan = (double)p[3] / 127.0;
  engine->efx_tap_left[0] = (float)(1.0 - pan);
  engine->efx_tap_right[0] = (float)pan;
  engine->efx_feedback =
    (float)(((double)p[2] - kEfxFeedbackZero) / kEfxFeedbackStep);
  {
    unsigned row = p[4];
    unsigned rows = 0;
    efx_table_shape(&engine->rom, kEfxDampTable, &rows, NULL);
    double a = row + 1u < rows ? efx_unit(engine, kEfxDampTable, row, 0) : 0.0;
    engine->efx_damp = (float)(a > 0.0 && a < 1.0 ? 1.0 - a : 0.0);
  }
  engine->efx_wet = (float)efx_unit(engine, kEfxBalanceTable, p[7], 0);
  engine->efx_dry = (float)efx_unit(engine, kEfxBalanceTable, p[7], 1);
  engine->efx_level = (float)efx_unit(engine, kEfxLevelTableIndex, p[8], 0);
  engine->efx_ready = engine->efx_buf[0] && engine->efx_buf[1];
}

/* The sweep shape, 0 at one end and 1 at the other.

   A SYMMETRIC triangle: 0 at the bottom, 1 at the top, equal slopes.

   The symmetry is measured, not assumed. Wet-only, this modulator makes the
   carrier's frequency deviation a SQUARE wave - peak/rms 1.030 to 1.055
   against 1.000 for an ideal square and 1.414 for a sinusoid, third
   harmonic 0.26 to 0.37 against 1/3 and fifth 0.18 to 0.24 against 1/5 -
   with a duty of 0.484 to 0.502. A sawtooth would put the duty far from a
   half and a strong second harmonic in the deviation; neither is there. */
float efx_mod_shape(unsigned shape, double phase)
{
  phase -= std::floor(phase);
  if (shape == kEfxModParabola) {
    /* zero at the middle of the cycle, full sweep at its ends, so the
       delay runs upward from the nominal and its VELOCITY ramps linearly
       and resets - which is the sawtooth deviation that is measured. */
    double u = 2.0 * phase - 1.0;
    return (float)(u * u);
  }
  return (float)(phase < 0.5 ? 2.0 * phase : 2.0 * (1.0 - phase));
}

/* MODULATED DELAY, for type 14 STEREO-CHORUS.

   One tap per channel and no feedback, which is measured: at full wet the
   per-channel envelope is flat to 0.52 dB, so nothing in the wet signal
   combs with anything else. The two channels sit near antiphase - an L/R
   correlation of -0.4487 at 523 Hz and -0.1405 at 262 Hz - so the stereo
   width of this effect IS its modulation, and at depth zero it collapses
   to a mono 1.00 ms comb at correlation +1.000.

   WHAT IS NOT BUILT, and why rather than silently: p1 FILTER TYPE and p2
   CUTOFF are unmeasured, and the factory patch has the filter off; p7,
   whose ceiling is 127 and whose factory value is 0, is unidentified.
   Neither is guessed at here. p8 and p9, the low and high gains, are the
   output shelves (THE OUTPUT SHELVES). */
void efx_modulated_refresh(struct Engine *engine)
{
  const uint8_t *p = engine->efx_parameter;
  const struct EfxModSpec *spec = efx_mod_spec(engine->efx_type);
  if (!spec)
    return;
  double scale = engine->output_rate / kXpNativeRate;
  double sweep_max = spec->sweepMs * engine->output_rate / 1000.0;
  engine->efx_shape = spec->shape;
  {
    unsigned n = 0;
    efx_table_shape(&engine->rom, spec->nominalTable, &n, NULL);
    uint16_t top = 0;
    if (n)
      efx_table_value(&engine->rom, spec->nominalTable, n - 1u, 0, &top);
    /* long enough for the furthest voice at the field's own ceiling, not
       for whatever it is set to now */
    double spread = spec->spread < 0 ? 0.0 :
      127.0 * spec->spreadMs * engine->output_rate / 1000.0;
    if (!efx_buf_ensure(engine,
                         (size_t)((double)top * scale + sweep_max +
                                   spread * (double)(spec->voices - 1u)) + 8u))
      return;
  }

  /* The still delay each channel sweeps from. On the chorus and the flanger
     that is one PRE-DELAY for both, measured at one point exactly: with the
     depth driven to zero the factory p3 of 10 gives a static 1.00 ms comb,
     and entry 10 of `0x038EC8` is 32 samples, 1.00 ms at the 32 kHz wave
     rate. On MODULATION-DELAY it is two independent delay slots. */
  for (unsigned c = 0; c < 2u; ++c) {
    uint16_t pre = 0;
    efx_table_value(&engine->rom, spec->nominalTable, p[spec->nominal[c]], 0,
                     &pre);
    engine->efx_nominal[c] = (double)pre * scale;
  }

  /* DEPTH, the sweep, ONE-SIDED UPWARD from the nominal - which is what
     the depth-zero reading shows, sitting exactly on the pre-delay's own
     value rather than half a sweep above it.

     THE SHAPE IS THE MASTER LEVEL CURVE `0x03856C`, AND THAT IS NOW
     MEASURED RATHER THAN BORROWED. Swept at seven depths on three types -
     14, 15 and 18 - the sweep normalised to its own top reads 0.000,
     0.121, 0.258, 0.418, 0.590, 0.783, 1.000 on all three, against the
     table's own 0.000, 0.121, 0.258, 0.418, 0.590, 0.783, 1.000. Worst
     difference 0.0005 over seven points and three types. Depth 0 gives a
     STATIC delay on every one of them, 0.003 to 0.004 % of the carrier,
     which is the analysis floor and the control that says the slot is the
     depth at all. */
  engine->efx_sweep =
    sweep_max * efx_unit(engine, kEfxLevelTableIndex, p[spec->depth], 0);

  /* p4 RATE: `table * 32000 / 2^24`, measured here at values 20 and 45 -
     1.0490 and 2.2983 Hz against 1.0490 and 2.2984 - and independently on
     the system chorus, which reads the same table through a different DSP
     program and gives the same constant (`M-110`). */
  {
    uint16_t raw = 0;
    efx_table_value(&engine->rom, kEfxLfoRateTable, p[spec->rate], 0,
                     &raw);
    double hz = (double)raw * kXpNativeRate / 16777216.0;
    engine->efx_lfo_step = hz / engine->output_rate;
  }
  /* The LFO FREE-RUNS across parameter writes; it is deliberately not reset
     here. Measured: on a slot that rewrites all twelve parameters and then
     holds a note for 1.2 s at the factory rate of 0.1487 Hz - a fifth of a
     cycle - the hardware's frequency deviation still VARIES across the
     window, 0.352 % of the carrier, because a turning point falls inside
     it. Restarting the sweep at each write makes the deviation constant
     over so short a window, and ours read 0.006 % until this was removed. */

  /* p6 PHASE, off the measured table (see above). */
  engine->efx_lfo_offset =
    spec->phase < 0 ? 0.0 : efx_chorus_phase(p[(unsigned)spec->phase]);
  engine->efx_invert_right = spec->invertRight;
  engine->efx_voices = spec->voices;
  engine->efx_spread = spec->spread < 0 ? 0.0 :
    (double)p[(unsigned)spec->spread] * spec->spreadMs *
    engine->output_rate / 1000.0;
  {
    double sum = 0.0;
    for (unsigned v = 0; v < spec->voices && v < 4u; ++v)
      sum += spec->weight[v];
    for (unsigned v = 0; v < 4u; ++v)
      engine->efx_weight[v] = (float)
        (v < spec->voices && sum > 0.0 ? spec->weight[v] / sum : 0.0);
  }

  /* FEEDBACK, where the type has one. The bipolar zero of 49 and the 2 %
     a step are the pure-delay family's own measured law (`M-101`); that
     this family's field reads the same way is the structural parallel and
     not a separate measurement, which is why the depth and phase sweeps
     put it AT that zero rather than trusting it. */
  engine->efx_feedback = spec->feedback < 0 ? 0.0f : (float)
    (((double)p[spec->feedback] - kEfxFeedbackZero) / kEfxFeedbackStep);

  /* HF damp in the feedback path, the same 18-row table the pure-delay
     family and the reverb read; its last row is a bypass rather than a
     pair. A type without the field damps nothing, and a damp of 0 makes
     the one-pole below the identity. */
  engine->efx_damp = 0.0f;
  if (spec->damp >= 0) {
    unsigned row = p[spec->damp];
    unsigned rows = 0;
    efx_table_shape(&engine->rom, kEfxDampTable, &rows, NULL);
    double a = row + 1u < rows ? efx_unit(engine, kEfxDampTable, row, 0)
                                : 0.0;
    engine->efx_damp = (float)(a > 0.0 && a < 1.0 ? 1.0 - a : 0.0);
  }

  /* p10 BALANCE, measured end to end: 0 is fully DRY (envelope flat to
     1.01 dB, L/R correlation +1.0000), 100 is fully WET, and the comb is
     deepest and symmetric at 50, which is what makes it a balance rather
     than a wet level. p11 LEVEL, measured: driving it to 0 silences the
     effect. */
  /* The amplitude modulator, where the type has one. Its envelope falls
     from unity by the depth, which the level curve scales. */
  engine->efx_trem_depth = 0.0f;
  engine->efx_trem_step = 0.0;
  if (spec->tremRate >= 0 && spec->tremDepth >= 0) {
    uint16_t raw = 0;
    efx_table_value(&engine->rom, kEfxLfoRateTable, p[spec->tremRate], 0,
                     &raw);
    engine->efx_trem_step =
      (double)raw * kXpNativeRate / 16777216.0 / engine->output_rate;
    engine->efx_trem_depth =
      (float)efx_unit(engine, kEfxLevelTableIndex, p[spec->tremDepth], 0);
  }

  engine->efx_wet =
    (float)efx_unit(engine, kEfxBalanceTable, p[spec->balance], 0);
  engine->efx_dry =
    (float)efx_unit(engine, kEfxBalanceTable, p[spec->balance], 1);
  engine->efx_level =
    (float)efx_unit(engine, kEfxLevelTableIndex, p[spec->level], 0);
  engine->efx_ready = engine->efx_buf[0] && engine->efx_buf[1];
}

void efx_mod_process(struct Engine *engine, const float *inL,
                      const float *inR, float *wetL, float *wetR,
                      size_t frames)
{
  for (size_t k = 0; k < frames; ++k) {
    /* Read before write, the same order the pure-delay family uses, so the
       two agree about what a delay of one sample means. */
    double ml = engine->efx_sweep *
      efx_mod_shape(engine->efx_shape, engine->efx_lfo_phase);
    double mr = engine->efx_sweep *
      efx_mod_shape(engine->efx_shape,
                     engine->efx_lfo_phase + engine->efx_lfo_offset);
    float l = 0.0f, r = 0.0f;
    for (unsigned v = 0; v < engine->efx_voices; ++v) {
      double off = (double)v * engine->efx_spread;
      l += engine->efx_weight[v] *
        efx_tap(engine, 0, engine->efx_nominal[0] + off + ml);
      r += engine->efx_weight[v] *
        efx_tap(engine, 1, engine->efx_nominal[1] + off + mr);
    }
    /* the damping one-pole, on the way round the loop; at damp 0 it is the
       identity and the two feedback-less types are unaffected */
    for (unsigned c = 0; c < 2u; ++c) {
      float y = c ? r : l;
      engine->efx_damp_state[c] = y * (1.0f - engine->efx_damp) +
        engine->efx_damp_state[c] * engine->efx_damp;
    }
    engine->efx_buf[0][engine->efx_pos] =
      inL[k] + engine->efx_feedback * engine->efx_damp_state[0];
    engine->efx_buf[1][engine->efx_pos] =
      inR[k] + engine->efx_feedback * engine->efx_damp_state[1];
    if (++engine->efx_pos >= engine->efx_len)
      engine->efx_pos = 0;
    if (engine->efx_trem_depth > 0.0f) {
      /* on the effect's own path, which is what TREMOLO-CHORUS names */
      float g = 1.0f - engine->efx_trem_depth *
        efx_mod_shape(kEfxModTriangle, engine->efx_trem_phase);
      l *= g;
      r *= g;
      engine->efx_trem_phase += engine->efx_trem_step;
      if (engine->efx_trem_phase >= 1.0)
        engine->efx_trem_phase -= 1.0;
    }
    wetL[k] = l;
    /* SPACE-D puts ONE delay in both channels and negates the right. */
    wetR[k] = engine->efx_invert_right ? -l : r;
    engine->efx_lfo_phase += engine->efx_lfo_step;
    if (engine->efx_lfo_phase >= 1.0)
      engine->efx_lfo_phase -= 1.0;
  }
}

void insert_reverb_refresh(struct Engine *engine)
{
  const struct InsertReverbSpec *spec =
    insert_reverb_spec(engine->efx_type);
  if (!spec)
    return;
  const uint8_t *p = engine->efx_parameter;
  double scale = engine->output_rate / kXpNativeRate;

  struct xp_efx_program prog;
  if (!efx_program_load(&engine->rom, engine->efx_type, &prog))
    return;
  struct xp_efx_site site[128];
  unsigned n = efx_program_sites(&prog, site, 128u);
  if (n > 128u)
    n = 128u;

  size_t lag[kInsertReverbMaxTaps];
  unsigned taps = 0, longest = 0;
  for (unsigned i = 0; i < n && taps < kInsertReverbMaxTaps; ++i) {
    if (site[i].write)
      continue;
    unsigned best = kXpEramSpace;
    for (unsigned j = 0; j < n; ++j) {
      if (!site[j].write)
        continue;
      unsigned d = (unsigned)((site[i].address - site[j].address) &
                               (kXpEramSpace - 1u));
      if (d && d < best)
        best = d;
    }
    if (best >= kXpEramSpace)
      continue;
    size_t L = (size_t)((double)best * scale);
    if (!L)
      L = 1u;
    lag[taps++] = L;
    if (L > longest)
      longest = (unsigned)L;
  }
  if (!taps)
    return;
  engine->rv_taps = taps;
  engine->rv_gated = spec->gate >= 0;

  /* The pre-delay, on `0x038EC8`, ahead of the network. */
  {
    uint16_t pre = 0;
    efx_table_value(&engine->rom, kEfxPreDelayTable, p[spec->preDelay], 0,
                     &pre);
    size_t want = (size_t)((double)pre * scale) + 8u;
    if (!engine->rv_pre_buf || engine->rv_pre_len < want) {
      std::free(engine->rv_pre_buf);
      engine->rv_pre_buf = (float *)std::calloc(want,
                                                 sizeof *engine->rv_pre_buf);
      engine->rv_pre_len = want;
      engine->rv_pre_pos = 0;
    }
    engine->rv_pre = engine->rv_pre_buf ? want - 8u : 0u;
  }

  /* HF damp, the 18-row table this engine reads everywhere; last row is a
     bypass. */
  engine->rv_damp = 0.0f;
  if (spec->damp) {
    unsigned row = p[spec->damp];
    unsigned rows = 0;
    efx_table_shape(&engine->rom, kEfxDampTable, &rows, NULL);
    double a2 = row + 1u < rows ? efx_unit(engine, kEfxDampTable, row, 0)
                                 : 0.0;
    engine->rv_damp = (float)(a2 > 0.0 && a2 < 1.0 ? 1.0 - a2 : 0.0);
  }

  if (engine->rv_gated) {
    engine->rv_gate =
      (size_t)((5.0 + 5.0 * (double)p[(unsigned)spec->gate]) *
                engine->output_rate / 1000.0);
    size_t need = (size_t)longest + 8u;
    if (!engine->rv_buf || engine->rv_len < need) {
      std::free(engine->rv_buf);
      engine->rv_buf = (float *)std::calloc(need, sizeof *engine->rv_buf);
      engine->rv_len = need;
      engine->rv_pos = 0;
    }
    if (!engine->rv_buf)
      return;
    for (unsigned i = 0; i < taps; ++i)
      engine->rv_lag[i] = lag[i];
    engine->rv_since = engine->rv_gate;
    engine->rv_env = 0.0f;
  } else {
    /* ONE COMB PER TAP, each gained from its OWN length. A single loop
       carrying every length has no single decay; a comb of length L does,
       and `g = 10^(-3L/(RT60*fs))` is exactly it. */
    double rt = spec->rt60Base *
      std::pow(2.0, (double)p[spec->time] / spec->rt60Steps);
    for (unsigned i = 0; i < taps; ++i) {
      size_t need = lag[i] + 8u;
      if (!engine->rv_comb[i] || engine->rv_comb_len[i] < need) {
        std::free(engine->rv_comb[i]);
        engine->rv_comb[i] = (float *)std::calloc(need,
                                                   sizeof **engine->rv_comb);
        engine->rv_comb_len[i] = need;
        engine->rv_comb_pos[i] = 0;
      }
      if (!engine->rv_comb[i])
        return;
      engine->rv_lag[i] = lag[i];
      double g = std::pow(10.0, -3.0 * (double)lag[i] /
                                 (rt * engine->output_rate));
      if (g > 0.999)
        g = 0.999;
      engine->rv_comb_g[i] = (float)g;
      engine->rv_comb_damp[i] = 0.0f;
    }
  }

  engine->efx_wet =
    (float)efx_unit(engine, kEfxBalanceTable, p[spec->balance], 0);
  engine->efx_dry =
    (float)efx_unit(engine, kEfxBalanceTable, p[spec->balance], 1);
  engine->efx_level =
    (float)efx_unit(engine, kEfxLevelTableIndex, p[spec->level], 0);
  engine->efx_ready = true;
}

void insert_reverb_process(struct Engine *engine, const float *inL,
                            const float *inR, float *wetL, float *wetR,
                            size_t frames)
{
  float norm = 1.0f / (float)engine->rv_taps;
  for (size_t k = 0; k < frames; ++k) {
    float in = 0.5f * (inL[k] + inR[k]);
    if (engine->rv_pre_buf && engine->rv_pre) {
      size_t j = engine->rv_pre_pos >= engine->rv_pre
        ? engine->rv_pre_pos - engine->rv_pre
        : engine->rv_pre_pos + engine->rv_pre_len - engine->rv_pre;
      float d = engine->rv_pre_buf[j];
      engine->rv_pre_buf[engine->rv_pre_pos] = in;
      if (++engine->rv_pre_pos >= engine->rv_pre_len)
        engine->rv_pre_pos = 0;
      in = d;
    }
    float sum = 0.0f, alt = 0.0f, g = norm;
    if (engine->rv_gated) {
      float mag = in < 0.0f ? -in : in;
      engine->rv_env = mag > engine->rv_env ? mag
                                             : engine->rv_env * 0.9995f;
      if (mag > 1e-5f && mag > 0.02f * engine->rv_env)
        engine->rv_since = 0;
      else if (engine->rv_since < engine->rv_gate)
        ++engine->rv_since;
      for (unsigned i = 0; i < engine->rv_taps; ++i) {
        size_t back = engine->rv_lag[i];
        size_t j = engine->rv_pos >= back ? engine->rv_pos - back
                                           : engine->rv_pos + engine->rv_len
                                             - back;
        float v = engine->rv_buf[j];
        sum += v;
        alt += (i & 1u) ? -v : v;
      }
      engine->rv_buf[engine->rv_pos] = in;
      if (++engine->rv_pos >= engine->rv_len)
        engine->rv_pos = 0;
      if (engine->rv_since >= engine->rv_gate)
        g = 0.0f;
    } else {
      for (unsigned i = 0; i < engine->rv_taps; ++i) {
        size_t back = engine->rv_lag[i];
        size_t pos = engine->rv_comb_pos[i];
        size_t len = engine->rv_comb_len[i];
        size_t j = pos >= back ? pos - back : pos + len - back;
        float v = engine->rv_comb[i][j];
        engine->rv_comb_damp[i] = v * (1.0f - engine->rv_damp) +
          engine->rv_comb_damp[i] * engine->rv_damp;
        engine->rv_comb[i][pos] =
          in + engine->rv_comb_g[i] * engine->rv_comb_damp[i];
        engine->rv_comb_pos[i] = pos + 1u >= len ? 0u : pos + 1u;
        sum += v;
        alt += (i & 1u) ? -v : v;
      }
    }
    wetL[k] = g * 0.5f * (sum + alt);
    wetR[k] = g * 0.5f * (sum - alt);
  }
}

/* OVERDRIVE and DISTORTION. Level and Pan are applied inside the effect,
   as the firmware applies them in its own registers, so the return is
   passed through at unity.

   A PARAMETER PAST ITS RANGE KEEPS ITS PREVIOUS VALUE. The machine
   discards such a write (`M-091`), and discards it per parameter:
   `efx_params/efx02_overdrive` rewrites all twelve bytes per slot with one
   driven to 127, and AmpType, LowGain and HiGain at 127 change nothing in
   the take while the same write's Pan, back at 64, does return to centre.
   A byte with no previous value to keep leaves the effect unbuilt. */
void efx_drive_refresh(struct Engine *engine)
{
  unsigned row = 0;
  if (!efx_drive_type(engine->efx_type, &row))
    return;
  bool previous = engine->efx_drive.ready && engine->efx_drive.row == row;
  uint8_t p[kEfxDriveParameters];
  for (unsigned i = 0; i < kEfxDriveParameters; ++i) {
    p[i] = engine->efx_parameter[i];
    if (!drive_parameter_valid(&engine->rom, i, p[i])) {
      if (!previous)
        return;
      p[i] = engine->efx_drive.param[i];
    }
  }
  if (!drive_set(&engine->rom, &engine->efx_drive, row, p))
    return;
  engine->efx_wet = 1.0f;
  engine->efx_dry = 0.0f;
  engine->efx_level = 1.0f;
  engine->efx_ready = true;
}

/* STEREO-EQ, with the drive family's per-byte discard: a byte past its
   range keeps its previous value, and with none to keep the effect stays
   unbuilt. Level is applied inside, in the firmware's register. */
void efx_stereo_eq_refresh(struct Engine *engine)
{
  bool previous = engine->efx_eq.ready;
  uint8_t p[XP_STEREO_EQ_PARAMETERS];
  for (unsigned i = 0; i < XP_STEREO_EQ_PARAMETERS; ++i) {
    p[i] = engine->efx_parameter[i];
    if (!stereo_eq_parameter_valid(i, p[i])) {
      if (!previous)
        return;
      p[i] = engine->efx_eq.param[i];
    }
  }
  if (!stereo_eq_set(&engine->rom, &engine->efx_eq, p))
    return;
  engine->efx_wet = 1.0f;
  engine->efx_dry = 0.0f;
  engine->efx_level = 1.0f;
  engine->efx_ready = true;
}

/* SPECTRUM and ENHANCER, with the same per-byte discard. Pan and Level
   are applied inside, in the firmware's registers. */
void efx_spectrum_refresh(struct Engine *engine)
{
  bool previous = engine->efx_spectrum.ready;
  uint8_t p[XP_SPECTRUM_PARAMETERS];
  for (unsigned i = 0; i < XP_SPECTRUM_PARAMETERS; ++i) {
    p[i] = engine->efx_parameter[i];
    if (!spectrum_parameter_valid(i, p[i])) {
      if (!previous)
        return;
      p[i] = engine->efx_spectrum.param[i];
    }
  }
  if (!spectrum_set(&engine->rom, &engine->efx_spectrum, p))
    return;
  engine->efx_wet = 1.0f;
  engine->efx_dry = 0.0f;
  engine->efx_level = 1.0f;
  engine->efx_ready = true;
}

void efx_enhancer_refresh(struct Engine *engine)
{
  bool previous = engine->efx_enhancer.ready;
  uint8_t p[XP_ENHANCER_PARAMETERS];
  for (unsigned i = 0; i < XP_ENHANCER_PARAMETERS; ++i) {
    p[i] = engine->efx_parameter[i];
    if (!enhancer_parameter_valid(i, p[i])) {
      if (!previous)
        return;
      p[i] = engine->efx_enhancer.param[i];
    }
  }
  if (!enhancer_set(&engine->rom, &engine->efx_enhancer, p))
    return;
  engine->efx_wet = 1.0f;
  engine->efx_dry = 0.0f;
  engine->efx_level = 1.0f;
  engine->efx_ready = true;
}

void efx_phaser_refresh(struct Engine *engine)
{
  bool previous = engine->efx_phaser.ready;
  uint8_t p[XP_PHASER_PARAMETERS];
  for (unsigned i = 0; i < XP_PHASER_PARAMETERS; ++i) {
    p[i] = engine->efx_parameter[i];
    if (!phaser_parameter_valid(i, p[i])) {
      if (!previous)
        return;
      p[i] = engine->efx_phaser.param[i];
    }
  }
  if (!phaser_set(&engine->rom, &engine->efx_phaser, p))
    return;
  engine->efx_wet = 1.0f;
  engine->efx_dry = 0.0f;
  engine->efx_level = 1.0f;
  engine->efx_ready = true;
}

/* ROTARY, with the same per-byte discard. Level is applied inside, in the
   firmware's register. Byte 11 is not read by the updater. */
void efx_rotary_refresh(struct Engine *engine)
{
  bool previous = engine->efx_rotary.ready;
  uint8_t p[XP_ROTARY_PARAMETERS];
  for (unsigned i = 0; i < XP_ROTARY_PARAMETERS; ++i) {
    p[i] = engine->efx_parameter[i];
    if (!rotary_parameter_valid(i, p[i])) {
      if (!previous)
        return;
      p[i] = engine->efx_rotary.param[i];
    }
  }
  if (!rotary_set(&engine->rom, &engine->efx_rotary, p))
    return;
  engine->efx_wet = 1.0f;
  engine->efx_dry = 0.0f;
  engine->efx_level = 1.0f;
  engine->efx_ready = true;
}

/* COMPRESSOR and LIMITER, with the same per-byte discard. Pan and Level
   are applied inside, in the firmware's registers. */
void efx_dynamics_refresh(struct Engine *engine)
{
  bool limiter = engine->efx_type == kEfxTypeLimiter;
  bool previous = engine->efx_dynamics.ready &&
    engine->efx_dynamics.limiter == limiter;
  unsigned n = limiter ? XP_LIMITER_PARAMETERS : XP_COMPRESSOR_PARAMETERS;
  uint8_t p[XP_DYNAMICS_PARAMETERS];
  for (unsigned i = 0; i < n; ++i) {
    p[i] = engine->efx_parameter[i];
    if (!dynamics_parameter_valid(&engine->rom, limiter, i, p[i])) {
      if (!previous)
        return;
      p[i] = engine->efx_dynamics.param[i];
    }
  }
  if (!dynamics_set(&engine->rom, &engine->efx_dynamics, limiter, p))
    return;
  engine->efx_wet = 1.0f;
  engine->efx_dry = 0.0f;
  engine->efx_level = 1.0f;
  engine->efx_ready = true;
}

/* The output shelves for the type in force, or none. */
void efx_shelf_refresh(struct Engine *engine)
{
  const struct EfxShelfSpec *spec = efx_shelf_spec(engine->efx_type);
  bool same = engine->efx_shelf_on &&
    engine->efx_shelf_type == engine->efx_type;
  if (!same) {
    std::memset(engine->efx_shelf, 0, sizeof engine->efx_shelf);
    engine->efx_shelf_on = false;
  }
  if (!spec)
    return;
  const unsigned at[2] = { spec->low, spec->high };
  for (unsigned k = 0; k < 2u; ++k) {
    uint8_t g = engine->efx_parameter[at[k]];
    if (!stereo_eq_shelf_set(&engine->rom, k == 1u, k == 0u, g,
                             &engine->efx_shelf[k])) {
      g = same ? engine->efx_shelf_gain[k] : (uint8_t)kEfxShelfFlat;
      if (!stereo_eq_shelf_set(&engine->rom, k == 1u, k == 0u, g,
                               &engine->efx_shelf[k])) {
        engine->efx_shelf_on = false;
        return;
      }
    }
    engine->efx_shelf_gain[k] = g;
  }
  engine->efx_shelf_type = engine->efx_type;
  engine->efx_shelf_on = true;
}

void efx_algorithm_refresh(struct Engine *engine)
{
  engine->efx_ready = false;
  efx_shelf_refresh(engine);
  if (engine->efx_type != kEfxTypePhaser)
    std::memset(&engine->efx_phaser, 0, sizeof engine->efx_phaser);
  if (engine->efx_type == kEfxTypePhaser) {
    efx_phaser_refresh(engine);
    return;
  }
  if (engine->efx_type != kEfxTypeRotary)
    std::memset(&engine->efx_rotary, 0, sizeof engine->efx_rotary);
  if (engine->efx_type == kEfxTypeRotary) {
    efx_rotary_refresh(engine);
    return;
  }
  if (engine->efx_type != kEfxTypeCompressor &&
      engine->efx_type != kEfxTypeLimiter)
    std::memset(&engine->efx_dynamics, 0, sizeof engine->efx_dynamics);
  if (engine->efx_type == kEfxTypeCompressor ||
      engine->efx_type == kEfxTypeLimiter) {
    efx_dynamics_refresh(engine);
    return;
  }
  if (engine->efx_type != kEfxTypeSpectrum)
    std::memset(&engine->efx_spectrum, 0, sizeof engine->efx_spectrum);
  if (engine->efx_type != kEfxTypeEnhancer)
    std::memset(&engine->efx_enhancer, 0, sizeof engine->efx_enhancer);
  if (engine->efx_type == kEfxTypeSpectrum) {
    efx_spectrum_refresh(engine);
    return;
  }
  if (engine->efx_type == kEfxTypeEnhancer) {
    efx_enhancer_refresh(engine);
    return;
  }
  if (engine->efx_type == kEfxTypeStereoEq) {
    efx_stereo_eq_refresh(engine);
    return;
  }
  std::memset(&engine->efx_eq, 0, sizeof engine->efx_eq);
  if (efx_drive_type(engine->efx_type, NULL)) {
    efx_drive_refresh(engine);
    return;
  }
  engine->efx_drive.ready = false;
  if (engine->efx_type == kEfxTypeStereoDelay)
    efx_stereo_delay_refresh(engine);
  else if (engine->efx_type == kEfxTypeTripleTap)
    efx_triple_tap_refresh(engine);
  else if (engine->efx_type == kEfxTypeTimeControl)
    efx_time_control_refresh(engine);
  else if (efx_mod_spec(engine->efx_type))
    efx_modulated_refresh(engine);
  else if (insert_reverb_spec(engine->efx_type))
    insert_reverb_refresh(engine);
}

/* The EFX output block: level, and the two sends the assign may mask out.

   THE BLOCK FOLLOWS THE EFFECT SOURCE, as the type and parameters do. The
   output-block fetcher `0x0A00931C` resolves the selector with the same
   arithmetic as the type/parameter reader `0x0A0094D0` - source 0 reads
   the performance common at `+0x1A`, any other source the selected part's
   patch image at `+0x19`, one byte lower as the rest of a patch's effect
   block is - and hands that pointer to the updater `0x0A00716C`; the
   working-copy fill at `0x0A0097C6` takes the same `+0x19` from the same
   image (FW-EXACT). So with a part as the source, the performance's own
   assign, level and sends are not the ones in force. */
void efx_refresh(struct Engine *engine)
{
  bool performance = true;
  unsigned image = 0;
  if (!efx_resolve_source(engine->common[kEfxSourceField], &performance,
                           &image))
    return;                      /* past the selector's range: leave it */
  const uint8_t *block = engine->common;
  unsigned shift = 0;
  if (!performance) {
    if (image >= kParts)
      return;
    block = engine->parts[image].common;
    shift = kEfxPatchBlockShift;
  }
  unsigned assign = block[kEfxOutputAssignField - shift];
  engine->efx_output_level = (float)
    (efx_output_level(&engine->rom,
                       block[kEfxOutputLevelField - shift]) / 8192.0);
  engine->efx_chorus_send = (float)
    (efx_send_level(&engine->rom, assign,
                     block[kEfxChorusSendField - shift]) / 8192.0);
  engine->efx_reverb_send = (float)
    (efx_send_level(&engine->rom, assign,
                     block[kEfxReverbSendField - shift]) / 8192.0);
}

/* THE EFFECT FOLLOWS A SYSEX WRITE TO ITS SOURCE PATCH. When the selector
   names a part, the type and parameters are that part's patch's, and a
   SysEx write reloads the effect as a performance edit does
   (`08_effects/dsp_program.md`, "Loading sequence"). A song that writes the
   performance common before the patch it names - `1_rise` sends
   `01 00 00 00` with source 12 and only then `02 0C 00 00`, whose type is
   DISTORTION - would otherwise keep whatever that part held when the
   common arrived. Rebuilt only when the block in force actually moved, so
   a write to any other part costs nothing and restarts nothing.

   A program change on the source part takes its own path,
   `efx_follow_program_change`. */
void efx_follow_source(struct Engine *engine)
{
  efx_refresh(engine);
  engine->efx_dirty = false;
  efx_block_refresh(engine);
  if (engine->efx_dirty)
    efx_algorithm_refresh(engine);
}

void reverb_refresh(struct Engine *engine)
{
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  if (!profile->reverbCharacters)
    return;
  unsigned type = engine->common[kReverbTypeField];
  if (type >= profile->reverbCharacters)
    type = 0u;
  if (type == kReverbTypeDelay || type == kReverbTypePanningDelay) {
    if (engine->reverb_ready) {
      reverb_destroy(&engine->reverb);
      engine->reverb_ready = false;
    }
    delay_refresh(engine, type == kReverbTypePanningDelay);
    return;
  }
  engine->delay_ready = false;
  if (!engine->reverb_ready || type != engine->reverb_character) {
    if (engine->reverb_ready)
      reverb_destroy(&engine->reverb);
    engine->reverb_ready = reverb_init(&engine->reverb, &engine->rom,
                                        (uint8_t)type, engine->output_rate);
    engine->reverb_character = (uint8_t)type;
  }
  reverb_apply_params(engine);
}

void part_controls(const struct Engine *engine, unsigned part,
                    struct XpJv1080PartControls *out)
{
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  const struct Part &p = engine->parts[part];
  out->patch_level = p.common[profile->patchFieldLevel];
  out->patch_pan = p.common[profile->patchFieldPan];
  out->part_level = p.part[profile->partFieldLevel];
  out->part_pan = p.part[profile->partFieldPan];
  out->volume = p.volume;
  out->key_shift = (int)(int8_t)p.part[profile->partFieldKeyShift];
  out->fine_tune = profile->partFieldFineTune == XP_VOICE_FIELD_NONE
    ? 0
    : (int)(int8_t)p.part[profile->partFieldFineTune];
  out->tune_cents = 100.0 * p.rpn_coarse + p.rpn_fine;
  out->clock_seconds = (double)engine->frames / engine->output_rate;
  out->lfo_seed = (uint32_t)engine->serial;
  out->tempo_bpm = engine->tempo_bpm;
  out->alternate_phase = p.alternate_next;
  matrix_sources(engine, p, out->matrix_source);
}

/* The part's assign, or the record's where the part defers to it. */
unsigned voice_destination(const struct Engine *engine, unsigned part,
                            const struct XpVoiceFieldMap *fields,
                            const uint8_t *bytes)
{
  const struct XpDeviceProfile *prof = xp_profile(&engine->rom);
  if (prof->partFieldOutputAssign == XP_VOICE_FIELD_NONE)
    return kOutputMix;
  unsigned assign = engine->parts[part].part[prof->partFieldOutputAssign];
  if (assign != prof->partOutputAssignPatch)
    return assign > kOutputTwo ? kOutputMix : assign;
  if (fields->outputAssign == XP_VOICE_FIELD_NONE)
    return kOutputMix;
  unsigned own = bytes[fields->outputAssign];
  return own > kOutputTwo ? kOutputMix : own;
}

/* `key` is the note as received, which a note-off matches; `sounded` is
   the note the record plays, which is the same key but for the patch's
   octave shift. */
struct Voice *start_record(struct Engine *engine, unsigned part,
                   const struct XpVoiceFieldMap *fields, const uint8_t *bytes,
                   unsigned key, unsigned velocity, unsigned sounded)
{
  size_t samples = 0;
  int keyShift = (int)(int8_t)engine->parts[part]
    .part[xp_profile(&engine->rom)->partFieldKeyShift];
  if (!jv1080_voice_span(&engine->rom, fields, bytes, sounded, velocity,
                         keyShift, &samples) || !samples)
    return nullptr;

  /* A KEY-INTERVAL tone with nothing to time against never sounds
     (`M-020`), so it takes no voice. */
  if (fields->toneDelayMode != XP_VOICE_FIELD_NONE &&
      bytes[fields->toneDelayMode] == kDelayKeyInterval &&
      !engine->parts[part].key_on_frame[key])
    return nullptr;

  struct Voice *voice = take_voice(engine);
  if (!voice)
    return nullptr;
  /* The decode is differential, so a sample's value depends on every delta
     before it in the block and the element's span is decoded whole at
     note-on rather than streamed, through the same decoded-PCM cache the
     sibling engine's renderer shares between its voices. */
  struct XpJv1080PartControls controls;
  part_controls(engine, part, &controls);
  if (!jv1080_voice_start(&engine->rom, fields, bytes, &controls, sounded,
                          velocity, engine->banks, engine->bank_sizes,
                          &engine->wave_cache, engine->output_rate,
                          &voice->voice))
    return nullptr;
  voice->wave_cache = &engine->wave_cache;
  voice->serial = ++engine->serial;
  voice->part = (uint8_t)part;
  voice->reverb_send = 0.0f;
  voice->chorus_send = 0.0f;
  voice->destination =
    (uint8_t)voice_destination(engine, part, fields, bytes);
  {
    const struct XpDeviceProfile *prof = xp_profile(&engine->rom);
    if (prof->partFieldReverbSend != XP_VOICE_FIELD_NONE)
      voice->reverb_send =
        (float)engine->parts[part].part[prof->partFieldReverbSend] / 127.0f;
    if (prof->partFieldChorusSend != XP_VOICE_FIELD_NONE)
      voice->chorus_send =
        (float)engine->parts[part].part[prof->partFieldChorusSend] / 127.0f;
  }
  voice->key = (uint8_t)key;
  voice->start_key = (uint8_t)key;
  voice->mute_group = fields->muteGroup == XP_VOICE_FIELD_NONE
    ? 0u : bytes[fields->muteGroup];
  voice->allocated = true;
  voice->key_down = true;
  voice->delay_mode = fields->toneDelayMode == XP_VOICE_FIELD_NONE
    ? kDelayNormal : bytes[fields->toneDelayMode];
  voice->delay = fields->toneDelayTime == XP_VOICE_FIELD_NONE ? 0u
    : (size_t)std::lround(tone_delay_ms(bytes[fields->toneDelayTime]) *
                          engine->output_rate / 1000.0);
  voice->release_in = 0u;
  {
    /* The bender reaches a record whose own switch lets it; a rhythm note
       carries one range for both directions where a tone takes the
       patch's two. RPN 0/0 sets the patch's range (`M-014`: 4 gives
       +-400 cents); whether it reaches a rhythm note is not measured, and
       it is not applied there. */
    const struct XpDeviceProfile *prof = xp_profile(&engine->rom);
    const struct Part &p = engine->parts[part];
    double up = 0.0, down = 0.0;
    bool bends = fields->benderSwitch == XP_VOICE_FIELD_NONE ||
      bytes[fields->benderSwitch] != 0u;
    if (bends && fields->benderRange != XP_VOICE_FIELD_NONE) {
      up = down = (double)bytes[fields->benderRange];
    } else if (bends && p.rpn_bend >= 0) {
      up = down = (double)p.rpn_bend;
    } else if (bends && prof->patchFieldBendUp != XP_VOICE_FIELD_NONE) {
      up = (double)p.common[prof->patchFieldBendUp];
      down = (double)p.common[prof->patchFieldBendDown];
    }
    voice->bend_up = 100.0 * up;
    voice->bend_down = 100.0 * down;
    voice->voice.bend_ratio = bend_ratio(p.bend, voice->bend_up,
                                         voice->bend_down);
    voice->holdable = fields->holdSwitch == XP_VOICE_FIELD_NONE ||
      bytes[fields->holdSwitch] != 0u;
    voice->sustained = false;
  }
  switch (voice->delay_mode) {
  case kDelayNormal:
  case kDelayHold:
    voice->wait = voice->delay;
    break;
  case kDelayKeyOffNormal:
    /* MEASURED (`M-113`): KEY-OFF-NORMAL sounds nothing while the key is
       held and starts a fresh attack at note-off + the delay - four slots
       of two gates and two delays agree to 0.33 dB - then releases the
       moment it reaches its sustain level, which a second envelope with a
       different sustain arrival moved to match. The voice is taken at the
       note-on and waits unrendered. NOT MEASURED, for either KEY-OFF mode:
       whether the machine holds a voice that early, what the hold pedal
       does to them - here it postpones their note-off as it does any
       other - what a new note-on does mid-envelope, and any dependence on
       velocity. An envelope whose times are all zero sounds here for the
       2 ms the attack table holds at value 0 - its measurement floor -
       before the zero-length release ends it, where the hardware's
       all-zero bench take is silent. */
    voice->wait = kWaitForKeyOff;
    voice->voice.release_at_sustain = true;
    break;
  case kDelayKeyOffDecay:
    /* MEASURED (`M-113`): KEY-OFF-DECAY runs its envelope from the note-on
       as if held but unheard, and is heard from note-off + the delay at
       whatever level that envelope has reached, releasing from there: at
       delay 64 on a 0.5 s gate it enters at -39.6 dB, the held envelope's
       level at that moment. NOT MEASURED: where the wave, the pitch and
       filter envelopes and the LFOs stand after the unheard stretch - a
       looped sine cannot show it - so they run on with the envelope here. */
    voice->wait = 0u;
    voice->muted = true;
    break;
  case kDelayKeyInterval:
    /* MEASURED (`M-020`): the panel's PLAY-MATE delays the tone by the
       interval since the previous note-on, exactly, to at least 4 s,
       ignoring the delay time field; a note with no previous note-on never
       sounds (handled before the voice is taken). The tone outlives its
       note-off - M-020's notes are gated 300 ms and still sound 4 s later -
       so the release is postponed by the same interval, NORMAL's rule;
       how long the machine lets it sound is NOT MEASURED. That the tone
       does not start at its note-on is seen on a factory patch too: PR-A
       105 Dulcimer's tones are thrown to opposite sides by alternate pan,
       and its `patches` take is at the floor on the side of its
       KEY-INTERVAL tone.

       NOT MEASURED BY M-020, which played key 60 alone: whether "previous"
       means the same key or any key of the part. This takes the same key.
       It is the reading the hardware's `3_stormwarning` take supports:
       Amazon Moon's tone 4 is KEY-INTERVAL, the song opens with a chord of
       four keys struck within 6 ms, and the take has none of tone 4 at any
       of their four pitches for the first 3 s - where any-key timing would
       have sounded three of them within 6 ms of the note-on. A rule that
       treats an interval of a few milliseconds as none would fit the take
       as well; a two-key stimulus separates the two. */
    voice->wait = (size_t)(engine->frames -
                           (engine->parts[part].key_on_frame[key] - 1u));
    voice->delay = voice->wait;
    break;
  default:
    /* CLOCK-SYNC and TAP-SYNC: `M-016` times them against a clock this
       engine does not keep, so they start at once. */
    voice->wait = 0u;
    break;
  }
  return voice;
}

/* Render a voice through its tone delay: nothing until it is due, then the
   voice from that frame, and a postponed release at its own frame. False
   once the voice has finished. `unpanned`, when given, receives the same
   output before the pan. */
bool render_voice(struct Voice *voice, float *l, float *r, size_t n,
                  float *unpanned = nullptr)
{
  size_t release = voice->release_in ? voice->release_in : SIZE_MAX;
  if (voice->release_in)
    voice->release_in = release > n ? release - n : 0u;
  if (voice->wait == kWaitForKeyOff)
    return true;
  if (voice->muted) {
    /* Unheard: rendered into scratch until the release begins. */
    static thread_local float scratchL[512], scratchR[512];
    size_t upto = release < n ? release : n;
    for (size_t done = 0; done < upto;) {
      size_t k = upto - done > 512u ? 512u : upto - done;
      std::memset(scratchL, 0, k * sizeof *scratchL);
      std::memset(scratchR, 0, k * sizeof *scratchR);
      if (!jv1080_voice_render(&voice->voice, scratchL, scratchR, k))
        return false;
      done += k;
    }
    if (release > n)
      return true;
    voice->muted = false;
    jv1080_voice_note_off(&voice->voice);
    return release == n ||
      jv1080_voice_render(&voice->voice, l + release, r + release,
                          n - release, unpanned ? unpanned + release : nullptr);
  }
  size_t start = 0;
  if (voice->wait) {
    if (release != SIZE_MAX && release <= voice->wait)
      return false;             /* released before it ever sounded */
    if (voice->wait >= n) {
      voice->wait -= n;
      return true;
    }
    start = voice->wait;
    voice->wait = 0u;
  }
  if (release != SIZE_MAX && release <= n) {
    if (!jv1080_voice_render(&voice->voice, l + start, r + start,
                             release - start,
                             unpanned ? unpanned + start : nullptr))
      return false;
    jv1080_voice_note_off(&voice->voice);
    return release == n ||
      jv1080_voice_render(&voice->voice, l + release, r + release,
                          n - release, unpanned ? unpanned + release : nullptr);
  }
  return jv1080_voice_render(&voice->voice, l + start, r + start, n - start,
                             unpanned ? unpanned + start : nullptr);
}

/* THE PART IS NOT THE CHANNEL. Every part carries its own receive channel,
   and more than one may carry the same one - which is how two patches
   layer, and this device's own first factory song does exactly that: part
   12 and part 16 both listen on channel 16 and nothing listens on channel
   12. Calls `visit` for each part receiving on `channel` and returns how
   many there were. */
template <typename F>
unsigned for_each_part_on(struct Engine *engine, unsigned channel, F visit)
{
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  unsigned found = 0;
  for (unsigned p = 0; p < kParts; ++p) {
    if (engine->parts[p].part[profile->partFieldReceiveChannel] != channel)
      continue;
    visit(p);
    ++found;
  }
  return found;
}

/* Which packed group a melodic bank's common and part blocks are. Read off
   the bank rather than written down, so the group numbering stays the
   profile's. */
bool patch_groups(const struct xp_rom *rom, unsigned *common, unsigned *tone)
{
  const struct XpDeviceProfile *profile = xp_profile(rom);
  if (!profile->packedMelodicBankCount)
    return false;
  const struct XpPackedBank &bank =
    profile->packedBanks[profile->packedMelodicBanks[0]];
  *common = bank.commonGroup;
  *tone = bank.partGroup;
  return true;
}

/* The prefetch thread's task: one span into the cache. Reads only the
   ROM and the bank bytes, which nothing writes after engine_create. */
void prefetch_wave(void *context, const BackgroundWorker::Item &item)
{
  struct Engine *engine = (struct Engine *)context;
  if (item.a >= XP_WAVE_BANK_COUNT)
    return;
  struct xp_wave_descriptor span;
  std::memset(&span, 0, sizeof span);
  span.address_a = item.b;
  span.address_c = item.c;
  (void)wave_cache_prefetch(&engine->wave_cache, xp_profile(&engine->rom),
                            engine->banks[item.a], engine->bank_sizes[item.a],
                            &span);
}

/* Every span a record can decode at any key, each queued once. */
void queue_record(struct Engine *engine, const struct XpVoiceFieldMap *fields,
                  const uint8_t *record, int keyShift,
                  BackgroundWorker::Item *seen, unsigned *seenCount,
                  unsigned seenCapacity, unsigned firstKey, unsigned keys)
{
  for (unsigned key = firstKey; key < firstKey + keys && key < 128u; ++key) {
    BackgroundWorker::Item item;
    if (!jv1080_voice_decode_span(&engine->rom, fields, record, key, keyShift,
                                  &item.a, &item.b, &item.c))
      continue;
    bool queued = false;
    for (unsigned i = 0; i < *seenCount && !queued; ++i)
      queued = seen[i].a == item.a && seen[i].b == item.b &&
        seen[i].c == item.c;
    if (queued)
      continue;
    if (*seenCount < seenCapacity)
      seen[(*seenCount)++] = item;
    engine->prefetch->post(item);
  }
}

/* The spans of every part and kit marked since the last block, resolved on
   the render thread - the records are its to read - and decoded on the
   prefetch thread. A burst of SysEx writes marks a part many times and
   queues it once. */
void queue_prefetch(struct Engine *engine)
{
  const uint32_t marked = engine->prefetch_parts;
  engine->prefetch_parts = 0;
  if (!engine->prefetch)
    return;
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  static thread_local BackgroundWorker::Item seen[512];
  for (unsigned part = 0; part < kParts; ++part) {
    if (!(marked & (UINT32_C(1) << part)) || part == profile->rhythmPartIndex)
      continue;
    unsigned count = 0;
    int keyShift = (int)(int8_t)engine->parts[part]
      .part[profile->partFieldKeyShift];
    for (unsigned t = 0; t < XP_JV1080_TONES_PER_PATCH; ++t)
      queue_record(engine, &profile->toneFields, engine->parts[part].tone[t],
                   keyShift, seen, &count, 512u, 0u, 128u);
  }
  if (marked & (UINT32_C(1) << kParts)) {
    unsigned count = 0;
    for (unsigned k = 0; k < kRhythmKeys; ++k)
      queue_record(engine, &profile->rhythmNoteFields, engine->rhythm.note[k],
                   0, seen, &count, 512u, profile->rhythmFirstKey + k, 1u);
  }
}

/* Load a factory patch into a part, which is what a program change does
   once a bank has been selected. */
bool load_patch(struct Engine *engine, unsigned part, unsigned bank,
                 unsigned program)
{
  struct xp_packed_record record;
  if (!packed_open(&engine->rom, bank, program, &record))
    return false;
  unsigned patchLevel = 0;
  unsigned patchPan = 64;
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  for (unsigned t = 0; t < XP_JV1080_TONES_PER_PATCH; ++t) {
    if (t >= record.part_count) {
      std::memset(engine->parts[part].tone[t], 0,
                  XP_JV1080_TONE_FIELDS);
      continue;
    }
    if (!jv1080_patch_tone(&engine->rom, &record, t,
                           engine->parts[part].tone[t], &patchLevel,
                           &patchPan))
      return false;
  }
  for (unsigned f = 0; f < XP_JV1080_PATCH_COMMON_FIELDS; ++f) {
    int value = 0;
    if (!packed_common_field(&engine->rom, &record, f, &value))
      return false;
    engine->parts[part].common[f] = (uint8_t)(value & 0xff);
  }
  (void)profile;
  engine->prefetch_parts |= UINT32_C(1) << part;
  return true;
}

}  // namespace

bool engine_program_change_one(struct Engine *engine, unsigned part,
                                unsigned program);

/* Every part as a reset leaves it, then the device's power-on selection.
   The patch is loaded through the program change's own resolution, so the
   record, the latch and the loaded patch agree the way the firmware's
   write-back leaves them. */
static void power_on_parts(struct Engine *engine)
{
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  for (unsigned p = 0; p < kParts; ++p)
    reset_part(&engine->rom, engine->parts + p, p);
  if (profile->powerOnMode != XP_POWER_ON_PATCH ||
      profile->partFieldPatchGroupType == XP_VOICE_FIELD_NONE)
    return;
  struct Part &part = engine->parts[kPatchModePart];
  part.part[profile->partFieldPatchGroupType] = 0u;
  part.part[profile->partFieldPatchGroupId] = profile->powerOnPatchGroupId;
  part.part[profile->partFieldPatchNumber] = profile->powerOnPatchNumber;
  part.part[profile->partFieldPatchNumber + 1u] = profile->powerOnPatchNumber;
  part.part[profile->partFieldReceiveChannel] = profile->powerOnPatchChannel;
  seed_latch(&engine->rom, &part);
  (void)engine_program_change_one(engine, kPatchModePart,
                                  profile->powerOnPatchNumber);
}

/* ---- the injected table's own functions ---------------------------- */

bool engine_create(void **state, const struct xp_rom *rom,
                    const uint8_t *const banks[], const size_t bankSizes[],
                    unsigned bankCount, double outputRate)
{
  if (!state || !rom || !rom->bytes || !banks || !bankSizes ||
      bankCount != XP_WAVE_BANK_COUNT || outputRate <= 0.0)
    return false;
  struct Engine *engine = (struct Engine *)std::calloc(1, sizeof *engine);
  if (!engine)
    return false;
  engine->rom = *rom;
  for (unsigned b = 0; b < XP_WAVE_BANK_COUNT; ++b) {
    if (!banks[b]) {
      std::free(engine);
      return false;
    }
    engine->banks[b] = banks[b];
    engine->bank_sizes[b] = bankSizes[b];
  }
  /* THE ENGINE RUNS AT 32 kHz. The machine computes its audio at that rate,
     and what its filters do in the top octave belongs to it: the hardware's
     low-pass there is a two-pole section at 32 kHz, which no section at
     another rate reproduces (`P-xxxx`, TASK-381). */
  engine->output_rate = kXpNativeRate;
  engine->resampling = outputRate != kXpNativeRate;
  if (engine->resampling &&
      !jv_resampler_init(&engine->resampler, kXpNativeRate, outputRate)) {
    std::free(engine);
    return false;
  }
  dc_blocker_init(&engine->dc_left, engine->output_rate);
  dc_blocker_init(&engine->dc_right, engine->output_rate);
  output_shelf_init(&engine->output_shelf, engine->output_rate);
  engine->max_voices = xp_profile(rom)->defaultMaxVoices;
  if (!engine->max_voices || engine->max_voices > kMaxVoices)
    engine->max_voices = kMaxVoices;
  /* The System Control Sources as the owner's unit reads them back, twice
     (`M-116`); the factory image's own bytes for them do not decode to a
     value in range, so what a factory reset leaves is not established. */
  engine->sys_ctrl[0] = 97u;
  engine->sys_ctrl[1] = 11u;
  /* Patch Remain as the owner's unit reads it back: OFF (system common
     byte `0B` of the corpus session's state probe). */
  engine->patch_remain = false;
  /* The manual's default for Default tempo; the unit's own power-on value
     is not read back. */
  engine->tempo_bpm = 120.0;
  engine->prefetch = new (std::nothrow) BackgroundWorker(prefetch_wave,
                                                        engine);
  engine->wave_cache.guard = engine->prefetch;
  power_on_parts(engine);
  *state = engine;
  return true;
}

void engine_free(void *state)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine)
    return;
  /* First: its thread decodes into the cache freed below. */
  delete engine->prefetch;
  engine->prefetch = nullptr;
  engine->wave_cache.guard = nullptr;
  for (unsigned i = 0; i < kMaxVoices; ++i)
    free_voice(engine->voices + i);
  wave_cache_clear(&engine->wave_cache);
  if (engine->reverb_ready)
    reverb_destroy(&engine->reverb);
  if (engine->chorus_ready)
    chorus_destroy(&engine->chorus);
  std::free(engine->delay_buf);
  std::free(engine->rv_buf);
  std::free(engine->rv_pre_buf);
  for (unsigned i = 0; i < kInsertReverbMaxTaps; ++i)
    std::free(engine->rv_comb[i]);
  std::free(engine->efx_buf[0]);
  std::free(engine->efx_buf[1]);
  if (engine->resampling)
    jv_resampler_free(&engine->resampler);
  voice_batch_free(engine->batch);
  std::free(engine);
}

void engine_reset(void *state)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine)
    return;
  for (unsigned i = 0; i < kMaxVoices; ++i)
    free_voice(engine->voices + i);
  engine->gm_mode = false;
  std::memset(&engine->rhythm, 0, sizeof engine->rhythm);
  power_on_parts(engine);
  /* A reset silences the effects too, or a tail outlives the notes that
     made it. The settings are kept: they belong to the performance
     common, which a reset does not rewrite. */
  if (engine->reverb_ready)
    reverb_reset(&engine->reverb);
  if (engine->chorus_ready)
    chorus_reset(&engine->chorus);
  if (engine->delay_buf)
    std::memset(engine->delay_buf, 0,
                engine->delay_len * sizeof *engine->delay_buf);
  dc_blocker_init(&engine->dc_left, engine->output_rate);
  dc_blocker_init(&engine->dc_right, engine->output_rate);
  output_shelf_init(&engine->output_shelf, engine->output_rate);
  if (engine->resampling)
    jv_resampler_reset(&engine->resampler);
  engine->serial = 0;
}

/* PORTAMENTO, patch common 0x35..0x39. MEASURED (`P-xxxx`, TASK-364's
   rig capture `jv1080-porta-capture`: nineteen takes on the bench Sine at
   pitch key follow 100 %, read as the instantaneous frequency by two
   independent readers, analytic phase and zero crossings, which agree on
   every slope below to within 1 %):

     shape      a glide is a straight line in cents from where it starts to
                the new key (residual 0.1 to 8 cents rms), up or down alike.
     RATE       the time field names a rate, whatever the interval: v 16
                glides 11321 c/s up and 11398 down, v 64 702 and 710, and at
                v 80 to 127 the rate reads 330.0, 160.0, 70.0 and 40.0 c/s
                over 8 s holds. kPortaRate below is each measured value.
     TIME       the time field names the glide's duration, whatever the
                interval: at v 40 every interval from 12 to 48 semitones,
                up or down, arrives in 0.494 to 0.515 s. kPortaTimeMs below.
                The two laws agree to 1 % from v 10 to 56 (1200 cents per
                the RATE is the TIME) and part from 64 up: 1700 against
                1681 ms at 64, 30.0 against 32.8 s at 127. So they are two
                tables, not one.
     DOWNWARD   a glide down starts at most 2400 cents above its key: from
                26, 30 and 36 semitones above, at RATE v 40, the pitch
                arrives in 0.991 to 1.005 s, as from 24 (TIME: the same
                plateau as 24 at 36). Upward nothing is clamped - a
                48-semitone rise arrives in 2.015 s, 4775 cents at 2370.
                The audible cap a downward glide also shows is the wave's
                playback ceiling, not portamento's (see step_ceiling).
     START      PITCH (0): the next glide starts where the part's glide
                stands, and that glide runs on after its note-off - 1.5 s
                of silence carries it 10.7 semitones on (predicted 62.75,
                read 62.8). NOTE (1): the next glide starts on the key of
                the previous note-on, wherever its glide had got to.
     MODE       NORMAL (0) glides every note; LEGATO (1) only a note played
                while another key of the part is held.
     POLY       each voice glides on its own from the part's source to its
                own key; a held note never moves. A chord's keys all glide
                from the same source, not each from the key sent before it:
                60, 67 and 76 over a held 48 stand at 0.60 of the way at
                +0.35 s alike, in either order.
     SEEN LATE  a note-on is not seen by the notes that follow it for a
                while: three keys 6 ms after a fourth glide from the key
                before it, and in LEGATO mode do not count it as held (the
                bench chord and Amazon Moon's own 6 ms pairs), where a key
                87 ms after one does. How long "a while" is, is NOT
                RECOVERED: the machine's control tick is not known
                (`L-09`). kPortaSeenFrames is 10 ms, a guess of the order -
                a few milliseconds, one processing cycle - bounded by 6 and
                87 ms, and every portamento note in the demo songs but four
                of SinusoidRave's (79-91 ms) falls outside that range.

   NOT MEASURED: CC5 (portamento time), CC65 (portamento switch) and CC84
   (portamento control), none of which is read; the glide at any pitch key
   follow but 100 %; PITCH start in POLY mode, where the part's glide is
   taken to be its newest note's; what a program change does to the part
   state (nothing, here). */
const unsigned kPortaRatePoints = 24u;
const unsigned kPortaRateValue[kPortaRatePoints] = {
  0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 14, 16, 20, 24, 27, 32, 40, 48, 56, 64,
  80, 96, 112, 127 };
/* Cents per second. Values 0 and 1 are too fast to fit a slope to: 4800
   cents arrive in 14 to 18 ms at both, which is what is written. */
const double kPortaRate[kPortaRatePoints] = {
  300000.0, 300000.0, 142000.0, 91500.0, 60100.0, 48100.0, 38500.0,
  27900.0, 21100.0, 16670.0, 13600.0, 11350.0, 8220.0, 6180.0, 5060.0,
  3720.0, 2370.0, 1560.0, 1050.0, 706.0, 330.0, 160.0, 70.0, 40.0 };
const unsigned kPortaTimePoints = 21u;
const unsigned kPortaTimeValue[kPortaTimePoints] = {
  3, 4, 5, 6, 8, 10, 12, 14, 16, 20, 24, 27, 32, 40, 48, 56, 64, 80, 96,
  112, 127 };
/* Milliseconds per glide. 3 to 8 are arrival times, good to about 3 ms;
   0 to 2 are instant (the first period after the note-on is on the key). */
const double kPortaTimeMs[kPortaTimePoints] = {
  12.0, 15.0, 23.0, 29.0, 40.0, 54.0, 70.0, 88.0, 106.0, 146.0, 195.0,
  237.0, 322.0, 506.0, 767.0, 1142.0, 1681.0, 3562.0, 7448.0, 15604.0,
  32764.0 };
const double kPortaDownClampCents = 2400.0;
const double kPortaSeenSeconds = 0.010;

/* Between measured values the law is interpolated in log, which is NOT
   MEASURED at the values in between. */
double porta_law(const unsigned *at, const double *law, unsigned n,
                 unsigned value)
{
  if (value <= at[0])
    return law[0];
  for (unsigned i = 1; i < n; ++i)
    if (value <= at[i]) {
      double t = (double)(value - at[i - 1]) / (double)(at[i] - at[i - 1]);
      return std::exp(std::log(law[i - 1]) +
                      t * (std::log(law[i]) - std::log(law[i - 1])));
    }
  return law[n - 1];
}

double porta_rate_cents(unsigned value)
{
  return porta_law(kPortaRateValue, kPortaRate, kPortaRatePoints, value);
}

double porta_time_seconds(unsigned value)
{
  if (value < kPortaTimeValue[0])
    return 0.0;
  return porta_law(kPortaTimeValue, kPortaTimeMs, kPortaTimePoints, value) /
    1000.0;
}

/* Take in every note-on the part has had for kPortaSeenSeconds. */
void porta_catch_up(const struct Engine *engine, struct Part &p)
{
  const double seen = kPortaSeenSeconds * engine->output_rate;
  unsigned taken = 0;
  while (taken < p.porta_pending_count &&
         (double)p.porta_pending[taken].frame + seen <= (double)engine->frames) {
    const struct Part::PortaNote &n = p.porta_pending[taken++];
    p.porta_key = n.key;
    p.porta_from = n.from;
    p.porta_to = n.to;
    p.porta_frame = n.frame;
    p.porta_frames = n.frames;
  }
  for (unsigned i = taken; i < p.porta_pending_count; ++i)
    p.porta_pending[i - taken] = p.porta_pending[i];
  p.porta_pending_count -= taken;
}

/* Where the part's glide stands now, in key cents. */
double porta_pitch(const struct Engine *engine, const struct Part &p)
{
  double into = (double)(engine->frames - p.porta_frame);
  if (!(p.porta_frames > into))
    return p.porta_to;
  return p.porta_to + (p.porta_from - p.porta_to) * (1.0 - into / p.porta_frames);
}

/* Whether a key of the part other than `key` is held and has been seen. */
bool porta_key_held(const struct Engine *engine, const struct Part &p,
                    unsigned key)
{
  const double seen = kPortaSeenSeconds * engine->output_rate;
  for (unsigned k = 0; k < 128u; ++k)
    if (k != key && p.held_velocity[k] && p.key_on_frame[k] &&
        (double)(p.key_on_frame[k] - 1u) + seen <= (double)engine->frames)
      return true;
  return false;
}

/* What a note on `key` glides from, in key cents, and at what rate; a rate
   of zero is no glide. `legato` is whether it is played over a held key. */
struct PortaGlide {
  double from;
  double rate;
};

struct PortaGlide porta_glide(const struct Engine *engine,
                              const struct Part &p, unsigned key, bool legato)
{
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  struct PortaGlide g = { 100.0 * (double)key, 0.0 };
  if (profile->patchFieldPortamentoSwitch == XP_VOICE_FIELD_NONE ||
      !p.common[profile->patchFieldPortamentoSwitch] || p.porta_key < 0)
    return g;
  if (p.common[profile->patchFieldPortamentoMode] && !legato)
    return g;
  double to = 100.0 * (double)key;
  double from = p.common[profile->patchFieldPortamentoStart]
    ? 100.0 * (double)p.porta_key : porta_pitch(engine, p);
  if (from - to > kPortaDownClampCents)
    from = to + kPortaDownClampCents;
  unsigned time = p.common[profile->patchFieldPortamentoTime];
  double rate;
  if (p.common[profile->patchFieldPortamentoType]) {
    double seconds = porta_time_seconds(time);
    rate = seconds > 0.0 ? std::fabs(to - from) / seconds : 0.0;
  } else {
    rate = porta_rate_cents(time);
  }
  if (from != to && rate > 0.0) {
    g.from = from;
    g.rate = rate;
  }
  return g;
}

/* A note-on the part will take in kPortaSeenSeconds from now. */
void porta_note(const struct Engine *engine, struct Part &p, unsigned key,
                const struct PortaGlide &g)
{
  if (p.porta_pending_count == sizeof p.porta_pending / sizeof *p.porta_pending) {
    /* Eight notes inside the window: the oldest is taken in early. */
    struct Part::PortaNote first = p.porta_pending[0];
    for (unsigned i = 1; i < p.porta_pending_count; ++i)
      p.porta_pending[i - 1] = p.porta_pending[i];
    --p.porta_pending_count;
    p.porta_key = first.key;
    p.porta_from = first.from;
    p.porta_to = first.to;
    p.porta_frame = first.frame;
    p.porta_frames = first.frames;
  }
  struct Part::PortaNote &n = p.porta_pending[p.porta_pending_count++];
  n.frame = engine->frames;
  n.key = (int)key;
  n.to = 100.0 * (double)key;
  n.from = g.rate > 0.0 ? g.from : n.to;
  n.frames = g.rate > 0.0
    ? std::fabs(n.to - n.from) / g.rate * engine->output_rate : 0.0;
}

/* One note-on on one melodic part. `returning` is a SOLO part going back
   to a key still held after the key it sounded came up: no note-on was
   received, so the key's own note-on time and the alternate pan are left
   alone. Returns how many voices sound the note. */
static unsigned part_note_on(struct Engine *engine, unsigned part,
                             unsigned key, unsigned velocity, bool returning)
{
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  /* MEASURED (`M-124`): the patch's octave shift moves the NOTE, not
     just the pitch - the zone a key selects is the shifted key's. PR-A
     057 at key 60 with its octave shift of +1 reads harmonics 1-8 within
     1.6 dB of our key 72 with no shift, and 14.6 dB off at harmonic 2
     with the pitch doubled on key 60's zone. What the machine does with
     a shift that carries a note past 0 or 127 is not measured; it is
     clamped here. */
  int shift = profile->patchFieldOctaveShift == XP_VOICE_FIELD_NONE ? 0
    : 12 * (int)(int8_t)engine->parts[part].common[profile->patchFieldOctaveShift];
  int shifted = (int)key + shift;
  unsigned sounded = shifted < 0 ? 0u : (shifted > 127 ? 127u : (unsigned)shifted);
  /* SOLO, key assign mode 1 (patch common 0x33): one note per part, the
     newest. MEASURED (`M-164`), `voice_allocation/key_assign_solo`,
     keys 60, 64 and 67 struck 800 ms apart and all held: each note-on
     replaces the note before it within ~4 ms with no gap in level, and
     the older keys' note-offs, while the newest still sounds, change
     nothing. The cut is the steal's own kill; the hardware's few
     milliseconds of overlap are not reproduced. It cuts a note already
     in its release too: MEASURED (`P-xxxx`, `solo_release_leg0/1` R3),
     a 60 released 300 ms before a 67 falls from 36 to -44 dB at the 67's
     onset, where the same phrase in POLY keeps decaying, 33, 30, 27 dB.

     SOLO LEGATO (0x34) WITH PORTAMENTO ON continues the sounding note
     onto the new key: MEASURED (`P-xxxx`, `solo_porta_leg1_normal` and
     `_legato`), a note over a held key and a return to a held key glide
     with no dip in level (+15 ms at -28..-30 dB against a -29 dB
     sustain, A-ENV attack 0.2 s), where Solo Legato off restarts the
     attack (-42..-47 dB) and still glides. The continued voices keep the
     wave and zone of the key they started on; whether the machine
     changes either is NOT MEASURED, the Sine cannot show it.

     SOLO LEGATO WITH PORTAMENTO OFF restarts the note here, whatever 0x34
     says. MEASURED (`M-164`) on the `1_rise` hardware take, whose PR-B
     009 Pick Bass is SOLO with Solo Legato on and portamento off: the
     80-400 Hz level rises at each of its 142 overlapping note-ons by a
     median 9.7 dB against 12.6 at its 277 detached ones, where a render
     that restarts every note reads 11.7 and 13.9, and it rises as much
     at overlaps over 40 ms (9.4 dB, 39 notes) as under 15 (9.8, 50). A
     note that continued the sounding voice would not rise at all.
     UNRESOLVED against `solo_release_leg1` (`P-xxxx`), the bench Sine in
     that same setting, whose A-ENV does not dip at a note over held
     keys nor at a return (-31 dB throughout against -55 with Solo Legato
     off): both hold if the machine restarts the wave and keeps the
     envelopes, which the Sine cannot show and nothing here models.

     The part is the unit, so a part layered on the same channel keeps
     its own notes. */
  struct Part &p = engine->parts[part];
  porta_catch_up(engine, p);
  const uint16_t keyAssign = profile->patchFieldKeyAssign;
  const bool solo = keyAssign != XP_VOICE_FIELD_NONE && p.common[keyAssign] != 0u;
  const bool soloLegato =
    profile->patchFieldSoloLegato != XP_VOICE_FIELD_NONE &&
    p.common[profile->patchFieldSoloLegato] != 0u;
  const bool portamento =
    profile->patchFieldPortamentoSwitch != XP_VOICE_FIELD_NONE &&
    p.common[profile->patchFieldPortamentoSwitch] != 0u;
  const bool legato = returning || porta_key_held(engine, p, key);
  const struct PortaGlide glide = porta_glide(engine, p, key, legato);
  unsigned started = 0;
  if (solo && soloLegato && portamento && legato) {
    for (unsigned i = 0; i < kMaxVoices; ++i) {
      struct Voice *voice = engine->voices + i;
      if (!voice->allocated || voice->part != part || !voice->key_down ||
          voice->voice.releasing)
        continue;
      double base = 100.0 * (double)voice->start_key;
      jv1080_voice_glide(&voice->voice, glide.from - base,
                         100.0 * (double)key - base, glide.rate);
      voice->key = (uint8_t)key;
      ++started;
    }
  }
  if (started) {
    porta_note(engine, p, key, glide);
    p.solo_key = (int)key;
    if (!returning) {
      p.held_velocity[key] = (uint8_t)velocity;
      p.alternate_next = -p.alternate_next;
      p.key_on_frame[key] = engine->frames + 1u;
    }
    return started;
  }
  if (solo)
    for (unsigned i = 0; i < kMaxVoices; ++i) {
      struct Voice *other = engine->voices + i;
      if (other->allocated && other->part == part)
        free_voice(other);
    }
  /* MEASURED (`P-xxxx`, `velocity/vel_range_switch_on` and `_off`): with
     the patch's velocity range switch off, a tone ranged 40-90 sounds at
     every velocity from 8 to 127 (velocity 1 is at the take's floor on
     the level law), and so do the `vel_range_low` (1-64) and `_high`
     (65-127) takes, whose patches leave the switch off; on, only 40 to 88
     sound. So off, the ranges are not read at all. */
  const uint16_t sw = profile->patchFieldVelocityRangeSwitch;
  const bool ranged = sw == XP_VOICE_FIELD_NONE ||
    engine->parts[part].common[sw] != 0u;
  const uint16_t vlo = profile->toneFields.velocityRangeLow;
  const uint16_t vhi = profile->toneFields.velocityRangeHigh;
  struct Voice *toneVoice[XP_JV1080_TONES_PER_PATCH] = {};
  for (unsigned t = 0; t < XP_JV1080_TONES_PER_PATCH; ++t) {
    const uint8_t *bytes = engine->parts[part].tone[t];
    uint8_t open[XP_JV1080_TONE_FIELDS];
    if (!ranged && vlo != XP_VOICE_FIELD_NONE && vhi != XP_VOICE_FIELD_NONE) {
      std::memcpy(open, bytes, sizeof open);
      open[vlo] = 1u;
      open[vhi] = 127u;
      bytes = open;
    }
    toneVoice[t] = start_record(engine, part, &profile->toneFields, bytes, key,
                             velocity, sounded);
    started += toneVoice[t] ? 1u : 0u;
  }
  /* THE STRUCTURES, types 2 to 10: a pair both of whose tones sound
     renders as one voice through the second tone (see `render_pair`),
     which carries the pair's output, pan and sends - the manual's own
     rule. A pair with one tone silent plays as type 1, as the manual
     says, and so, here, does a pair either of whose tones waits on a
     tone delay: how a delayed tone joins its pair is not measured. */
  const uint16_t structField[2] = {profile->patchFieldStructure12,
                                   profile->patchFieldStructure34};
  const uint16_t boostField[2] = {profile->patchFieldBooster12,
                                  profile->patchFieldBooster34};
  for (unsigned k = 0; k < 2u; ++k) {
    if (structField[k] == XP_VOICE_FIELD_NONE)
      continue;
    unsigned type = (unsigned)engine->parts[part].common[structField[k]] + 1u;
    struct Voice *first = toneVoice[2u * k];
    struct Voice *second = toneVoice[2u * k + 1u];
    if (type < 2u || type > 10u || !first || !second || first->wait ||
        second->wait || first->muted || second->muted)
      continue;
    unsigned boosted = boostField[k] == XP_VOICE_FIELD_NONE ? 0u
      : (unsigned)engine->parts[part].common[boostField[k]];
    first->pair = second;
    first->pair_feed = true;
    second->pair = first;
    jv1080_voice_pair(&first->voice, &second->voice, type, boosted);
  }
  /* The glide, on every tone the note started. */
  if (glide.rate > 0.0)
    for (unsigned t = 0; t < XP_JV1080_TONES_PER_PATCH; ++t)
      if (toneVoice[t])
        jv1080_voice_glide(&toneVoice[t]->voice,
                           glide.from - 100.0 * (double)key, 0.0, glide.rate);
  porta_note(engine, p, key, glide);
  p.solo_key = solo ? (int)key : -1;
  if (!returning) {
    p.held_velocity[key] = (uint8_t)velocity;
    p.alternate_next = -p.alternate_next;
    p.key_on_frame[key] = engine->frames + 1u;
  }
  return started;
}

bool engine_note_on_jv(void *state, unsigned channel, unsigned key,
                        unsigned velocity)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine || channel >= kParts || key > 127u || velocity == 0u ||
      velocity > 127u)
    return false;
  /* The four tones of a patch start together and in phase, which is
     measured: one to four tones sum coherently to within 0.02 dB of
     `20*log10(N)` (`M-037`). */
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  unsigned started = 0;
  for_each_part_on(engine, channel, [&](unsigned part) {
    if (part == profile->rhythmPartIndex) {
      /* The rhythm part plays one record per key, not a four-tone patch,
         and the key selects the record rather than the pitch. */
      if (key < profile->rhythmFirstKey ||
          key >= (unsigned)profile->rhythmFirstKey + kRhythmKeys)
        return;
      unsigned index = key - profile->rhythmFirstKey;
      const uint8_t *note = engine->rhythm.note[index];
      unsigned group = note[profile->rhythmNoteFields.muteGroup];
      /* A mute group stops every other rhythm voice in it - one hi-hat
         closing the other. Group zero is "no group" and mutes nothing.
         The stop is a kill, not a release: on `rhythm/mute_group_hats`
         the take is at its -97 dB floor 0.22 s after a closed hat chokes
         the open one, where releasing the open hat at its own time 4
         leaves -76 dB there, and -84 with this kill. The rest is the
         closed hat's own decay. */
      if (group) {
        for (unsigned i = 0; i < kMaxVoices; ++i) {
          struct Voice *other = engine->voices + i;
          if (other->allocated && other->mute_group == group)
            free_voice(other);
        }
      }
      started += start_record(engine, part, &profile->rhythmNoteFields, note,
                               key, velocity, key) ? 1u : 0u;
      engine->parts[part].alternate_next = -engine->parts[part].alternate_next;
      return;
    }
    started += part_note_on(engine, part, key, velocity, false);
  });
  return started != 0u;
}

/* What the key coming up does to one voice, through its tone delay. The
   hold pedal postpones this, whole, to the pedal's release. */
static void key_off(struct Voice *voice)
{
  switch (voice->delay_mode) {
  case kDelayNormal:
  case kDelayKeyInterval:
    /* The note-off is postponed by the delay, as the start was: the
       NORMAL take at delay 64 stops sounding 647 ms after its
       note-off, against the 640 ms this gives. */
    if (voice->delay)
      voice->release_in = voice->delay;
    else
      jv1080_voice_note_off(&voice->voice);
    break;
  case kDelayHold:
    /* HOLD releases at the key, and a key that comes up before the
       delay has run cancels the tone: the hold take stops at the
       note-off, and `tone_delay_hold_cancel` - delay 96, key held
       400 ms - is silent from start to end. */
    if (voice->wait)
      free_voice(voice);
    else
      jv1080_voice_note_off(&voice->voice);
    break;
  case kDelayKeyOffNormal:
    voice->wait = voice->delay;
    break;
  case kDelayKeyOffDecay:
    if (voice->delay) {
      voice->release_in = voice->delay;
    } else {
      voice->muted = false;
      jv1080_voice_note_off(&voice->voice);
    }
    break;
  default:
    jv1080_voice_note_off(&voice->voice);
    break;
  }
}

/* The key coming up on one voice of a part. The firmware's own note-off
   handler reads the part's hold byte and leaves a held voice sounding
   (`04_protocol/midi.md`); a record whose Hold-1 switch is off ignores
   the pedal. */
static void voice_note_off(struct Engine *engine, struct Voice *voice)
{
  voice->key_down = false;
  if (engine->parts[voice->part].hold && voice->holdable)
    voice->sustained = true;
  else
    key_off(voice);
}

bool engine_note_off_jv(void *state, unsigned channel, unsigned key)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine || channel >= kParts || key > 127u)
    return false;
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  unsigned released = 0;
  for_each_part_on(engine, channel, [&](unsigned part) {
    struct Part &p = engine->parts[part];
    p.held_velocity[key] = 0u;
    /* A SOLO part whose sounding key comes up while others are held goes
       back to the HIGHEST of them. MEASURED (`P-xxxx`,
       `solo_release_leg0/1`, portamento off): keys pressed 64 60 72 67,
       then 69 on and off, return to 72, and 72 released to 64; pressed 67
       72 60 64 they return to 72, then 67 - neither the newest held key
       nor the first. With portamento on the return glides from the key
       that came up (`solo_porta_*` S2 and S5: 60 to 48, 60 to 55, 55 to
       48, each over the TIME law's own duration), and it retriggers or
       not as a note over a held key does. NOT MEASURED: what the hold
       pedal changes - with it down nothing returns here - and the
       velocity of the returning note, which is its own note-on's. */
    if (part != profile->rhythmPartIndex && (int)key == p.solo_key && !p.hold) {
      int back = -1;
      for (int k = 127; k >= 0 && back < 0; --k)
        if (p.held_velocity[k])
          back = k;
      if (back >= 0) {
        released += part_note_on(engine, part, (unsigned)back,
                                 p.held_velocity[back], true);
        return;
      }
      p.solo_key = -1;
    }
    for (unsigned i = 0; i < kMaxVoices; ++i) {
      struct Voice *voice = engine->voices + i;
      if (!voice->allocated || !voice->key_down || voice->part != part ||
          voice->key != key)
        continue;
      voice_note_off(engine, voice);
      ++released;
    }
  });
  return released != 0u;
}

/* A matrix source moved: every sounding voice of the part follows at once,
   as the bender's do. The pan then slews to its new place (the voice's
   set_pan); whether the machine eases the other destinations is not
   measured. */
void matrix_refresh(struct Engine *engine, unsigned part)
{
  double source[3];
  matrix_sources(engine, engine->parts[part], source);
  for (unsigned i = 0; i < kMaxVoices; ++i) {
    struct Voice *voice = engine->voices + i;
    if (voice->allocated && voice->part == part && voice->voice.matrix_used)
      jv1080_voice_set_matrix(&voice->voice, source);
  }
}

/* The controllers this device acts on. CC7 is measured (`M-081`); the bank
   pair is held until a program change resolves it, since either may arrive
   first. CC11 expression has no level of its own: it is measured INERT on
   this machine, under 0.1 dB across seventeen values (`M-048`), with the
   System Volume control source at VOLUME (`M-116`'s readback). Every
   controller below 96 is also a matrix source. */
namespace {

/* Hold 1 off: every note-off the pedal held reaches its voice at once. */
void release_hold(struct Engine *engine, unsigned part)
{
  struct Part &p = engine->parts[part];
  if (p.hold)
    for (unsigned i = 0; i < kMaxVoices; ++i) {
      struct Voice *voice = engine->voices + i;
      if (voice->allocated && voice->part == part && voice->sustained) {
        voice->sustained = false;
        key_off(voice);
      }
    }
  p.hold = false;
}

/* ALL SOUND OFF cuts rather than releases: the chord goes from -29 to
   -100 dBFS inside the 0.2 s `M-049` resolves, where a release of the same
   notes falls 14.5 dB a second. The cut's own shape inside that is not
   measured. */
void all_sound_off(struct Engine *engine, unsigned part)
{
  struct Part &p = engine->parts[part];
  for (unsigned i = 0; i < kMaxVoices; ++i) {
    struct Voice *voice = engine->voices + i;
    if (voice->allocated && voice->part == part)
      free_voice(voice);
  }
  std::memset(p.held_velocity, 0, sizeof p.held_velocity);
  p.solo_key = -1;
}

/* RESET ALL CONTROLLERS runs the part reset `0x0A0137C0`, which rewrites
   the part's whole controller block. CC11's matrix source is MEASURED at
   0.0 after CC11 64 (`M-160`, TASK-424).

   @provenance class=DOCUMENTED devices=JV-1080
   The manual's Reset All Controllers list (MIDI implementation) includes
   Hold 1 off, modulation 0, channel pressure 0 and pitch bend at its
   centre. Hold 1 off releases what the pedal held, as the pedal's own
   release does; a reset that only cleared the flag left those notes
   sounding for good. */
void reset_controllers(struct Engine *engine, unsigned part)
{
  struct Part &p = engine->parts[part];
  p.cc[11] = 0u;
  release_hold(engine, part);
  p.modulation = 0u;
  p.cc[1] = 0u;
  p.pressure = 0u;
  p.bend = 0;
  for (unsigned i = 0; i < kMaxVoices; ++i) {
    struct Voice *voice = engine->voices + i;
    if (voice->allocated && voice->part == part)
      voice->voice.bend_ratio =
        bend_ratio(p.bend, voice->bend_up, voice->bend_down);
  }
  matrix_refresh(engine, part);
}

/* ALL NOTES OFF, and OMNI OFF and ON with it: each releases the part's keys
   as their own note-offs would, within 1 dB of one another at every frame,
   and with the hold pedal down nothing changes at all (`M-049`). What the
   Omni messages do to the receive mode itself is not measured and nothing
   here changes it. */
void all_notes_off(struct Engine *engine, unsigned part)
{
  struct Part &p = engine->parts[part];
  for (unsigned i = 0; i < kMaxVoices; ++i) {
    struct Voice *voice = engine->voices + i;
    if (voice->allocated && voice->key_down && voice->part == part)
      voice_note_off(engine, voice);
  }
  std::memset(p.held_velocity, 0, sizeof p.held_velocity);
  p.solo_key = -1;
}

}  // namespace

bool engine_control_change(void *state, unsigned channel, unsigned controller,
                            unsigned value)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine || channel >= kParts || value > 127u)
    return false;
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  return for_each_part_on(engine, channel, [&](unsigned part) {
    struct Part &p = engine->parts[part];
    if (controller < 128u)
      p.cc[controller] = (uint8_t)value;
    switch (controller) {
    case 0: p.bank_msb = (uint8_t)value; break;
    case 32: p.bank_lsb = (uint8_t)value; break;
    case 7:
      p.volume = (uint8_t)value;
      for (unsigned i = 0; i < kMaxVoices; ++i) {
        struct Voice *voice = engine->voices + i;
        if (voice->allocated && voice->part == part)
          jv1080_voice_set_volume(&voice->voice, p.volume);
      }
      break;
    case 1: p.modulation = (uint8_t)value; break;
    case 64:
      /* HOLD-1, on at 64 and above (`04_protocol/controllers.md`). Its
         release hands every note-off it held to the voice at once. */
      if (value >= 64u)
        p.hold = true;
      else
        release_hold(engine, part);
      break;
    case 120:
      all_sound_off(engine, part);
      break;
    case 123:
    case 124:
    case 125:
      all_notes_off(engine, part);
      break;
    /* @provenance class=FW-EXACT devices=JV-1080 ref=0x0A013C24,0x0A013DC4,0x0A013EB0
       MONO runs the All Notes Off handler - skipped for a value above 15 -
       then writes key assign SOLO into the patch common; POLY runs it with
       the value forced to 0, then writes POLY (`04_protocol/midi.md`). The
       manual: "The key assign mode of the patch common parameter is changed
       to SOLO ... Serves as All Notes Off". */
    case 126:
      if (value <= 15u)
        all_notes_off(engine, part);
      if (profile->patchFieldKeyAssign != XP_VOICE_FIELD_NONE)
        p.common[profile->patchFieldKeyAssign] = 1u;
      break;
    case 127:
      all_notes_off(engine, part);
      if (profile->patchFieldKeyAssign != XP_VOICE_FIELD_NONE)
        p.common[profile->patchFieldKeyAssign] = 0u;
      break;
    case 121:
      reset_controllers(engine, part);
      break;
    case 101: p.rpn_msb = (uint8_t)value; break;
    case 100: p.rpn_lsb = (uint8_t)value; break;
    case 6:
      /* RPN 0/0, the bend range, in semitones (`M-014`). The up field's
         own ceiling is 12 and nothing above it has been measured through
         this path. What the manual's RPN RESET, 127/127, does to a range
         set here is not measured; it is taken as the deselect it is on
         other receivers. */
      if (p.rpn_msb == 0u && p.rpn_lsb == 0u)
        p.rpn_bend = (int)(value > 12u ? 12u : value);
      /* MEASURED (`M-084`): RPN 0/2 is coarse tune, `msb - 64` semitones
         clamped to +-32, and RPN 0/1 fine tune, `(msb - 64) * 50/32`
         cents clamped to +-50 - both exact on the device. A data entry of
         0 changes neither, which is what the take shows; whether that is
         the machine validating the value or treating 0 as unset is not
         settled. They reach the notes that start after them; whether a
         sounding note follows is not measured. */
      if (p.rpn_msb == 0u && p.rpn_lsb == 2u && value) {
        int semitones = (int)value - 64;
        p.rpn_coarse = semitones < -32 ? -32 : (semitones > 32 ? 32 : semitones);
      }
      if (p.rpn_msb == 0u && p.rpn_lsb == 1u && value) {
        double cents = ((double)value - 64.0) * 50.0 / 32.0;
        p.rpn_fine = cents < -50.0 ? -50.0 : (cents > 50.0 ? 50.0 : cents);
      }
      break;
    default: break;
    }
    if (controller < 96u)
      matrix_refresh(engine, part);
  }) != 0u;
}

void engine_panic_jv(void *state)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine)
    return;
  for (unsigned part = 0; part < kParts; ++part) {
    all_sound_off(engine, part);
    reset_controllers(engine, part);
  }
}

bool engine_pitch_bend(void *state, unsigned channel, unsigned value)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine || channel >= kParts || value > 16383u)
    return false;
  return for_each_part_on(engine, channel, [&](unsigned part) {
    struct Part &p = engine->parts[part];
    p.bend = (int)value - 8192;
    /* Every sounding voice of the part follows at once: the firmware's
       bend handler stores the value and calls its recompute-affected-
       voices routine directly (`04_protocol/controllers.md`). */
    for (unsigned i = 0; i < kMaxVoices; ++i) {
      struct Voice *voice = engine->voices + i;
      if (voice->allocated && voice->part == part)
        voice->voice.bend_ratio =
          bend_ratio(p.bend, voice->bend_up, voice->bend_down);
    }
  }) != 0u;
}

bool engine_channel_pressure(void *state, unsigned channel, unsigned value)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine || channel >= kParts || value > 127u)
    return false;
  return for_each_part_on(engine, channel, [&](unsigned part) {
    engine->parts[part].pressure = (uint8_t)value;
    matrix_refresh(engine, part);
  }) != 0u;
}

/* Load a rhythm SET, which is what a program change does on the rhythm
   part. A set is one packed record of a common block and sixty-four key
   records, the same two groups the temporary-area SysEx writes reach, so
   the voice path needs nothing new to play it.

   A rhythm part does not read the patch source its bank select names. The
   firmware keeps a per-part rhythm flag - `u8[0x09001D0C + part]`, 1 for a
   rhythm part - and dispatches the SAME resolved group through the rhythm
   loader table `0x058DB8` rather than the patch table `0x058D88`
   (`04_protocol/program_bank.md`, FW-EXACT). Without this a bank select
   and program change on the rhythm part loaded a MELODIC patch into a part
   whose note-on path never reads one, which is silence. */
bool load_rhythm_set(struct Engine *engine, unsigned bank, unsigned program)
{
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  struct xp_packed_record record;
  if (!packed_open(&engine->rom, bank, program, &record))
    return false;
  for (unsigned f = 0; f < kRhythmCommonFields; ++f) {
    int value = 0;
    if (!packed_common_field(&engine->rom, &record, f, &value))
      return false;
    engine->rhythm.common[f] = (uint8_t)value;
  }
  unsigned keys = record.part_count < kRhythmKeys
    ? record.part_count : kRhythmKeys;
  for (unsigned k = 0; k < keys; ++k)
    for (unsigned f = 0; f < kRhythmNoteFields; ++f) {
      int value = 0;
      if (!packed_part_field(&engine->rom, &record, k, f, &value))
        return false;
      engine->rhythm.note[k][f] = (uint8_t)value;
    }
  for (unsigned k = keys; k < kRhythmKeys; ++k)
    std::memset(engine->rhythm.note[k], 0, kRhythmNoteFields);
  engine->prefetch_parts |= UINT32_C(1) << kParts;
  (void)profile;
  return true;
}

bool engine_program_change_one(struct Engine *engine, unsigned part,
                                unsigned program)
{
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  struct Part &p = engine->parts[part];
  bool rhythm = part == profile->rhythmPartIndex;
  const unsigned none = profile->packedBankSelectCount;
  /* The group comes from the part's latch, not from whether a bank select
     preceded this message. GM mode overrides the latch and rewrites it; a
     latch that names no group defers to the part record's own group. See
     the profile for the firmware. */
  unsigned i;
  if (engine->gm_mode && profile->gmPartTemplate) {
    i = profile->gmBankSelect;
    p.bank_msb = profile->packedBankSelect[i].msb;
    p.bank_lsb = profile->packedBankSelect[i].lsb;
  } else {
    i = select_by_pair(profile, p.bank_msb, p.bank_lsb);
    if (i == none)
      i = select_by_record(&engine->rom, &p);
  }
  if (i == none)
    return false;
  const struct XpBankSelect &select = profile->packedBankSelect[i];
  unsigned bank = rhythm ? select.rhythmBank : select.bank;
  if (bank == XP_PACKED_BANK_NONE)
    return false;                /* a group with no image here */
  bool loaded = rhythm ? load_rhythm_set(engine, bank, program)
                       : load_patch(engine, part, bank, program);
  /* The record then says what was loaded, as `0x0A018A0C` writes it:
     type 0, id = group + 1 (`0x0A014ACE`), and the number with its
     alias. */
  if (loaded && profile->partFieldPatchGroupType != XP_VOICE_FIELD_NONE) {
    p.part[profile->partFieldPatchGroupType] = 0u;
    p.part[profile->partFieldPatchGroupId] = (uint8_t)(select.group + 1u);
    p.part[profile->partFieldPatchNumber] = (uint8_t)program;
    p.part[profile->partFieldPatchNumber + 1u] = (uint8_t)program;
  }
  /* A PROGRAM CHANGE CUTS THE PART'S SOUNDING VOICES. After the load,
     `0x0A018A0C` calls `0x0A0193D4(part)` (`0x0A01955C` for the rhythm
     part, the same code on part 9); every program-change path reaches the
     same routine. With Patch Remain (`u8[0x090138F1]`, system common `0B`)
     off, it calls the internal-ROM routine 0x3F38(part) and resets the
     per-voice state of every voice the part owns; with it on, it only
     marks those voices and leaves them sounding. A number that fails
     validation returns before the load and cuts nothing. A bank select
     only latches (`0x0A0118F4`), so it never cuts.

     0x3F38's body is not in the PRG image. What it does is measured: on
     the factory-patch corpus, 228 of 229 slots with a voice still over
     15 dB above the floor at the next program change fall to the floor,
     about 30 dB inside 0.5 ms - a hard cut, not a release. The cut lands
     5-31 ms (median 11) after the program change on hardware, the time
     the load takes; that delay is not modelled. */
  if (loaded && !engine->patch_remain)
    for (unsigned v = 0; v < kMaxVoices; ++v) {
      struct Voice *voice = engine->voices + v;
      if (voice->allocated && voice->part == part)
        free_voice(voice);
    }
  /* A program change puts the part's alternate pan back on its "+" side
     (`P-xxxx`, see `note_pan_offset`). A bank select alone does not. That
     a failed load leaves the counter alone, and that the rhythm part
     resets too, are not measured. */
  if (loaded)
    p.alternate_next = 1;
  return loaded;
}

/* GM System On, as `0x0A00E0A8` runs it; the profile has the firmware.
   What it does NOT model: the performance common, whose effect settings
   the device takes from a record in its own system memory
   (`0x023800CF`) that this implementation does not hold, so the effects
   stay as they were; the system-area bytes the same routine stages; and
   the receive switch in system memory that gates the message. */
bool engine_gm_system_on(void *state)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine)
    return false;
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);
  if (!profile->gmPartTemplate ||
      profile->gmPartTemplate + kPartFields > engine->rom.size)
    return false;
  /* `0x0A010F94` clears every part's notes on the way into the mode, the
     same call All Sound Off makes. */
  for (unsigned v = 0; v < kMaxVoices; ++v)
    free_voice(engine->voices + v);
  engine->gm_mode = true;
  for (unsigned n = 0; n < kParts; ++n) {
    struct Part &p = engine->parts[n];
    std::memcpy(p.part, engine->rom.bytes + profile->gmPartTemplate,
                kPartFields);
    p.part[profile->partFieldReceiveChannel] = (uint8_t)n;
    p.volume = profile->gmVolume;
    engine_program_change_one(engine, n,
                              p.part[profile->partFieldPatchNumber]);
  }
  return true;
}

/* THE EFFECT RELOADS WHEN ITS SOURCE PART TAKES A PROGRAM CHANGE. The
   per-part handler `0x0A018A0C` copies the patch into the part's image
   (`0x0A01993C`) and then calls `0x0A007D78(part)`, which maps the part
   onto the selector's numbering (part + 1 below the rhythm part, part
   above it, the rhythm part itself never) and, when that equals the
   performance's EFX:Source, posts event bit 2 to the DSP task. The task
   (`0x0A00A0C8`) dispatches bit 2 through `0x0A059C0C[2]` to
   `0x0A009D60`, which runs a level ramp, loads the idle program, and
   re-reads the type, the twelve parameters AND the four output bytes from
   the source image (`0x0A0096EC`, offsets +12..+28) before loading the
   new program and output block (FW-EXACT). The reload is unconditional:
   it happens even when the new patch's effect equals the old one.

   What is not modelled: that ramp (a counter stepped to 127 with a
   one-tick wait per step, writing CRAM 135 and 145, whose roles are not
   identified), the send ramps, and the idle program's gap. */
static void efx_follow_program_change(struct Engine *engine, unsigned part)
{
  bool performance = true;
  unsigned image = 0;
  if (!efx_resolve_source(engine->common[kEfxSourceField], &performance,
                           &image) ||
      performance || image != part)
    return;
  efx_refresh(engine);
  efx_block_refresh(engine);
  efx_algorithm_refresh(engine);
}

bool engine_program_change(void *state, unsigned channel, unsigned program)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine || channel >= kParts || program > 127u)
    return false;
  unsigned loaded = 0;
  for_each_part_on(engine, channel, [&](unsigned part) {
    if (engine_program_change_one(engine, part, program)) {
      ++loaded;
      efx_follow_program_change(engine, part);
    }
  });
  return loaded != 0u;
}

/* One DT1 payload. This device's parameter address is four seven-bit
   bytes - group, part, block, offset - and a write may start at any
   parameter of a block and run past its end, which is the ordinary
   editing case: the front panel and every editor address one parameter at
   a time. `device_spec.md` S10 records that a DT1 may run past a block
   boundary and that the machine relies on it - the tone record is 130
   parameters where a block addresses 128, so its last two live in the
   next block along. */
bool engine_sysex_block(void *state, const uint8_t *address,
                         unsigned addressBytes, const uint8_t *data,
                         size_t count)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine || !address || !data || !count || addressBytes != 4u)
    return false;
  unsigned commonGroup = 0;
  unsigned toneGroup = 0;
  if (!patch_groups(&engine->rom, &commonGroup, &toneGroup))
    return false;
  const struct XpDeviceProfile *profile = xp_profile(&engine->rom);

  unsigned a1 = address[0];
  unsigned part = address[1];
  unsigned block = address[2];
  unsigned within = address[3];

  /* Only the temporary patch areas are acted on. The device addresses them
     two ways - `02 pp bb xx` for performance-mode part pp, `03 00 bb xx`
     for patch mode - and the second is the first with part zero. */
  /* The temporary performance's own part blocks, `01 00 1n xx`: where each
     part's receive channel, level, pan and key shift come from. */
  if (a1 == 0x01u && part == 0x00u && (block & 0xf0u) == 0x10u) {
    unsigned index = block & 0x0fu;
    if (index >= kParts || within >= kPartFields)
      return false;
    uint8_t channel = engine->parts[index].part[profile->partFieldReceiveChannel];
    packed_apply_wire_block(&engine->rom,
                             xp_profile(&engine->rom)
                               ->packedPerformancePartGroup,
                             within, data, count, engine->parts[index].part,
                             kPartFields);
    /* A part whose receive channel changes is reset by `0x0A011104`
       through `0x0A0137C0`, and its CC11 matrix source reads 0 after it
       (`M-160`, TASK-424); a write of the same channel leaves it. */
    if (engine->parts[index].part[profile->partFieldReceiveChannel] !=
        channel) {
      engine->parts[index].cc[11] = 0u;
      matrix_refresh(engine, index);
    }
    /* A write that completes the patch number re-points the part's latch
       at the record's group: the two DT1 sites that assemble the number
       from its nibbles both call `0x0A0140FA`, which seeds the latch
       through `0x0A014EBE`. That routine also LOADS the named patch; that
       half is not modelled, so a song must still send the patch data it
       plays. Whether a write of the group alone re-seeds is not traced. */
    unsigned number = profile->partFieldPatchNumber;
    if (number != XP_VOICE_FIELD_NONE && within <= number + 1u &&
        within + count > number)
      seed_latch(&engine->rom, engine->parts + index);
    return true;
  }
  /* The temporary performance's common block, `01 00 00 xx`: this device
     puts its reverb parameters there rather than in the patch. */
  if (a1 == 0x01u && part == 0x00u && block == 0x00u) {
    if (within >= kPerfCommonFields)
      return false;
    packed_apply_wire_block(&engine->rom,
                             profile->packedPerformanceCommonGroup,
                             within, data, count, engine->common,
                             kPerfCommonFields);
    if (within <= kDefaultTempoField && within + count > kDefaultTempoField) {
      unsigned bpm = engine->common[kDefaultTempoField];
      if (bpm >= 20u && bpm <= 250u)
        engine->tempo_bpm = (double)bpm;
    }
    reverb_refresh(engine);
    chorus_refresh(engine);
    efx_refresh(engine);
    efx_block_refresh(engine);
    efx_algorithm_refresh(engine);
    return true;
  }
  /* System common, `00 00 00 xx`: Patch Remain and the two System Control
     Sources are held, one byte each on the wire as the readback shows
     them. */
  if (a1 == 0x00u && part == 0x00u && block == 0x00u) {
    bool held = false;
    for (size_t i = 0; i < count; ++i) {
      unsigned at = within + (unsigned)i;
      if (at == 0x0bu && data[i] <= 1u) {
        engine->patch_remain = data[i] != 0u;
        held = true;
      }
      if ((at == 0x12u || at == 0x13u) && data[i] <= 97u) {
        engine->sys_ctrl[at - 0x12u] = data[i];
        held = true;
      }
    }
    if (held)
      for (unsigned p = 0; p < kParts; ++p)
        matrix_refresh(engine, p);
    return held;
  }
  if (a1 == 0x03u)
    part = kPatchModePart;
  else if (a1 != 0x02u)
    return false;                /* the rest of those areas: unheld */

  /* The rhythm set has its own part index: `02 09 00 xx` is its common
     block and `02 09 kk xx` its record for key kk. */
  if (a1 == 0x02u && part == profile->rhythmPartIndex) {
    if (!block) {
      if (within >= kRhythmCommonFields)
        return false;
      packed_apply_wire_block(&engine->rom, profile->packedRhythmCommonGroup,
                               within, data, count, engine->rhythm.common,
                               kRhythmCommonFields);
      return true;
    }
    if (block < profile->rhythmFirstKey ||
        block >= (unsigned)profile->rhythmFirstKey + kRhythmKeys ||
        within >= kRhythmNoteFields)
      return false;
    packed_apply_wire_block(&engine->rom, profile->packedRhythmNoteGroup,
                             within, data, count,
                             engine->rhythm.note[block -
                                                 profile->rhythmFirstKey],
                             kRhythmNoteFields);
    engine->prefetch_parts |= UINT32_C(1) << kParts;
    return true;
  }
  if (part >= kParts)
    return false;

  if (block == kBlockPatchCommon) {
    if (within >= XP_JV1080_PATCH_COMMON_FIELDS)
      return false;
    packed_apply_wire_block(&engine->rom, commonGroup, within, data, count,
                             engine->parts[part].common,
                             XP_JV1080_PATCH_COMMON_FIELDS);
    efx_follow_source(engine);
    return true;
  }
  /* A tone's 130 parameters span two blocks, so the block index carries
     both which tone and which page of it: the low bit of the offset from
     the first tone block is the page, and the rest is the tone. */
  if (block >= kBlockFirstTone &&
      block < kBlockFirstTone +
                kBlockToneStride * XP_JV1080_TONES_PER_PATCH) {
    unsigned relative = block - kBlockFirstTone;
    unsigned tone = relative / kBlockToneStride;
    unsigned page = relative % kBlockToneStride;
    unsigned offset = page * kBlockParameters + within;
    if (offset >= XP_JV1080_TONE_FIELDS)
      return false;
    packed_apply_wire_block(&engine->rom, toneGroup, offset, data, count,
                             engine->parts[part].tone[tone],
                             XP_JV1080_TONE_FIELDS);
    engine->prefetch_parts |= UINT32_C(1) << part;
    return true;
  }
  return false;                  /* a rhythm key, or a block not held */
}

namespace {

/* The most frames jv_render_native renders its voices for at once. */
const size_t kRenderChunk = 512u;

/* Where one voice's output goes this block. */
enum VoiceRoute : uint8_t {
  kRouteDropped,      /* rendered, then not heard */
  kRouteInsert,       /* the insert's own bus */
  kRouteSends,        /* the mix, and the chorus and reverb sends */
  kRouteMix           /* the mix alone */
};

VoiceRoute voice_route(const struct Engine *engine, const struct Voice *voice)
{
  /* A voice assigned to OUTPUT1 or OUTPUT2 leaves the machine by its own
     jacks and is not on the MIX bus the chorus and reverb return to, so it
     is rendered - it still holds its slot and runs its envelopes - and then
     dropped. Whether its per-voice sends survive that routing is NOT
     established, so it sends nothing rather than sending something
     unverified.

     MEASURED EXPOSURE, before this changed anything: across all three
     factory demo songs exactly one record asks for OUTPUT1 - tone 1 of part
     1 in `1080 rave` - and that part's own assign reads EFX, so the part
     decides and the tone's OUTPUT1 never applies. No voice in any of the
     three is affected. */
  if (voice->destination == kOutputOne || voice->destination == kOutputTwo)
    return kRouteDropped;
  /* A voice bound for the insert goes to the insert's own bus, but only
     while there is an insert to go to: an effect type this engine does not
     render leaves its voices on the mix, where they have been all along,
     rather than dropping them into a bus nothing reads. */
  if (voice->destination == kOutputEfx && engine->efx_ready)
    return kRouteInsert;
  /* The reverb bus is live whichever shape the module has taken: the tank
     on types 0..5, the panning delay on type 7. */
  bool reverbLive = engine->reverb_ready || engine->delay_ready;
  if ((reverbLive && voice->reverb_send > 0.0f) ||
      (engine->chorus_ready && voice->chorus_send > 0.0f))
    return kRouteSends;
  return kRouteMix;
}

/* A sending voice's block, rendered on its own, into the mix and both
   sends. */
void mix_with_sends(const struct Voice *voice, const float *vl,
                    const float *vr, const float *vc, float *left,
                    float *right, float *send, float *csend, size_t n)
{
  /* THE CHORUS SEND IS TAKEN BEFORE THE PAN. On the device a centred
     voice's first chorus echo, at depth 0 and feedback 0, is 3.14 dB above
     its own dry signal on both channels, on every depth-0 tick of
     `chorus_fb/system_feedback_ticks` (`P-xxxx`). The pan is
     constant-power, so a centred voice reaches each channel at 0.7071 of
     its unpanned level: a send taken before the pan predicts +3.01 dB, and
     the post-pan mean of the two channels predicts 0 dB. Only the centred
     case is measured; that an off-centre voice sends the same is what
     "before the pan" means, and no capture yet varies the pan with the
     chorus open. The reverb send stays on the post-pan mean: its returns
     were calibrated through it, and PAN-DLY's first repeat reads +2.7 dB
     over the dry against the device's +3.0 that way. */
  for (size_t k = 0; k < n; ++k) {
    float mono = 0.5f * (vl[k] + vr[k]);
    left[k] += vl[k];
    right[k] += vr[k];
    send[k] += voice->reverb_send * mono;
    csend[k] += voice->chorus_send * vc[k];
  }
}

/* One block of every voice, for rendering them on several threads. Each
   voice renders into a block of its own, which holds exactly what the
   single-threaded loop would have added into the shared buses: a voice
   adds each sample once into a zeroed span, and 0 + x is x. The blocks are
   then summed in voice order, as that loop sums them. */
struct VoiceBlock {
  float left[kRenderChunk];
  float right[kRenderChunk];
  float unpanned[kRenderChunk];
};

struct VoiceBatch {
  size_t frames;
  unsigned count;
  struct Voice *voice[kMaxVoices];
  VoiceRoute route[kMaxVoices];
  bool alive[kMaxVoices];
  struct VoiceBlock block[kMaxVoices];
};

/* Below this many frames a chunk renders its voices on the calling thread:
   the resampling path asks for one native frame at a time, which leaves a
   worker nothing to do but wait. */
const size_t kThreadedMinFrames = 32u;

void voice_batch_free(struct VoiceBatch *batch)
{
  delete batch;
}

void render_batch_voice(void *context, unsigned k)
{
  struct VoiceBatch *batch = (struct VoiceBatch *)context;
  struct VoiceBlock *block = batch->block + k;
  size_t n = batch->frames;
  std::memset(block->left, 0, n * sizeof *block->left);
  std::memset(block->right, 0, n * sizeof *block->right);
  bool unpanned = batch->route[k] == kRouteSends;
  if (unpanned)
    std::memset(block->unpanned, 0, n * sizeof *block->unpanned);
  batch->alive[k] = render_voice(batch->voice[k], block->left, block->right,
                                 n, unpanned ? block->unpanned : nullptr);
}

}  // namespace

/* `frames` of the engine's own 32 kHz output, added into `stereo`. */
void jv_render_native(struct Engine *engine, float *stereo, size_t frames)
{
  engine->frames += frames;
  /* The caller's buffer is interleaved; the voice model writes into two
     planar spans, so the sum is built planar and interleaved once. A
     block-sized scratch pair rather than a per-sample transpose.

     A voice that sends to the reverb is rendered into its own scratch pair
     first, because the send bus needs that voice on its own: its share of
     the bus is its PART's send, which is the live one on this device and
     the tone's is inert (`M-039`, `M-052`). A voice with no send is summed
     straight into the mix as before. */
  static const size_t kChunk = kRenderChunk;
  float left[kChunk];
  float right[kChunk];
  float vl[kChunk];
  float vr[kChunk];
  float vc[kChunk];
  float send[kChunk];
  float csend[kChunk];
  float cwet[kChunk * 2u];
  float efxL[kChunk];
  float efxR[kChunk];
  float efxWetL[kChunk];
  float efxWetR[kChunk];
  size_t done = 0;
  while (done < frames) {
    size_t n = frames - done;
    if (n > kChunk)
      n = kChunk;
    std::memset(left, 0, n * sizeof *left);
    std::memset(right, 0, n * sizeof *right);
    std::memset(send, 0, n * sizeof *send);
    std::memset(csend, 0, n * sizeof *csend);
    std::memset(efxL, 0, n * sizeof *efxL);
    std::memset(efxR, 0, n * sizeof *efxR);
    bool sending = false;
    if (engine->pool && n >= kThreadedMinFrames) {
      struct VoiceBatch *batch = engine->batch;
      batch->frames = n;
      batch->count = 0;
      for (unsigned i = 0; i < kMaxVoices; ++i) {
        struct Voice *voice = engine->voices + i;
        if (!voice->allocated || voice->pair_feed)
          continue;
        batch->voice[batch->count] = voice;
        batch->route[batch->count] = voice_route(engine, voice);
        ++batch->count;
      }
      engine->pool->run(batch->count, render_batch_voice, batch);
      for (unsigned k = 0; k < batch->count; ++k) {
        struct Voice *voice = batch->voice[k];
        const struct VoiceBlock *block = batch->block + k;
        switch (batch->route[k]) {
        case kRouteDropped:
          break;
        case kRouteInsert:
          for (size_t f = 0; f < n; ++f) {
            efxL[f] += block->left[f];
            efxR[f] += block->right[f];
          }
          break;
        case kRouteSends:
          mix_with_sends(voice, block->left, block->right, block->unpanned,
                         left, right, send, csend, n);
          sending = true;
          break;
        case kRouteMix:
          for (size_t f = 0; f < n; ++f) {
            left[f] += block->left[f];
            right[f] += block->right[f];
          }
          break;
        }
        if (!batch->alive[k])
          free_voice(voice);
      }
    } else {
      for (unsigned i = 0; i < kMaxVoices; ++i) {
        struct Voice *voice = engine->voices + i;
        if (!voice->allocated || voice->pair_feed)
          continue;
        switch (voice_route(engine, voice)) {
        case kRouteDropped:
          std::memset(vl, 0, n * sizeof *vl);
          std::memset(vr, 0, n * sizeof *vr);
          if (!render_voice(voice, vl, vr, n))
            free_voice(voice);
          break;
        case kRouteInsert:
          std::memset(vl, 0, n * sizeof *vl);
          std::memset(vr, 0, n * sizeof *vr);
          if (!render_voice(voice, vl, vr, n))
            free_voice(voice);
          for (size_t k = 0; k < n; ++k) {
            efxL[k] += vl[k];
            efxR[k] += vr[k];
          }
          break;
        case kRouteSends:
          std::memset(vl, 0, n * sizeof *vl);
          std::memset(vr, 0, n * sizeof *vr);
          std::memset(vc, 0, n * sizeof *vc);
          if (!render_voice(voice, vl, vr, n, vc))
            free_voice(voice);
          mix_with_sends(voice, vl, vr, vc, left, right, send, csend, n);
          sending = true;
          break;
        case kRouteMix:
          if (!render_voice(voice, left, right, n))
            free_voice(voice);
          break;
        }
      }
    }
    for (size_t k = 0; k < n; ++k) {
      stereo[(done + k) * 2u] +=
        dc_blocker_step(&engine->dc_left, left[k]);
      stereo[(done + k) * 2u + 1u] +=
        dc_blocker_step(&engine->dc_right, right[k]);
    }
    /* The insert returns to the mix through its own balance and level, and
       feeds the chorus and reverb through the two sends the output assign
       may have masked to zero. The dry side of the balance is the signal
       the effect was fed, which is why the bus is kept rather than summed
       into the mix on the way in.

       EFX:Output Level scales the return to the output bus and NOT the
       sends. Its coefficient lands in CRAM 246 and 249 (`0x03856C[v]`,
       unity at 127, FW-EXACT), and 246 and 249 are the instruction slots
       the output assign re-points between MIX, OUTPUT1 and OUTPUT2
       (`0x37D8`, `0x37E4` = PRAM 246, 249) - the writes into the bus. The
       sends are instructions 222 and 224, earlier in the program, so they
       read the effect's output before the level multiplies it (FW-STRUCT:
       the slots and order are exact; what each opcode computes is not
       decoded). */
    if (engine->efx_ready) {
      if (efx_drive_type(engine->efx_type, NULL))
        drive_process(&engine->efx_drive, efxL, efxR, efxWetL, efxWetR, n);
      else if (engine->efx_type == kEfxTypeStereoEq)
        stereo_eq_process(&engine->efx_eq, efxL, efxR, efxWetL, efxWetR, n);
      else if (engine->efx_type == kEfxTypeSpectrum)
        spectrum_process(&engine->efx_spectrum, efxL, efxR, efxWetL,
                         efxWetR, n);
      else if (engine->efx_type == kEfxTypeEnhancer)
        enhancer_process(&engine->efx_enhancer, efxL, efxR, efxWetL,
                         efxWetR, n);
      else if (engine->efx_type == kEfxTypePhaser)
        phaser_process(&engine->efx_phaser, efxL, efxR, efxWetL, efxWetR, n);
      else if (engine->efx_type == kEfxTypeRotary)
        rotary_process(&engine->efx_rotary, efxL, efxR, efxWetL, efxWetR, n);
      else if (engine->efx_type == kEfxTypeCompressor ||
               engine->efx_type == kEfxTypeLimiter)
        dynamics_process(&engine->efx_dynamics, efxL, efxR, efxWetL,
                         efxWetR, n);
      else if (insert_reverb_spec(engine->efx_type))
        insert_reverb_process(engine, efxL, efxR, efxWetL, efxWetR, n);
      else if (efx_mod_spec(engine->efx_type))
        efx_mod_process(engine, efxL, efxR, efxWetL, efxWetR, n);
      else if (engine->efx_type == kEfxTypeTripleTap ||
               engine->efx_type == kEfxTypeTimeControl)
        efx_tap_process(engine, efxL, efxR, efxWetL, efxWetR, n);
      else
        efx_process(engine, efxL, efxR, efxWetL, efxWetR, n);
      for (size_t k = 0; k < n; ++k) {
        float l = engine->efx_level *
          (engine->efx_wet * efxWetL[k] + engine->efx_dry * efxL[k]);
        float r = engine->efx_level *
          (engine->efx_wet * efxWetR[k] + engine->efx_dry * efxR[k]);
        if (engine->efx_shelf_on)
          for (unsigned s = 0; s < 2u; ++s) {
            struct xp_stereo_eq_shelf *sh = &engine->efx_shelf[s];
            /* Gain 15 is the triple (1, -a, a), an identity; it is run as
               one, with the state kept on the signal it would have held,
               so a factory-flat type renders exactly as it did. */
            if (engine->efx_shelf_gain[s] == kEfxShelfFlat) {
              sh->x1[0] = sh->y1[0] = l;
              sh->x1[1] = sh->y1[1] = r;
              continue;
            }
            l = stereo_eq_shelf_run(sh, 0, l);
            r = stereo_eq_shelf_run(sh, 1, r);
          }
        stereo[(done + k) * 2u] += engine->efx_output_level * l;
        stereo[(done + k) * 2u + 1u] += engine->efx_output_level * r;
        float mono = 0.5f * (l + r);
        csend[k] += engine->efx_chorus_send * mono;
        send[k] += engine->efx_reverb_send * mono;
        if (engine->efx_reverb_send > 0.0f)
          sending = true;
      }
    }
    /* The chorus's wet signal is formed once and then returned through up
       to two paths, because that is what the firmware does: the level
       lands in the dry-mix return register `XP 0x333E`, the reverb-send
       register `XP 0x333C`, or both, and the output assign is what picks
       (`08_effects/chorus.md`, `FW-EXACT`). Routing it into the reverb is
       the only way a chorused note leaves a tail - `M-044` measured MIX at
       -99.7 dBFS after the note against -60.7 for REVERB and MIX+REV. */
    if (engine->chorus_ready) {
      unsigned assign = engine->common[kChorusOutputField];
      std::memset(cwet, 0, n * 2u * sizeof *cwet);
      chorus_process(&engine->chorus, csend, cwet, n);
      if (assign != kChorusOutReverb)
        for (size_t k = 0; k < n * 2u; ++k)
          stereo[done * 2u + k] += cwet[k];
      if (assign != kChorusOutMix) {
        for (size_t k = 0; k < n; ++k)
          send[k] += 0.5f * (cwet[k * 2u] + cwet[k * 2u + 1u]);
        sending = true;
      }
    }
    /* The reverb adds its stereo return to the dry mix already in place.
       It is stepped whenever it holds a tail, not only while something is
       sending, or a note's reverb would stop with the note. */
    if (engine->reverb_ready && (sending || engine->reverb.active))
      reverb_process(&engine->reverb, send, stereo + done * 2u, n);
    else if (engine->delay_ready && engine->delay_buf)
      delay_process(engine, send, stereo + done * 2u, n);
    /* The block is the finished mix here: the caller hands it over zeroed
       (device_render), so nothing but this engine's output is filtered. */
    output_shelf_run(&engine->output_shelf, stereo + done * 2u, n);
    done += n;
  }
}

/* The host's frames. Each one takes the engine forward only as far as it
   needs, so a parameter or note arriving between two host frames reaches
   the next 32 kHz sample. That sample sits half a kernel ahead of the one
   the host frame is centred on: an event reaches the output
   kJvResampleHalf / 32000 s - 1 ms - after the frame it arrived at. */
void engine_render_jv(void *state, float *stereo, size_t frames)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine || !stereo || !frames)
    return;
  if (engine->prefetch_parts)
    queue_prefetch(engine);
  if (!engine->resampling) {
    jv_render_native(engine, stereo, frames);
    return;
  }
  for (size_t f = 0; f < frames; ++f) {
    while (jv_resampler_needs_input(&engine->resampler)) {
      float native[2] = {0.0f, 0.0f};
      jv_render_native(engine, native, 1u);
      jv_resampler_push(&engine->resampler, native[0], native[1]);
    }
    float l, r;
    jv_resampler_pull(&engine->resampler, &l, &r);
    stereo[2u * f] += l;
    stereo[2u * f + 1u] += r;
  }
}

bool engine_set_worker_pool_jv(void *state, WorkerPool *pool)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine)
    return false;
  engine->pool = nullptr;
  if (!pool || pool->threads() <= 1)
    return true;
  if (!engine->batch) {
    engine->batch = new (std::nothrow) struct VoiceBatch;
    if (!engine->batch)
      return false;
  }
  engine->pool = pool;
  return true;
}

bool engine_set_max_voices_jv(void *state, unsigned maxVoices)
{
  struct Engine *engine = (struct Engine *)state;
  if (!engine || !maxVoices || maxVoices > kMaxVoices)
    return false;
  for (unsigned i = maxVoices; i < kMaxVoices; ++i)
    free_voice(engine->voices + i);
  engine->max_voices = maxVoices;
  return true;
}

unsigned engine_active_voices(const void *state)
{
  const struct Engine *engine = (const struct Engine *)state;
  if (!engine)
    return 0;
  unsigned active = 0;
  for (unsigned i = 0; i < kMaxVoices; ++i)
    active += engine->voices[i].allocated ? 1u : 0u;
  return active;
}

}}  // namespace EmuSC::Xp

extern "C" {

const struct XpVoiceEngineOps JV1080_VOICE_ENGINE = {
  EmuSC::Xp::engine_create,
  EmuSC::Xp::engine_free,
  EmuSC::Xp::engine_reset,
  EmuSC::Xp::engine_note_on_jv,
  EmuSC::Xp::engine_note_off_jv,
  EmuSC::Xp::engine_control_change,
  EmuSC::Xp::engine_program_change,
  EmuSC::Xp::engine_sysex_block,
  EmuSC::Xp::engine_render_jv,
  EmuSC::Xp::engine_set_max_voices_jv,
  EmuSC::Xp::engine_active_voices,
  EmuSC::Xp::engine_gm_system_on,
  EmuSC::Xp::engine_pitch_bend,
  EmuSC::Xp::engine_channel_pressure,
  EmuSC::Xp::engine_set_worker_pool_jv,
  EmuSC::Xp::engine_panic_jv,
};

}  // extern "C"
