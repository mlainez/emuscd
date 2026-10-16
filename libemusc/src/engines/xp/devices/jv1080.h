/* SPDX-License-Identifier: CC0-1.0 */
/*
 *  Roland JV-1080 constants for the XP engine (engines/xp/).
 *
 *  This device runs the same sound chip as the SC-88 and carries the
 *  identical Roland part number for it, but its firmware is a different
 *  thing entirely: the SC-88's is in an external ROM and disassembles,
 *  while the JV-1080's synthesis engine lives in the SH7034's 64 KB
 *  internal mask ROM, which has never been dumped.
 *
 *  SO NOTHING HERE IS A FIRMWARE PORT, AND NOTHING HERE MAY BE READ AS
 *  FIRMWARE-EXACT. What the external ROM holds - the preset records, the
 *  parameter map, the wave tables, the effect coefficient data - is read
 *  from it and is exact. Everything the voice path does with those values
 *  is a BEHAVIOURAL MODEL fitted to laws measured on the hardware: a
 *  transfer function that responds the way the machine does, not the
 *  arithmetic the machine uses to get there. engines/xp/README.md says the
 *  same thing about this device from the other side, and each law below
 *  carries the measurement it comes from.
 *
 *  The device facts themselves live in struct XpDeviceProfile
 *  (devices/profile.h), populated in jv1080.cc.
 */
#ifndef EMUSC_XP_DEVICES_JV1080_H
#define EMUSC_XP_DEVICES_JV1080_H

#include "profile.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* This device's control ROM image size, for the test suite's own buffers;
   XpDeviceProfile::romSize is what the engine reads. */
inline constexpr unsigned XP_JV1080_CONTROL_ROM_SIZE = 0x100000u;

/* Which field of the tone group each role is. The index within a group is
   also the parameter's SysEx address offset on this device, because its
   descriptor table doubles as its parameter address map - so these are the
   manual's own offsets, checked field by field against the descriptors'
   declared ranges for all ten groups. They are here rather than in the
   profile only where a caller needs the whole run of a block; the profile
   carries the ones the shared voice path reads by role. */
inline constexpr unsigned XP_JV1080_TONE_FIELDS = 130u;
inline constexpr unsigned XP_JV1080_PATCH_COMMON_FIELDS = 75u;
inline constexpr unsigned XP_JV1080_TONES_PER_PATCH = 4u;

extern const struct XpDeviceProfile JV1080_PROFILE;

/* This device's own voice engine, for XpDeviceProfile::voiceEngine. Its
   state is opaque: the shared device layer holds it as a void * and only
   ever calls through this table. */
extern const struct XpVoiceEngineOps JV1080_VOICE_ENGINE;

/* ONE TONE, SOUNDING. A BEHAVIOURAL MODEL, NOT THE CHIP.
 *
 *   Everything below responds the way the hardware measures - the wave it
 *   reads and the rate it reads it at, the amplitude envelope, the filter's
 *   corner, the pan split - and none of it is the arithmetic the machine
 *   uses, because that arithmetic is in an undumped mask ROM. What IS in
 *   this device's readable ROM is the wave data and the parameter values;
 *   those are exact, and this turns them into sound by the measured
 *   transfer functions listed against each field in jv1080.cc.
 *
 *   What this model does NOT yet include, so that nothing reading it
 *   mistakes silence for a measurement: the two LFOs and all their
 *   destinations, the pitch envelope, FXM, the booster, the
 *   ten structures (six of which ring-modulate), tone delay, the TVA bias,
 *   every key-follow field INCLUDING the cutoff's own (M-077 measures it
 *   exactly and it is not wired here), the resonance's velocity
 *   sensitivity, the alternate and random pan depths, the A-ENV's velocity
 *   curves 1 to 6 (measured in M-029, not yet transcribed here), the
 *   element record's own attenuation and fine-tune fields (units open,
 *   U-R3-03), and the insert, chorus and reverb effects, which are
 *   bypassed rather than approximated.
 */
/* Everything outside the tone that scales or places one of its voices.
   Each term is a measured one: the patch and part levels index the same
   square law the tone level does (`M-009`), CC7 indexes it with a floor
   (`M-081`), the two pans sum as offsets from centre into one table
   (`M-002`, `M-015`), and the part's key shift moves the pitch. */
struct XpJv1080PartControls {
  unsigned patch_level;
  unsigned patch_pan;
  unsigned part_level;
  unsigned part_pan;
  unsigned volume;
  int key_shift;
  int fine_tune;                 /* the part's own detune, in cents */
  double tune_cents;             /* the RPN master coarse and fine tune */
  /* The engine's own running clock at note-on, which a free-running LFO
     takes its phase from, and a seed for the drawn LFO waveforms. */
  double clock_seconds;
  uint32_t lfo_seed;
  /* The tempo an EXT SYNC LFO counts its clock pulses against, BPM. */
  double tempo_bpm;
  /* The three matrix controllers' sources as they stand, 0..1. */
  double matrix_source[3];
  /* Which way this note's alternate pan throws: +1 on the part's first
     note after a program change, then flipping on each note-on after it. */
  int alternate_phase;
};

/* One of a voice's two LFOs: its waveform, its rate, the offset its
   output is shifted by, and the delay and fade that shape its depth. */
struct XpJv1080Lfo {
  unsigned form;
  double frequency;              /* Hz */
  double phase;                  /* cycles, 0..1 */
  double offset;                 /* -1..+1 of the waveform's amplitude */
  double delay;                  /* seconds */
  unsigned fade_mode;            /* ON-IN, ON-OUT, OFF-IN, OFF-OUT */
  double fade;                   /* seconds */
  double held;                   /* the drawn forms' current value */
  uint32_t seed;
  double since_on;               /* seconds since the note-on */
  double since_off;              /* seconds since the note-off, < 0 before */
};

struct XpJv1080Voice {
  bool active;
  bool releasing;
  /* A NO-SUSTAIN rhythm note, and a note-off it has deferred. */
  bool one_shot;
  bool pending_release;
  /* Release on reaching the sustain level, with no key to wait for. */
  bool release_at_sustain;

  /* The two LFOs and what each moves: pitch in cents, amplitude in dB,
     cutoff in the cutoff parameter's own units and pan in pan-table
     distance, each a signed peak for a waveform of +-1. lfo_active is
     false when every depth is zero, and then none of this is read. */
  struct XpJv1080Lfo lfo[2];
  bool lfo_active;
  double lfo_pitch_cents[2];
  double lfo_amp_db[2];
  double lfo_cutoff_units[2];
  double lfo_pan_units[2];
  int pan_offset;                /* the voice's own pan, before the LFO */
  double lfo_pitch_ratio;
  double lfo_gain;
  double lfo_cutoff;
  size_t lfo_period;
  size_t lfo_countdown;

  /* The controller matrix: each slot's destination and signed depth,
     controller c owning slots 4c..4c+3, and the values its destinations
     act on before it. matrix_used is false when no slot is routed, and
     then none of this is read. */
  bool matrix_used;
  uint8_t matrix_dest[12];
  double matrix_depth[12];
  double tone_level_gain;        /* the tone level's own square law */
  double velocity_gain;          /* the velocity curve's own gain */
  double fade_gain;              /* the velocity cross fade's own gain */
  double outer_level_gain;       /* the patch and part levels' */
  unsigned volume;               /* CC7, as the voice last received it */
  unsigned resonance_base;
  unsigned resonance_value;      /* with the matrix's RES applied */
  double resonance_q_base;
  double lfo_base_cents[2];
  double lfo_base_frequency[2];
  double lfo_base_cutoff_units[2];
  double matrix_pitch_ratio;
  double matrix_cutoff;
  double matrix_pan;             /* pan-table distance the matrix adds */

  /* The decoded element, and where in it the read head is. */
  const int32_t *pcm;
  size_t pcm_count;
  double position;               /* fractional index into pcm */
  double increment;              /* wave samples per output sample */
  double step_ceiling;           /* the most the read head may advance */
  double bend_ratio;             /* the bender's share, 1 at centre */
  /* Portamento: the glide's pitch factor, 1 at rest on the voice's own
     key. porta_cents is the same offset in key cents (a key is 100),
     before the tone's pitch key follow, porta_kf, turns it into pitch.
     While porta_left samples remain both move by one step per sample;
     on arrival they are set to the end values exactly. */
  double porta_ratio;
  double porta_cents;
  double porta_factor;
  double porta_cents_step;
  double porta_end_ratio;
  double porta_end_cents;
  double porta_kf;
  size_t porta_left;
  size_t loop_first;
  size_t loop_last;
  bool looping;
  bool reverse;
  /* A ping-pong loop (`loop type 1`) does not read forward. Its turn is a
     REFLECTION, not a time reversal, because the wave format is a
     differential one: running the address back down the stream while still
     accumulating what it reads gives the loop backwards and reflected about
     the value it turned at, continuous in value AND in slope. Once the head
     has passed the loop's end, `position` stops being an index into `pcm`
     and becomes a position in a cycle of `2 * (loop_last - loop_first + 1)`
     - the reflected descending pass, then the forward ascending one - and
     `in_cycle` says which of the two meanings it currently has. */
  bool ping_pong;
  bool in_cycle;

  /* Amplitude. static_gain is everything but the envelope: the level
     fields' square law, CC7, the velocity curve and the wave gain. Only
     CC7 moves it once the note has started. */
  double static_gain;
  /* static_gain without the wave gain: what a booster pair's second tone
     applies after the clip, its wave gain having gone in before it. */
  double static_gain_unwaved;
  /* The factors static_gain is the product of, around the CC7 one, so a
     volume change can form it again in the same order. */
  double gain_levels;
  double gain_velocity;
  double gain_wave;
  double gain_mix;
  double gain_fade;
  double gain_left;
  double gain_right;
  /* The pan gains a move is heading for. gain_left and gain_right reach
     them by the pan slew, one step per pan tick (see pan_tick). */
  double pan_target_left;
  double pan_target_right;
  bool pan_moving_left;
  bool pan_moving_right;
  bool pan_started;              /* false until the first tick snaps */
  size_t pan_period;             /* samples per pan tick */
  size_t pan_countdown;

  /* The four-segment amplitude envelope. level[] is linear amplitude and
     level_units[] the same levels as the record's own 0-127 values;
     time[0] is the attack's duration and time[1..3] each segment's time
     for a 20 dB fall on a full traverse. */
  double level[4];
  double level_units[4];
  double time[4];
  unsigned segment;
  double envelope;               /* current linear amplitude */
  double segment_start;          /* in level units, from segment 1 on */
  double segment_total;          /* this segment's own duration, seconds */
  double segment_remaining;      /* seconds left in this segment */
  double sample_period;
  /* The level units the envelope last read through the level table and
     the amplitude that read gave. A segment between two equal levels asks
     for the same units on every sample. Zero units is zero amplitude, so
     a zeroed voice starts with a matching pair. */
  double envelope_units;
  double envelope_units_amplitude;
  /* EMUSC_LEGACY_DSP_FAST carries the level read between exact reads by
     one ratio per sample, valid inside one segment and one interval of the
     level table. Declared in both tiers: the struct crosses the engine's
     boundary, and its size must not depend on a build option. */
  double envelope_ratio;
  unsigned envelope_ratio_interval;
  unsigned envelope_ratio_countdown;
  bool envelope_ratio_valid;

  /* The filter, as a two-pole section: its direct-form coefficients, and
     the state-variable realisation of the same transfer function that
     runs it - g and k set the poles, m_hp/m_bp/m_lp mix the three outputs
     into the numerator, s1/s2 are the two integrator states. */
  int filter_type;
  double b0, b1, b2, a1, a2;
  double svf_g, svf_k, m_hp, m_bp, m_lp;
  double svf_h1, svf_h2, svf_h3;   /* set_svf's, from g and k */
  double s1, s2;

  /* The filter envelope, which moves the CUTOFF PARAMETER and not a
     frequency (`M-082`), so everything here is in cutoff units on the
     0..127 scale and the corner law is applied to the sum.

     cutoff_offset is the whole sweep at full envelope level, signed by the
     depth; fenv_value is the fraction of it the envelope currently stands
     at. fenv_time[] is each segment's duration, whatever levels it runs
     between. */
  double cutoff_base;
  double cutoff_offset;
  double resonance_q;
  double fenv_level[4];          /* fraction of full scale per segment */
  double fenv_time[4];           /* seconds per segment */
  unsigned fenv_segment;
  double fenv_value;
  double fenv_start;
  double fenv_total;
  double fenv_remaining;
  /* The modulator is re-evaluated once per block rather than per sample -
     `M-074` measures a step completing inside one carrier cycle, so a
     block of a millisecond or less is indistinguishable from the machine,
     and it is what makes a moving corner affordable without a coefficient
     solve per sample. */
  size_t control_period;
  size_t control_countdown;
  double output_rate;

  /* The pitch envelope, in cents: penv_level[] is each segment's target,
     penv_time[] its duration, and penv_ratio the pitch factor it currently
     stands at. penv_active is false when the depth is zero or every level
     is centre, and then none of this is read. */
  /* A structured tone pair: set on the pair's second tone, which renders
     both, `partner` being the first. structure is the panel's type, 2 to
     10. tone_gain is this voice's own share of its level - tone level,
     matrix LEV, velocity and wave gain - without the patch's, the part's,
     CC7 or the mix scale, which a pair applies once, through the second
     tone. */
  struct XpJv1080Voice *partner;
  unsigned structure;
  unsigned booster;              /* the pair's booster, 0..3 */
  /* In a pair, a wave that has played out and a voice whose envelope has
     ended are different things: the other tone still sounds through this
     one's filter, and through the second tone's TVA. */
  bool wave_done;
  bool envelope_done;
  double tone_gain;

  /* FXM: the read rate alternates between fxm_ratio[0] and fxm_ratio[1],
     each held for fxm_half seconds; fxm_clock is the time into the pair. */
  bool fxm_active;
  double fxm_ratio[2];
  double fxm_half;
  double fxm_clock;

  bool penv_active;
  double penv_level[4];
  double penv_time[4];
  unsigned penv_segment;
  double penv_value;
  double penv_start;
  double penv_remaining;
  double penv_total;
  double penv_ratio;
  size_t penv_period;
  size_t penv_countdown;
};

#ifdef __cplusplus
}

struct xp_rom;
struct xp_packed_record;
struct xp_wave_cache;

namespace EmuSC { namespace Xp {

/* Set a voice up to play one tone. `tone` is the tone's 130 decoded bytes -
 * the form the packed record decodes to, and the same form the device's own
 * SysEx tone frames carry - and patchLevel/patchPan are the patch common's.
 * `banks` are the eight descrambled 1 MiB wave banks. The element's decoded
 * PCM comes from `cache` (wave_cache_acquire; null decodes a private copy)
 * and is `voice->pcm`, which the caller gives back with exactly one
 * wave_cache_release(cache, voice->pcm) once the voice is done with it.
 * False means this tone does not sound for this key and velocity, which is
 * not an error, and holds no PCM.
 */
bool jv1080_voice_start(const struct xp_rom *rom,
                         const struct XpVoiceFieldMap *fields,
                         const uint8_t *tone,
                         const struct XpJv1080PartControls *controls,
                         unsigned key, unsigned velocity,
                         const uint8_t *const banks[XP_WAVE_BANK_COUNT],
                         const size_t bankSizes[XP_WAVE_BANK_COUNT],
                         struct xp_wave_cache *cache,
                         double outputRate, struct XpJv1080Voice *voice);

/* Decode one patch's tone into `tone` (130 bytes) and its patch common's
 * level and pan, ready for jv1080_voice_start. */
bool jv1080_patch_tone(const struct xp_rom *rom,
                        const struct xp_packed_record *patch, unsigned index,
                        uint8_t *tone, unsigned *patchLevel,
                        unsigned *patchPan);

/* How many decoded samples this tone reads at this key, without decoding
 * them. False where the tone does not sound, the same cases
 * jv1080_voice_start refuses short of a failed decode. `keyShift`
 * is the part's key shift, which jv1080_voice_start reads from its
 * controls: both move the zone the key selects. */
bool jv1080_voice_span(const struct xp_rom *rom,
                        const struct XpVoiceFieldMap *fields,
                        const uint8_t *tone, unsigned key, unsigned velocity,
                        int keyShift, size_t *samples);

/* The span jv1080_voice_start decodes for this tone at this key, as the
 * decoded-PCM cache keys it: the bank, and the first and last wave
 * addresses. Velocity does not move it. False where no element is
 * resolved at that key. */
bool jv1080_voice_decode_span(const struct xp_rom *rom,
                               const struct XpVoiceFieldMap *fields,
                               const uint8_t *tone, unsigned key,
                               int keyShift, unsigned *bank,
                               uint32_t *first, uint32_t *last);

void jv1080_voice_release(struct XpJv1080Voice *voice);
/* Link a tone pair under structure `type` (the panel's 2..10): `second`
   renders both from then on, and `first` must no longer be rendered on its
   own. `booster` is the pair's booster setting, 0..3, read by types 3 and
   4. */
void jv1080_voice_pair(struct XpJv1080Voice *first, struct XpJv1080Voice *second,
                       unsigned type, unsigned booster);
/* A CC7 value reaching a voice that is already sounding. */
void jv1080_voice_set_volume(struct XpJv1080Voice *voice, unsigned volume);
/* The matrix controllers' sources moving under a sounding voice. */
void jv1080_voice_set_matrix(struct XpJv1080Voice *voice,
                              const double source[3]);
/* The key coming up: a release, or on a NO-SUSTAIN voice still in its
   first three segments, a release deferred to their end. */
void jv1080_voice_note_off(struct XpJv1080Voice *voice);
/* A portamento glide: from `fromCents` to `toCents`, both key cents
   relative to the key the voice was started on, moving linearly at
   `centsPerSecond`. A rate of zero or less, or no distance, sets the end
   at once. */
void jv1080_voice_glide(struct XpJv1080Voice *voice, double fromCents,
                        double toCents, double centsPerSecond);
/* Where the voice's glide stands now, in the same key cents. */
double jv1080_voice_glide_cents(const struct XpJv1080Voice *voice);

/* The filter envelope's two measured pieces, exposed so a test can check
 * them against the takes they come from without a ROM.
 *
 * jv1080_filter_env_curve is velocity curve `curve` (0..6) at `velocity`,
 * as the fraction of the envelope's travel in cutoff units at velocity
 * sensitivity +50 - one at velocity 127.
 *
 * jv1080_filter_env_offset is this record's whole sweep at full envelope
 * level, in CUTOFF-PARAMETER units and signed by the depth field.
 *
 * jv1080_filter_env_segment_seconds is how long a segment with time field
 * `value` lasts, whatever levels it runs between, before key and velocity
 * scaling.
 */
double jv1080_filter_env_curve(unsigned curve, unsigned velocity);
double jv1080_filter_env_segment_seconds(unsigned value);
double jv1080_filter_env_offset(const struct XpVoiceFieldMap *fields,
                                 const uint8_t *record, unsigned velocity);

/* The magnitude, in dB, of the filter section a voice of filter type `type`
 * (0 OFF, 1 LPF, 2 BPF, 3 HPF, 4 PKG) runs at cutoff parameter `cutoff` and
 * resonance `resonance`, output rate `rate`, at frequency `hz` - exposed
 * so a test can check the section against the takes without a ROM. */
double jv1080_tvf_response_db(int type, double cutoff, unsigned resonance,
                              double rate, double hz);

/* The amplitude factor the TVA key bias (direction 0..3, point, level enum
 * 0..14) puts on a note at `key`. */
double jv1080_bias_gain(unsigned direction, unsigned point, unsigned level,
                        unsigned key);

/* The factor the A-ENV's velocity-time sensitivity (enum 0..14) puts on its
 * attack time at note-on velocity `velocity`. */
double jv1080_amp_env_velocity_time_scale(unsigned enumValue,
                                          unsigned velocity);

/* Adds this voice's output into l/r and, when `unpanned` is given, the same
 * output before the pan into it. Returns false once it has finished,
 * having written whatever it had left. */
bool jv1080_voice_render(struct XpJv1080Voice *voice, float *l, float *r,
                          size_t frames, float *unpanned = nullptr);

}}  // namespace EmuSC::Xp
#endif

#endif
