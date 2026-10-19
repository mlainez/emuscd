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
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with libEmuSC. If not, see <http://www.gnu.org/licenses/>.
 *
 *  Device profiles: where each Roland ROM keeps its records and how those
 *  records are laid out.
 *
 *  The readers in control_rom.cc are generic. Everything that differs between
 *  devices - table offsets, record strides, the byte position of a field inside
 *  a record - lives here as data, selected once at initialisation from the synth
 *  model. Nothing device-specific belongs in the engine itself.
 *
 *  Every offset in device_profile.cc carries the evidence that fixed it. Those
 *  comments are the point of the file: an offset without provenance is a guess,
 *  and a guess that happens to be in range is the most expensive kind.
 */
#ifndef DEVICE_PROFILE_H
#define DEVICE_PROFILE_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace EmuSC
{

// The waveform record table: one record per playable waveform.
struct WaveformTableLayout
{
  uint32_t offset;
  int      stride;
  int      nameLength;
  int      zones;
};

// The sample table: one record per sample slot referenced by a waveform zone.
struct SampleTableLayout
{
  uint32_t offset;
  int      stride;
  int      volume;                      // per-sample attenuation
};

// Byte offsets within one tone record.
struct ToneFieldMap
{
  int enabled;
  int waveform;
  int velocityLow, velocityHigh;
  int coarseTune;                       // signed semitones
  int fineTune;                         // signed, a few cents either way
  int pitchKeyFollow;                   // low nibble indexes the percent table
  int pan;                              // 0..127 = L64..63R, 128 = random
  int filterCutoff, filterResonance;
  int level, dryLevel;                // multiplied to give the tone's level
  int envTime1, envLevel1;
  int envTime2, envLevel2;
  int envTime3, envLevel3;
  int envRelease;
  int reverbSend, chorusSend;
  int tvaVelCurve;                      // +71 bits 0-2: which of the seven curves
  int tvaVelLevelSens;                  // +72 signed -63..+63; 0 = no velocity effect

  // The time-variant filter's own fields. The JV's cutoff chain reads eleven of
  // them and reading only two - cutoff and resonance - is why the filter had to
  // be left disabled (P-0390).
  //
  // filterMode and tvfVelCurve share ONE byte: bits 3-4 are the mode
  // (0 OFF, 1 LPF, 2 HPF after the shift) and bits 0-2 select the envelope's
  // velocity curve. So do resonance and its SOFT/HARD mode bit, which is why
  // resoMode names the same offset as filterResonance.
  int filterMode;                       // bits 3-4
  int resoMode;                         // bit 7 of the resonance byte
  int cutoffKeyFollow;                  // bits 0-3 index the cents-per-semitone table
  int tvfVelCurve;                      // bits 0-2 of the filterMode byte
  int tvfVelLevelSens;                  // signed; with the curve, an attenuation
  int tvfEnvDepth;                      // signed -63..+63
  int tvfEnv;                           // first of four interleaved time/level pairs
  int lfo1TvfDepth, lfo2TvfDepth;       // signed

  // Which bit of `enabled` is the Tone Switch. The byte carries other fields
  // beside it, so a truthiness test reports a switched-off tone as on the
  // moment any of them is set.
  int enabledBit;

  // The envelope TIME-sense triple of the TVA and TVF blocks (scdb
  // devices/jv880 D-27, FW-EXACT). One byte holds two nibbles - bits 0-3
  // "T1 velocity", bits 4-7 "T4 velocity" - and the high nibble of a second
  // byte is "time KF"; each is 0-14 with 7 neutral, the manual's -100..+100
  // in fifteen steps. The firmware copies both bytes verbatim into the voice
  // descriptor (ROM1 0x4B19-0x4B3A) and reads them from there. 0 means the
  // device has no such fields, and the engine applies the law only when the
  // two tables it needs (RomLookup::JVEnvTimeVelDepth/KeyFollow) were read.
  int tvaTimeVelocity, tvaTimeKeyFollow;
  int tvfTimeVelocity, tvfTimeKeyFollow;

  // TVA Delay Time. Its bit 7 is the KEY-OFF setting, and a KEY-OFF tone's
  // release reads the note-ON velocity where every other tone reads the
  // note-off's (ROM1 0x39FE sets the flag, 0x1FCD tests it).
  int tvaDelayTime;

  // The pitch block and the two LFOs (scdb devices/jv880 D-37, FW-EXACT: the
  // Patch Tone descriptor table at ROM2 0x3A524, the readers in ROM1's note-on
  // voice setup and the pitch combine at ROM1 0x40AB). Appended last, so a
  // profile written before these existed still compiles and reads nothing:
  // pitchEnv == 0 means no pitch envelope fields, lfo1Rate == 0 no LFO fields.
  int pitchEnv;                         // first of T1 L1 T2 L2 T3 L3 T4 L4, L signed
  int pitchEnvDepth;                    // signed -12..+12: 100 cents per unit at level 63
  int pitchEnvVelSens;                  // signed -63..+63, velocity curve 0
  int pitchTimeVelocity;                // bits 0-3 "T1 velocity", 4-7 "T4 velocity"
  int pitchTimeKeyFollow;               // bits 4-7 "time KF"
  int lfo1Params, lfo1Rate, lfo1Delay, lfo1Fade;   // form b0-2, offset b3-5, sync b6, fade OUT b7; rate; delay (b7 KEY-OFF); fade time
  int lfo2Params, lfo2Rate, lfo2Delay, lfo2Fade;
  int lfo1PitchDepth, lfo2PitchDepth;   // signed; |d| indexes RomLookup::JVLfoPitchDepth
  // The two per-tone CC enables the manual calls the VOLUME switch and the
  // HOLD-1 switch (1-3-2, SysEx 0x08 and 0x09), both bits of the tone byte the
  // TVA velocity curve also lives in: bit 7 and bit 6 of +0x47. Volume off
  // takes the tone out of CC7's reach; Hold-1 off takes it out of the hold
  // pedal's. Both name a BYTE offset; the bits are fixed. 0 means the device
  // has no such switch and both are always on, which is the Sound Canvas.
  // scdb D-30.
  int      toneVolumeSwitch;            // bit 7
  int      toneHold1Switch;             // bit 6

  // The two LFO -> TVA depths, signed, appended last for the same reason the
  // pitch block was: a profile written before they existed still compiles and
  // reads nothing. 0 means the device has no such field. On the JV they are
  // SysEx 0x2C / 0x37 at +0x21 / +0x24 and they are two's complement, not the
  // Sound Canvas's sign-and-magnitude - which is why they cannot share
  // TVALFO1Depth / TVALFO2Depth with the Sound Canvas path. scdb D-75.
  int      lfo1TvaDepth, lfo2TvaDepth;

  // Random Pitch Depth, an index 0-15 into RomLookup::JVRandomPitch, drawn
  // once per voice. A rhythm note has had this since D-71; a patch tone keeps
  // the same index in the LOW nibble of +0x27, and the port read it for the
  // kit and not for the patches. 0 means the device has no such field.
  int      toneRandomPitch;             // bits 0-3

  // TVA Level Key Follow and Pan Key Follow, each a 0-14 index into
  // RomLookup::JVKeyFollowPct with 7 neutral. Both name a BYTE and a
  // SHIFT because the JV puts one in a low nibble and the other in a
  // high one. 0 means the device has no such field. scdb D-77.
  int      levelKeyFollow, levelKeyFollowShift;
  int      panKeyFollow,   panKeyFollowShift;

  // Tone Delay Mode, bits 4-5 of the byte named here: 0 NORMAL, 1 HOLD,
  // 2 PLAY-MATE. It selects how the Tone Delay Time at `tvaDelayTime` is
  // used, so a device with a delay time but no mode byte leaves this 0 and
  // gets NORMAL. scdb D-78.
  int      toneDelayMode;               // bits 4-5

  // The per-TONE controller matrix (scdb D-79). Three controllers - the
  // modulation wheel, channel aftertouch and expression - each with FOUR
  // destination/sense slots. Per controller this names the first of TWO
  // consecutive bytes holding the four destination nibbles (slot 1 low, slot 2
  // high of the first byte; slots 3 and 4 of the second) and the first of FOUR
  // consecutive signed sense bytes. Rows of zero mean the device has no such
  // matrix. On the JV they are +0x05/+0x07, +0x0B/+0x0D and +0x11/+0x13.
  int      ctrlMatrix[3][2];            // {first destination byte, first sense byte}

  // FXM: the byte whose bit 7 is the switch and bits 0-3 the depth. 0 = the
  // device has no such field. scdb D-80.
  int      fxm;
};

// @provenance class=FW-EXACT devices=JV-880 ref=D-79,ROM1:0x617F
// The per-tone controller matrix's arithmetic (scdb D-79: the
// contribution helper at ROM1 0x617F, the scale at ROM1 0x620E and the twelve
// destination accumulators the note-on builder at ROM1 0x56B0-0x617C fills).
//
// Each of the three controllers contributes once per destination: the senses of
// every slot pointing at that destination are SUMMED, the sum is clamped to
// +-63 and expanded to a 0-255 gain, and the controller's 0-127 value is scaled
// by it. The three contributions are then summed and scaled again, this time
// by a per-destination full-scale word with a per-destination clamp - which is
// what puts each accumulator in the units of the parameter it reaches.
//
// Destination order is the manual's: 1 PITCH, 2 CUTOFF, 3 RESONANCE, 4 LEVEL,
// 5/6 PITCH LFO1/2, 7/8 TVF LFO1/2, 9/10 TVA LFO1/2, 11/12 LFO1/2 RATE.
// Index 0 is OFF and is never scaled.
struct CtrlMatrixJvLaw
{
  int enabled;                          // 0 = the device has no such matrix

  // What the expression controller rests at before any CC11 arrives. The JV
  // treats CC11 as a modulation SOURCE and not as a part volume, so it rests
  // at 0 where a Sound Canvas rests at 127 - measured: with no CC11 at all the
  // machine renders exactly its CC11 = 0 render, and a tone routing expression
  // to LEVEL at -63 is SILENCED by CC11 = 127.
  int expressionRest;

  int full[13];                         // the 0x620E scale word, per destination
  int clamp[13];                        // and its clamp
};

// A bank of patches, and the tone records inside each patch.
// Positional initialisers: the field order here IS the initialiser order in
// devices/*.cc. Inserting a field without moving the values with it silently
// shifts every later member - it once turned `stride` into 0 and made every
// patch read from the same offset.
struct PatchLayout
{
  uint32_t bankOffset;                  // first bank; the rest follow at
  uint32_t bankStride;                  // +bankStride each
  int      banks, perBank;

  // Which MIDI bank select (CC0) reaches the preset banks, and which bank index
  // the first ROM bank answers to. A program change alone reaches the first
  // bank only; the machine puts the rest behind a bank select.
  int      midiBankFirst;               // CC0 value for bank 0 of the ROM
  int      midiBankPresets;             // CC0 value that reaches the presets
  int      stride, nameLength;
  int      level;                       // patch-common byte
  int      firstTone, toneStride, tones;
  ToneFieldMap tone;

  // Patch-common byte holding Analog Feel, the manual's "1/f fluctuation".
  // Appended last so a profile written before it existed still compiles, and
  // zero means the device has no such parameter.
  int      analogFeel;

  // Patch-common byte whose bit 7 is Key Assign (0 POLY, 1 SOLO), bit 5 Solo
  // Legato and bit 6 the Portamento switch: the JV-880's +0x18 (SysEx 0x1C,
  // 0x1D, 0x1E). ROM1 0x2B72-0x2B8B reads the same bits into the allocator's
  // per-part flags @0x8C50/@0x8C52. Zero when the device has no such byte.
  int      keyAssign;

  // Patch Common byte holding the "Velocity Switch" in bit 7. It gates the
  // per-Tone Velocity Range: with the bit clear the firmware never reads the
  // range bytes (ROM1 0xBDF). Appended last, so a profile written before this
  // existed leaves it 0 = no such switch, and the range then always applies -
  // which is the Sound Canvas behaviour. scdb D-54.
  int      velocitySwitch;
};

// Byte offsets within one performance part record.
struct PerformancePartMap
{
  int patch;
  int level, pan;
  int coarseTune;                       // signed semitones
  int channel;                          // also carries the two effect switches
  int channelMask;
  int reverbSwitchBit, chorusSwitchBit;
};

// A performance: a common block of effect settings, then one record per part.
struct PerformanceLayout
{
  uint32_t offset;
  int      stride, commonSize, partStride, parts;
  int      bootIndex;                   // the performance the device powers on in

  // The performance a host reset selects, in the same `bank flag | program`
  // selector the control-channel Performance select takes (the encoding is at
  // presetABase below). Synth::reset() fires it through the device's own
  // mechanism - Settings::select_performance(), the call a control-channel
  // program change makes - so a host reset leaves the machine where the trigger
  // every ROM demo song opens with leaves it, rather than in the boot
  // performance the field above names. Negative when the device has no such
  // state to reach.
  //
  // Not appended at the end of this struct, because 0 is a valid selector
  // (Internal performance 0) and so cannot double as "none": a device that
  // fills a PerformanceLayout must say what it wants here.
  int      resetSelector;

  int      reverbType, reverbLevel, reverbTime, reverbFeedback;

  // The chorus block. chorusType shares its byte with reverbType (bits 3-5),
  // and chorusLevel's bit 7 is the Chorus Output switch: set means the chorus
  // returns into the reverb rather than the mix.
  int      chorusType, chorusLevel, chorusRate, chorusDepth, chorusFeedback;
  PerformancePartMap part;

  // The reverb TYPE records. reverbTypeTable is a table of big-endian pointers,
  // one per type; the byte at reverbReturnCoeff inside the record it points at
  // scales the return level (P-0382, P-0395). The table is longer than the
  // reverb type list - on the JV-880 it has ELEVEN entries, the eight 0x3C-byte
  // reverb records followed by the three 0x0A-byte chorus ones - so
  // reverbTypeCount says how many of them are reverb records and the rest are
  // left alone. Zero disables the lookup.
  //
  // The coefficients land in ControlRom::LookupTables::JVReverbReturnCoeff and
  // the arithmetic that uses them is in reverb.cc, selected by
  // ReverbLaw::returnLaw - the return has to follow the type, and baking one
  // number into the boot performance's level byte could not.
  uint32_t reverbTypeTable;
  int      reverbTypeCount, reverbReturnCoeff;

  // Byte offsets of the two delay-time scale WORDS inside the same record,
  // big-endian: +0x0C is the left tap's and +0x0E the right's. Zero when the
  // device has no such records.
  int      reverbTapScaleA, reverbTapScaleB;

  // Byte offset, inside the common block, of the eight Voice Reserve bytes,
  // one per part, 0-28. On the JV-880 they are Performance Common SysEx
  // 0x17-0x1E and the firmware's own descriptor table (ROM2 0x3A204) puts
  // them at +0x14..+0x1B, max 0x1C; ROM1 0x2BA6 copies them to the allocator's
  // reserve array. The allocator lets no other part take a part's newest
  // `reserve` voices (scdb devices/jv880/06_voice_engine/allocation.md, D-44).
  // Last, so an initialiser written before it existed leaves it 0 = none.
  int      voiceReserve;

  // Performance BANKS, for the control-channel Performance select (D-46).
  //
  // A program change whose channel equals the control channel does not set a
  // part's patch: in Performance mode it loads a whole Performance, and with it
  // every part's patch, receive channel, level, pan, coarse tune and the common
  // effect settings (ROM1 0x6D83 stages it, ROM2 0x301B1 loads it). The
  // selector is `bank flag | program`, the flag coming from CC0 on the same
  // channel (only 80 -> 0x00 and 81 -> 0x80 are stored, and it persists):
  //
  //   0x00-0x3F Internal   `offset` above
  //   0x40-0x7F Card       absent on a bare machine: the select is REJECTED
  //   0x80-0xBF Preset A   presetABase
  //   0xC0-0xFF Preset B   presetBBase
  //
  // and the index within the bank is the selector's low four bits - sixteen
  // performances per bank (ROM2 0x301D7 masks with 0xCF, clearing bits 4-5).
  //
  // Every JV-880 demo song opens with CC0 81 then PC 72 here, which is selector
  // 0xC8 = Preset B index 8, "for CompuMix": one part on each of channels 1-7
  // and 10 at level 127. Without this the songs play through the boot
  // performance, whose map is [1,1,1,2,2,3,4,10] - three parts stacked on
  // channel 1 at coarse tune 0/-12/+12 and nothing at all on channels 5-7.
  //
  // controlChannel is 0-based; the firmware's 0x10 (= 16) means OFF, so 16 here
  // disables the whole path, as does a zero preset base.
  uint32_t presetABase, presetBBase;
  int      controlChannel;
};

// A rhythm set: one record per key.
//
// The JV's rhythm note is a BIT-PACKED record of 52 parameters, not a byte
// array, so a field that is not a whole byte names its shift and its width as
// data. Every offset below is the firmware's own, out of the field-descriptor
// table the DT1 apply routine walks.
//
// Positional initialisers, as everywhere in this header: the field order here
// IS the initialiser order in devices/*.cc.
struct RhythmLayout
{
  uint32_t offset;
  int      stride, keys, firstKey;
  int      enabled, waveform, playKey;
  int      level, pan;
  int      reverbSend, chorusSend;

  // Everything from here down was unread until 2026-09-03 (D-01): 44 of the
  // 52 parameters, which left every drum sharing one hardcoded envelope, no
  // filter, no velocity response and no choke groups.
  int      enabledBit;                  // bit of `enabled` that is Tone Switch
  int      muteGroup;                   // bits 0-4; 0 = ungrouped, 1-31 a group
  int      fineTune;                    // signed cents
  int      dryLevel;
  int      filterMode, filterModeShift; // two bits: 0 OFF, 1 LPF, 2 HPF
  int      filterCutoff;
  int      filterResonance;             // bits 0-6
  int      resoMode;                    // bit 7 of the resonance byte
  int      tvfVelLevelSens;             // signed -63..+63
  int      tvfEnvDepth;                 // signed -63..+63
  int      tvfEnv;                      // four interleaved time/level pairs
  int      tvaVelLevelSens;             // signed -63..+63
  int      tvaEnv;                      // T1 L1 T2 L2 T3 L3, then T4 = release

  // A rhythm note has no cutoff key follow - drums are per-key, not
  // keyboard-tracked - so the engine's key-follow index has to be told which
  // entry of the device's cents-per-semitone table means zero. Entry 0 of the
  // JV's table is -100, so leaving the index at zero detunes every drum's
  // filter by a semitone's worth of cents per key away from 60.
  int      cutoffKeyFollowNeutral;

  // The envelope TIME-sense nibbles, added 2026-09-03 with D-27. A rhythm note
  // carries ONE "time velocity" nibble per envelope rather than the patch tone's
  // pair, and the firmware mirrors it into both the T1 and T4 slots while forcing
  // the time key-follow to neutral (ROM1 0x4E85-0x4E8D writes 0x70) - drums are
  // per-key, so key-following their envelope times would be meaningless. Low
  // nibble of each byte; 7 is neutral, as entry 7 of both depth tables is 0.
  // Appended last so a profile written before these existed still compiles.
  int      tvaTimeVelocity;             // rhythm record +0x21, bits 0-3
  int      tvfTimeVelocity;             // rhythm record +0x14, bits 0-3

  // Rhythm BANKS. The JV keeps one rhythm set per memory bank, and a program
  // change on the rhythm part selects between them by its BANK BITS ALONE -
  // the program number within the bank is ignored (ROM2 0x30446,
  // devices/jv880/04_protocol/program_bank.md). ROM2 holds them at a 0x8000
  // stride alongside the patch banks: 0x0E760 the Internal factory set,
  // 0x16760 Preset A, 0x1E760 Preset B.
  //
  // Every one of the seven demo songs asks for a PRESET set - song 1 and song 7
  // for Preset A, songs 3 and 6 for Preset B - so with one set loaded from the
  // Internal offset they all played the wrong kit. Appended last; 0 means one
  // bank, which is what a profile written before this existed wants.
  int      banks;
  uint32_t bankStride;

  // The pitch block, the rhythm note's WG modulation - the same five fields a
  // patch tone carries in ToneFieldMap, in the same encoding, read by the same
  // JV pitch envelope (Pitch::_jv_init). Zero pitchEnv means the record has no
  // pitch envelope and the whole block is left neutral. The rhythm note has
  // ONE time-velocity nibble for this envelope, mirrored into T1 and T4 like
  // the TVA's and TVF's, and no time key-follow.
  int      pitchEnv;                    // first of T1 L1 T2 L2 T3 L3 T4 L4, L signed
  int      pitchEnvDepth;               // signed -12..+12
  int      pitchEnvVelSens;             // signed -63..+63
  int      pitchTimeVelocity;           // low nibble, 0-14, 7 neutral

  // Random Pitch Depth, an index 0-15 into RomLookup::JVRandomPitch. The
  // rhythm note keeps it in the HIGH nibble of the time-velocity byte where a
  // patch tone keeps it in a low nibble, hence the shift.
  int      randomPitch, randomPitchShift;

  // Pitch Bend Range, 0-12 semitones in the low nibble, per NOTE: a rhythm
  // note bends by its own range and not the part's, and the factory kits
  // leave it at 0 on 176 of 183 notes, so a bend on the rhythm channel moves
  // only the seven that ask for it.
  int      bendRange;

  // The DT1 FIELD-DESCRIPTOR TABLE for a rhythm note: eight bytes per SysEx
  // parameter - nibble mode, min, max, bias, shift-1, preserve mask, record
  // offset, recalc bit - which is what turns an incoming DT1 byte into a bit
  // field of the record. Reading it out of the ROM is what lets a rhythm-note
  // edit reach the engine without a second copy of the parameter map here.
  // Zero means the device has none and rhythm DT1 is ignored.
  //
  // `descriptorsAlt` is the same table in the later firmware revision, used
  // exactly as RomLookupTable::offsetAlt is.
  uint32_t descriptors, descriptorsAlt;
  int      descriptorCount;
};

// Curves the synthesis engine reads straight out of the ROM. The id says which
// engine table the bytes belong to, so a device lists what it has and the reader
// stays a switch rather than a chain of name comparisons.
enum class RomLookup
{
  TVFResonanceFreq,
  TVFResonance,
  EnvSegmentCurve,
  TVAPanKeyFollow,
  TVALevelIndex,
  EnvTimeKeyFollowSens,
  LFOSine,

  // The LFO RATE table: 128 big-endian words indexed by the tone's rate
  // parameter, giving the LFO's phase increment. Documented FW-EXACT in
  // scdb devices/jv880/07_synthesis/lfo.md (ROM2 0x4C58, u16 BE, reached from
  // descriptor 0x18), and the lane that found it established that its only
  // four readers are all the same LFO phase accumulator. The port has been
  // filling this table with ZEROS, so the JV's LFO has never moved. D-38.
  LFORateTable,
  PitchCoarseExp,
  EnvelopeTime,
  JVLevel,
  JVLevelEnv,
  JVLevelEnvSlope,
  JVVelCurves,

  // The JV's time-variant filter tables, all in its control ROM (P-0390). The
  // two exponential tables turn the cents accumulator into a frequency ratio;
  // TVFBase is the coefficient that ratio multiplies; the LIMIT/DAMP pairs are
  // the per-resonance clamp and damping, one pair per resonance mode; and
  // TVFCutoffKF is the manual's own key-follow list in cents per semitone.
  JVTvfExpCoarse,
  JVTvfExpFine,
  JVTvfLimitSoft,
  JVTvfDampSoft,
  JVTvfLimitHard,
  JVTvfDampHard,
  JVTvfBase,
  JVTvfCutoffKF,
  // The tone key-follow percentage list, ROM2 0x57FE: 16 signed words,
  // -910..+910 at 9.1 units per displayed percent, index 7 neutral. Shared
  // by TVA Level Key Follow and Pan Key Follow (scdb D-77).
  JVKeyFollowPct,

  // The JV's PAN law: 128 big-endian words, packed (L << 8) | R, one per pan
  // position. The Sound Canvas keeps a single 129-byte ramp read through its CPU
  // map and mirrored as T[p] against T[0x80 - p]; the JV cannot be expressed that
  // way, because its centre is ASYMMETRIC - (88, 90) of 127 - so the two channels
  // are stored independently and must be read independently. Shape is Unchecked
  // on purpose: as a word sequence it is neither rising nor falling, since a step
  // that leaves L alone and lifts R raises the word while a step that drops L
  // lowers it by 255.
  JVPanLaw,

  // The JV's chorus type records: three records of five big-endian words, laid
  // end to end, holding the effect-PSRAM addresses its chorus sweeps between
  // and the sweep-rate base (P-0394).
  JVChorusRecords,

  // The JV's envelope TIME-sense tables (scdb D-27): the signed velocity
  // depth words the "T1/T4 velocity" nibbles index, and the signed
  // per-semitone key-follow bytes the "time KF" nibble indexes.
  JVEnvTimeVelDepth,
  JVEnvTimeKeyFollow,

  // The JV's TVA attack shape (scdb D-35): the 256-byte table its TVA envelope
  // stepper reads for SEGMENT 0 ONLY, indexed by the high byte of the segment's
  // remaining-time counter and multiplied by the tone's L1. Every other
  // segment, and every segment of the pitch and TVF envelopes, is the linear
  // pre-curve ramp.
  JVTvaAttackCurve,
  JVLfoRate,                            // 128 u16 phase increments per 16 ms tick
  JVLfoOffset,                          // 8 s8 offsets added to the waveform sample
  JVLfoWaves,                           // 4 x 256 s8 waveform tables
  JVLfoPitchDepth,                      // 64 u16: |depth| -> cents/4 at full swing
  JVFxmRatio,                           // 17 u16: 65536 * 2^(-n/16), n = 1..16

  // The manual's Random Pitch Depth list in cents, indices 1..15 (ROM2 0x57A0;
  // the word before it is not entry 0 - the firmware never indexes 0). A voice
  // draws once at note-on and adds (draw * cents) >> 16 to its base pitch
  // (scdb devices/jv880/07_synthesis/pitch.md).
  JVRandomPitch
};

// Which way a curve must go for the reading to be trusted. Four JV tables have
// been caught by this check and two of them were entered with it switched off.
enum class Monotonic
{
  Unchecked,
  Rising,
  Falling
};

struct RomLookupTable
{
  RomLookup id;
  uint32_t  offset;
  int       entries;
  int       width;                      // bytes per entry, 1 or 2
  Monotonic shape;

  // A second address for devices whose firmware revisions move their tables.
  // `offset` is the address in the EARLIER revision and this the LATER one; 0
  // means the table does not move and `offset` is used for both. Appended last
  // on purpose: a five-field initialiser still compiles and reads 0 here.
  //
  // The JV-880 needs it. Its v1.0.1 grows the page-3 code image by 318 bytes,
  // which carries every table above the insertions with it, and three of ours
  // live up there. Mixing the two revisions' addresses is not a subtle error:
  // at the wrong revision TVAPanKeyFollow reads ASCII string data as a pan
  // curve, and its shape check is Unchecked, so nothing stops it.
  uint32_t  offsetAlt;
};


// A ROM that carries its waveforms, patches, performances and rhythm set in one
// image, addressed by record offset. The JV family is laid out this way.
struct RecordRomLayout
{
  WaveformTableLayout waveform;
  SampleTableLayout   sample;
  PatchLayout         patch;
  PerformanceLayout   performance;
  RhythmLayout        rhythm;

  // A device whose patch bank has not been mapped yet still plays its waveforms.
  bool has_patches(void) const { return patch.bankOffset != 0; }
};


// Where the Sound Canvas family keeps the tables its program and CPU ROMs hold.
struct ProgramRomMap
{
  int VelocityCurves;
  int KeyMapperIndex;
  int KeyMapper;

  // TVA pan key follow curve, or 0 if this generation has none. The SC-55 has
  // none: no instrument in its control ROM selects one (Instrument::panKeyFlw is
  // 0 on all 386 records) and the SC-55mkII's address holds unrelated 16-bit data
  // in the SC-55 ROM. The SC-55mkII's curve is a 128-byte table of pan positions
  // biased by 0x40, preceded by a four entry selector whose first entry is 0xffff
  // and whose remaining three all point at this curve (PROVENANCE.md P-0122).
  int TVAPanKeyFollow;
};

struct CpuRomMap
{
  int PitchParamScale;
  int EnvTimeKeyFollowSens;
  int EnvTimeScale;
  int EnvelopeTime;
  int LFORate;
  int LFODelayTime;
  int LFOTVFDepth;
  int LFOTVPDepth;
  int LFOSine;
  int TVFCutoffFreqKF;
  int TVFCutoffVSens;
  int TVFEnvDepth;
  int TVFCutoffFreq;
  int TVFResonanceFreq;
  int TVFResonance;
  int PitchEnvVelSens1;
  int PitchEnvVelSens2;
  int PitchEnvDepth;
  int TVFEnvScale;
  int PortamentoRate;
  int EnvSegmentStep;
  int EnvSegmentCurve;
  int TVAEnvExpChange;
  int TVABiasLevel;
  int TVAPanpot;
  int TVALevelIndex;
  int TVALevel;
  int PitchFineExp;
  int PitchCoarseExp;
};

// A ROM split across a control ROM of banked instrument records and a separate
// CPU ROM holding the synthesis curves. The Sound Canvas family is laid out
// this way.
struct SoundCanvasLayout
{
  const std::vector<uint32_t> *banks;
  const ProgramRomMap         *program;
  const CpuRomMap             *cpu;
  int      velocityCurves;

  // Drum sets follow the drum map array in equal blocks, ending here.
  uint32_t drumSetTableEnd;
  int      drumSetStride;

  // The key mapper is stored at a ROM address; the engine wants it relative to
  // the bank this base names.
  uint32_t keyMapperBase;

  uint32_t demoSongOffset;              // bundled demo songs
  bool     demoSongRunsToRomEnd;        // otherwise it ends at the first bank

  uint32_t introAnimOffset;             // 0 if this device has no intro animation
  uint32_t introAnimLength;             // each animation is this long, laid end
  int      introAnimCount;              // to end from introAnimOffset
};


// The arithmetic of a device's level law. The engine does the algebra; these say
// what the device's firmware shifts and scales by, so no device constant sits in
// engine code. The JV's values come from its disassembly (P-0381).
struct LevelLaw
{
  int toneIndexShift;        // (toneLevel x sampleLevel) >> this, to index the curve
  int dynamicsShift;         // (partLevel x patchLevel) >> this
  int staticShift;           // (gain x dynamics) >> this, to an 8-bit static level
  int keyFollowUnitsPerPct;  // engine key-follow units per ten percent
  // WITHDRAWN and no longer consumed. It encoded the assumption that the
  // envelope scales the static level linearly, "no attenuation" at 127. It does
  // not: the firmware converts the envelope value through its own (curve, slope)
  // pair and writes the result to a SEPARATE chip register (P-0398). Kept as a
  // named zero so the profile's field order is not silently reshuffled; set it
  // to 0 for a device whose envelope register has its own curve.
  int envelopeFullScale;

  // What the device's envelope time table holds. The JV's is in MILLISECONDS
  // (P-0383); the Sound Canvas's is already in engine control ticks, which is
  // what 0 means here. The engine's tick is 256 samples of its 32 kHz clock, 8 ms
  // - the same service period the JV's firmware uses, measured independently.
  int envelopeTimeUsPerUnit; // microseconds per table entry, or 0 if already ticks
  int envelopeInstantTicks;  // segment this short snaps instantly

  // Whether the generic dynamic-level path may apply the part level. A device
  // whose own level law already multiplies the part level in (the JV reads
  // T[(partLevel x patchLevel) >> 7]) must say 0 here, or the attenuation is
  // applied twice and the part-level curve comes out about twice as steep in
  // decibels. 1 = the Sound Canvas behaviour, part level applied here.
  int partLevelInDynamics;

  // How CC7 Volume joins the level index, and this is a family split rather than
  // a constant. Under GS, CC7 IS the part level: one slot, one curve, and
  // writing the controller straight into the part level is correct there. The JV
  // keeps the two apart - a Performance part level and a separate CC7 - and
  // multiplies them into ONE index (scdb D-28, FW-EXACT):
  //
  //   idx = ((2*L + (L >= 64)) * volume) >> volumeIndexShift
  //
  // where L is the part-level product and the expansion is the same 0..127 ->
  // 0..255 widening the firmware applies to every gain byte. ROM1 0x44c8-0x44d4
  // on the JV-880, hence 8 there. Note that this is a /127.5 scale, not /127:
  // at CC7 = 127 the index still lands one or two steps below L, so full volume
  // is NOT unity on this device.
  //
  // 0 means "not this device": the index is used unscaled and CC7 arrives as the
  // part level. That is the Sound Canvas behaviour and it is the default, so a
  // profile that says nothing about volume keeps it.
  int volumeIndexShift;
};


// Which arithmetic a device's level chain uses. The families differ in kind, not
// in constants: the Sound Canvas accumulates attenuations in a log index domain
// and subtracts them, while the JV forms a product of two curve lookups and
// keeps the envelope as a separate multiplier (PROVENANCE.md P-0381).
enum class LevelLawKind
{
  SoundCanvasLogIndex,
  JVCurveProduct
};


// How a sample address in a waveform record maps onto the wave ROM images.
struct WaveAddressing
{
  bool     flat;          // one flat space, no bank bits: the JV family
  uint32_t bank2Offset;   // where bank id 2 lands; the mkII differs from the mk1
};


// The reverb unit's device-specific numbers.
//
// The network in reverb.cc is SHARED, and that is a hardware fact rather than a
// convenience. The SC-55 mk1, the SC-55mkII and the JV-880 all drive the same
// reverb DSP: Roland's JV-880 service notes state that its TC6116AF is the mk1's
// TC24SC201AF PCM engine packaged together with the mk1's HG62E11B23FS I/O gate
// array, and the SC-55mkII's parts list carries that same Roland part number
// (15239229). So one sibling differs from another only in the numbers below,
// never in the topology.
// Which arithmetic turns the Reverb Level parameter into the wet return gain.
// A KIND rather than a constant, as with ChorusLawKind and LevelLawKind: the
// two families do not differ by a factor here, they differ in whether the
// reverb TYPE takes part at all.
enum class ReverbReturnLaw
{
  LevelOverDivisor,
  JVTypeCoefficient
};


// How the Reverb Time parameter becomes a delay-line tap address on reverb
// characters 6 and 7 - the two DELAY types. Again a KIND, not a constant: the
// Sound Canvas's tap is a straight line in the parameter and the JV's is a
// fixed-point multiply by a word taken from the selected type's own record,
// with a sign-extension step in the middle that is worth 61 samples over half
// the range (P-0397).
enum class ReverbDelayTapLaw
{
  LinearPerTime,
  JVRecordScale
};


// WHERE the reverb program's registers come from. The program itself - the
// chip's twelve-writer diffuser-and-tank network in reverb.cc - is the same on
// every device in the family.
//
// SoundCanvasProgram: the per-character register blocks in reverb.h.
//
// JVRecordProgram: the device's own ROM records, one 30-word record per reverb
// type, streamed by its firmware into the same registers (scdb devices/jv880/
// 08_effects/dsp_program.md). The JV-880's eight types do not line up with the
// Sound Canvas's eight characters - three of its records ARE Sound Canvas
// blocks under other names (ROOM2 = Plate, HALL2 = Room2, STAGE2 = Hall2) and
// the rest are its own - which is why an earlier attempt to drive the JV from
// the reverb.h blocks by type index left per-type offsets spanning 21 dB
// (P-0395 §2), and why a later attempt to explain those records as a different
// network altogether ("one line, nine stereo tap pairs", 2026-09-03/04) had to
// be retracted once its onset, polarity and loop-mode predictions were tested
// against the reference (scdb notes/track_reverb_program_2026-09-05.md).
enum class ReverbNetworkKind
{
  SoundCanvasProgram,
  JVRecordProgram
};


// How the Reverb Time parameter becomes the program's recirculation gain, for
// the six reverb types. (The two delay types take it from Delay Feedback on
// every device: one register, two parameter names, one role.)
//
// SoundCanvasTimeLine is the rounded straight line into the loop gain that
// reverb.cc has always used, inverted from this project's own measured
// T60(gLoop) curve against each machine's T60 (P-0296, P-0300, P-0303).
//
// JVFirmwareRegister is the JV-880 firmware's own arithmetic, confirmed
// instruction by instruction at ROM2 0x71C3-0x71DC (an unsigned mulxu.b, no
// sign extension, the >>8 falling out of overwriting the product's low byte):
//
//     F010.hi = (expand(Time) * record[+0x36]) >> 8
//
// and the program reads that register as byte / 128, exactly as it reads the
// Sound Canvas's own time byte and the delay arm's Feedback in the same
// register (reverb.cc). An earlier "falsification" of this law - two types
// computing the identical byte and decaying 2.7 dB apart per loop - was an
// artefact of the network it was measured under: with two tank loops of
// different lengths per type there is no single per-pass gain to compare, and
// the FIT law that replaced it for a day (loop length times Time squared) was
// removed with that network (scdb notes/track_reverb_program_2026-09-05.md).
enum class ReverbFeedbackLaw
{
  SoundCanvasTimeLine,
  JVFirmwareRegister
};


struct ReverbLaw
{
  // Reverb time -> loop gain, a rounded straight line saturating at timeCap.
  // Inverted from our own measured T60(gLoop) curve against each machine's own
  // T60; the mk1 decays about 1.09x slower than the mkII at the same reverb
  // time, so one line cannot fit both (P-0296, P-0300, P-0303, P-0304).
  float timeSlope, timeOffset;
  int   timeCap;

  // Return level. Both laws divide by levelDivisor, and 64 = unity is the
  // chip's own scale for a byte-sized coefficient field, established four
  // independent ways in the JV-880's firmware (P-0395).
  //
  // LevelOverDivisor is the Sound Canvas path: gain = level / levelDivisor, up
  // to 1.98. It is NOT the mk1's firmware law either - that writes the return as
  // (level << 8) into a 16-bit register - but it is what the family has been
  // measured against and it stays until the mk1's own return is traced.
  //
  // JVTypeCoefficient is the JV-880's, from ROM2 0x71EC-0x7227 (P-0395):
  //     expand(v) = 2 * v + (v >= 64)
  //     target    = (expand(Level) * rec[+0x38]) >> 8      types 0-5
  //     target    = Level >> 1                             types 6-7
  // out of the same 64. An earlier attempt at this law divided the target a
  // second time, by 255 rather than by 64, and measured 12.7 dB QUIET (P-0387);
  // that arithmetic is the whole of the discrepancy the ReverbLaw comment used
  // to attribute to the network's internal gain.
  float levelDivisor;
  ReverbReturnLaw returnLaw;

  // Pre-LPF: a complementary one-pole pair whose halves sum to pairSum, stepped
  // by stepPerLevel and capped at maxLevel.
  int preLpfStep, preLpfPairSum, preLpfMaxLevel;

  // Delay and panning-delay taps.
  //
  // delayTapBase is a property of the reverb program's own register layout, not
  // of the device's delay time: it is the address one past the highest WRITE
  // pointer in the character's register set, which is what makes the shortest
  // tap one sample of delay rather than a wrap of the whole 16384-word buffer.
  // For reverb.cc's `_crDelayBase` the highest writer is 0x15, so the base is
  // 0x16. The JV's own delay records put their ten write pointers at 0..9 and
  // their base at `word[+0x1A] + 1 = 10` - the same rule, a different program.
  // The 12-word difference is not a divergence in the Time law and must not be
  // "corrected" by moving one program's base onto the other's (D-16 reads it as
  // an error; P-0397 shows it is not).
  //
  // LinearPerTime:  tap = base + perTime * reverbTime, and the panning type's
  //                 left tap at half the spacing. Measured on the Sound Canvas.
  // JVRecordScale:  tap = base + ((M(time) * w) >> 16), with w the selected
  //                 type record's own scale word - +0x0C for the left tap and
  //                 +0x0E for the right, which is where the panning type's 2:1
  //                 ratio comes from rather than from a hardcoded halving.
  //                 perTime is then unused. See ControlRom::LookupTables::
  //                 JVReverbTapScale for M and for the firmware trace.
  int delayTapBase, delayTapPerTime;
  ReverbDelayTapLaw delayTapLaw;

  // Where the registers come from, and which Time -> loop-gain law. See the two
  // enums above. Under JVRecordProgram the registers come out of
  // ControlRom::LookupTables::JVReverbRecord and NONE of the constants above
  // apply except levelDivisor and returnLaw: timeSlope/timeOffset/timeCap, the
  // pre-LPF triple and delayTapBase are all properties of the reverb.h blocks,
  // because the JV's pre-LPF, tap base and Time scale are all in its records.
  ReverbNetworkKind network;
  ReverbFeedbackLaw feedbackLaw;

  // What a SEND of 127 means, appended so every profile written before it
  // still compiles and keeps the Sound Canvas value: 128, so 127 is just under
  // unity. The JV-880 briefly ran 64 here (D-39, 2026-09-04) to close a 6 dB
  // reverb deficit that turned out to belong to the reverb network of the
  // time, not to the send; with the chip's own program every JV type lands
  // within about a dB of the reference at 128 (engines/gp/devices/jv880.cc).
  float sendDivisor = 128.0f;

  // What a CHORUS send of 127 means, when that is not the same as a reverb
  // send. Zero keeps them equal, which is what every Sound Canvas profile
  // wants. The JV needs them apart: its reverb send really is 64 = unity and
  // D-39 measured it, but its chorus send is a 0-127 PARAMETER feeding the
  // same byte coefficient field, and such a parameter is written into the
  // field shifted down by one - which is exactly what ChorusJvLaw::levelShift
  // already does to the chorus LEVEL. Measured on nine single-tone factory
  // patches spanning send 8-127 with the reverb closed, taking the wet as the
  // sample difference between a chorus-open and chorus-closed render of the
  // same engine: at 64 = unity ours ran +2.7 to +5.4 dB hot from send 31 up,
  // mean +4.3, and the reference's implied coefficient tracked send/128 to
  // within a factor 1.07-1.47. scdb D-60.
  float chorusSendDivisor = 0.0f;

  // How long after a performance load the return stays silent and the line
  // empty, in milliseconds; 0 for a device with no such sequence. The JV's
  // effect drivers rebuild reverb and chorus in sequence with waits between
  // their register writes - 200 and 300 ticks in the reverb arm, with slot
  // 0x1E's F012-F01E zeroed for the second wait, then 50 in the chorus arm -
  // and the reverb return opens when they are done, at the same moment as the
  // chorus's (ChorusJvLaw::loadHoldMs).
  int loadHoldMs = 0;
};


// Which mechanism a device's chorus is. As with the level and filter laws the
// two families differ in KIND, not in constants: the Sound Canvas modulates a
// delay LENGTH from an LFO whose rate comes from a rate law, while the JV sweeps
// a READ POINTER between two effect-PSRAM addresses taken from a per-type record
// and lets the sweep speed set the period (PROVENANCE.md P-0394).
//
// Driving the JV's bytes through the Sound Canvas law is what the owner heard as
// "ça saccade": it modulated at 15.9 Hz where the machine modulates at 2 Hz, and
// its span reached 39.7 ms against CHORUS1's real 5.75 ms (P-0392).
enum class ChorusLawKind
{
  SoundCanvasLfo,
  JVSweptPointer
};


// The constants of the JV's chorus driver, so the engine holds the arithmetic
// and the device holds its numbers. Every one is an instruction's immediate in
// ROM2 0x745B-0x74E3 or 0x738C-0x73A4 (P-0394); the per-type records themselves
// come from the ROM through RomLookup::JVChorusRecords.
//
// The driver hands the chip a start address, an end address, a position and a
// read-rate increment on pseudo-voice slot 0x1F:
//
//   span    = w3 - w2
//   window  = 2 * hi16(f * (span - hi16(2 * Rate * rateScale * span)))
//   incr    = 2 * hi16(f * w4)          with f = Depth * depthScale + depthOffset
//
// so f(Depth) = 2 * (Depth * 225 + 4096) / 65536 scales BOTH the window and the
// increment while g(Rate) = 1 - 2 * Rate * 241 / 65536 shrinks the window alone.
// The modulation period is window / increment, so f cancels and the period
// depends only on Rate; the excursion IS the window and depends only on Depth.
// That exact decoupling is what identifies the mechanism - no coincidence
// produces it - and it is why Rate must reach g() and Depth f(): see the
// performance field map in engines/gp/devices/jv880.cc.
struct ChorusJvLaw
{
  int   types;              // type records in the table, five words each
  int   depthScale;         // f = Depth * this + depthOffset
  int   depthOffset;
  int   rateScale;          // g = 1 - (2 * Rate * this) / 65536
  int   levelShift;         // wet gain = ((level & 0x7f) >> this) / coeffUnity
  int   feedbackShift;      // feedback  = (feedback >> this) / coeffUnity
  float coeffUnity;         // the byte coefficient that means 1.0

  // How long after a performance load the return stays silent and the read
  // pointer parked at its start address, in milliseconds. The effect drivers
  // rebuild both effects in sequence with waits between the register writes,
  // and the sweep only begins when they are done; so the PHASE of the sweep at
  // every later note is set by this, and a phase that is off makes a wet copy
  // at 18-20 ms of delay add where the machine's cancels, partial by partial.
  int   loadHoldMs;
};


// How a part's TVF Cutoff Frequency parameter reaches the partial's cutoff.
struct TvfCutoffLaw
{
  // Whether a POSITIVE parameter value raises the partial's base cutoff. On the
  // mkII generation it does, one cutoff step per parameter step, and the
  // resonance the cutoff admits is read at the raised position. On the mk1 a
  // positive value does nothing at all. Measured on Warm Pad, C4, over the whole
  // parameter range (PROVENANCE.md P-0133).
  bool offsetRaises;

  // The two ceilings this parameter is clamped to: on (parameter - 0x40) where
  // the partial's base cutoff is offset, and on the parameter itself where the
  // phase iterator recomputes it.
  //
  // The mk1 restricts this parameter where later devices do not. Its ceiling is
  // 0x50 against the mkII's 0x7f - and that restriction is the mk1's own, not
  // the family's: the SCC-1 generation's firmware allows 0x72, both in the ROM
  // byte and in its manual's own parameter chart. libEmuSC currently answers an
  // SCC-1 ROM with this profile, so it inherits the mk1's narrower ceiling; that
  // is a KNOWN DIVERGENCE, not a measurement, and correcting it needs an SCC-1
  // reference render rather than a change made on paper.
  int offsetMax;
  int paramMax;
};


// Which arithmetic a device's filter chain uses. As with the level law the two
// families differ in kind, not in constants: the Sound Canvas assembles a cutoff
// INDEX and looks the coefficient up, while the JV accumulates modulation in
// CENTS, exponentiates it and multiplies the base coefficient by the result
// (PROVENANCE.md P-0390). An earlier attempt to drive the JV's fields through
// the Sound Canvas chain came out close in colour and 35 dB wrong in level.
enum class TvfLawKind
{
  SoundCanvasIndex,
  JVCentsRatio
};


// The constants of the JV's filter chain, so the engine holds the arithmetic and
// the device holds its numbers. Every one is read from the firmware (P-0390).
struct TvfJvLaw
{
  int envDepthScale;      // TVF-ENV Depth: sign(d) * ((|2d| << 8) * this >> 16)
  int lfoDepthScale;      // LFO -> TVF depth: int8 * this
  int zeroResLimit;       // at resonance 0 the cutoff word is capped here
  int zeroResDampBase;    // and the damping is this minus (word >> 3),
  int zeroResDampFloor;   // never below this
  int resSlewPerTick;     // resonance moves at most this far per envelope tick
  int cutoffUnity;        // cutoff word that means F1 = 1.0
  int dampUnity;          // damping word that means Q1 = 1.0
  int envTickPeriods;     // engine control periods per filter envelope tick
};


// The JV's pitch envelope constant (scdb devices/jv880 D-37). ROM1 0x48BF forms
// the depth word as sign(d) * ((|2d| << 8) * envDepthScale >> 16) and the pitch
// combine at ROM1 0x40BF multiplies the velocity-attenuated envelope level
// (L << 8) by it, high word taken: depth +12 at level +63 is exactly 1200
// cents. 0 means the device has no such law and the engine adds nothing.
struct PitchJvLaw
{
  int envDepthScale;

  // The wave key-zone search: a zone whose breakpoint is B is the LAST key that
  // uses it (scdb devices/jv880 D-83, M-100). "The first breakpoint at or above
  // the note" already says that - what does not is the value handed to it.
  // Pitch::_basePitchC is not the note's semitone: for a key below C4 the
  // key-follow branch leaves the whole missing semitone in _basePitchF as a full
  // 1000, so the search reads one semitone low and every zone keeps its
  // breakpoint key plus one. Set on a device whose boundary has been measured
  // against the machine; the Sound Canvas family's has not.
  bool zoneBreakIsLastKey;
};


// Everything the engine needs to know about one device. Exactly one of the two
// layout pointers is set: it says which family the ROM belongs to, and so which
// reader can walk it.
// ---------------------------------------------------------------------------
// THE ANALOG STAGE: what the device does to the finished mix, after the chip.
//
// Everything above this line is the digital machine. A real module then puts
// the samples through a converter and an analog board, and that board is part
// of the instrument's sound. This is the single place it is declared, as data,
// so that adding the next module's output stage is a few numbers in
// devices/<model>.cc and nothing at all in the engine. The code that runs it is
// analog_stage.cc and it never learns which device it is serving.
//
// It is measured, not assumed. A device whose board nobody has measured leaves
// this default-constructed and its mix is passed through bit for bit - which is
// where every Sound Canvas profile stands, and where it must stay until someone
// measures one. The JV-880's response is emphatically NOT the SC-55's: they are
// different boards, and the temptation to reuse a curve that "sounds more like
// hardware" is exactly what the evidence classes exist to stop.

struct OutputSection
{
  enum Kind { LowShelf, Peaking, HighShelf };

  Kind  kind;
  float freq;                   // Hz: the shelf corner, or the peak's centre
  float gainDb;
  float q;                      // shelf slope, or the peak's width
};

struct AnalogStageProfile
{
  // A cascade of RBJ cookbook biquads, applied to the finished stereo mix at
  // the host sample rate. The forms matter as much as the numbers: these were
  // fitted WITH the cookbook equations, so another realisation of "a low shelf
  // at 118 Hz" would be a different curve from the measured one.
  const OutputSection *response = nullptr;
  int                  sections = 0;

  // Flat make-up gain applied with the response, for a board whose absolute
  // gain is known. Zero is not "no opinion", it is the convention the JV's
  // response is stated in: the curve is referenced to the 250-500 Hz bands and
  // is unity there, because a long-term spectrum measured off a recording of
  // unknown gain can fix a SHAPE and cannot fix a level.
  float trimDb = 0.0f;
};


struct DeviceProfile
{
  const char *name;

  // Size of the control ROM image, and how many 2 MB wave ROM banks the device
  // addresses. Used to tell one device of a family from another.
  size_t romSize;
  int    waveRomBanks;

  uint8_t maxPolyphony;

  // Fade applied to a partial when its voice is given to another note, in dB per
  // millisecond. Measured by taking a sounding voice away and comparing the
  // render against one where it was left alone: the level falls linearly in dB,
  // independent of the tone's own release (PROVENANCE.md P-0080).
  float voiceDampRate;

  const RecordRomLayout   *records;
  const SoundCanvasLayout *soundCanvas;

  const RomLookupTable *lookupTables;
  int                   lookupTableCount;

  LevelLaw level;

  LevelLawKind   levelLawKind;
  ReverbLaw      reverb;
  TvfCutoffLaw   tvfCutoff;
  WaveAddressing wave;

  // Last, so that a device profile written before these existed still compiles
  // and still means what it meant: an omitted initialiser leaves the kind at
  // SoundCanvasIndex and the JV law zeroed and unread.
  TvfLawKind     tvfLawKind;
  TvfJvLaw       tvfJv;

  // The same reasoning for the chorus: an omitted initialiser leaves the kind at
  // SoundCanvasLfo, which is the behaviour every Sound Canvas profile had before
  // this field existed.
  ChorusLawKind  chorusLawKind;
  ChorusJvLaw    chorusJv;

  // And for the pitch: omitted, the scale is 0 and the JV pitch path is off.
  PitchJvLaw     pitchJv;

  // And for the analog board: omitted, no sections, and the mix is untouched.
  AnalogStageProfile analog;

  // And for the per-tone controller matrix: omitted, enabled is 0 and the
  // matrix is not built. scdb D-79.
  CtrlMatrixJvLaw ctrlJv;

  // Whether a variation-table miss falls back to the bank's group base (the
  // bank number with its low three bits cleared), and failing that to bank 0,
  // rather than leaving the part silent. Only the SC-55 does this, and only
  // for bank < 64 and program < 120 (part.cc has the full rule and its
  // provenance). Last and omitted everywhere else, so it defaults to false -
  // the correct, no-fallback behaviour for every other profile.
  bool variationFallback;

  // Whether the SysEx receiver acts on Roland (manufacturer 0x41) messages
  // only, dropping every universal message - GM System On and Master Volume
  // among them. Such a device has no GM mode for GM System On or a GS_GM host
  // reset to enter. Omitted, false: the Sound Canvas receivers act on both.
  bool rolandSysExOnly;

  // Whether the GS model-ID (0x42) DT1 handler acts on GS Scale Tuning only -
  // address 40 0x 40 or 40 1x 40, x = 0-7, with exactly 12 data bytes - and
  // drops every other GS DT1, the GS Reset 40 00 7F among them. x names part
  // ((x - 1) & 7), so x = 0 is the eighth part. Omitted, false: a Sound Canvas
  // acts on the whole GS map.
  bool gsScaleTuningOnly;

  // Whether CC120 All Sound Off goes unrecognised, as a device whose MIDI
  // implementation chart leaves it out ignores it. Omitted, false: it cuts
  // the part's sound.
  bool ignoresAllSoundOff;

  // Whether Mono (CC126) and Poly (CC127) cut the part's sound, as All Sound
  // Off does, rather than releasing its notes as All Notes Off does.
  // Omitted, false: they release.
  bool monoPolyCutSound;
};

// The Sound Canvas family's shared behaviour, for a generation whose ROM layout
// has not been mapped yet and so has no profile of its own. It carries only what
// the synthesis engine reads - never a ROM offset - and its values are exactly
// what the engine used to hold in its own else branches.
extern const DeviceProfile SOUND_CANVAS_DEFAULT_PROFILE;

extern const DeviceProfile JV880_PROFILE;
extern const DeviceProfile SC55_PROFILE;
extern const DeviceProfile SC55MKII_PROFILE;

}

#endif  // DEVICE_PROFILE_H
