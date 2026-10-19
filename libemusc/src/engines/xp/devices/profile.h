/* SPDX-License-Identifier: CC0-1.0 */
/*
 *  The XP engine's per-device parameter block, and this engine's own fixed
 *  array sizes.
 *
 *  struct XpDeviceProfile is the vocabulary type engines/xp/'s generic code
 *  reads every device-specific fact through, injected at runtime via
 *  xp_rom::profile / xp_engine::profile. A device's values live in its own
 *  devices/<device>.cc beside their provenance comments; nothing in this
 *  file names a device, so a second device on this engine adds no field
 *  here that only it would fill.
 *
 *  A handful of constants stay compile-time values below instead: they size
 *  this engine's own fixed C arrays, which requires the value to be known
 *  at compile time wherever it is used, not just at one definition site.
 *  The older engine has the identical exception
 *  (DeviceProfile::MAX_PARTIALS) - this isn't a compromise unique to this
 *  engine.
 */
#ifndef EMUSC_XP_DEVICES_PROFILE_H
#define EMUSC_XP_DEVICES_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct xp_rom;

#ifdef __cplusplus
namespace EmuSC { namespace Xp { class WorkerPool; } }

extern "C" {
#endif

/* This engine's own fixed array sizes and hard ceilings: concurrently
   active slots, notes and parts, tone components per note, wave chips
   and banks, reverb buffers/taps, output filter sections, TVF sections,
   the per-tone controller matrix, and the two record sizes used to size
   a stack buffer in the test suite. Regardless of which device's default
   polyphony is in effect (see XpDeviceProfile::defaultMaxVoices) - these
   are the engine's, not the device's. */
inline constexpr unsigned XP_ENGINE_SLOT_COUNT = 64u;
inline constexpr unsigned XP_ENGINE_NOTE_COUNT = 64u;
inline constexpr unsigned XP_ENGINE_PART_COUNT = 32u;
inline constexpr unsigned XP_MAX_TONE_COMPONENTS = 2u;
inline constexpr unsigned XP_WAVE_CHIP_COUNT = 4u;
inline constexpr unsigned XP_WAVE_BANK_COUNT = 8u;
inline constexpr unsigned XP_MIDI_PORT_COUNT = 2u;
inline constexpr unsigned XP_MATRIX_SOURCE_COUNT = 6u;
inline constexpr unsigned XP_MATRIX_DEST_COUNT = 11u;
/* A record part this device does not carry (see the reverb word indices). */
inline constexpr uint8_t XP_REVERB_WORD_NONE = 0xffu;

/* How a device sweeps its chorus delay; see `chorusModulator`. */
inline constexpr uint8_t XP_CHORUS_MOD_SINE = 0u;
inline constexpr uint8_t XP_CHORUS_MOD_TRIANGLE_UP = 1u;

/* What a device's chorus feeds back; see `chorusFeedbackTap`. */
inline constexpr uint8_t XP_CHORUS_FB_TAP_MEAN = 0u;
inline constexpr uint8_t XP_CHORUS_FB_TAP_LEFT = 1u;

/* What a device selects at power-on and on a host reset; see
   `powerOnMode`. */
inline constexpr uint8_t XP_POWER_ON_NONE = 0u;
inline constexpr uint8_t XP_POWER_ON_PATCH = 1u;

inline constexpr unsigned XP_REVERB_BUFFERS = 12u;
inline constexpr unsigned XP_REVERB_TAPS = 8u;
inline constexpr unsigned XP_REVERB_HALF_BUFFERS = 4u;
inline constexpr int XP_OUTPUT_HOLD_TAPS = 31;
inline constexpr int XP_OUTPUT_ANALOG_SECTIONS = 5;
inline constexpr int XP_OUTPUT_MAX_SECTIONS = 4;
inline constexpr unsigned XP_TVF_SECTIONS = 1u;
inline constexpr unsigned XP_DRUM_FIELDS = 10u;

/* Ceilings for the descriptor-packed record tables below. Sized by what a
   member of this family declares, not by what one device happens to have:
   the widest wave ROM permutation is sixteen data lines and twenty-one
   address lines, and the largest record schema seen so far is ten groups
   and thirteen banks. */
inline constexpr unsigned XP_WAVE_DATA_LINES_MAX = 16u;
inline constexpr unsigned XP_WAVE_ADDRESS_LINES_MAX = 21u;
inline constexpr unsigned XP_PACKED_GROUP_MAX = 12u;
inline constexpr unsigned XP_PACKED_BANK_MAX = 16u;
inline constexpr uint8_t XP_PACKED_BANK_NONE = 0xffu;
inline constexpr unsigned XP_EFX_TABLE_MAX = 32u;

/* One parameter-conversion table: a run of big-endian words a parameter
   value indexes. `columns` is 2 where an entry is a pair - a pan table's
   two sides, a one-pole's two coefficients - and 1 otherwise. */
struct XpEfxTable {
  uint32_t base;
  uint16_t count;
  uint8_t columns;
};
inline constexpr unsigned XP_WAVE_SOURCE_MAX = 2u;
inline constexpr unsigned XP_MULTISAMPLE_BANK_MAX = 2u;

/* A DEVICE'S OWN VOICE PATH, INJECTED.
 *
 *   Two members of this family need two of these and share nothing
 *   per-voice. One is a firmware port: its firmware is dumped, its voice
 *   state is the chip's own register and RAM layout, and its scheduler
 *   recomposes every voice from ROM tables once per control period
 *   (engine.h, renderer.h). The other's synthesis engine is in an undumped
 *   internal mask ROM, so its voice path is a behavioural model fitted to
 *   measured hardware and has no control period, no amplitude register and
 *   no ROM table to read. A shared per-voice struct would be a lie about
 *   both.
 *
 *   What IS shared is everything above the voice: the top-level MIDI
 *   device, its channel state, the wave ROM descramble, the record readers
 *   and the output mixing. So a device supplies this table and the shared
 *   device layer calls through it, naming no device.
 *
 *   A device that leaves XpDeviceProfile::voiceEngine null is served by
 *   engine.h/renderer.h instead, which is where this family started.
 *
 *   `rom` and `banks` stay owned by the caller and outlive the state.
 *   `sysex_block` receives one DT1 payload whole, with its address as the
 *   bytes that arrived. Whole, because a device's own frames may run past
 *   an address-byte boundary and a per-address call has nowhere to carry;
 *   as bytes, because how those bytes pack into a number is the device's
 *   own convention and not something shared code should decide. */
struct XpVoiceEngineOps {
  bool (*create)(void **state, const struct xp_rom *rom,
                  const uint8_t *const banks[], const size_t bankSizes[],
                  unsigned bankCount, double outputRate);
  void (*destroy)(void *state);
  void (*reset)(void *state);
  bool (*note_on)(void *state, unsigned part, unsigned key,
                   unsigned velocity);
  bool (*note_off)(void *state, unsigned part, unsigned key);
  bool (*control_change)(void *state, unsigned channel, unsigned controller,
                          unsigned value);
  bool (*program_change)(void *state, unsigned part, unsigned program);
  bool (*sysex_block)(void *state, const uint8_t *address,
                       unsigned addressBytes, const uint8_t *data,
                       size_t count);
  void (*render)(void *state, float *stereo, size_t frames);
  bool (*set_max_voices)(void *state, unsigned maxVoices);
  unsigned (*active_voices)(const void *state);
  /* The universal GM System On. Null on a device with no GM mode. */
  bool (*gm_system_on)(void *state);
  /* Pitch bend as its 14-bit value, 8192 the centre, and channel
     aftertouch. Null on a device that does not act on them. */
  bool (*pitch_bend)(void *state, unsigned channel, unsigned value);
  bool (*channel_pressure)(void *state, unsigned channel, unsigned value);
  /* Renders the voices on `pool`'s threads, or on the calling thread when
     pool is null or has one thread, with the same output either way. False,
     leaving the voices on the calling thread, if the engine cannot take it.
     Null on a device whose voice path renders on the calling thread only. */
  bool (*set_worker_pool)(void *state, EmuSC::Xp::WorkerPool *pool);
  /* The host's panic, not a MIDI message: every part's voices cut, its
     pedals released and its controllers reset, whatever its receive channel
     and whatever the device's channel-mode messages would do. Programs and
     patches stay. Null on a device with nothing to stop. */
  void (*panic)(void *state);
};

/* Where a record type keeps each field a voice needs, by role. A device's
   melodic tone and its rhythm note are two record types with the same
   parts in different places, so the voice path reads through one of these
   rather than through a fixed set of offsets. XP_VOICE_FIELD_NONE marks a
   role a record type does not have. */
inline constexpr uint16_t XP_VOICE_FIELD_NONE = 0xffffu;

struct XpVoiceFieldMap {
  uint16_t enable;
  uint16_t waveGroup;
  uint16_t waveGroupId;
  uint16_t waveNumber;
  uint16_t waveGain;
  uint16_t level;
  uint16_t pan;
  /* The other per-note pan sources: pan key follow (an index 0..14, 7
     neutral), random pan depth (0..63), alternate pan depth (1..127, 64
     neutral). */
  uint16_t panKeyFollow;
  uint16_t randomPanDepth;
  uint16_t alternatePanDepth;
  uint16_t coarseTune;
  uint16_t fineTune;
  /* Random pitch depth, an index into the displayed-cents list 0..30. */
  uint16_t randomPitchDepth;
  /* FXM: its switch, its colour 0..3 and its depth 0..15. */
  uint16_t fxmSwitch;
  uint16_t fxmColor;
  uint16_t fxmDepth;
  /* Whether the record answers the bender and the hold pedal, and the
     bend range a record carries itself where it has no patch around it. */
  uint16_t benderSwitch;
  uint16_t holdSwitch;
  uint16_t benderRange;
  /* The A-ENV's velocity sensitivity, signed, 0 insensitive, and which of
     the device's velocity curves it reads through. */
  uint16_t ampVelocitySens;
  uint16_t ampVelocityCurve;
  /* The TVA key bias: direction 0 LOWER, 1 UPPER, 2 LOWER&UPPER, 3 ALL,
     the break key, and the level enum 0..14 (7 neutral). */
  uint16_t biasDirection;
  uint16_t biasPoint;
  uint16_t biasLevel;
  /* The two LFOs' eight fields each - waveform, key trigger, rate, level
     offset, delay, fade mode, fade time, external sync - from these
     indices on, and the first of each destination's two depths. */
  uint16_t lfoFirst[2];
  uint16_t pitchLfoDepth;
  uint16_t filterLfoDepth;
  uint16_t ampLfoDepth;
  uint16_t panLfoDepth;
  /* The controller matrix: three controllers of four destination/depth
     pairs each, twenty-four fields from this index on. */
  uint16_t matrixFirst;
  /* The rhythm note's envelope mode: 0 NO-SUSTAIN, 1 SUSTAIN. */
  uint16_t envelopeMode;
  /* The tone delay: which mode it runs in, and its time. */
  uint16_t toneDelayMode;
  uint16_t toneDelayTime;
  /* Indices into XpDeviceProfile::keyFollowTable. */
  uint16_t pitchKeyFollow;
  uint16_t cutoffKeyFollow;
  /* The key the wave is played at, where the record names one instead of
     transposing by the key that triggered it - which is what makes a drum
     a drum rather than a sample pitched to whatever key struck it. */
  uint16_t sourceKey;
  /* Where this record's audio leaves the chip: the mix, the insert effect,
     or one of the separate output pairs. A part's own assign OVERRIDES it
     unless the part says PATCH, which is the part deferring to whatever
     its records say (`M-006`, `M-019`). XP_VOICE_FIELD_NONE where a record
     type carries no such field. */
  uint16_t outputAssign;
  uint16_t cutoff;
  uint16_t resonance;
  uint16_t filterType;
  /* The filter envelope. Its depth is signed and its levels are not: the
     sign of the whole sweep is the depth's. A record type may carry one
     velocity-time sensitivity where another carries two, and may carry
     neither a velocity curve nor a time key follow at all - hence the
     separate indices rather than one run. */
  uint16_t filterEnvDepth;
  uint16_t filterEnvVelCurve;
  uint16_t filterEnvVelSens;
  uint16_t filterEnvVelTime1;
  uint16_t filterEnvVelTime4;
  uint16_t filterEnvTimeKeyFollow;
  uint16_t filterEnvTime1;       /* four times run from here */
  uint16_t filterEnvLevel1;      /* four levels run from here */
  /* The pitch envelope: its signed depth, then four times and four signed
     levels, each run from its first index. */
  uint16_t pitchEnvDepth;
  uint16_t pitchEnvVelSens;
  uint16_t pitchEnvVelTime1;
  uint16_t pitchEnvVelTime4;
  uint16_t pitchEnvTimeKeyFollow;
  uint16_t pitchEnvTime1;
  uint16_t pitchEnvLevel1;
  uint16_t ampTime1;             /* four times run from here */
  uint16_t ampLevel1;            /* three levels run from here */
  uint16_t ampEnvTimeKeyFollow;
  uint16_t ampEnvVelTime1;
  uint16_t keyRangeLow;
  uint16_t keyRangeHigh;
  uint16_t velocityRangeLow;
  uint16_t velocityRangeHigh;
  uint16_t velocityCrossFade;
  uint16_t muteGroup;
};

/* Which packed bank a bank-select pair names, as the device's own selector
   resolves CC0 and CC32. */
struct XpBankSelect {
  uint8_t msb;
  uint8_t lsb;
  uint8_t bank;
  /* The SAME pair reaches a different source on a rhythm part. The
     JV-1080's bank select resolves CC0/CC32 to a GROUP, and a group holds
     both a patch source and a rhythm source; which one is read is decided
     by the part's own rhythm flag, not by the bank select
     (`04_protocol/program_bank.md`, FW-EXACT). XP_PACKED_BANK_NONE where a
     group has no rhythm source this implementation holds an image for. */
  uint8_t rhythmBank;
  /* The device's own number for this group - on the JV-1080 the twelve-
     group enum of `0x0A014EF6` (USER 0, CARD 1, PR-A 2, PR-B 3, PR-C 4,
     GM 5, ...). A performance part record names its patch by group, and
     this is what the record's group id resolves to. */
  uint8_t group;
};

/* One group of a descriptor-packed record schema: a run of field
   descriptors that together tile one record (or one of its sub-records)
   with no hole and no overlap. A group is addressed by role rather than by
   number - see XpDeviceProfile::packedBanks - so generic code never needs
   to know which index is which. */
struct XpPackedGroup {
  uint16_t firstDescriptor;  /* index of this group's first descriptor */
  uint16_t fieldCount;       /* decoded bytes the group produces */
  uint16_t packedSize;       /* bytes the group occupies in the record */
};

/* A bank of descriptor-packed records: `count` records of `recordSize`
   packed bytes, each one a common block followed by `partCount` identical
   sub-records. partGroup is XP_PACKED_NO_GROUP where a record has no
   sub-records. */
inline constexpr uint8_t XP_PACKED_NO_GROUP = 0xffu;

struct XpPackedBank {
  uint32_t base;
  uint16_t count;
  uint16_t recordSize;
  uint8_t commonGroup;
  uint8_t partGroup;
  uint8_t partCount;
};

/* One wave-number namespace: the two parallel lookup tables that turn a
   stored wave number into a (multisample bank, multisample row) pair, and
   how many numbers the namespace holds. */
struct XpWaveSource {
  uint32_t listRowTable;    /* u16 BE per wave number */
  uint32_t listBankTable;   /* u8 per wave number */
  uint16_t listCount;
  uint8_t multisampleBank;  /* unused; the row table's own bank byte wins */
  uint8_t elementDirectory; /* which element directory this source reads */
};

/* A table of fixed-stride records, used for both the multisample tables
   and the wave-element directories. */
struct XpRecordTable {
  uint32_t base;
  uint16_t count;
  uint16_t stride;
};

/* Where a wave-element record keeps each field. Byte offsets inside the
   record; a u24 field is big-endian and three bytes wide. */
struct XpElementLayout {
  uint8_t attenuation;
  uint8_t start;            /* u24 */
  uint8_t loop;             /* u24 */
  uint8_t end;              /* u24, inclusive */
  uint8_t control;          /* loop mode in the low bits, reverse flag */
  uint8_t rootKey;
  uint8_t fineTune;         /* u16 */
  uint8_t loopFineTune;     /* u16 */
  uint8_t reverseMask;      /* control bit selecting reverse playback */
  uint8_t loopModeMask;     /* control bits holding the loop mode */
};

/* Where a multisample record keeps its name, its key split points and its
   element references. */
struct XpMultisampleLayout {
  uint8_t name;
  uint8_t nameLength;
  uint8_t splitPoints;
  uint8_t splitCount;
  uint8_t elementRefs;      /* u16 BE each, 0xffff for an unused slot */
  uint8_t refCount;
};

/* Every other device fact: ROM table addresses, measured/fitted values and
   record sizes. See a device's own devices/<device>.cc for the values and
   their provenance comments. A device leaves at zero every field its own
   ROM has no counterpart for; the reader of each field checks that before
   using it, so a zero means "this device has no such table" rather than
   "address zero". */
struct XpDeviceProfile {
  /* Identification: rom_init() reads these, never a device's own bytes -
     the size, and two 16-byte signatures memcmp'd at offset 0 and at
     identSecondOffset, matching this data against the incoming ROM image.
     This is the only role these four fields play; everything else in
     the struct is read after identification has already chosen this
     profile. A device picks the second span for being fixed content its
     ROM carries at a known address, which is not the same question as
     which table that address belongs to. */
  size_t romSize;
  uint8_t identVectors[16];
  uint8_t identFirstDirectory[16];
  uint32_t identSecondOffset;

  unsigned defaultMaxVoices;

  uint32_t envelopeRateTable;
  uint32_t rateScaleTable;
  uint32_t releasePedalTable;

  unsigned toneCommonSize;
  unsigned componentSize;

  uint32_t pointerTableBase;
  uint32_t pointerBankSize;
  uint32_t melodicMapBase;

  uint32_t directoryBase;
  uint32_t directoryEnd;
  uint32_t descriptorBase;
  uint32_t descriptorEnd;

  uint32_t toneBase;
  uint32_t toneEnd;

  uint32_t drumMapBase;
  uint32_t drumPointerTable;
  uint32_t drumKitCount;
  uint32_t drumKitStride;
  uint32_t drumKitBase;

  uint32_t levelTable;
  uint32_t coarseGainTable;
  uint32_t fineGainTable;
  uint32_t ampCurve1Table;
  uint32_t ampCurve0Table;

  uint32_t portamentoRateTable;

  uint32_t baseTable;
  uint32_t limitTable;

  uint32_t rateTable;
  uint32_t delayTable;
  uint32_t sineTable;
  uint32_t table10;
  uint32_t table12;
  uint32_t table14;
  uint32_t table16;
  uint32_t tablePoints;

  uint16_t interpolateBelow;

  uint32_t panTable;
  uint32_t sendTable;

  uint32_t eqLow200;
  uint32_t eqLow400;
  uint32_t eqHigh3k;
  uint32_t eqHigh6k;
  uint8_t eqGainMin;
  uint8_t eqGainMax;

  uint32_t delayCentreTable;
  uint32_t delayRatioTable;
  uint32_t delayMacroTable;
  double delayUnitsPerMs;
  unsigned delayMaxUnits;
  double delayMaxMs;

  uint32_t reverbPointers;
  uint32_t reverbPage;
  uint32_t reverbMacroTable;
  uint16_t reverbAllpassPairA;
  uint16_t reverbAllpassPairB;
  float reverbAllpassG;
  uint32_t reverbImage0Cram;
  uint8_t headWord[XP_REVERB_BUFFERS];
  uint8_t farWord[XP_REVERB_BUFFERS];
  uint8_t tapWord[XP_REVERB_TAPS];
  uint8_t allpassBuffer[8];
  uint8_t tapInstruction[XP_REVERB_TAPS];

  /* WHERE EACH PART OF A CHARACTER RECORD SITS, IN WORDS FROM ITS START.
     The two devices carry the SAME record - eight allpass coefficient
     pairs, two damping pairs, then twelve buffer heads, twelve far-end
     reads and eight taps as thirty-two delay-memory addresses - and they
     differ only in where those parts begin, so the loader reads offsets
     rather than knowing them. A device whose record has no such part sets
     XP_REVERB_WORD_NONE and the loader leaves that field at its default.

     `reverbInputWord` is the one part the two do not share: a per-character
     input one-pole, which the SC-88 does not carry in its record at all
     (its pre-LPF comes from a parameter instead). */
  uint8_t reverbCharacters;
  uint8_t reverbRecordWords;
  uint8_t reverbAllpassWord;
  uint8_t reverbDampWord;
  uint8_t reverbAddressWord;
  uint8_t reverbTrimWord;
  uint8_t reverbInputWord;
  /* How the per-character pointer table is written. The SC-88 stores a u16
     offset from `reverbPage`. A device storing whole pointers sets
     `reverbPointerBytes` to 4, and `reverbPointerBase` to the address its
     ROM is mapped at, which is subtracted to get a file offset. */
  uint8_t reverbPointerBytes;
  uint32_t reverbPointerBase;
  /* Two tables a device may drive its reverb parameters through, zero
     where it has neither. `reverbDampTable` is 18 rows of (a, 0x1FFF-a)
     whose `a` is the one-pole coefficient of a lowpass at the row's own
     corner (`M-064`, exact to the unit, row 17 the bypass);
     `reverbLevelTable` is 128 monotonic words against 8192 = unity. */
  uint32_t reverbDampTable;
  uint32_t reverbLevelTable;

  uint32_t chorusMacroTable;
  double chorusMaxMs;
  /* WHICH SHAPE THE CHORUS DELAY IS SWEPT WITH, and how far.

     `XP_CHORUS_MOD_SINE` sweeps symmetrically about the nominal delay,
     which is what this engine has always done; the SC-88's own shape has
     never been measured and that is the assumption it keeps.

     `XP_CHORUS_MOD_TRIANGLE_UP` sweeps a straight line from the nominal
     delay UPWARD and back, never below it - measured on the JV-1080, where
     the delay traces +1.2280 and +1.2372 ms/s on the two halves of its
     rise and -1.2112 on its fall, straight to a few hundredths of a
     millisecond, against the factor of two a sine would put between its
     own early and late slopes. */
  uint8_t chorusModulator;
  /* WHICH TAP THE CHORUS FEEDS BACK into the head of its line.

     `XP_CHORUS_FB_TAP_MEAN` feeds back the mean of the two output taps.
     The SC-88's own feedback topology has never been measured and that is
     the assumption it keeps.

     `XP_CHORUS_FB_TAP_LEFT` feeds back the left output tap alone -
     measured on the JV-1080 (`M-112`): with the taps apart, the left
     channel's second-order echo sits at twice the left delay only and the
     right channel's at the sum of the two delays only, the other terms
     at -0.006 and -0.003 of the first order. The mean of two taps at
     different delays is a comb inside the loop that discards energy every
     pass, which is what the mean cannot reproduce here: a modulated
     feedback-127 ring that decays at -2.3 to -2.4 dB/s on hardware. */
  uint8_t chorusFeedbackTap;
  /* THE HIGH-PASS INSIDE THE CHORUS'S FEEDBACK LOOP, as the corner of a
     one-pole section in hertz; zero for none.

     Measured on the JV-1080 (`P-xxxx`): the static feedback-127 ring's
     comb teeth each lose a fixed amount per pass, and that loss falls as
     1/f^2 from 0.085 dB at 142 Hz to a flat floor above about 1.5 kHz.
     Fitted over 54 teeth from 142 Hz to 3.9 kHz, a one-pole high-pass
     reads 19.68 +- 0.04 Hz with 0.0003 dB per pass left over; the same
     analysis reads a known 19.96 Hz section, simulated at the same loop
     length and loss, as 19.67 Hz, so the corner is 19.97 +- 0.04 Hz.
     The DSP coefficient behind it is not decoded. A filter outside the loop
     would scale a tooth, not change how fast it decays, so this one is in
     the loop. Whether it sits on the fed-back signal or on the whole line
     input the measurement cannot tell; at 20 Hz the difference to the
     first pass is nil, and it is applied to the fed-back signal so a
     feedback of zero leaves the chorus exactly as it was. The SC-88 has
     never been measured and keeps none. */
  double chorusLoopHighpassHz;
  /* The tables a device drives its chorus from, zero where it has none.
     `chorusRateAccumulator` is the modulus the rate table's entry is a
     per-control-period increment on, so the modulation is
     `table[rate] / accumulator / control period` hertz. `chorusDepthMaxMs`
     is the peak-to-peak sweep at the top of the depth field, which the
     depth table's own shape scales. */
  uint32_t chorusPreDelayTable;
  uint32_t chorusRateTable;
  double chorusRateAccumulator;
  uint32_t chorusDepthTable;
  double chorusDepthMaxMs;
  uint32_t chorusLevelTable;

  uint32_t pitchCurveCentre;

  unsigned waveDescriptorSize;
  unsigned waveBankSize;
  unsigned waveChipSize;

  /* The wave ROM board's bit permutations, and the shape of the ROM they
     undo (wave_descramble_chip). A board wires its scrambling to whatever
     the chips are: waveUnitBytes is the width of one addressable storage
     unit in the dump - 1 for a byte-wide ROM, 2 for a 16-bit word ROM
     dumped little-endian within the word - and the two line counts are how
     many data and (unit-)address lines that board actually permutes. The
     arrays are sized for the widest member of the family and only the
     first waveDataLineCount / waveAddressLineCount entries are read. */
  unsigned waveUnitBytes;
  unsigned waveDataLineCount;
  unsigned waveAddressLineCount;
  uint8_t waveDataLinePermutation[XP_WAVE_DATA_LINES_MAX];
  uint8_t waveAddressLinePermutation[XP_WAVE_ADDRESS_LINES_MAX];

  /* The plaintext header each ROM carries, which the board leaves out of
     the scrambling: waveHeaderBytes bytes at every multiple of
     waveHeaderStride inside a chip. A stride equal to waveChipSize means
     one header per chip. */
  unsigned waveHeaderBytes;
  uint32_t waveHeaderStride;

  double belowPower;
  double partialPower;

  uint8_t selectors[XP_WAVE_BANK_COUNT];

  /* --- Descriptor-packed preset records --------------------------------
     A device whose preset records are bit-packed carries one field
     descriptor table: packedDescriptorCount records of
     packedDescriptorStride bytes at packedDescriptorBase, each naming a
     mask, a byte offset inside the record, a shift and a signed bias. The
     groups partition that table into record schemas and the banks say
     where the records themselves are. packedDescriptorBase is zero on a
     device whose records are flat fixed-offset ones instead, and then
     nothing in packed_rom.h applies to it. */
  uint32_t packedDescriptorBase;
  unsigned packedDescriptorStride;
  unsigned packedDescriptorCount;
  /* Byte offsets inside one descriptor. */
  uint8_t packedMaskOffset;       /* u16 BE */
  uint8_t packedByteOffset;
  uint8_t packedShiftOffset;
  uint8_t packedBiasOffset;       /* i8 */
  uint8_t packedMinOffset;
  uint8_t packedMaxOffset;

  struct XpPackedGroup packedGroups[XP_PACKED_GROUP_MAX];
  unsigned packedGroupCount;
  struct XpPackedBank packedBanks[XP_PACKED_BANK_MAX];
  unsigned packedBankCount;

  /* Which bank a melodic program change selects, as an index into
     packedBanks, one entry per selectable bank in program-change order.
     A rhythm part reads packedRhythmBanks the same way. */
  uint8_t packedMelodicBanks[XP_PACKED_BANK_MAX];
  unsigned packedMelodicBankCount;
  uint8_t packedRhythmBanks[XP_PACKED_BANK_MAX];
  unsigned packedRhythmBankCount;

  /* Which packed bank each bank-select pair reaches. A pair not listed
     here names a card or expansion group this implementation has no
     image for, and selects nothing. */
  struct XpBankSelect packedBankSelect[XP_PACKED_BANK_MAX];
  unsigned packedBankSelectCount;

  /* Where the performance part record names its patch: a group type, a
     group id within that type, and a patch number followed by its alias.
     A program change whose latched bank pair does not resolve falls back
     to the group these name, and a successful one writes them back. */
  uint16_t partFieldPatchGroupType;
  uint16_t partFieldPatchGroupId;
  uint16_t partFieldPatchNumber;
  /* PRG table turning a type-0 group id into a group, and how many ids it
     holds. An id past the end, or any other type, names a card or board. */
  uint32_t partGroupIdTable;
  uint8_t partGroupIdCount;

  /* GM MODE. The 20-byte part record the device's GM System On copies
     into every part, its only per-part change being the receive channel;
     which packedBankSelect entry the mode forces every program change to;
     and the CC7 volume the mode leaves on each part. gmPartTemplate is 0
     on a device with no GM mode. */
  uint32_t gmPartTemplate;
  uint8_t gmBankSelect;
  uint8_t gmVolume;

  /* THE POWER-ON SELECTION, which a host reset also leaves behind.
     `XP_POWER_ON_PATCH` puts one patch on the patch-mode part, receiving
     on `powerOnPatchChannel` (0-based), named the way a part record names
     one: a type-0 group id resolved through `partGroupIdTable`, and a
     number within that group. `XP_POWER_ON_NONE` leaves every part as
     `reset` makes it. */
  uint8_t powerOnMode;
  uint8_t powerOnPatchGroupId;
  uint8_t powerOnPatchNumber;
  uint8_t powerOnPatchChannel;

  /* WHAT A HOST RESET LEAVES BEHIND, as distinct from power-on. True puts
     the device in its GM mode through its own GM System On, whatever
     sound map the host asked for, so a MIDI file that assumes sixteen
     GM parts finds them; the power-on state above is what a host that
     sends no reset gets. False leaves a host reset at the device reset. */
  bool hostResetEntersGm;

  /* Where the melodic tone record and the rhythm note record keep each
     field the voice path reads. Each index is the descriptor's own index
     within its group, which on a device whose descriptor table doubles as
     its SysEx address map is also the parameter's SysEx offset. */
  struct XpVoiceFieldMap toneFields;
  struct XpVoiceFieldMap rhythmNoteFields;

  /* The value list pitch and cutoff key follow index: `keyFollowCount`
     fixed-width ASCII entries of `keyFollowWidth` characters, each a
     signed percentage - of one semitone per key for pitch, of one octave
     of corner per octave of key for cutoff. Zero where a device has none,
     and then pitch tracks the key at 100 % and cutoff not at all. */
  uint32_t keyFollowTable;
  uint8_t keyFollowCount;
  uint8_t keyFollowWidth;

  /* The value list the envelopes' time key follow indexes, in the same
     format: each a signed percentage, +-100 % being one doubling of time
     per octave of key. Zero where a device has none, and then envelope
     times do not follow the key. */
  uint32_t timeKeyFollowTable;
  uint8_t timeKeyFollowCount;
  uint8_t timeKeyFollowWidth;

  /* The rhythm set: which packed groups its common and per-key records
     are, which part index addresses it, and the first key it holds. */
  uint8_t packedRhythmCommonGroup;
  uint8_t packedRhythmNoteGroup;
  uint8_t rhythmPartIndex;
  uint8_t rhythmFirstKey;
  uint8_t rhythmKeyCount;

  /* Field indices inside the patch-common group. The octave shift is a
     whole-patch transposition that adds to the tone's own coarse tune, and
     is XP_VOICE_FIELD_NONE on a patch record that has no such field. */
  uint16_t patchFieldName;
  uint16_t patchFieldNameLength;
  uint16_t patchFieldLevel;
  uint16_t patchFieldPan;
  uint16_t patchFieldOctaveShift;
  /* The structure of tones 1 and 2 and of tones 3 and 4, 0..9 for the
     panel's types 1 to 10, and each pair's booster, 0..3. */
  uint16_t patchFieldStructure12;
  uint16_t patchFieldBooster12;
  uint16_t patchFieldStructure34;
  uint16_t patchFieldBooster34;
  /* The patch's bend range, up and down, in semitones. */
  uint16_t patchFieldBendUp;
  uint16_t patchFieldBendDown;
  /* The sources of matrix controllers 2 and 3; controller 1's is fixed. */
  uint16_t patchFieldControlSource2;
  uint16_t patchFieldControlSource3;
  /* The patch's velocity range switch: off, its tones' velocity ranges do
     not gate them. */
  uint16_t patchFieldVelocityRangeSwitch;
  /* The patch's key assign mode: 0 POLY, 1 SOLO. */
  uint16_t patchFieldKeyAssign;
  /* SOLO legato, 0 OFF, 1 ON, and portamento: switch, mode (0 NORMAL,
     1 LEGATO), type (0 RATE, 1 TIME), start (0 PITCH, 1 NOTE) and time
     0..127. */
  uint16_t patchFieldSoloLegato;
  uint16_t patchFieldPortamentoSwitch;
  uint16_t patchFieldPortamentoMode;
  uint16_t patchFieldPortamentoType;
  uint16_t patchFieldPortamentoStart;
  uint16_t patchFieldPortamentoTime;

  /* The performance-part group, and the fields of it that decide where a
     note goes and how loud it is. A part is addressed by its own receive
     channel rather than by its index, so two parts may share a channel and
     layer - which this device's factory songs rely on. */
  uint8_t packedPerformanceCommonGroup;
  uint8_t packedPerformancePartGroup;
  uint16_t partFieldReceiveChannel;
  uint16_t partFieldLevel;
  uint16_t partFieldPan;
  uint16_t partFieldKeyShift;
  /* The part's own output assign, which decides for the whole part unless
     it reads PATCH - the one value that hands the decision to the record.
     `partOutputAssignPatch` is that value. */
  uint16_t partFieldOutputAssign;
  uint8_t partOutputAssignPatch;
  /* The part's own reverb send. `M-039`/`M-052`: the PART's send is the
     live one and a melodic tone's is inert, so a voice carries its part's
     at note-on. XP_VOICE_FIELD_NONE where a device has no such field. */
  uint16_t partFieldReverbSend;
  uint16_t partFieldChorusSend;
  uint16_t partFieldFineTune;    /* cents, XP_VOICE_FIELD_NONE where absent */

  /* --- Wave selection through the multisample directories -------------
     The chain a stored wave reference walks on a device whose wave ROM is
     described by multisample records rather than by one flat descriptor
     table: a wave group ID selects a source, the source's two lookup
     tables give a multisample bank and row, the row gives key splits and
     element references, and the element record gives the ROM addresses.
     waveGroupSourceTable is zero on a device with no such chain. */
  uint32_t waveGroupSourceTable;
  unsigned waveGroupSourceCount;
  struct XpWaveSource waveSources[XP_WAVE_SOURCE_MAX];
  unsigned waveSourceCount;
  struct XpRecordTable multisampleBanks[XP_MULTISAMPLE_BANK_MAX];
  unsigned multisampleBankCount;
  struct XpMultisampleLayout multisampleLayout;
  struct XpRecordTable elementDirectories[XP_MULTISAMPLE_BANK_MAX];
  unsigned elementDirectoryCount;
  struct XpElementLayout elementLayout;

  /* The wave-address space element records are written in: how wide one
     chip's slice of it is, so that an element address resolves to a chip
     and an offset inside it, and which chip each directory's slot zero
     is. Zero where the element record carries its own bank byte instead. */
  uint32_t waveSourceSlotSize;
  uint8_t elementDirectoryChipBase[XP_MULTISAMPLE_BANK_MAX];

  /* This device's System Exclusive identity and address width. Roland's
     own model id, and how many seven-bit bytes its parameter address
     takes - three on a GS device, four on this family's JV member. */
  uint8_t sysexModelId;
  uint8_t sysexAddressBytes;

  /* True on a device the shared firmware-port engine serves whose firmware
     has a GM mode: the universal GM System On, 7E 7F 09 01, is received
     and puts every part in it. A device with its own voice path says the
     same through XpVoiceEngineOps::gm_system_on instead, and leaves this
     false. */
  bool firmwarePortGmMode;

  /* How far one voice sits below the mix's full scale. A device that sums
     many voices into a fixed-width mixer needs headroom for their sum, and
     on a device whose mixer is not recovered this is the one number that
     stands for it. Zero means unity - the shared firmware-port engine
     composes its amplitude from the ROM's own headroom instead and has no
     use for this. */
  double voiceMixScale;

  /* --- The insert effect's program bank -------------------------------
     A device whose insert effect is a loadable DSP program keeps a bank of
     fixed-stride slots - one program image then one coefficient image to a
     slot - and a table mapping the effect TYPE onto a slot. Several types
     share a slot, being the same program under different coefficients,
     which is why the bank is smaller than the type list.

     `efxBankBase` is zero on a device with no such bank, and then nothing
     in `efx.cc` will read anything. `efxPointerBase` is the address the ROM
     is mapped at, which the table's entries are written in and which is
     subtracted to reach a file offset. */
  uint32_t efxBankBase;
  uint32_t efxSlotStride;
  uint16_t efxSlotCount;
  uint32_t efxTypeTable;
  uint16_t efxTypeCount;
  uint32_t efxPointerBase;
  /* The level table the EFX output block reads, shared in this device's ROM
     with the chorus's and the reverb's but named here per block because it
     is that block's own reference. */
  uint32_t efxLevelTable;
  /* The conversion tables the effect parameters read, in the order a
     device's own extraction lists them. Named indices for the ones whose
     meaning is established are in efx.h; the rest are reachable by index
     and carry no claim about what they convert. */
  struct XpEfxTable efxTables[XP_EFX_TABLE_MAX];
  unsigned efxTableCount;

  /* Null on a device the shared firmware-port engine serves; see
     struct XpVoiceEngineOps above. */
  const struct XpVoiceEngineOps *voiceEngine;
};

/* Never-null: falls back to the profile of whichever device this engine
   was first written for, since several existing tests build a bare struct
   xp_rom by hand (bypassing rom_init(), so .profile is never set).
   Mirrors ControlRom::device()'s own fallback (the older engine's), not
   its nullable profile(). A device whose ROM rom_init() has identified
   gets its own profile back instead. */
const struct XpDeviceProfile *xp_profile(const struct xp_rom *rom);

#ifdef __cplusplus
}
#endif

#endif
