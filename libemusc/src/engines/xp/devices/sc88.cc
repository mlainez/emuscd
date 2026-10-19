/* SPDX-License-Identifier: CC0-1.0 */
/*
 *  Roland SC-88 device profile for the XP engine (engines/xp/).
 *
 *  The single instance below is this device's DeviceProfile equivalent -
 *  see sc88.h for the struct and the injection mechanism (xp_rom::profile,
 *  xp_engine::profile, xp_profile()).
 */
#include "sc88.h"
#include "../rom.h"

extern "C" {

const struct XpDeviceProfile SC88_PROFILE = {
  /* rom_init()'s own identification data - never referenced past that
     one function. */
  .romSize = XP_CONTROL_ROM_SIZE,
  .identVectors = {
    0x00, 0x00, 0x02, 0x00, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x01, 0xf4, 0x00, 0x00, 0x01, 0xf4 },
  .identFirstDirectory = {
    0x00, 0x00, 'P', 'i', 'a', 'n', 'o', ' ',
    '1', 'A', ' ', ' ', ' ', ' ', 0x03, 0xff },
  /* The first tone directory record, which is also .directoryBase below. */
  .identSecondOffset = 0x30000u,

  .defaultMaxVoices = 64u,          /* the real hardware's polyphony */

  /* The envelope rate, key/velocity rate-scale and hold-pedal release-scale
     ROM tables, shared by the pitch, TVF and TVA envelopes alike. */
  .envelopeRateTable = 0x1543eu,
  .rateScaleTable = 0x1573eu,
  .releasePedalTable = 0x78a02u,

  /* The fixed-size tone/component records rom_open_tone and
     rom_open_component step by. */
  .toneCommonSize = 34u,
  .componentSize = 148u,

  /* Melodic lookup: the physical-bank pointer table, its per-bank stride,
     and the map that resolves a map/variation pair to a physical bank
     (rom_select_melodic). */
  .pointerTableBase = 0x20000u,
  .pointerBankSize = 384u,
  .melodicMapBase = 0x2fc00u,

  /* The tone directory and wave descriptor regions (rom_select_zone). */
  .directoryBase = 0x30000u,
  .directoryEnd = 0x3606cu,
  .descriptorBase = 0x36100u,
  .descriptorEnd = 0x3f714u,

  /* The tone data region: common header plus components (rom_open_tone). */
  .toneBase = 0x40000u,
  .toneEnd = 0x75000u,

  /* Drum kit lookup: map to physical kit index, kit pointer table, and the
     kit records themselves (rom_select_drum). */
  .drumMapBase = 0x2fd00u,
  .drumPointerTable = 0x2b550u,
  .drumKitCount = 24u,
  .drumKitStride = 0x50cu,
  .drumKitBase = 0x23c30u,

  /* The level-word table, converting a 0..127 level - master, secondary,
     part, expression, or a rhythm note's own level - into an attenuation
     (tva_gain_from_headroom_q17). */
  .levelTable = 0x14f3eu,
  /* Coarse and fine gain tables; their paired 16-bit product converts an
     attenuation into a linear gain. */
  .coarseGainTable = 0x1503eu,
  .fineGainTable = 0x1523eu,
  /* Linear (family 1) and exponential (family 0) envelope-rate curve
     tables, selected by the component's own per-stage curve-family bit. */
  .ampCurve1Table = 0x1553eu,
  .ampCurve0Table = 0x1563eu,

  /* Portamento rate table: 128 big-endian 32-bit entries indexed by the
     raw CC5 byte (portamento_rate). */
  .portamentoRateTable = 0x78502u,

  /* The per-key cutoff (TVF-F) base table: one semitone per index, in the
     same log-sine domain kXpTvfQUnity/kXpTvfOctaveUnits/XP_TVF_NYQUIST_WORD
     (devices/sc88.h) below decode (tvf_word_to_hz). */
  .baseTable = 0x78702u,
  /* @provenance class=FW-EXACT devices=SC-88 ref=0x78802
   * The limit table is the filter's own topology, written down.
   *
   * 0x78802 holds one cutoff ceiling per resonance index. Read in the base
   * table's domain - f = 2*sin(pi*fc/32000), the Chamberlin coefficient
   * the base table already is - and with q = resonance_index/64, every
   * one of its 128 entries satisfies
   *
   *     f*f + f*q = 2
   *
   * to a rounding unit: the residual against the exact law is -0.53 +-
   * 0.29 table units and lies in [-0.99, 0], which is the signature of a
   * floor(), not of a fit. In octaves that is 0.33 cent of cutoff.
   *
   * That expression is not a general fact about two-pole filters. It is
   * the trace of the state matrix of the forward-Euler state-variable
   * filter
   *
   *     lp[n] = lp[n-1] + f*bp[n-1]
   *     hp[n] = x[n] - lp[n] - q*bp[n-1]
   *     bp[n] = bp[n-1] + f*hp[n]
   *
   * whose trace is 2 - f*f - f*q and whose determinant is 1 - f*q. Setting
   * the trace to zero puts the pole pair at exactly +-90 degrees: the chip
   * refuses to let the filter's realised resonance climb past fs/4, and
   * the ceiling it writes for each resonance is the cutoff at which that
   * happens. A trapezoidal (bilinear) realisation is stable for every
   * positive coefficient and has no such ceiling to tabulate; its own
   * coefficient g = tan(pi*fc/fs) puts g*g + g*q at 0.906..1.000 over the
   * same table, and reading the word as a frequency instead of a sine
   * puts it at 3.211..3.414. The topology is forward-Euler.
   *
   * This is the SC-55's filter. libEmuSC's svf.cc is the same three
   * lines, and the SC-55's control ROM carries the same law twice:
   * TVFResonance[] is 128*(sqrt(q*q+4) - q) and TVFResonanceFreq[] is
   * 32*(4-f*f)/f, the two sides of that state matrix's stability edge,
   * with q = r/64 and f = x/128 (P-0130, P-0131). The SC-88 tabulates the
   * same topology at a tighter bound.
   *
   * Two things follow that the audio path could not have been told any
   * other way. The coefficient is the ROM's own word and is not warped
   * again; and a tone whose computed cutoff is clamped to this ceiling is
   * a tone the chip has been asked to leave alone - at q = 1 the law
   * gives f = 1 exactly, where the difference equations above collapse to
   * y[n] = x[n-1], a bare sample of delay. "Wide open" is realised as
   * transparent, which is what asking for a cutoff above the ceiling
   * should mean. */
  .limitTable = 0x78802u,

  /* SC88-CTL v1.01 offsets. The two increment tables sit immediately
     after the 16-entry callback dispatch table at 0x29ba. */
  .rateTable = 0x29dau,
  .delayTable = 0x2adau,

  /* The sine table and the four selectable waveform tables (lfo_waveform
     selectors 0x00, 0x10, 0x12, 0x14, 0x16), and their shared point count
     (lfo_table_sample). */
  .sineTable = 0x1492cu,
  .table10 = 0x14524u,
  .table12 = 0x14626u,
  .table14 = 0x14728u,
  .table16 = 0x1482au,
  .tablePoints = 129u,

  /* Below this increment, lfo_table_sample interpolates between adjacent
     table entries instead of reading the nearer one alone. */
  .interpolateBelow = UINT16_C(0x0200),

  /* The pan and effect-send gain tables. The effect sends do NOT read the
     pan table. They have their own, and it is a different shape: 128
     words at 0x15eb6, indexed by the control value WHOLE rather than by
     `value - 1`, and exactly `64 * floor((value * 512 + 63) / 127)` on
     every one of the 128 - a linear Q15 gain with 0x8000 for unity
     (`P-xxxx`). The firmware reaches it from a different routine than the
     pan pair does. The two tables agree at only three points, so reading
     one for the other is audible: it opens the send 1.4 dB too far
     around the middle of the range, and at control 1 the pan table's
     first word is 0, which closes a send the chip would have left open. */
  .panTable = 0x15db6u,
  .sendTable = 0x15eb6u,

  /* The EQ's four 25-record blocks, six bytes each, indexed by gain -
     0x34. */
  .eqLow200 = 0x1609cu,
  .eqLow400 = 0x16132u,
  .eqHigh3k = 0x161c8u,
  .eqHigh6k = 0x1625eu,
  .eqGainMin = 0x34u,
  .eqGainMax = 0x4cu,

  .delayCentreTable = 0x15fb4u,
  .delayRatioTable = 0x165cau,
  .delayMacroTable = 0x158beu,
  /* The delay memory counts in 1/32 ms in this path, and both the centre
     time and the side taps cap at 0x7d00 above the 0x8000 base: one
     second. */
  .delayUnitsPerMs = 32.0,
  .delayMaxUnits = 0x7d00u,
  .delayMaxMs = 1000.0,

  /* SC88-CTL v1.01. The character pointers are offsets **within the
   * 0x10000 page**; read as absolute addresses they land in unrelated
   * code and decode to plausible nonsense, which is the trap recorded in
   * `M-008`. */
  .reverbPointers = 0x1595eu,
  .reverbPage = 0x10000u,
  /* The eight macro presets, 8 bytes each, read by SC88-CTL handler
   * 0x3388 and by the power-on loader at 0x4476. The reset image at ROM
   * 0x13104 - the patch common block whose first sixteen bytes are the
   * default patch name - carries macro 4 and that macro's own seven
   * bytes, so a GS reset is this table's Hall 2 row and not a separate
   * set of defaults. */
  .reverbMacroTable = 0x1583eu,
  .reverbAllpassPairA = 0x3000u,   /* -0.5 under the XP law */
  .reverbAllpassPairB = 0x1000u,   /* +0.5 */
  .reverbAllpassG = 0.5f,
  /* The single-module DSP image and its coefficient RAM. CRAM[i] is the
   * coefficient of PRAM[i] - no field selects it (`M-173`) - so the gain
   * of a tap is the word at the tap's own instruction index. */
  .reverbImage0Cram = 0x78b02u + 0x480u,

  /* The reverb's delay-line graph, read out of the DSP program in the
   * control ROM (`M-173`, scdb `08_effects/dsp_program.md`). Twelve ERAM
   * buffers, eight output taps:
   *
   *   B0 B1 B2 B3   four series allpasses at g = 0.5, the input diffuser
   *   B4 B5 B6 B7   tank half 1: allpass, delay, allpass, delay
   *   B8 B9 B10 B11 tank half 2: allpass, delay, allpass, delay
   *   eight taps    read inside the tank, two per pair of program slots
   *
   * The twelve buffer heads are the twelve instructions with the ERAM
   * write enable (bit 24) set; the twenty reads have it clear, and each
   * buffer's far end sits one address below the next head. The eight
   * taps are reads that land inside a buffer rather than at its end, and
   * they are the early field.
   *
   * Which of the record's 32 delay-memory addresses is which: the record
   * always carries them in program order of the instructions they patch -
   * writes at 45 49 53 57 65 67 69 71 81 83 85 87, far-end reads at
   * 41 43 47 51 59 61 63 55 75 77 79 73 and taps at
   * 89 91 93 95 97 99 101 103 for the first module, the same sequence
   * shifted for the other two layouts. These three arrays are that order
   * read back as buffer roles. */
  .headWord = { 0u, 2u, 4u, 6u, 8u, 10u, 14u, 16u, 20u, 22u, 26u, 28u },
  .farWord = { 1u, 3u, 5u, 7u, 9u, 13u, 15u, 19u, 21u, 25u, 27u, 31u },
  .tapWord = { 11u, 17u, 23u, 29u, 12u, 18u, 24u, 30u },
  /* The eight coefficient pairs belong to the eight buffers whose write
   * instruction carries +0.5, in the same order the record lists them. */
  .allpassBuffer = { 0u, 1u, 2u, 3u, 4u, 6u, 8u, 10u },
  /* PRAM indices of the eight taps in the single-module image. */
  .tapInstruction = { 131u, 133u, 135u, 137u, 139u, 141u, 143u, 145u },
  /* This device's own record map, in words from the record's start: the
     eight allpass coefficient pairs at 0, the two damping pairs at 16, the
     thirty-two delay-memory addresses at 20, and the return trim at 52.
     Ten characters of 53 words each, reached through a u16 offset from
     `reverbPage`. There is no per-character input filter in the record -
     the pre-LPF is a parameter on this device (`M-010`) - so that index is
     absent. */
  .reverbCharacters = 10u,
  .reverbRecordWords = 53u,
  .reverbAllpassWord = 0u,
  .reverbDampWord = 16u,
  .reverbAddressWord = 20u,
  .reverbTrimWord = 52u,
  .reverbInputWord = XP_REVERB_WORD_NONE,
  .reverbPointerBytes = 2u,
  .reverbPointerBase = 0u,
  .reverbDampTable = 0u,
  .reverbLevelTable = 0u,

  /* The eight macro presets, 8 bytes each, read by SC88-CTL handler
     0x3400 and by the power-on loader at 0x44a8. The reset image at ROM
     0x13104 carries macro 2 and that macro's own eight bytes, which are
     the manual's printed chorus defaults byte for byte. */
  .chorusMacroTable = 0x1587eu,
  /* `3*p` reaches 381 samples and the sweep is added on top, so the
     chorus line is sized for the longest delay the register can ask for
     plus the deepest sweep, with a margin for interpolation. Both the
     chorus and delay lines count in samples at 32 kHz, the rate the
     sound chip runs its lines at, so every recovered length is converted
     from that. */
  .chorusMaxMs = 64.0,
  /* Unmeasured on this device: the mean of both taps is an assumption,
     not a reading (see `chorusFeedbackTap`). */
  .chorusFeedbackTap = XP_CHORUS_FB_TAP_MEAN,

  /* Centre of the 255-word bipolar pitch-control curve at
   * 0x78304..0x78502, indexed -127..127 about this address
   * (`02_rom/tables.md`). */
  .pitchCurveCentre = 0x78402u,

  .waveDescriptorSize = 20u,
  .waveBankSize = 0x100000u,
  .waveChipSize = 0x200000u,

  /* The wave ROM board's bit-permutation tables (wave_descramble_chip).
     This board carries byte-wide mask ROMs, so one storage unit is one
     byte, eight data lines are permuted, and the address permutation runs
     over the 21 lines that address 2 MiB of bytes. */
  .waveUnitBytes = 1u,
  .waveDataLineCount = 8u,
  .waveAddressLineCount = 21u,
  .waveDataLinePermutation = { 2u, 0u, 4u, 5u, 7u, 6u, 3u, 1u },
  .waveAddressLinePermutation = {
    0u, 4u, 2u, 3u, 1u, 13u, 7u, 12u, 5u, 10u, 16u,
    9u, 6u, 8u, 14u, 17u, 11u, 15u, 18u, 19u, 20u },

  /* One 32-byte plaintext header per 1 MiB logical bank - two per chip -
     and the board leaves both out of the scrambling. */
  .waveHeaderBytes = 0x20u,
  .waveHeaderStride = 0x100000u,

  /* -15 dB and -7 dB as power ratios; see wave_loop_reads_double's own
     comment for the measured gaps these sit in the middle of. */
  .belowPower = 0.0316227766016838,
  .partialPower = 0.199526231496888,

  /* The wave-bank selector byte for each of the eight banks across the
     four wave ROM chips. */
  .selectors = { 0x00u, 0x01u, 0x10u, 0x11u, 0x20u, 0x21u, 0x30u, 0x31u },

  /* GS: model id 42, three address bytes. */
  .sysexModelId = 0x42u,
  .sysexAddressBytes = 3u,

  /* The universal non-real-time handler at 0x0e70 takes 7E 7F 09 and
     hands the sub-ID on; 0x17e2 sends 01 to the GM System On at 0x128c6.
     scdb 03_disassembly/sysex_handlers.md. */
  .firmwarePortGmMode = true,

  /* This device's voice path is the shared firmware port in engine.h and
     renderer.h, which is what a null here selects. */
  .voiceEngine = nullptr,
};

const struct XpDeviceProfile *xp_profile(const struct xp_rom *rom)
{
  return (rom && rom->profile) ? rom->profile : &SC88_PROFILE;
}

}  // extern "C"
