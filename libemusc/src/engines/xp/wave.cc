/* SPDX-License-Identifier: CC0-1.0 */
#include "wave.h"

#include "common/constants.h"
#include "devices/sc88.h"

#include <cmath>
#include <climits>

namespace EmuSC { namespace Xp {

namespace {

uint16_t be16(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

uint32_t be24(const uint8_t *p)
{
  return ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
}

int16_t s16(uint16_t value)
{
  return value <= INT16_MAX
    ? (int16_t)value
    : (int16_t)(-1 - (int32_t)(UINT16_MAX - value));
}

uint32_t descramble_data(const struct XpDeviceProfile *profile, uint32_t value)
{
  uint32_t result = 0;
  for (unsigned bit = 0; bit < profile->waveDataLineCount; ++bit)
    result |= ((value >> profile->waveDataLinePermutation[bit]) & 1u) << bit;
  return result;
}

uint32_t descramble_address(const struct XpDeviceProfile *profile, uint32_t value)
{
  uint32_t result = 0;
  for (unsigned bit = 0; bit < profile->waveAddressLineCount; ++bit)
    result |= ((value >> profile->waveAddressLinePermutation[bit]) & 1u) << bit;
  return result;
}

/* Goertzel magnitude squared of `x[0..count)` at DFT bin `k` of a
   `count`-point transform.  Stable where a complex recurrence over several
   thousand samples is not. */
double bin_power(const int32_t *x, size_t count, size_t k)
{
  static const double twoPi = 6.283185307179586476925286766559;
  double omega = twoPi * (double)k / (double)count;
  double coeff = 2.0 * std::cos(omega);
  double s1 = 0.0;
  double s2 = 0.0;

  for (size_t n = 0; n < count; ++n) {
    double s0 = (double)x[n] + coeff * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

}  // namespace

/* UNDOING THE BOARD'S SCRAMBLING IS ONE LOOP FOR BOTH ROM WIDTHS.
 *
 *   The wave ROMs sit on the sound chip's own bus, and every board in this
 *   family permutes both their address and their data lines. What differs
 *   between boards is the width of the chips: a byte-wide mask ROM permutes
 *   eight data lines and addresses its bytes directly, while a 16-bit word
 *   ROM permutes sixteen and addresses words, so the same 2 MiB needs one
 *   address line fewer. Both are the same operation over a storage unit of
 *   XpDeviceProfile::waveUnitBytes bytes, and both are described entirely by
 *   the two line counts and the two permutation tables, so neither board
 *   appears in this function.
 *
 *   A multi-byte unit is little-endian in the dump - byte 0 of the pair is
 *   the word's low byte - and stays little-endian after decoding, because
 *   the decoded image is read back one byte per sample by the FCE decoder
 *   below.
 */
bool wave_descramble_chip(const struct XpDeviceProfile *profile,
                           const uint8_t *raw, size_t rawSize,
                           uint8_t *decoded, size_t decodedSize)
{
  if (!profile)
    profile = &SC88_PROFILE;
  const unsigned unit = profile->waveUnitBytes;
  if (!raw || !decoded || raw == decoded || rawSize != profile->waveChipSize ||
      decodedSize < profile->waveChipSize || unit < 1u || unit > 4u ||
      profile->waveDataLineCount > 8u * unit ||
      profile->waveDataLineCount > XP_WAVE_DATA_LINES_MAX ||
      profile->waveAddressLineCount > XP_WAVE_ADDRESS_LINES_MAX ||
      profile->waveChipSize % unit != 0u ||
      profile->waveHeaderStride == 0u)
    return false;

  const uint32_t units = (uint32_t)(profile->waveChipSize / unit);
  if (descramble_address(profile, units - 1u) >= units)
    return false;                /* the permutation escapes the chip */

  /* A line permutation moves each bit on its own, so it is the OR of what
     it does to each byte of its input taken alone: four 256-entry tables
     per permutation, built by the functions above, give the same word as
     walking every line of every unit. */
  uint32_t addressBytes[4][256], dataBytes[4][256];
  for (unsigned k = 0; k < 4u; ++k)
    for (uint32_t v = 0; v < 256u; ++v) {
      addressBytes[k][v] = descramble_address(profile, v << (8u * k));
      dataBytes[k][v] = descramble_data(profile, v << (8u * k));
    }

  for (uint32_t source = 0; source < units; ++source) {
    const uint32_t byteAddress = source * unit;
    /* The plaintext headers pass through untouched, at their own address
       and with their own bytes. */
    if (byteAddress % profile->waveHeaderStride < profile->waveHeaderBytes) {
      for (unsigned b = 0; b < unit; ++b)
        decoded[byteAddress + b] = raw[byteAddress + b];
      continue;
    }
    uint32_t value = 0;
    for (unsigned b = 0; b < unit; ++b)
      value |= dataBytes[b][raw[byteAddress + b]];
    const uint32_t target =
      (addressBytes[0][source & 0xffu] |
       addressBytes[1][(source >> 8) & 0xffu] |
       addressBytes[2][(source >> 16) & 0xffu] |
       addressBytes[3][source >> 24]) * unit;
    for (unsigned b = 0; b < unit; ++b)
      decoded[target + b] = (uint8_t)(value >> (8u * b));
  }
  return true;
}

bool wave_descriptor_parse(const struct XpDeviceProfile *profile,
                            const uint8_t *raw, size_t size,
                            struct xp_wave_descriptor *out)
{
  if (!profile)
    profile = &SC88_PROFILE;
  if (!raw || !out || size < profile->waveDescriptorSize)
    return false;

  out->bank_select = raw[0];
  out->address_a = be24(raw + 1);
  out->base_pitch_correction = s16(be16(raw + 4));
  out->root_key = raw[6];
  out->address_b = be24(raw + 7);
  out->control = raw[10];
  out->address_c = be24(raw + 11);
  out->alternate_pitch_correction = s16(be16(raw + 14));
  out->start_offset = be16(raw + 16);
  out->state_a = s16(be16(raw + 18));
  return true;
}

bool wave_descriptor_loop_type(const struct xp_wave_descriptor *desc,
                                enum xp_wave_loop_type *out)
{
  if (!desc || !out)
    return false;

  switch (desc->control) {
  case 0x00:
    *out = XP_WAVE_FORWARD_LOOP;
    return true;
  case 0x10:
    /* Ping-pong, and the ROM says so by itself.

       Read forward - which is what this returned until TASK-183 - every
       control-0x10 loop jumps in phase once per traversal, and that is the
       pop the owner reported on the French Horn for weeks. It is not one
       instrument: over the whole wave ROM, the correlation ACROSS the join
       of a loop played forward is +0.968 at the median for the 1518
       descriptors that carry 0x00 and +0.197 for the 215 that carry 0x10.
       The 0x00 loops tile; the 0x10 loops do not, and 0x10 is on exactly
       the material a designer cannot cut to a whole number of periods -
       Choir, Strings, Church Organ, French Horn, Tremolo Strings,
       Overdrive, Timpani.

       But a ping-pong turn in a DIFFERENTIAL format is not a time reversal.
       The decoder is an accumulator over deltas; running the address back
       down the stream while still adding what it reads gives

           y[m] = x[c] + d[c] + ... + d[c-m+1] = 2*x[c] - x[c-m]

       - the loop backwards AND reflected about the value it turned at,
       which is continuous in value and in SLOPE. A plain time reversal
       leaves a corner instead, and measures worse than the forward read,
       which is how the forward reading came to look right.

       The ROM is cut for the reflected turn: over all 1733 looping
       descriptors the deltas from address_b to address_c sum to EXACTLY
       zero, so x[c] == x[b-1] without a single exception. That is what
       lands the reflected pass on x[b-1] as its address reaches b-1, so
       the cycle closes with no step and no drift.

       Measured, French Horn C4, worst period-to-period correlation through
       the sustain: forward 0.780, plain time reversal -0.424, reflected
       0.987, against the hardware recording's 0.992. Over the eighteen
       control-0x10 instruments that have an SC-88 recording, p50 goes from
       +0.370 to +0.941 where the hardware's own is +0.926, and where the
       hardware dips we now dip with it. The 0x00 renders are unchanged
       byte for byte, and the corrected 63-instrument audit does not move:
       median MAD 2.1 dB, 46 within 3 dB, 7 past 6. */
    *out = XP_WAVE_PING_PONG_LOOP;
    return true;
  case 0x80:
    *out = XP_WAVE_FORWARD_ONE_SHOT;
    return true;
  case 0x88:
    *out = XP_WAVE_REVERSE_ONE_SHOT;
    return true;
  default:
    return false;
  }
}

/* THE DESCRIPTOR CARRIES TWO PITCH CORRECTIONS AND THE TUNING IS THEIR SUM.

   A looping descriptor's `+4` and `+14` are both signed words in the pitch
   domain where one octave is 16384 units.  `+4` alone is what the voice path
   used, and it leaves 686 of the ROM's 1713 looping descriptors out of tune.

   The ROM says so by itself, because a looping wave's period is not a matter
   of opinion: the loop from `address_b` to `address_c` is cut to a whole
   number of cycles of the note the descriptor names as its root key.  Predict
   the loop's own rate as `32000 / (c - b + 1) * 2**(correction / 16384)`,
   take the nearest whole number of root-key cycles it spans, and read off the
   error in cents.  Over every looping descriptor in 0x36100..0x3f714:

       group                       correction   median  p90   within 2 cents
       +14 == 0   (n = 1024)       base          0.22   7.10      82.7 %
       +14 != 0   (n =  685)       base          1.77  10.10      53.6 %
       +14 != 0   (n =  685)       base + alt    0.13   0.36      97.1 %
       |+14| >= 20 (n =  392)      base          3.80  12.69      22.2 %
       |+14| >= 20 (n =  392)      base + alt    0.21   0.37      98.2 %
       |+14| >= 20 (n =  392)      base - alt    7.67  25.11       2.3 %

   The first row is the null and it validates the method: where `+14` is
   already zero the two readings are the same expression and the loops land
   on their root key to a fifth of a cent.  The `base - alt` row is the same
   magnitude with the wrong sign and it is twice as bad as doing nothing, so
   the measurement can fail and does.  Paired on the 379 descriptors whose
   whole-cycle count is the same under both readings, `base + alt` is the
   closer of the two on 377.

   THE FIRMWARE ADDS IT, UNDER A CONDITION THAT IS NOT RECOVERED.  `0x612c`
   `ed 04 83` loads the descriptor's `+4` into the pitch sum; `0x612f`
   `f1 25 da f0` tests bit 0 of the per-voice byte at DP:`0x25da` + slot and
   `0x6133` skips over `0x6135` `ed 0e 23`, which adds `+14`, when it is
   clear.  That flag is cleared for every slot at power-on (`0x1154f`) and
   again at `0x5dfc`, three instructions before the same routine calls the
   pitch composition at `0x6077`; it is SET at `0x5334` `f1 25 da c0`.  The
   corrected listing desynchronises across both `0x5334` and `0x5dfc` - it
   reads them as `bcs.b` and `bclr.w` inside other instructions - so the
   setter was believed not to exist; the raw bytes are unambiguous.  What
   reaches `0x5334` is not traced, so WHEN the chip applies the alternate
   correction is open, and adding it always is a choice rather than a
   recovered condition.

   It is the choice the ROM's own tuning supports, and the hardware agrees on
   the tone that raised the question.  `Charang` (program 84) sounds a
   `DIST_GT` zone whose `+14` is zero beside a `SITAR` zone at 0x3b4ac whose
   `+14` is +132.  At C5 the first plays at 523.99 Hz; the second plays at
   519.65 Hz on `+4` alone and at 522.56 Hz with both, so the pair beats at
   8.4 Hz or at 2.8 Hz at the octave.  The hardware recording of that note
   carries no 8.4 Hz modulation in its 1.0-1.1 kHz band at all, and the
   render on `+4` alone carries it as the strongest line after the note's own
   envelope. */
int16_t wave_pitch_correction(const struct xp_wave_descriptor *desc,
                               bool alternate)
{
  if (!desc)
    return 0;
  uint16_t value = (uint16_t)desc->base_pitch_correction;
  if (alternate)
    value = (uint16_t)(value + (uint16_t)desc->alternate_pitch_correction);
  return s16(value);
}

bool wave_prepare_registers(const struct XpDeviceProfile *profile,
                             const struct xp_wave_descriptor *desc,
                             bool suppressStartOffset,
                             struct xp_wave_registers *out)
{
  if (!profile)
    profile = &SC88_PROFILE;
  if (!desc || !out || desc->address_a >= profile->waveBankSize ||
      desc->address_b >= profile->waveBankSize ||
      desc->address_c >= profile->waveBankSize)
    return false;

  out->bank_flags = UINT32_C(0x8000) |
    ((uint32_t)(desc->control & 0x7f) << 8) | desc->bank_select;
  out->state_a = 0;

  uint32_t first;
  uint32_t third;
  if (desc->control & 0x08) {
    if ((desc->control & 0x80) && desc->address_a == 0)
      return false;
    first = desc->address_c;
    third = desc->address_a;
    if (desc->control & 0x80)
      --third;
  } else {
    first = desc->address_a;
    if (!suppressStartOffset) {
      if (desc->start_offset >= profile->waveBankSize - first)
        return false;
      first += desc->start_offset;
      out->state_a = ((desc->state_a < 0 ? UINT32_C(3) : 0) << 16) |
        (uint16_t)desc->state_a;
    }
    third = desc->address_c;
  }

  out->start = first;
  out->end = third;
  out->loop = (desc->control & 0x80) ? third : desc->address_b;
  out->initial_state = UINT32_C(0x18);
  return true;
}

bool fce_decoder_reset(const struct XpDeviceProfile *profile,
                        struct xp_fce_decoder *decoder,
                        uint32_t sampleStart)
{
  if (!profile)
    profile = &SC88_PROFILE;
  if (!decoder || sampleStart >= profile->waveBankSize)
    return false;
  decoder->next_address = sampleStart & ~UINT32_C(0x0f);
  decoder->accumulator = 0;
  decoder->profile = profile;
  return true;
}

bool fce_decoder_read(struct xp_fce_decoder *decoder,
                       const uint8_t *bank, size_t bankSize,
                       int32_t *pcm24)
{
  if (!decoder || !decoder->profile || !bank || !pcm24 ||
      bankSize < decoder->profile->waveBankSize ||
      decoder->next_address >= decoder->profile->waveBankSize)
    return false;

  uint32_t address = decoder->next_address++;
  int32_t delta = bank[address] <= INT8_MAX
    ? bank[address]
    : (int32_t)bank[address] - 256;
  uint8_t packed = bank[address >> 5];
  unsigned exponent = (address & 0x10) ? packed >> 4 : packed & 0x0f;
  decoder->accumulator += (int64_t)delta * (INT64_C(1) << exponent);
  int64_t scaled = decoder->accumulator * INT64_C(128);
  if (scaled > INT32_C(0x7fffff))
    scaled = INT32_C(0x7fffff);
  else if (scaled < -INT32_C(0x800000))
    scaled = -INT32_C(0x800000);
  *pcm24 = (int32_t)scaled;
  return true;
}

bool fce_decode_descriptor(const struct XpDeviceProfile *profile,
                            const uint8_t *bank, size_t bankSize,
                            const struct xp_wave_descriptor *desc,
                            int32_t *output, size_t capacity,
                            size_t *written)
{
  if (!profile)
    profile = &SC88_PROFILE;
  if (written)
    *written = 0;
  if (!bank || !desc || !output || !written ||
      desc->address_a > desc->address_c ||
      desc->address_c >= profile->waveBankSize)
    return false;

  size_t count = (size_t)(desc->address_c - desc->address_a) + 1;
  struct xp_fce_decoder decoder;
  if (capacity < count || !fce_decoder_reset(profile, &decoder, desc->address_a))
    return false;

  size_t index = 0;
  for (uint32_t address = decoder.next_address; address <= desc->address_c;
       ++address) {
    int32_t sample;
    if (!fce_decoder_read(&decoder, bank, bankSize, &sample))
      return false;
    if (address >= desc->address_a)
      output[index++] = sample;
  }
  *written = index;
  return true;
}

bool fce_decode_storage(const struct XpDeviceProfile *profile,
                         const uint8_t *bank, size_t bankSize,
                         const struct xp_wave_descriptor *desc,
                         int32_t *output, size_t capacity,
                         uint32_t *baseAddress, size_t *written)
{
  if (!profile)
    profile = &SC88_PROFILE;
  if (baseAddress)
    *baseAddress = 0;
  if (written)
    *written = 0;
  if (!bank || !desc || !output || !baseAddress || !written ||
      desc->address_a > desc->address_c ||
      desc->address_c >= profile->waveBankSize)
    return false;

  uint32_t base = desc->address_a & ~UINT32_C(0x0f);
  size_t count = (size_t)(desc->address_c - base) + 1;
  struct xp_fce_decoder decoder;
  if (capacity < count || !fce_decoder_reset(profile, &decoder, desc->address_a))
    return false;
  for (size_t index = 0; index < count; ++index)
    if (!fce_decoder_read(&decoder, bank, bankSize, output + index))
      return false;
  *baseAddress = base;
  *written = count;
  return true;
}

/* The thirty organ descriptors the SC-88 reads at twice the rate.

   The behaviour is settled and measured against hardware, and the mechanism
   is NOT recovered.  The Drawbar Organ C3 recording carries the nine Hammond
   drawbar footages with 40 dB holes at harmonics 5, 7, 9 and 11; the ROM loop
   the key-48 zone selects reproduces that hole pattern only when the loop is
   read at 2x: on Percussive Organ C3 the fifth harmonic is -54.2 dB against
   the recording's -55.2 read at 2x, and 16.2 dB out read at 1x.  Read at 1x, which is what the
   arithmetic in `xp_renderer_static_pitch_word` gives, the registration
   moves half an octave down, the holes fill, and both organs sound an octave
   below the note.

   This predicate is a model of that BEHAVIOUR, not of the machine.  Neither
   the H8/520 nor the XP can compute it at note-on: it is a property of the
   decoded waveform, and the CPU never decodes a sample.  Three lanes searched
   for the mechanism and closed every source this project can read - all 160
   descriptor bits, the whole 512 KB control ROM and all eight 1 MiB wave banks
   at every base and strides 1/2/4, the SC-88 Pro's own firmware and descriptor
   table, the seven demo songs, and Sound Canvas VA's records and code.  The
   sibling device says what the missing field would look like and that the
   SC-88 does not have it: on the JV-880 the Marimba samples carry the same
   mismatch the other way up - their loops hold half a root-key period per
   waveform cycle - and its tone record reconciles it with a stored coarse
   tune of -12 semitones at +37.  The SC-88 has that field, component +0x16,
   with a -36..+24 semitone range that 65 components use; it reads zero on
   every organ component, and it is per-component while the defect is
   per-zone - E.Organ 1's single component reaches two clean zones and nine
   affected ones.

   The rule, on the loop's own decoded content, with `L` the loop length and
   `root` the descriptor's root key:

     P = 32000 / (440 * 2^((root - 69) / 12))    root-key period, in samples
     u = L / P                                   root-key cycles in the loop
     h = u / 2

   `u` must be within 0.02 of an even integer at least two, so that bin `h`
   exists.  Then, with `A_k` the loop's DFT magnitude at bin k:

     the loudest bin below h is at most -15 dB relative to A_h, and
     A_h is at least -7 dB relative to A_u.

   Together: the loop is a harmonic tone whose own fundamental sits an octave
   below its root pitch.  Both thresholds are the middle of a measured gap
   over the 113 forward-looping descriptors whose loop spans an even whole
   number of root-key cycles.  The first clause separates -26.6 dB (the worst
   of the thirty) from +4.5 dB (the best of the rest), 31.1 dB wide; the
   second separates -2.45 dB from -11.35 dB, 8.90 dB wide.  The selection is
   exactly those thirty for every first threshold in -25..-6 dB crossed with
   every second in -11.3..-2.5 dB.

   `pcm` is the decoded sample, `base_address` the wave address its first
   entry holds. */
bool wave_loop_reads_double(const struct XpDeviceProfile *profile,
                             const int32_t *pcm, size_t count,
                             uint32_t baseAddress,
                             const struct xp_wave_descriptor *desc)
{
  if (!profile)
    profile = &SC88_PROFILE;
  enum xp_wave_loop_type mode;
  if (!pcm || !desc || !wave_descriptor_loop_type(desc, &mode) ||
      mode != XP_WAVE_FORWARD_LOOP || desc->root_key > 127 ||
      desc->address_c <= desc->address_b || desc->address_b < baseAddress)
    return false;
  size_t first = (size_t)(desc->address_b - baseAddress);
  size_t length = (size_t)(desc->address_c - desc->address_b) + 1u;
  if (length < 16u || first > count || count - first < length)
    return false;

  /* How many periods of the note the descriptor claims fit in the loop. */
  double period = kXpNativeRate /
    (440.0 * std::pow(2.0, ((double)desc->root_key - 69.0) / 12.0));
  double cycles = (double)length / period;
  double rounded = std::floor(cycles + 0.5);
  if (rounded < 2.0 || std::fabs(cycles - rounded) > 0.02)
    return false;
  size_t whole = (size_t)rounded;
  if (whole % 2u)
    return false;
  size_t half = whole / 2u;
  if (whole >= length / 2u)
    return false;

  double atHalf = bin_power(pcm + first, length, half);
  double atRoot = bin_power(pcm + first, length, whole);
  if (atHalf <= 0.0)
    return false;

  /* Nothing below the half-root partial: it is the loop's own fundamental
     and not a bin of some inharmonic body.  This is what keeps the Melodic
     Tom out - its loop spans thirty root-key cycles and carries 37 dB more
     energy below bin fifteen than in it. */
  for (size_t k = 1u; k < half; ++k) {
    if (bin_power(pcm + first, length, k) > atHalf * profile->belowPower)
      return false;
  }
  /* And it is a partial, not a seam artefact: the clean low zones of these
     same organ sets carry a bin there 30 dB down. */
  return atHalf >= atRoot * profile->partialPower;
}

bool wave_cursor_init(const struct XpDeviceProfile *profile,
                       struct xp_wave_cursor *cursor,
                       const struct xp_wave_registers *registers,
                       enum xp_wave_loop_type mode)
{
  if (!profile)
    profile = &SC88_PROFILE;
  if (!cursor || !registers || registers->start >= profile->waveBankSize ||
      registers->loop >= profile->waveBankSize ||
      registers->end >= profile->waveBankSize ||
      mode == XP_WAVE_REVERSE_ONE_SHOT)
    return false;
  if (registers->start > registers->end ||
      (mode != XP_WAVE_FORWARD_ONE_SHOT && registers->loop > registers->end))
    return false;

  cursor->position = registers->start;
  cursor->loop = registers->loop;
  cursor->end = registers->end;
  cursor->mode = mode;
  cursor->direction = 1;
  cursor->ended = false;
  return true;
}

bool wave_cursor_current(const struct xp_wave_cursor *cursor,
                          uint32_t *address)
{
  if (!cursor || !address || cursor->ended)
    return false;
  *address = cursor->position;
  return true;
}

bool wave_cursor_advance(struct xp_wave_cursor *cursor)
{
  if (!cursor || cursor->ended)
    return false;

  switch (cursor->mode) {
  case XP_WAVE_FORWARD_LOOP:
    cursor->position = cursor->position == cursor->end
      ? cursor->loop : cursor->position + 1;
    return true;

  case XP_WAVE_PING_PONG_LOOP:
    if (cursor->direction > 0) {
      if (cursor->position == cursor->end)
        cursor->direction = -1;
      else
        ++cursor->position;
    } else {
      if (cursor->position == cursor->loop)
        cursor->direction = 1;
      else
        --cursor->position;
    }
    return true;

  case XP_WAVE_FORWARD_ONE_SHOT:
    if (cursor->position == cursor->end)
      cursor->ended = true;
    else
      ++cursor->position;
    return true;

  case XP_WAVE_REVERSE_ONE_SHOT:
    break;
  }
  return false;
}

}}  // namespace EmuSC::Xp
