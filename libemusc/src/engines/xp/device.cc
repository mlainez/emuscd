/* SPDX-License-Identifier: CC0-1.0 */
#include "device.h"

#include "common/worker_pool.h"
#include "devices/sc88.h"

#include <cstdlib>
#include <cstring>

namespace EmuSC { namespace Xp {

namespace {

void syncPart(Device *device, uint8_t part)
{
  const ChannelState *channel = device->channels + part;
  struct xp_tva_levels levels;
  struct xp_pan_controls pan;
  levels.master = device->master_volume;
  levels.secondary = device->secondary_level;
  levels.part = channel->volume;
  levels.expression = channel->expression;
  pan.master = device->master_pan;
  /* GS pan 0 is RANDOM, and is passed through as zero: the engine draws a
     fresh 1..127 position for each voice, which is what spreads a part
     across the image instead of stacking it in the centre. */
  pan.part = channel->pan;
  if (channel->pan == 0)
    ++device->random_pan_requests;
  engine_set_part_levels(&device->engine, part, &levels);
  engine_set_part_pan(&device->engine, part, &pan);
}

void syncChorus(Device *device)
{
  chorus_set_params(&device->renderer.rom, &device->chorus,
                     device->chorus_level, device->chorus_feedback,
                     device->chorus_delay, device->chorus_rate,
                     device->chorus_depth, device->chorus_pre_lpf);
}

/* Writing the chorus macro copies eight bytes over pre-LPF through delay
   send - handler 0x3400, the reverb handler 0x3388's sibling, through the
   same copy helper and the same 8-byte record stride. The delay send is
   single-module only and this engine does not hold it. */
bool loadChorusMacro(Device *device, uint8_t macro)
{
  uint8_t p[8];
  if (macro > 7 || !chorus_macro(&device->renderer.rom, macro, p))
    return false;
  device->chorus_macro = macro;
  device->chorus_pre_lpf = p[0] > 7 ? 7 : p[0];
  device->chorus_level = p[1];
  device->chorus_feedback = p[2];
  device->chorus_delay = p[3];
  device->chorus_rate = p[4];
  device->chorus_depth = p[5];
  device->chorus_send_to_reverb = p[6];
  syncChorus(device);
  return true;
}

void syncLfo(Device *device, uint8_t part)
{
  const ChannelState *channel = device->channels + part;
  struct xp_lfo_controls controls;
  controls.rate = channel->vibrato_rate;
  controls.delay = channel->vibrato_delay;
  controls.depth = channel->vibrato_depth;
  engine_set_part_lfo_controls(&device->engine, part, &controls);
}

void syncEq(Device *device)
{
  (void)eq_set_params(&device->renderer.rom, &device->eq,
                       device->eq_low_frequency, device->eq_low_gain,
                       device->eq_high_frequency, device->eq_high_gain);
}

void syncPitch(Device *device, uint8_t part)
{
  const ChannelState *channel = device->channels + part;
  int64_t numerator = ((int32_t)channel->pitch_bend - 8192) *
    (int32_t)channel->pitch_bend_sensitivity * 16384;
  int32_t offset = (int32_t)(numerator / (8192 * 12));
  /* 0x4000 pitch-word units to the octave */
  offset += ((int32_t)channel->key_shift - 64) * 16384 / 12;
  engine_set_part_pitch_offset(&device->engine, part, offset);
}

/* An H8 arithmetic right shift, written as a division so that shifting a
   negative value does not depend on the C implementation's choice. */
int32_t shiftRight(int32_t value, unsigned bits)
{
  int32_t divisor = INT32_C(1) << bits;
  int32_t quotient = value / divisor;
  if (value < 0 && value % divisor != 0)
    --quotient;
  return quotient;
}

int16_t s16(uint16_t value)
{
  return value <= INT16_MAX
    ? (int16_t)value
    : (int16_t)(-1 - (int32_t)(UINT16_MAX - value));
}

/* `sub.b #0x40:8` on the depth byte, read back as the signed byte the
   multiply that follows treats it as. */
int centredDepth(uint8_t depth)
{
  uint8_t byte = (uint8_t)(depth - 0x40u);
  return byte <= 0x7fu ? (int)byte : (int)byte - 0x100;
}

/* One source's contribution to a bipolar destination: the centred depth
   times the source's own value, wrapping into the destination word.
   `0x11876`..`0x1189e` is one of the four, written in the ROM as a
   magnitude multiply with the sign reapplied afterwards. */
uint16_t matrixTerm(uint8_t depth, uint8_t value)
{
  return (uint16_t)(centredDepth(depth) * (int)value);
}

void syncTvf(Device *device, uint8_t part);
void syncLfo1PitchDepth(Device *device, uint8_t part);
void syncTva(Device *device, uint8_t part);
void syncToneMap(Device *device, uint8_t part);

bool setReverbCharacter(Device *device, uint8_t character);
bool loadReverbMacro(Device *device, uint8_t macro);
bool sysexWrite(Device *device, uint8_t port, uint32_t address,
                 uint8_t value);
bool blockPart(uint8_t block, uint8_t port, uint8_t *part);

bool initCommon(Device *device, const uint8_t *controlRom,
                 size_t controlRomSize,
                 const uint8_t *const chips[XP_WAVE_CHIP_COUNT],
                 const size_t sizes[XP_WAVE_CHIP_COUNT], double outputRate,
                 enum xp_fractional_wrap wrap, bool raw)
{
  size_t romSize;
  if (!device || !controlRom || !chips || !sizes)
    return false;
  /* Identifies the device before anything below needs its profile - the
     real identification rom_init() does; renderer_init() below repeats
     it on the same bytes once device->control_rom exists, harmlessly.
     It is also the size check: matches() accepts an image only at the
     profile's own romSize, so a wrong-length image has no profile and
     never reaches the allocation below. */
  struct xp_rom identifyRom;
  if (!rom_init(&identifyRom, controlRom, controlRomSize))
    return false;
  const struct XpDeviceProfile *profile = xp_profile(&identifyRom);
  romSize = profile->romSize;
  std::memset(device, 0, sizeof *device);
  device->control_rom = (uint8_t *)std::malloc(romSize);
  if (!device->control_rom)
    goto fail;
  std::memcpy(device->control_rom, controlRom, romSize);
  for (unsigned chip = 0; chip < XP_WAVE_CHIP_COUNT; ++chip) {
    if (!chips[chip] || sizes[chip] != profile->waveChipSize)
      goto fail;
    device->decoded_chips[chip] = (uint8_t *)std::malloc(profile->waveChipSize);
    if (!device->decoded_chips[chip])
      goto fail;
    if (raw) {
      if (!wave_descramble_chip(profile, chips[chip], sizes[chip],
                                device->decoded_chips[chip],
                                profile->waveChipSize))
        goto fail;
    } else {
      std::memcpy(device->decoded_chips[chip], chips[chip],
                  profile->waveChipSize);
    }
    device->banks[chip * 2].selector = profile->selectors[chip * 2];
    device->banks[chip * 2].bytes = device->decoded_chips[chip];
    device->banks[chip * 2].size = profile->waveBankSize;
    device->banks[chip * 2 + 1].selector = profile->selectors[chip * 2 + 1];
    device->banks[chip * 2 + 1].bytes =
      device->decoded_chips[chip] + profile->waveBankSize;
    device->banks[chip * 2 + 1].size = profile->waveBankSize;
  }
  /* A device whose voice path is its own takes the banks and stops here:
     the renderer, engine and effects below are the shared firmware port's,
     and this device has no firmware to port. The ROM view is still built,
     because that is what the injected engine reads its own tables
     through. */
  device->voice_ops = profile->voiceEngine;
  if (device->voice_ops) {
    const uint8_t *bankBytes[XP_WAVE_BANK_COUNT];
    size_t bankSizes[XP_WAVE_BANK_COUNT];
    for (unsigned b = 0; b < XP_WAVE_BANK_COUNT; ++b) {
      bankBytes[b] = device->banks[b].bytes;
      bankSizes[b] = device->banks[b].size;
    }
    if (!rom_init(&device->renderer.rom, device->control_rom, romSize) ||
        !device->voice_ops->create(&device->voice_state,
                                    &device->renderer.rom, bankBytes,
                                    bankSizes, XP_WAVE_BANK_COUNT,
                                    outputRate))
      goto fail;
    device->output_rate = outputRate;
    device->initialized = true;
    return true;
  }
  if (!renderer_init(&device->renderer, device->control_rom,
                      romSize, device->banks,
                      XP_WAVE_BANK_COUNT, outputRate, wrap))
    goto fail;
  device->output_rate = outputRate;
  output_init(&device->output, outputRate);
  if (!tvf_coefficients_init(&device->tvf_coefficients, outputRate))
    goto fail;
  renderer_set_tvf_audio_transfer(&device->renderer,
                                   tvf_audio_process_provisional,
                                   &device->tvf_coefficients);
  if (!chorus_init(&device->chorus, outputRate, profile))
    goto fail;
  if (!delay_init(&device->delay, outputRate, profile))
    goto fail;
  eq_init(&device->eq);
  if (!engine_init(&device->engine, &device->renderer))
    goto fail;
  /* Hall 2 is the character a reset selects; the reverb reads its own delay
     lines and diffuser count out of the ROM (`M-008`). A ROM that carries no
     character records is not a reason to refuse the device: an effect is not
     a precondition for the voice path, so the device renders dry and says so
     through `reverb.active`. */
  (void)reverb_init(&device->reverb, &device->renderer.rom, 4, outputRate);
  device->initialized = true;
  /* The selected tone map is set here and not in the reset, because it is
     the one part parameter a GS Reset leaves alone. */
  for (unsigned part = 0; part < XP_ENGINE_PART_COUNT; ++part)
    device->channels[part].tone_map_selected = XP_TONE_MAP_SC88;
  device_reset_controllers(device);
  return true;

fail:
  device_destroy(device);
  return false;
}

/* The matrix's cached cutoff word, SC88-CTL 0x11871..0x11982, stored at
   DP:1c34 + part. The four byte-controller products are summed with
   wrapping word adds and halved by one arithmetic shift (`0x1191e`);
   pitch bend takes its own path, forming `(bend - 0x2000) * 4`, keeping
   bits 23..8 of its product with the centred depth (`0x11958`) and adding
   the signed high word of that times `0x7f00` (`0x1195c`). The final add
   wraps. Pitch is the one destination that does not halve and whose bend
   constant is 0xfe16 instead; amplitude and the two LFO rates share this
   shape exactly, and would be wired from here. */
void syncTvf(Device *device, uint8_t part)
{
  const ChannelState *channel = device->channels + part;
  struct xp_tvf_controls controls;
  controls.part_cutoff = channel->cutoff;
  controls.secondary_cutoff = 64;
  controls.part_resonance = channel->resonance;
  controls.secondary_resonance = 64;
  controls.matrix_cutoff = device_matrix_cutoff_word(channel);
  engine_set_part_tvf_controls(&device->engine, part, &controls);
}

/* The one LFO-depth destination this engine consumes. Those depths are
   unsigned and their four byte-controller products are shifted right two
   rather than one (`04_protocol/controllers.md`); modulation is the only
   source modelled, so the sum is the one term. */
void syncLfo1PitchDepth(Device *device, uint8_t part)
{
  const ChannelState *channel = device->channels + part;
  unsigned depth =
    channel->matrix_depth[XP_MATRIX_MODULATION][XP_MATRIX_LFO1_PITCH_DEPTH];
  engine_set_part_lfo1_pitch_depth(
    &device->engine, part, (uint16_t)((depth * channel->modulation) >> 2));
}

void syncTva(Device *device, uint8_t part)
{
  const ChannelState *channel = device->channels + part;
  struct xp_tva_controls controls;
  controls.part_attack = channel->attack;
  controls.secondary_attack = 64;
  controls.part_decay = channel->decay;
  controls.secondary_decay = 64;
  engine_set_part_tva_controls(&device->engine, part, &controls);
}

/* The part's bank word, resolved the way `2d3e` and `2e7a` resolve it: the
   forced byte governs unless it is zero, and then the part's own selected
   map does. A forced byte above 2 resolves to no tone at all on the device;
   it is refused where it is received instead, so nothing ever reaches this
   holding a map the ROM lookup has no row for. */
void syncToneMap(Device *device, uint8_t part)
{
  const ChannelState *channel = device->channels + part;
  engine_set_part_tone_map(&device->engine, part,
                            channel->tone_map_forced
                              ? channel->tone_map_forced
                              : channel->tone_map_selected);
}

/* Both the manual's block numbering and the reverb macro's meaning are in
   `04_protocol/sysex.md`: for the sixteen blocks of a group, x=1..9 selects
   parts 1..9, x=0 selects part 10 and x=a..f selects parts 11..16. */
bool blockPart(uint8_t block, uint8_t port, uint8_t *part)
{
  uint8_t index;
  if (block <= 0x09)
    index = block == 0 ? 9u : (uint8_t)(block - 1);
  else if (block <= 0x0f)
    index = block;
  else
    return false;
  *part = (uint8_t)(port * 16u + index);
  return true;
}

bool setReverbCharacter(Device *device, uint8_t character)
{
  struct xp_reverb replacement;
  if (character == device->reverb_character)
    return true;
  /* A character is a different set of delay lines read out of the ROM, so
     it is a new reverb rather than a new parameter. The old one is kept
     until the new one is known to have been built. */
  if (!reverb_init(&replacement, &device->renderer.rom, character,
                    device->reverb.output_rate))
    return false;
  /* A render that has the effects switched off keeps them off across a
     character change; a freshly built reverb comes up enabled. */
  replacement.active = device->reverb.active;
  reverb_destroy(&device->reverb);
  device->reverb = replacement;
  device->reverb_character = character;
  reverb_set_params(&device->reverb, device->reverb_level,
                     device->reverb_time, device->reverb_pre_lpf);
  return true;
}

/* Writing the reverb macro address copies the preset's seven bytes over
   character, pre-LPF, level, time, delay feedback, the reserved byte and
   predelay. SC88-CTL handler 0x3388 is the delay handler 0x342b's sibling:
   both clamp the written byte through a bounds record, store it, and then,
   only when the index is zero, copy the rest of the block out of a ROM table
   (`0x1583e + 8*macro` here, `0x158be + 16*macro` there) through the same
   pair of copy helpers. The power-on loader at 0x4476 reads the same table. */
bool loadReverbMacro(Device *device, uint8_t macro)
{
  uint8_t p[7];
  if (macro > 7 || !reverb_macro(&device->renderer.rom, macro, p))
    return false;
  device->reverb_macro = macro;
  if (!setReverbCharacter(device, p[0] > 7 ? 7 : p[0]))
    return false;
  device->reverb_pre_lpf = p[1] > 7 ? 7 : p[1];
  device->reverb_level = p[2];
  device->reverb_time = p[3];
  device->reverb_delay_feedback = p[4];
  /* p[5] is the block's reserved byte; `40 01 36` has no parameter and the
     firmware refuses a write to it. */
  device->reverb_predelay = p[6];
  reverb_set_predelay(&device->reverb, device->reverb_predelay);
  reverb_set_params(&device->reverb, device->reverb_level,
                     device->reverb_time, device->reverb_pre_lpf);
  return true;
}

/* One address of a DT1 packet. `true` means the write was acted on. */
bool sysexWrite(Device *device, uint8_t port, uint32_t address,
                 uint8_t value)
{
  uint8_t part;
  if (address == 0x00007f) {
    /* System Mode Set. Which parameters exist depends on it, so it is
       held even though nothing yet varies with it. */
    if (value > 1)
      return false;
    device->system_mode = value;
    /* @provenance class=DOCUMENTED devices=SC-88
       "The operating mode will be set, and at the same time all internal
       parameters (except for the map settings of each Part) will be reset
       to the initial state" (om-128). The SC-88's own demo songs open with
       it in place of a GS reset. Only the voice side of that reset is done
       here, the cut the GS reset makes; the parameter reset is not, since
       the demos send settings ahead of it that a reset would clear, and
       which of them the device keeps is not established. */
    for (unsigned part = 0; part < XP_ENGINE_PART_COUNT; ++part)
      engine_all_sound_off(&device->engine, (uint8_t)part);
    /* @provenance class=FW-STRUCT devices=SC-88 ref=0x37a8,0x12a10,0x129dd,0x13184
       Both candidates for this address's handler run the module reset,
       which copies the part image at 0x13184 with +18 = 00, so Rx.Bank
       Select and its LSB come back on, as the manual says (om-125). What
       they leave Rx.NRPN at differs - `0x12a10` turns it on, `0x129dd`
       off again afterwards - and which one this address reaches is not
       established, so Rx.NRPN is left as it was. */
    for (unsigned part = 0; part < XP_ENGINE_PART_COUNT; ++part) {
      device->channels[part].rx_bank_select = true;
      device->channels[part].rx_bank_select_lsb = true;
    }
    return true;
  }
  if (address == 0x40007f) {
    device_reset_controllers(device);
    return true;
  }
  switch (address) {
  case 0x400004:
    device_set_master_volume(device, value);
    return true;
  case 0x400006:
    device_set_master_pan(device, value);
    return true;
  case 0x400130:
    /* The macro reloads the whole block from its preset; see the loader. */
    return value <= 7 && loadReverbMacro(device, value);
  case 0x400131:
    return value <= 7 && setReverbCharacter(device, value);
  case 0x400132:
    if (value > 7)
      return false;
    device->reverb_pre_lpf = value;
    reverb_set_params(&device->reverb, device->reverb_level,
                       device->reverb_time, device->reverb_pre_lpf);
    return true;
  case 0x400133:
    device->reverb_level = value;
    reverb_set_params(&device->reverb, device->reverb_level,
                       device->reverb_time, device->reverb_pre_lpf);
    return true;
  case 0x400137:
    /* Predelay, in milliseconds, single-module only. */
    if (value > 127)
      return false;
    device->reverb_predelay = value;
    reverb_set_predelay(&device->reverb, value);
    return true;
  case 0x400135:
    device->reverb_delay_feedback = value;
    return true;
  case 0x400134:
    device->reverb_time = value;
    reverb_set_params(&device->reverb, device->reverb_level,
                       device->reverb_time, device->reverb_pre_lpf);
    return true;
  /* The macro's eight names are the manual's chorus types. Which fields
     each preset loads is not recovered - the delay macro demonstrably
     copies a ten-byte preset, and this one may too - so the macro is held
     and the fields it would carry are left to the song, which sends them. */
  case 0x400200:
    if (value > 1)
      return false;
    device->eq_low_frequency = value;
    syncEq(device);
    return true;
  case 0x400201:
    if (value < 0x34 || value > 0x4c)
      return false;
    device->eq_low_gain = value;
    syncEq(device);
    return true;
  case 0x400202:
    if (value > 1)
      return false;
    device->eq_high_frequency = value;
    syncEq(device);
    return true;
  case 0x400203:
    if (value < 0x34 || value > 0x4c)
      return false;
    device->eq_high_gain = value;
    syncEq(device);
    return true;
  case 0x400150:
    /* Writing the macro copies its ten bytes over pre-LPF through reverb
       send; writing any other field changes only that field. */
    if (value > 9)
      return false;
    device->delay_macro = value;
    if (!delay_macro(&device->renderer.rom, value, device->delay_params))
      return false;
    return delay_set_params(&device->renderer.rom, &device->delay,
                             device->delay_params);
  case 0x400151: case 0x400152: case 0x400153: case 0x400154:
  case 0x400155: case 0x400156: case 0x400157: case 0x400158:
  case 0x400159: case 0x40015a:
    device->delay_params[address - 0x400151u] = value;
    return delay_set_params(&device->renderer.rom, &device->delay,
                             device->delay_params);
  case 0x400138:
    return loadChorusMacro(device, value);
  case 0x400139:
    if (value > 7)
      return false;
    device->chorus_pre_lpf = value;
    syncChorus(device);
    return true;
  case 0x40013a:
    device->chorus_level = value;
    syncChorus(device);
    return true;
  case 0x40013b:
    device->chorus_feedback = value;
    syncChorus(device);
    return true;
  case 0x40013c:
    device->chorus_delay = value;
    syncChorus(device);
    return true;
  case 0x40013d:
    device->chorus_rate = value;
    syncChorus(device);
    return true;
  case 0x40013e:
    device->chorus_depth = value;
    syncChorus(device);
    return true;
  case 0x40013f:
    device->chorus_send_to_reverb = value;
    return true;
  default:
    break;
  }
  /* `41 mf rr`: one per-note kit parameter. m is the drum setup, f the
     field, rr the note; `41 m0 00` is the setup's twelve-byte name, which
     has no bearing on the sound. */
  if ((address & 0xff0000u) == 0x410000u) {
    uint8_t setup = (uint8_t)(((address >> 12) & 0x0fu) + 1u);
    uint8_t field = (uint8_t)((address >> 8) & 0x0fu);
    uint8_t note = (uint8_t)(address & 0xffu);
    if (field == 0)
      return true;                     /* the name */
    return engine_set_drum_parameter(&device->engine, setup, field, note,
                                      value);
  }
  /* The `40 4x` block, the part parameters the SC-88 adds. */
  if ((address & 0xf0f000u) == 0x404000u) {
    if ((address & 0xffu) == 0x20u) {
      /* The equaliser switch. It is one global effect, so any block's
         write governs it. */
      if (value > 1)
        return false;
      device->eq.enabled = value != 0;
      return true;
    }
    if (blockPart((uint8_t)((address >> 8) & 0x0f), port, &part)) {
      switch (address & 0xffu) {
      case 0x00:
        /* The same forcing byte CC32 writes: `46de` and `317c` store it in
           the same place. */
        if (value > 2)
          return false;
        device->channels[part].tone_map_forced = value;
        syncToneMap(device, part);
        return true;
      case 0x01:
        /* The part's own map, which the forcing byte defers to. `46e9`
           carries the range 01..02 in its parameter-table entry at
           `13e94` and refuses anything else. */
        if (value < XP_TONE_MAP_SC55 || value > XP_TONE_MAP_SC88)
          return false;
        device->channels[part].tone_map_selected = value;
        syncToneMap(device, part);
        return true;
      default:
        return false;
      }
    }
    return false;
  }
  /* `40 2x ss`, the controller destination matrix: six source groups
     sixteen apart, eleven destinations each. Every depth is held; only
     the cutoff column and modulation's LFO1 pitch depth are consumed so
     far, and the sync below recomposes the cached cutoff word whichever
     one moved, because a depth the matrix does not read yet still has to
     survive to the merge that reads it. */
  if ((address & 0xf0f000u) == 0x402000u &&
      blockPart((uint8_t)((address >> 8) & 0x0f), port, &part)) {
    ChannelState *state = device->channels + part;
    uint8_t group = (uint8_t)((address & 0xffu) >> 4);
    uint8_t destination = (uint8_t)(address & 0x0fu);
    if (group >= XP_MATRIX_SOURCE_COUNT ||
        destination >= XP_MATRIX_DEST_COUNT || value > 127)
      return false;
    state->matrix_depth[group][destination] = value;
    syncTvf(device, part);
    if (group == XP_MATRIX_MODULATION &&
        destination == XP_MATRIX_LFO1_PITCH_DEPTH)
      syncLfo1PitchDepth(device, part);
    return true;
  }
  /* Patch-part block. Family 40 addresses the group on the same side as the
     port the message arrived on and family 50 the opposite group, so the
     caller's port is what decides which sixteen parts `1x` counts within. */
  if ((address & 0xf0f000u) == 0x401000u &&
      blockPart((uint8_t)((address >> 8) & 0x0f), port, &part)) {
    ChannelState *state = device->channels + part;
    switch (address & 0xffu) {
    case 0x0a:
      /* @provenance class=FW-EXACT devices=SC-88 ref=0x13854,0x456b,0x360b
         Rx.NRPN: entry 0a of the `40 1x` table at 0x13804 names the
         switch-word writer `456b` and bit 15. Turning it off also runs
         `360b`, which leaves no NRPN selected. The handler takes any
         nonzero value as on; values above the manual's 00..01 are refused
         here, as this file refuses other out-of-range writes. */
      if (value > 1)
        return false;
      state->rx_nrpn = value != 0;
      if (!state->rx_nrpn) {
        state->nrpn_msb = 127;
        state->nrpn_lsb = 127;
      }
      return true;
    case 0x23:
    case 0x24:
      /* @provenance class=FW-EXACT devices=SC-88 ref=0x1391c,0x13924,0x4825
         Rx.Bank Select and Rx.Bank Select LSB. Both table entries name
         `4825`, which sets the part's receive-off bit for 00 and clears
         it otherwise; setting either bit also clears the stored CC0
         value at `d860 + 2*part`. Values above 01 are refused, as for
         Rx.NRPN. */
      if (value > 1)
        return false;
      if ((address & 0xffu) == 0x23u)
        state->rx_bank_select = value != 0;
      else
        state->rx_bank_select_lsb = value != 0;
      if (value == 0)
        state->variation = 0;
      return true;
    case 0x14:
      if (value > 2)
        return false;
      state->same_note_mode = value == 0 ? XP_SAME_NOTE_SINGLE
        : value == 1 ? XP_SAME_NOTE_LIMITED_MULTI
        : XP_SAME_NOTE_FULL_MULTI;
      return true;
    case 0x15:
      /* Off, drum setup MAP1 or MAP2 - the only way a song can put drums
         on a part other than 10, and the switch that says which of the two
         working copies of a kit the part's own `41 mf rr` edits reach. */
      if (value > 2)
        return false;
      engine_set_part_rhythm(&device->engine, part, value);
      return true;
    case 0x30:
      state->vibrato_rate = value;
      syncLfo(device, part);
      return true;
    case 0x31:
      state->vibrato_depth = value;
      syncLfo(device, part);
      return true;
    case 0x37:
      state->vibrato_delay = value;
      syncLfo(device, part);
      return true;
    case 0x13:
      if (value > 1)
        return false;
      state->mono_mode = value;
      return true;
    case 0x1f:
      if (value > 0x5f)
        return false;
      state->cc1_assign = value;
      return true;
    case 0x20:
      if (value > 0x5f)
        return false;
      state->cc2_assign = value;
      return true;
    case 0x16:
      if (value < 0x28 || value > 0x58)
        return false;
      state->key_shift = value;
      syncPitch(device, part);
      return true;
    case 0x19:
      state->volume = value;
      syncPart(device, part);
      return true;
    case 0x1c:
      state->pan = value;
      syncPart(device, part);
      return true;
    case 0x34:
      state->attack = value;
      syncTva(device, part);
      return true;
    case 0x35:
      state->decay = value;
      syncTva(device, part);
      return true;
    case 0x36:
      state->release = value;
      return true;
    case 0x21:
      state->chorus_send = value;
      engine_set_part_chorus_send(&device->engine, part, value);
      return true;
    case 0x2c:
      state->delay_send = value;
      engine_set_part_delay_send(&device->engine, part, value);
      return true;
    case 0x22:
      state->reverb_send = value;
      engine_set_part_reverb_send(&device->engine, part, value);
      return true;
    case 0x25:
      /* Not in any held parameter map. Skatey Eight writes it once. It is
         accepted and counted so the packet is not refused wholesale, and
         so the count says something is being carried and discarded. */
      ++device->unhandled_sysex;
      return true;
    default:
      return false;
    }
  }
  return false;
}

}  // namespace

bool device_init_raw(Device *device, const uint8_t *controlRom,
                      size_t controlRomSize,
                      const uint8_t *const rawChips[XP_WAVE_CHIP_COUNT],
                      const size_t rawSizes[XP_WAVE_CHIP_COUNT],
                      double outputRate, enum xp_fractional_wrap wrap)
{
  return initCommon(device, controlRom, controlRomSize, rawChips, rawSizes,
                     outputRate, wrap, true);
}

bool device_init_decoded(
  Device *device, const uint8_t *controlRom,
  size_t controlRomSize,
  const uint8_t *const decodedChips[XP_WAVE_CHIP_COUNT],
  const size_t decodedSizes[XP_WAVE_CHIP_COUNT], double outputRate,
  enum xp_fractional_wrap wrap)
{
  return initCommon(device, controlRom, controlRomSize, decodedChips,
                     decodedSizes, outputRate, wrap, false);
}

void device_destroy(Device *device)
{
  if (!device)
    return;
  if (device->voice_ops) {
    device->voice_ops->destroy(device->voice_state);
    device->voice_state = nullptr;
  } else if (device->initialized) {
    engine_destroy(&device->engine);
  }
  renderer_destroy(&device->renderer);
  reverb_destroy(&device->reverb);
  tvf_coefficients_destroy(&device->tvf_coefficients);
  std::free(device->send_bus);
  std::free(device->chorus_bus);
  std::free(device->delay_bus);
  chorus_destroy(&device->chorus);
  delay_destroy(&device->delay);
  for (unsigned chip = 0; chip < XP_WAVE_CHIP_COUNT; ++chip)
    std::free(device->decoded_chips[chip]);
  std::free(device->control_rom);
  std::memset(device, 0, sizeof *device);
}

bool device_gm_system_on(Device *device)
{
  if (!device || !device->initialized)
    return false;
  if (device->voice_ops) {
    if (!device->voice_ops->gm_system_on)
      return false;
    return device->voice_ops->gm_system_on(device->voice_state);
  }
  if (!xp_profile(&device->renderer.rom)->firmwarePortGmMode)
    return false;
  /* @provenance class=FW-EXACT devices=SC-88 ref=0x128c6,0x128fb,0x1291a,0x129a1,0x129c0,0x13184
     GM System On runs the same module reset a GS Reset does (`0x12aa1`,
     `0x12ac5`, from the part image at 0x13184) and differs from it in two
     bits per part: it sets +18 bit 0, Rx.Bank Select off, where the GS
     Reset instead sets +2 bit 15, Rx.NRPN on, which the image leaves off.
     Rx.Bank Select LSB stays on, but CC32 is refused while bit 0 is set.
     The firmware's reset reaches one module in MODE-2 and both in MODE-1;
     this reset, like the GS Reset here, reaches every part. */
  device_reset_controllers(device);
  return device_gm_receive_off(device);
}

bool device_gm_receive_off(Device *device)
{
  if (!device || !device->initialized || device->voice_ops ||
      !xp_profile(&device->renderer.rom)->firmwarePortGmMode)
    return false;
  for (unsigned part = 0; part < XP_ENGINE_PART_COUNT; ++part) {
    device->channels[part].rx_bank_select = false;
    device->channels[part].rx_nrpn = false;
  }
  return true;
}

bool device_has_gm_mode(const Device *device)
{
  if (!device || !device->initialized)
    return false;
  if (device->voice_ops)
    return device->voice_ops->gm_system_on != nullptr;
  return xp_profile(&device->renderer.rom)->firmwarePortGmMode;
}

bool device_has_gs_mode(const Device *device)
{
  return device && device->initialized &&
         xp_profile(&device->renderer.rom)->sysexModelId == 0x42;
}

bool device_host_reset_enters_gm(const Device *device)
{
  if (!device || !device->initialized)
    return false;
  return xp_profile(&device->renderer.rom)->hostResetEntersGm;
}

void device_reset_controllers(Device *device)
{
  if (!device || !device->initialized)
    return;
  if (device->voice_ops) {
    /* No effect macros to reload: this device's effects are bypassed
       rather than approximated, and its own reset lives in its engine. */
    device->voice_ops->reset(device->voice_state);
    for (unsigned part = 0; part < XP_ENGINE_PART_COUNT; ++part)
      device->channels[part] = ChannelState();
    return;
  }
  /* @provenance class=UNVERIFIED devices=SC-88
     The reset cuts every part's voices before restoring the defaults, as
     the shared Part/Note engine's reset does for the SC-55 family and as
     the JV-1080's GM System On does from its firmware. Neither the SC-88's
     manual nor its traced firmware says what a GS reset does to notes
     still sounding. Left sounding, notes a song never released (a player
     stopped mid-song) were brought back to full level by the next song's
     reset and held for good. */
  for (unsigned part = 0; part < XP_ENGINE_PART_COUNT; ++part)
    engine_all_sound_off(&device->engine, (uint8_t)part);
  device->master_volume = 127;
  device->secondary_level = 127;
  device->master_pan = 64;
  /* The reverb block a reset leaves behind is macro 4's own preset row.
     The power-on image at ROM 0x13104 - the patch common block, recognisable
     by the default patch name in its first sixteen bytes - holds
     `04 04 00 40 40 00 00 00` there: the macro, then the seven bytes macro 4
     copies. So this is a table read, not a second set of constants. */
  (void)loadReverbMacro(device, 4);
  reverb_reset(&device->reverb);
  /* The chorus block a reset leaves behind is macro 2's own preset row, the
     same way the reverb's is macro 4's; the power-on image carries it. */
  (void)loadChorusMacro(device, 2);
  chorus_reset(&device->chorus);
  /* Delay macro 0 and the ten bytes it copies over pre-LPF through reverb
     send, which is what writing the macro address does. */
  /* Both corners at their first setting and both gains at the centre,
     which is the identity, and enabled - as `40 4x 20` defaults to. */
  device->eq_low_frequency = 0;
  device->eq_high_frequency = 0;
  device->eq_low_gain = 0x40;
  device->eq_high_gain = 0x40;
  device->eq.enabled = true;
  syncEq(device);
  eq_reset(&device->eq);
  output_reset(&device->output);
  device->delay_macro = 0;
  if (delay_macro(&device->renderer.rom, 0, device->delay_params))
    (void)delay_set_params(&device->renderer.rom, &device->delay,
                            device->delay_params);
  delay_reset(&device->delay);
  for (unsigned part = 0; part < XP_ENGINE_PART_COUNT; ++part) {
    ChannelState *channel = device->channels + part;
    channel->variation = 0;
    channel->tone_map_forced = 0;
    channel->program = 0;
    channel->volume = 100;
    channel->expression = 127;
    channel->pan = 64;
    channel->hold1 = 0;
    /* GS's own default part reverb send is 40, and its chorus send 0
       (SC88-OM), so a song that wants chorus has to ask for it. */
    channel->reverb_send = 40;
    engine_set_part_reverb_send(&device->engine, (uint8_t)part, 40);
    channel->chorus_send = 0;
    engine_set_part_chorus_send(&device->engine, (uint8_t)part, 0);
    channel->delay_send = 0;
    engine_set_part_delay_send(&device->engine, (uint8_t)part, 0);
    channel->cutoff = 64;
    channel->resonance = 64;
    channel->attack = 64;
    channel->decay = 64;
    channel->release = 64;
    channel->modulation = 0;
    channel->key_shift = 64;
    channel->cc1_assign = 0x10;
    channel->cc2_assign = 0x11;
    channel->cc1_value = 0;
    channel->cc2_value = 0;
    channel->mono_mode = 1;
    channel->vibrato_rate = 64;
    channel->vibrato_depth = 64;
    channel->vibrato_delay = 64;
    syncLfo(device, (uint8_t)part);
    channel->portamento_time = 0;
    channel->portamento_switch = 0;
    /* 0xff, not 0: the source key is a one-shot and 0xff is its "unset",
       which `0x3569` and `0x4606` write on the reset paths. Zero would name
       key 0 and glide every note up from it. */
    channel->portamento_control = 0xffu;
    engine_set_part_portamento(&device->engine, (uint8_t)part, false);
    engine_set_part_portamento_time(&device->engine, (uint8_t)part, 0);
    engine_set_part_portamento_control(&device->engine, (uint8_t)part, 0xffu);
    /* The matrix's reset depths are ROM data: the part image at SC88-CTL
       0x13184, whose alignment is pinned by +0x26/+0x27 = 10/11 (the CC1
       and CC2 assignments), +0x08 = 0x64 (part level 100), +0x0f = 0x28
       (reverb send 40) and +0x12/+0x13 = 0x40/0x40. Its six matrix groups
       sit twelve bytes apart from +0x28 and every one of them resets
       pitch, cutoff, amplitude and both LFO rates to the neutral 0x40 and
       the six LFO depths to zero. Two bytes differ: modulation's LFO1
       pitch depth is 0x0a, which is the manual's 47 cents, and pitch
       bend's pitch depth is 0x42, its two semitones.

       So a controller moves nothing but pitch until a song sends `40 2x`,
       which is why wiring the cutoff destination leaves every render of
       this corpus untouched. */
    std::memset(channel->matrix_depth, 0, sizeof channel->matrix_depth);
    for (unsigned source = 0; source < XP_MATRIX_SOURCE_COUNT; ++source) {
      channel->matrix_depth[source][XP_MATRIX_PITCH] = 0x40;
      channel->matrix_depth[source][XP_MATRIX_CUTOFF] = 0x40;
      channel->matrix_depth[source][XP_MATRIX_AMPLITUDE] = 0x40;
      channel->matrix_depth[source][XP_MATRIX_LFO1_RATE] = 0x40;
      channel->matrix_depth[source][XP_MATRIX_LFO2_RATE] = 0x40;
    }
    channel->matrix_depth[XP_MATRIX_MODULATION]
                         [XP_MATRIX_LFO1_PITCH_DEPTH] = 0x0a;
    channel->matrix_depth[XP_MATRIX_PITCH_BEND][XP_MATRIX_PITCH] = 0x42;
    channel->channel_pressure = 0;
    syncLfo1PitchDepth(device, (uint8_t)part);
    channel->pitch_bend = 8192;
    channel->pitch_bend_sensitivity = 2;
    channel->rpn_msb = 127;
    channel->rpn_lsb = 127;
    channel->nrpn_msb = 127;
    channel->nrpn_lsb = 127;
    /* The part image at 0x13184 has +18 = 00, both bank switches on; the
       GS Reset's `0x129a1`/`0x129c0` then set Rx.NRPN, +2 bit 15. */
    channel->rx_bank_select = true;
    channel->rx_bank_select_lsb = true;
    channel->rx_nrpn = true;
    channel->same_note_mode = XP_SAME_NOTE_LIMITED_MULTI;
    /* GS puts the rhythm part on MIDI channel 10, i.e. part 9 of each port,
       and a GS reset restores exactly that. It plays from drum setup MAP1:
       `04_protocol/sysex.md` records the manual's "part 10 initially map
       1", every `41 mf rr` edit in the corpus is addressed to MAP1 - demo
       song 3 sets its kick's panpot with `41 04 23` - and the firmware
       agrees, since `474b` maps the value 01 to the flags byte 0x30 whose
       bit 5 points the part at the first of the two kit working areas.
       Which kit set it plays is a different axis: the tone map, which the
       part image the firmware copies at reset (`131f4`, first word 0002)
       and SC88-OM printed 7-31 both leave on the SC-88. Map 1 would be
       wrong as well as different - it holds only the ten SC-55 kits, and
       three of the seven demo songs ask channel 10 for wire program 49,
       ETHNIC, which exists in the SC-88 map alone. */
    engine_set_part_rhythm(&device->engine, (uint8_t)part,
                            (part % 16u) == 9u ? 1u : 0u);
    syncToneMap(device, (uint8_t)part);
    engine_hold_value(&device->engine, (uint8_t)part, 0);
    engine_hold(&device->engine, (uint8_t)part, false);
    engine_sostenuto(&device->engine, (uint8_t)part, false);
    syncPart(device, (uint8_t)part);
    syncPitch(device, (uint8_t)part);
    syncTvf(device, (uint8_t)part);
  }
}

bool device_set_max_voices(Device *device, unsigned maxVoices)
{
  if (!device || !device->initialized)
    return false;
  if (device->voice_ops)
    return device->voice_ops->set_max_voices(device->voice_state, maxVoices);
  return engine_set_max_voices(&device->engine, maxVoices);
}

bool device_set_worker_pool(Device *device, WorkerPool *pool)
{
  if (!device || !device->initialized)
    return false;
  if (device->voice_ops) {
    if (!device->voice_ops->set_worker_pool)
      return !pool || pool->threads() <= 1;
    return device->voice_ops->set_worker_pool(device->voice_state, pool);
  }
  return engine_set_worker_pool(&device->engine, pool);
}

void device_panic(Device *device)
{
  if (!device || !device->initialized)
    return;
  if (device->voice_ops) {
    if (device->voice_ops->panic)
      device->voice_ops->panic(device->voice_state);
    return;
  }
  for (unsigned part = 0; part < XP_ENGINE_PART_COUNT; ++part) {
    engine_all_sound_off(&device->engine, (uint8_t)part);
    (void)device_midi(device, (uint8_t)(part / 16u),
                      (uint8_t)(0xb0u | (part % 16u)), 121, 0);
  }
}

void device_set_master_volume(Device *device, uint8_t value)
{
  if (!device || !device->initialized || value > 127)
    return;
  device->master_volume = value;
  for (unsigned part = 0; part < XP_ENGINE_PART_COUNT; ++part)
    syncPart(device, (uint8_t)part);
}

void device_set_master_pan(Device *device, uint8_t value)
{
  if (!device || !device->initialized || value < 1 || value > 127)
    return;
  device->master_pan = value;
  for (unsigned part = 0; part < XP_ENGINE_PART_COUNT; ++part)
    syncPart(device, (uint8_t)part);
}

bool device_sysex(Device *device, uint8_t port,
                   const uint8_t *data, size_t size)
{
  if (!device || !device->initialized || !data ||
      port >= XP_MIDI_PORT_COUNT)
    return false;
  /* accept the message with or without its framing bytes */
  if (size && data[0] == 0xf0) {
    ++data;
    --size;
  }
  if (size && data[size - 1] == 0xf7)
    --size;
  /* The universal GM System On, 7E 7F 09 01, on a device whose own voice
     path has a GM mode. Only the broadcast ID is compared: the JV-1080's
     dispatcher at `0x0A016F8C` accepts 7F and nothing else. */
  if (device->voice_ops && size == 4u && data[0] == 0x7e &&
      data[1] == 0x7f && data[2] == 0x09 && data[3] == 0x01)
    return device_gm_system_on(device);
  const struct XpDeviceProfile *profile = xp_profile(&device->renderer.rom);
  /* @provenance class=FW-EXACT devices=SC-88 ref=0x0e70,0x17e2,0x128c6,0x1296c
     The same message on a device the firmware port serves, when its
     firmware has a GM mode. `0x0e70` compares 7F and 09 and nothing else
     of the header, so here too only the broadcast ID is received; `0x17e2`
     then acts on two sub-IDs: 01, GM System On, and 02, which runs the
     GS Reset's own module reset (`0x1296c`). Both are gated by the panel's
     Rx GM On switch, which this device does not model and leaves on. */
  if (!device->voice_ops && profile->firmwarePortGmMode && size == 4u &&
      data[0] == 0x7e && data[1] == 0x7f && data[2] == 0x09) {
    if (data[3] == 0x01)
      return device_gm_system_on(device);
    if (data[3] == 0x02) {
      device_reset_controllers(device);
      return true;
    }
    return false;
  }
  /* 41 dev <model> 12, the device's own address bytes, at least one data
     byte, checksum. The model id and the address width are the device's,
     so they come from its profile: three bytes on a GS device, four on
     this family's JV member. */
  const size_t addressBytes = profile->sysexAddressBytes;
  if (addressBytes < 1u || addressBytes > 4u)
    return false;
  if (size < 6u + addressBytes || data[0] != 0x41 ||
      data[2] != profile->sysexModelId || data[3] != 0x12)
    return false;
  /* the unit's own device ID, or the broadcast ID every unit answers */
  if (data[1] != 0x10 && data[1] != 0x7f)
    return false;
  for (size_t i = 4; i < size; ++i)
    if (data[i] > 0x7f)
      return false;
  unsigned sum = 0;
  for (size_t i = 4; i + 1 < size; ++i)
    sum += data[i];
  if (((unsigned)(-(int)sum) & 0x7fu) != data[size - 1])
    return false;
  const size_t payloadAt = 4u + addressBytes;

  /* A device with its own voice path is handed the address bytes as they
     arrived and the payload whole. Its own frames may run past an
     address-byte boundary - the machine relies on it - and there is
     nowhere for a per-address loop to carry. */
  if (device->voice_ops) {
    if (!device->voice_ops->sysex_block(device->voice_state, data + 4,
                                        (unsigned)addressBytes,
                                        data + payloadAt,
                                        size - payloadAt - 1u))
      ++device->unhandled_sysex;
    return true;
  }
  if (addressBytes != 3u)
    return false;
  uint32_t address =
    ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 8) | data[6];
  /* Family 50 is the same map addressed to the other group. Normalising it
     to 40 against the opposite port keeps one set of address cases. */
  uint8_t group = port;
  if ((address & 0xf00000u) == 0x500000u) {
    address = (address & 0x0fffffu) | 0x400000u;
    group = (uint8_t)(XP_MIDI_PORT_COUNT - 1u - port);
  }
  /* A packet may carry several consecutive addresses. Each is applied in
     turn, so a packet naming one address this implementation does not know
     does not discard the rest of it. */
  for (size_t i = payloadAt; i + 1 < size; ++i)
    if (!sysexWrite(device, group, address + (uint32_t)(i - payloadAt),
                    data[i]))
      ++device->unhandled_sysex;
  return true;
}

/* The channel messages a device with its own voice path receives. Bank
   select and program change are held apart because either may arrive
   first and the device resolves the pair only when the program change
   does. Anything else is accepted and dropped: accepted, because a song
   sending a controller this device does not act on is not a malformed
   song, and counted nowhere because a dropped controller is not a
   dropped parameter write. */
bool midiToVoiceEngine(Device *device, uint8_t part, uint8_t status,
                        uint8_t data1, uint8_t data2)
{
  const struct XpVoiceEngineOps *ops = device->voice_ops;
  void *voices = device->voice_state;
  switch (status & 0xf0) {
  case 0x80:
    return ops->note_off(voices, part, data1);
  case 0x90:
    return data2 == 0 ? ops->note_off(voices, part, data1)
                      : ops->note_on(voices, part, data1, data2);
  case 0xc0:
    return ops->program_change(voices, part, data1);
  case 0xb0:
    ops->control_change(voices, part, data1, data2);
    return true;
  case 0xe0:
    if (ops->pitch_bend)
      ops->pitch_bend(voices, part, (unsigned)data1 | ((unsigned)data2 << 7));
    return true;
  case 0xd0:
    if (ops->channel_pressure)
      ops->channel_pressure(voices, part, data1);
    return true;
  default:
    return true;
  }
}

bool device_midi(Device *device, uint8_t port, uint8_t status,
                  uint8_t data1, uint8_t data2)
{
  if (!device || !device->initialized || port >= XP_MIDI_PORT_COUNT ||
      (status & 0x80) == 0 || data1 > 127 || data2 > 127)
    return false;
  uint8_t channel = status & 0x0f;
  uint8_t part = (uint8_t)(port * 16 + channel);
  ChannelState *state = device->channels + part;
  /* A device with its own voice path answers the messages that reach it
     and nothing else. Every case below this line writes the shared
     firmware port's own part registers, which such a device does not
     have; falling through would write state nothing reads and, worse,
     call into an engine that was never initialised. */
  if (device->voice_ops)
    return midiToVoiceEngine(device, part, status, data1, data2);
  switch (status & 0xf0) {
  case 0x80:
    return engine_note_off(&device->engine, part, data1);
  case 0x90:
    if (data2 == 0)
      return engine_note_off(&device->engine, part, data1);
    return engine_note_on(&device->engine, part, state->variation,
                           state->program, data1, data2, 0,
                           state->same_note_mode, 1.0f);
  case 0xb0:
    switch (data1) {
    case 0:
      /* `3172`: refused while Rx.Bank Select is off. */
      if (!state->rx_bank_select)
        return true;
      state->variation = data2;
      return true;
    case 1:
      /* Modulation is a matrix source, not a vibrato control: the depth
         each of its eleven destinations gets is a separate parameter, and
         after a reset only LFO1 pitch depth is nonzero. */
      state->modulation = data2;
      syncLfo1PitchDepth(device, part);
      syncTvf(device, part);
      return true;
    case 91:
      state->reverb_send = data2;
      engine_set_part_reverb_send(&device->engine, part, data2);
      return true;
    case 94:
      state->delay_send = data2;
      engine_set_part_delay_send(&device->engine, part, data2);
      return true;
    case 5:
      state->portamento_time = data2;
      engine_set_part_portamento_time(&device->engine, part, data2);
      return true;
    case 65:
      /* `0x31e4` tests the receive switch, then bit 6 of the value: the
         64 threshold, on at or above it. */
      state->portamento_switch = data2;
      engine_set_part_portamento(&device->engine, part, data2 >= 64);
      return true;
    case 84:
      state->portamento_control = data2;
      engine_set_part_portamento_control(&device->engine, part, data2);
      return true;
    /* @provenance class=FW-EXACT devices=SC-88 ref=0x127b8,0x352d,0x3611,0x361c,0x362e
       The controller table at 0x127b8 sends 120 to 0x352d, 123..125 to
       0x3611, 126 to 0x361c and 127 to 0x362e, and each acts only on a
       value of 0 (`tst r0; bne`). 126 and 127 set the part's mode bit and
       then run the 120 handler: All Sound Off with the mode change, as the
       manual has it (om-127). */
    case 120:
      if (data2 == 0)
        engine_all_sound_off(&device->engine, part);
      return true;
    case 123:
    case 124:
    case 125:
      if (data2 == 0)
        engine_all_notes_off(&device->engine, part);
      return true;
    case 126:
      if (data2 == 0) {
        state->mono_mode = 0;
        engine_all_sound_off(&device->engine, part);
      }
      return true;
    case 127:
      if (data2 == 0) {
        state->mono_mode = 1;
        engine_all_sound_off(&device->engine, part);
      }
      return true;
    case 93:
      state->chorus_send = data2;
      engine_set_part_chorus_send(&device->engine, part, data2);
      return true;
    case 6:
      if (state->rpn_msb == 0 && state->rpn_lsb == 0) {
        state->pitch_bend_sensitivity = data2 > 24 ? 24 : data2;
        syncPitch(device, part);
        return true;
      }
      if (state->nrpn_msb == 1 && state->nrpn_lsb == 0x20) {
        state->cutoff = data2;
        syncTvf(device, part);
        return true;
      }
      if (state->nrpn_msb == 1 && state->nrpn_lsb == 0x21) {
        state->resonance = data2;
        syncTvf(device, part);
        return true;
      }
      /* The envelope-time modifiers. `07_synthesis/tva.md` gives the law:
         stages 0 and 1 take the attack modifier, stages 2 and 3 the decay
         one, doubled about their centre. Release is held but not applied -
         the release path's own part modifier is not recovered. */
      if (state->nrpn_msb == 1 && state->nrpn_lsb == 0x63) {
        state->attack = data2;
        syncTva(device, part);
        return true;
      }
      if (state->nrpn_msb == 1 && state->nrpn_lsb == 0x64) {
        state->decay = data2;
        syncTva(device, part);
        return true;
      }
      if (state->nrpn_msb == 1 && state->nrpn_lsb == 0x66) {
        state->release = data2;
        return true;
      }
      /* The drum-instrument block: the MSB names the parameter and the LSB
         is the note. It reaches the same per-note overrides the `41 mf rr`
         addresses do, except that its pitch is a centred offset. */
      if (state->nrpn_msb >= 0x18 && state->nrpn_msb <= 0x1f) {
        static constexpr uint8_t kFieldOf[8] = {
          10u,  /* 18 pitch, relative */
          0u,   /* 19 unassigned */
          2u,   /* 1a TVA level */
          0u,   /* 1b unassigned */
          4u,   /* 1c panpot */
          5u,   /* 1d reverb send */
          6u,   /* 1e chorus send */
          9u    /* 1f delay send */
        };
        uint8_t field = kFieldOf[state->nrpn_msb - 0x18u];
        uint8_t setup = device->engine.parts[part].rhythm_setup;
        if (!field || !setup)
          return false;
        return engine_set_drum_parameter(&device->engine, setup, field,
                                          state->nrpn_lsb, data2);
      }
      if (state->nrpn_msb == 1 && state->nrpn_lsb == 0x08) {
        state->vibrato_rate = data2;
        syncLfo(device, part);
        return true;
      }
      if (state->nrpn_msb == 1 && state->nrpn_lsb == 0x09) {
        state->vibrato_depth = data2;
        syncLfo(device, part);
        return true;
      }
      if (state->nrpn_msb == 1 && state->nrpn_lsb == 0x0a) {
        state->vibrato_delay = data2;
        syncLfo(device, part);
        return true;
      }
      return false;
    case 7:
      state->volume = data2;
      syncPart(device, part);
      return true;
    case 10:
      state->pan = data2;
      syncPart(device, part);
      return true;
    case 11:
      state->expression = data2;
      syncPart(device, part);
      return true;
    case 32:
      /* The tone map, 0 the part's selected map and 1 and 2 forcing the
       * SC-55 and SC-88 maps. `317c` stores the value in the high byte of
       * the part's bank word, which both the melodic selector `2d3e` and
       * the drum one `2e7a` read, so this moves either kind of part onto
       * either map.
       * The device stores 3..127 as well and lets the next Program Change
       * resolve no tone at all, which is a silenced part rather than a
       * changed one; this refuses the message instead, and no corpus file
       * sends one. */
      /* `317c`: refused while either bank switch is off. */
      if (!state->rx_bank_select || !state->rx_bank_select_lsb)
        return true;
      if (data2 > 2)
        return false;
      state->tone_map_forced = data2;
      syncToneMap(device, part);
      return true;
    case 64:
      /* Two things, and only the second was being done: the pedal both
         retains notes past Note Off and scales their release. */
      state->hold1 = data2;
      engine_hold(&device->engine, part, data2 >= 64);
      engine_hold_value(&device->engine, part, data2);
      return true;
    case 66:
      engine_sostenuto(&device->engine, part, data2 >= 64);
      return true;
    /* `3253` and `3261` leave RPN unselected whatever Rx.NRPN says, and
       store the NRPN number only while it is on. */
    case 98:
      if (state->rx_nrpn)
        state->nrpn_lsb = data2;
      state->rpn_msb = 127;
      state->rpn_lsb = 127;
      return true;
    case 99:
      if (state->rx_nrpn)
        state->nrpn_msb = data2;
      state->rpn_msb = 127;
      state->rpn_lsb = 127;
      return true;
    case 100:
      state->rpn_lsb = data2;
      state->nrpn_msb = 127;
      state->nrpn_lsb = 127;
      return true;
    case 101:
      state->rpn_msb = data2;
      state->nrpn_msb = 127;
      state->nrpn_lsb = 127;
      return true;
    case 121:
      /* @provenance class=DOCUMENTED devices=SC-88
         The manual's Reset All Controllers list (om-127): pitch bend
         centre, polyphonic and channel pressure 0, modulation 0, expression
         127, Hold 1, portamento, sostenuto and soft 0, RPN and NRPN
         unset with their data unchanged. */
      state->expression = 127;
      state->hold1 = 0;
      state->pitch_bend = 8192;
      state->modulation = 0;
      state->channel_pressure = 0;
      state->portamento_switch = 0;
      engine_set_part_portamento(&device->engine, part, false);
      syncLfo1PitchDepth(device, part);
      state->rpn_msb = 127;
      state->rpn_lsb = 127;
      state->nrpn_msb = 127;
      state->nrpn_lsb = 127;
      engine_hold_value(&device->engine, part, 0);
      engine_sostenuto(&device->engine, part, false);
      syncPart(device, part);
      syncPitch(device, part);
      syncTvf(device, part);
      return true;
    default:
      /* An ordinary controller below CC120 is compared with this part's
         CC1/CC2 assignment before being refused, which is the order
         `04_protocol/controllers.md` gives. Every matrix depth those two
         feed is zero after a reset, so this receives them and applies the
         zero. */
      if (data1 < 120) {
        if (data1 == state->cc1_assign) {
          state->cc1_value = data2;
          syncTvf(device, part);
          return true;
        }
        if (data1 == state->cc2_assign) {
          state->cc2_value = data2;
          syncTvf(device, part);
          return true;
        }
      }
      return false;
    }
  case 0xc0:
    if (device->engine.parts[part].rhythm_setup && data1 != state->program)
      engine_clear_drum_overlay(&device->engine,
                                 device->engine.parts[part].rhythm_setup);
    /* A rhythm part's program change is honoured whatever the bank MSB is.
       `04_protocol/program_bank.md` records the firmware as ignoring it
       while the MSB is nonzero, but the music contradicts that: Brass
       Nation is a full orchestra - tuba, horns, trombone, five string
       parts, flute, oboe, clarinet, bassoon, piccolo, harp, timpani,
       tubular bell, celesta, xylophone, glockenspiel - and its drum part
       asks for program 48, which is the ORCHESTRA kit in both maps. Opus 88
       asks for program 49, whose ETHNIC kit exists only in the SC-88 map,
       with a zero MSB. Honouring it satisfies both; ignoring it leaves an
       orchestra playing a pop kit (`M-024`). */
    state->program = data1;
    return true;
  case 0xd0:
    /* Channel aftertouch. Handler `0x30f7` stores the byte at DP:d6a0 and
       arms the matrix; every destination it reaches is a matrix depth, so
       nothing else here consumes it. */
    state->channel_pressure = data1;
    syncTvf(device, part);
    return true;
  case 0xe0:
    state->pitch_bend = (uint16_t)(data1 | ((uint16_t)data2 << 7));
    syncPitch(device, part);
    syncTvf(device, part);
    return true;
  default:
    return false;
  }
}

int16_t device_matrix_cutoff_word(const ChannelState *channel)
{
  uint16_t sum = matrixTerm(
    channel->matrix_depth[XP_MATRIX_MODULATION][XP_MATRIX_CUTOFF],
    channel->modulation);
  sum = (uint16_t)(sum + matrixTerm(
    channel->matrix_depth[XP_MATRIX_CHANNEL_PRESSURE][XP_MATRIX_CUTOFF],
    channel->channel_pressure));
  sum = (uint16_t)(sum + matrixTerm(
    channel->matrix_depth[XP_MATRIX_CC1][XP_MATRIX_CUTOFF],
    channel->cc1_value));
  sum = (uint16_t)(sum + matrixTerm(
    channel->matrix_depth[XP_MATRIX_CC2][XP_MATRIX_CUTOFF],
    channel->cc2_value));
  sum = (uint16_t)shiftRight(s16(sum), 1);

  int32_t bend = ((int32_t)channel->pitch_bend - INT32_C(0x2000)) * 4;
  int32_t product = (int32_t)centredDepth(
    channel->matrix_depth[XP_MATRIX_PITCH_BEND][XP_MATRIX_CUTOFF]) * bend;
  /* Bits 23..8, assembled at `0x11958` from the product's high byte and
     the high byte of its low word, then read as a signed word. */
  product = s16((uint16_t)(shiftRight(product, 8) & 0xffff));
  product = shiftRight(product * INT32_C(0x7f00), 16);
  return s16((uint16_t)(sum + (uint16_t)product));
}

void device_render(Device *device, float *stereo, size_t frames)
{
  if (!device || !device->initialized || !stereo)
    return;
  /* A device with its own voice path renders dry. Its insert, chorus and
     reverb effects are BYPASSED, not approximated: all forty insert types
     are characterised behaviourally in the research but their DSP
     topology needs the chip's instruction set and is open, and the output
     stage below is the other device's measured analogue front end, which
     is not this one's. Both are gaps, and a gap is honest where a guess
     would not be. */
  if (device->voice_ops) {
    std::memset(stereo, 0, frames * 2u * sizeof *stereo);
    device->voice_ops->render(device->voice_state, stereo, frames);
    return;
  }
  /* The send bus grows to whatever block the caller asks for and is kept,
     so a render loop does not allocate per block. */
  if (frames > device->send_capacity) {
    float *grown =
      (float *)std::realloc(device->send_bus, frames * sizeof *grown);
    if (!grown) {
      engine_render(&device->engine, stereo, frames);
      return;
    }
    device->send_bus = grown;
    float *grownChorus = (float *)std::realloc(device->chorus_bus,
                                                frames * sizeof *grownChorus);
    if (!grownChorus) {
      engine_render(&device->engine, stereo, frames);
      return;
    }
    device->chorus_bus = grownChorus;
    float *grownDelay = (float *)std::realloc(device->delay_bus,
                                               frames * sizeof *grownDelay);
    if (!grownDelay) {
      engine_render(&device->engine, stereo, frames);
      return;
    }
    device->delay_bus = grownDelay;
    device->send_capacity = frames;
  }
  /* The three buses are handed over uncleared: engine_render_with_send()
     assigns every one of the `frames` reverb, chorus and delay frames it is
     given, and the effects below read only what it wrote. Its own early
     return cannot leave them untouched from here, either - `initialized` is
     set only once engine_init() has succeeded, and that is what gives the
     engine the renderer the early return tests for. */
  engine_render_with_send(&device->engine, stereo, device->send_bus,
                           device->chorus_bus, device->delay_bus, frames);
  /* The delay runs before the reverb reads its bus, because it has its own
     send into the reverb and that send is recovered. */
  if (device->delay.active)
    delay_process(&device->delay, device->delay_bus, stereo,
                   device->send_bus, frames);
  /* The chorus runs before the reverb reads its bus, because the chorus
     has its own send into the reverb; that send is received and held but
     not yet routed, so the two effects are still parallel here. */
  if (device->chorus.active)
    chorus_process(&device->chorus, device->chorus_bus, stereo, frames);
  if (device->reverb.active)
    reverb_process(&device->reverb, device->send_bus, stereo, frames);
  eq_process(&device->eq, stereo, frames);
  /* Last: the output stage. Everything above this line is the digital
     machine; output.cc is the one place that carries behaviour we
     have measured but not derived. */
  output_process(&device->output, stereo, frames);
}

}}  // namespace EmuSC::Xp
