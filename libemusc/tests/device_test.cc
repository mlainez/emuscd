/* SPDX-License-Identifier: CC0-1.0 */
#include "engines/xp/device.h"
#include "engines/xp/common/worker_pool.h"

#include <cassert>
#ifdef NDEBUG
#error "this test is assertion-driven; NDEBUG compiles it away"
#endif
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

void put16(uint8_t *p, uint16_t value)
{
  p[0] = (uint8_t)(value >> 8);
  p[1] = (uint8_t)value;
}

void put24(uint8_t *p, uint32_t value)
{
  p[0] = (uint8_t)(value >> 16);
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)value;
}

/* One GS DT1 write, framed as the wire carries it. */
bool dt1(EmuSC::Xp::Device *device, uint8_t a, uint8_t b, uint8_t c,
         uint8_t value)
{
  uint8_t packet[10];
  unsigned sum = (unsigned)a + b + c + value;
  packet[0] = 0xf0; packet[1] = 0x41; packet[2] = 0x10;
  packet[3] = 0x42; packet[4] = 0x12;
  packet[5] = a; packet[6] = b; packet[7] = c; packet[8] = value;
  packet[9] = (uint8_t)((0u - sum) & 0x7fu);
  return EmuSC::Xp::device_sysex(device, 0, packet, 10);
}

uint8_t *read_exact(const char *path, size_t size)
{
  FILE *file = fopen(path, "rb");
  uint8_t *bytes = (uint8_t *)malloc(size);
  assert(file && bytes);
  assert(fread(bytes, 1, size, file) == size);
  assert(fgetc(file) == EOF);
  fclose(file);
  return bytes;
}

void make_control(uint8_t *control)
{
  static const uint8_t vectors[16] = {
    0x00, 0x00, 0x02, 0x00, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x01, 0xf4, 0x00, 0x00, 0x01, 0xf4
  };
  unsigned i;
  memcpy(control, vectors, sizeof vectors);
  memcpy(control + 0x30000, "\0\0Piano 1A    \3\377", 16);
  control[0x2fc80] = 0;
  put24(control + 0x20000, 0x40000);
  memcpy(control + 0x40000, "Test Tone   ", 12);
  control[0x40000 + 30] = 1;
  control[0x40000 + 32] = 3;
  control[0x40000 + 33] = 2;
  put16(control + 0x40000 + 0x0e, 0xbad0);
  put16(control + 0x40000 + 0x10, 0xb6d0);
  put16(control + 0x40000 + 34, 0);
  put16(control + 0x40000 + 34 + 0x14, 0x4000);
  control[0x40000 + 34 + 0x3e] = 0xff;
  /* A stage level word is an **attenuation**: zero is full level. Storing
     0xffff here once looked like full level because the conversion was
     inverted, which the ROM disproves - a piano's last two stages store
     0xffff and its tail is silent. */
  put16(control + 0x40000 + 34 + 0x78, 0x0000);
  control[0x40000 + 34 + 0x80] = 1;
  /* The component's velocity window, +6c..+6d inclusive. A calloc'd
     fixture states 0..0, which sounds nothing: every note this file
     plays would be refused. The ROM's own tones all reach 127. */
  control[0x40000 + 34 + 0x6c] = 0;
  control[0x40000 + 34 + 0x6d] = 127;
  control[0x30010] = 127;
  control[0x30011] = 0xff;
  put16(control + 0x30014, 0x6100);
  put24(control + 0x36101, 0x8000);
  control[0x36106] = 60;
  put24(control + 0x36107, 0x8000);
  put24(control + 0x3610b, 0x8001);
  /* The coarse and fine level tables, as a monotone ramp. The real tables
     are a dB curve; what matters to a fixture is that an intermediate
     attenuation converts to an intermediate gain, because an envelope
     stage ramps its attenuation and reads the tables all the way along. */
  for (i = 0; i < 256; ++i) {
    put16(control + 0x1503e + i * 2, (uint16_t)((i + 1) * 256 - 1));
    put16(control + 0x1523e + i * 2, (uint16_t)((i + 1) * 256 - 1));
  }
  put16(control + 0x15db6 + 63 * 2, 0x4c00);
  put16(control + 0x1573e + 64 * 2, 0xffff);
  put16(control + 0x1543e + 2, 0xffff);
  /* The stage's interpolation word, which the chip is handed beside the
     target: the component takes the exponential table at `0x1563e`, and
     this is the SC-88's own entry at the rate index above. Left at zero a
     fixture says "never move", and the stage would hold at its start
     level for its whole dwell. */
  put16(control + 0x1563e + 2, 0x0517);
}

/* The held ROMs' own paths are read from the environment at run time, so
   ctest's own environment carries them whatever the tree was configured
   with:
     SC88_CONTROL_ROM  the control ROM
     SC88_WAVE_ROMS    the four wave chips, comma separated, in chip order
   Without them the caller skips rather than passing while checking only
   the synthetic fixtures above. */
bool split_wave_roms(const char *csv, char paths[4][512])
{
  const char *p = csv;
  unsigned i;
  for (i = 0; i < 4; ++i) {
    const char *comma = strchr(p, ',');
    size_t len = comma ? (size_t)(comma - p) : strlen(p);
    if (len == 0 || len >= 512)
      return false;
    memcpy(paths[i], p, len);
    paths[i][len] = '\0';
    if (i < 3) {
      if (!comma)
        return false;
      p = comma + 1;
    } else if (comma) {
      return false;                      /* exactly four entries expected */
    }
  }
  return true;
}

void test_held_raw(char **paths)
{
  uint8_t *control = read_exact(paths[0], XP_CONTROL_ROM_SIZE);
  uint8_t *chips[XP_WAVE_CHIP_COUNT];
  const uint8_t *inputs[XP_WAVE_CHIP_COUNT];
  size_t sizes[XP_WAVE_CHIP_COUNT];
  EmuSC::Xp::Device device;
  float output[8192];
  double energy = 0.0;
  unsigned i;

  for (i = 0; i < XP_WAVE_CHIP_COUNT; ++i) {
    chips[i] = read_exact(paths[i + 1], SC88_PROFILE.waveChipSize);
    inputs[i] = chips[i];
    sizes[i] = SC88_PROFILE.waveChipSize;
  }
  assert(EmuSC::Xp::device_init_raw(&device, control, XP_CONTROL_ROM_SIZE,
                                     inputs, sizes, 48000.0,
                                     XP_WRAP_FULL_CARRY));
  assert(EmuSC::Xp::device_midi(&device, 0, 0x90, 60, 100));
  EmuSC::Xp::device_render(&device, output, 4096);
  for (i = 0; i < 8192; ++i)
    energy += fabs(output[i]);
  assert(energy > 0.0);
  EmuSC::Xp::device_destroy(&device);
  for (i = 0; i < XP_WAVE_CHIP_COUNT; ++i)
    free(chips[i]);
  free(control);
}

/* The three ways the engine can run its voices give the same bytes: frame
   by frame (what a stage tap forces), a segment at a time on the calling
   thread, and a segment at a time on a worker pool. The notes overlap on
   three channels at different velocities, so every frame sums voices of
   different levels and an out-of-order sum shows; the blocks are of uneven
   length, so segments end mid-period and voices run out mid-segment; and
   a four-voice ceiling makes notes steal slots and their voices ramp out
   on the stopping list. */
void test_render_paths_match(const uint8_t *control,
                             const uint8_t *const chips[],
                             const size_t sizes[])
{
  static const size_t blocks[] = {1, 37, 300, 1000, 5, 257, 2048, 64, 999};
  static float framedOut[2 * 2048], segmentedOut[2 * 2048];
  static float threadedOut[2 * 2048], tap[2048];
  EmuSC::Xp::Device framed, segmented, threaded;
  EmuSC::Xp::Device *all[3] = {&framed, &segmented, &threaded};
  EmuSC::Xp::WorkerPool pool(4);
  for (EmuSC::Xp::Device *d : all) {
    assert(EmuSC::Xp::device_init_decoded(d, control, XP_CONTROL_ROM_SIZE,
                                          chips, sizes, 32000.0,
                                          XP_WRAP_FULL_CARRY));
    assert(EmuSC::Xp::device_set_max_voices(d, 4));
  }
  struct xp_engine_stage_taps taps = {};
  taps.oscillator = tap;
  EmuSC::Xp::engine_set_stage_taps(&framed.engine, &taps);
  assert(EmuSC::Xp::device_set_worker_pool(&threaded, &pool));
  assert((threaded.engine.pool != nullptr) == (pool.threads() > 1));
  unsigned sounded = 0;
  auto render_all = [&](size_t frames) {
    EmuSC::Xp::device_render(&framed, framedOut, frames);
    EmuSC::Xp::device_render(&segmented, segmentedOut, frames);
    EmuSC::Xp::device_render(&threaded, threadedOut, frames);
    assert(std::memcmp(framedOut, segmentedOut,
                       frames * 2 * sizeof *framedOut) == 0);
    assert(std::memcmp(framedOut, threadedOut,
                       frames * 2 * sizeof *framedOut) == 0);
    unsigned slots = EmuSC::Xp::engine_active_slots(&framed.engine);
    assert(EmuSC::Xp::engine_active_slots(&segmented.engine) == slots);
    assert(EmuSC::Xp::engine_active_slots(&threaded.engine) == slots);
    for (size_t i = 0; i < frames * 2; ++i)
      sounded += framedOut[i] != 0.0f;
  };
  unsigned note = 0;
  for (unsigned round = 0; round < 6; ++round)
    for (size_t frames : blocks) {
      uint8_t ch = (uint8_t)(note % 3);
      uint8_t key = (uint8_t)(48 + (note * 7) % 24);
      for (EmuSC::Xp::Device *d : all) {
        EmuSC::Xp::device_midi(d, 0, (uint8_t)(0x90 | ch), key,
                               (uint8_t)(60 + (note * 13) % 67));
        if (note >= 2)
          EmuSC::Xp::device_midi(d, 0, (uint8_t)(0x80 | ((note - 2) % 3)),
                                 (uint8_t)(48 + ((note - 2) * 7) % 24), 64);
      }
      ++note;
      render_all(frames);
    }
  assert(sounded > 0);
  /* Three voices of one pitch struck together run out on the same frame of
     a segment, and the free list takes their slots back in the order the
     frame loop frees them - which decides which slot the next three notes
     get, and so the order three different levels are summed in. */
  for (EmuSC::Xp::Device *d : all)
    EmuSC::Xp::device_midi(d, 0, 0xb0, 123, 0);
  for (unsigned i = 0; i < 4; ++i)
    render_all(2048);
  for (unsigned chord = 0; chord < 4; ++chord) {
    for (EmuSC::Xp::Device *d : all)
      for (uint8_t ch = 0; ch < 3; ++ch)
        EmuSC::Xp::device_midi(d, 0, (uint8_t)(0x90 | ch),
                               (uint8_t)(50 + chord * 5),
                               (uint8_t)(127 - 40 * ch - chord));
    for (unsigned i = 0; i < 6; ++i)
      render_all(997);
  }
  /* Detached, the pool's device renders on the calling thread, in step. */
  assert(EmuSC::Xp::device_set_worker_pool(&threaded, nullptr));
  assert(threaded.engine.pool == nullptr);
  render_all(2048);
  for (EmuSC::Xp::Device *d : all)
    EmuSC::Xp::device_destroy(d);
}

}  // namespace

int main(void)
{
  uint8_t *control = (uint8_t *)calloc(XP_CONTROL_ROM_SIZE, 1);
  uint8_t *chip = (uint8_t *)calloc(SC88_PROFILE.waveChipSize, 1);
  const uint8_t *chips[XP_WAVE_CHIP_COUNT] = {chip, chip, chip, chip};
  const size_t sizes[XP_WAVE_CHIP_COUNT] = {
    SC88_PROFILE.waveChipSize, SC88_PROFILE.waveChipSize,
    SC88_PROFILE.waveChipSize, SC88_PROFILE.waveChipSize
  };
  EmuSC::Xp::Device device;
  float output[514];

  assert(control && chip);
  make_control(control);
  chip[0x8000] = 1;
  chip[0x8001] = 1;
  test_render_paths_match(control, chips, sizes);
  assert(EmuSC::Xp::device_init_decoded(&device, control, XP_CONTROL_ROM_SIZE,
                                        chips, sizes, 32000.0,
                                        XP_WRAP_FULL_CARRY));
  assert(device.channels[0].volume == 100);
  assert(device.channels[0].cutoff == 64);
  /* A reset makes MIDI channel 10 of each port a rhythm part playing from
     drum setup MAP1, and leaves every part's tone map on the SC-88, which
     is the kit set channel 10's programs index. The two are separate
     axes: the setup says where a song's `41 mf rr` edits land, the tone
     map says which kits exist. */
  assert(device.engine.parts[9].rhythm_setup == 1);
  assert(device.engine.parts[25].rhythm_setup == 1);
  assert(device.engine.parts[0].rhythm_setup == 0);
  assert(device.engine.parts[9].tone_map == XP_TONE_MAP_SC88);
  assert(device.engine.parts[0].tone_map == XP_TONE_MAP_SC88);
  /* CC32 forces a map; 0 goes back to the selected one, which nothing
     resets off the SC-88. */
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 32, 1));
  assert(device.engine.parts[0].tone_map == XP_TONE_MAP_SC55);
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 32, 0));
  assert(device.engine.parts[0].tone_map == XP_TONE_MAP_SC88);
  assert(EmuSC::Xp::device_midi(&device, 0, 0x90, 60, 100));
  assert(EmuSC::Xp::engine_active_slots(&device.engine) == 1);
  {
    double unity_step = device.engine.slots[0].component.oscillator.step;
    assert(EmuSC::Xp::device_midi(&device, 0, 0xe0, 127, 127));
    assert(device.engine.slots[0].component.oscillator.step > unity_step);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 101, 0));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 100, 0));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 6, 12));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xe0, 127, 127));
    assert(fabs(device.engine.slots[0].component.oscillator.step /
                unity_step - 1.9998) < 0.001);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 121, 0));
    assert(fabs(device.engine.slots[0].component.oscillator.step -
                unity_step) < 1e-12);
  }
  /* The controller destination matrix. Its reset depths are the part
     image at SC88-CTL 0x13184: every group's cutoff depth is the neutral
     0x40, so a controller at any position leaves the cached word at zero
     and the filter exactly where it was. That is why wiring this moves
     no render of a song that never writes `40 2x`. */
  {
    EmuSC::Xp::ChannelState *state = device.channels;
    unsigned source;
    for (source = 0; source < XP_MATRIX_SOURCE_COUNT; ++source)
      assert(state->matrix_depth[source][XP_MATRIX_CUTOFF] == 0x40);
    assert(state->matrix_depth[XP_MATRIX_MODULATION]
                              [XP_MATRIX_LFO1_PITCH_DEPTH] == 0x0a);
    assert(state->matrix_depth[XP_MATRIX_PITCH_BEND]
                              [XP_MATRIX_PITCH] == 0x42);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 1, 127));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xd0, 127, 0));
    assert(device.channels[0].channel_pressure == 127);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 16, 127));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 17, 127));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xe0, 127, 127));
    assert(EmuSC::Xp::device_matrix_cutoff_word(state) == 0);
    assert(device.engine.parts[0].tvf_controls.matrix_cutoff == 0);

    /* `40 21 01`: modulation to cutoff on part 1, at full positive
       depth. 63 * 127 halved is 4000, which is exactly where the
       consumer's clamp sits, so the term reaches its own full scale. */
    assert(dt1(&device, 0x40, 0x21, 0x01, 0x7f));
    assert(state->matrix_depth[XP_MATRIX_MODULATION]
                              [XP_MATRIX_CUTOFF] == 0x7f);
    assert(EmuSC::Xp::device_matrix_cutoff_word(state) == 4000);
    assert(device.engine.parts[0].tvf_controls.matrix_cutoff == 4000);
    assert(EmuSC::Xp::tvf_matrix_cutoff_term(4000) == 8191);
    /* Released, the wheel puts it back where it was. */
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 1, 0));
    assert(device.engine.parts[0].tvf_controls.matrix_cutoff == 0);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 1, 127));
    /* A depth below centre darkens instead. */
    assert(dt1(&device, 0x40, 0x21, 0x01, 0x00));
    assert(EmuSC::Xp::device_matrix_cutoff_word(state) == -4064);
    /* Neutral again, and the wheel goes back to moving nothing. */
    assert(dt1(&device, 0x40, 0x21, 0x01, 0x40));
    assert(EmuSC::Xp::device_matrix_cutoff_word(state) == 0);
    /* `40 4x 20` is the equaliser switch and `40 2x 20` the channel
       aftertouch group's pitch depth. The two are distinct addresses and
       neither reaches the other. */
    assert(dt1(&device, 0x40, 0x21, 0x20, 0x00));
    assert(device.eq.enabled);
    assert(state->matrix_depth[XP_MATRIX_CHANNEL_PRESSURE]
                              [XP_MATRIX_PITCH] == 0);
    assert(dt1(&device, 0x40, 0x41, 0x20, 0x00));
    assert(!device.eq.enabled);
    assert(dt1(&device, 0x40, 0x41, 0x20, 0x01));
    assert(device.eq.enabled);
    /* Put every source back at rest for the tests that follow. */
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 1, 0));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xd0, 0, 0));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 16, 0));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 17, 0));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 121, 0));
  }
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 99, 1));
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 98, 0x20));
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 6, 127));
  assert(device.channels[0].cutoff == 127);
  assert(device.engine.parts[0].tvf_dirty);
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 98, 0x21));
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 6, 96));
  assert(device.channels[0].resonance == 96);
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 10, 1));
  assert(device.engine.slots[0].component.pan_target_position == 1);
  EmuSC::Xp::device_render(&device, output, 257);
  assert(!device.engine.parts[0].tvf_dirty);
  assert(device.engine.slots[0].component.pan_position == 63);
  assert(output[0] > 0.0f && output[0] == output[1]);

  put16(device.control_rom + 0x14f3e, 0xffff);
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 7, 0));
  /* Silencing the part silences the voices, but the output stage is
     AC-coupled and rings briefly on any step, so what is asserted is that
     it settles rather than that it is zero on the next sample. The
     settled level is asserted against an absolute bound: comparing two
     samples of the tail against each other compares float noise once
     more than one section is in the path. */
  EmuSC::Xp::device_render(&device, output, 64);
  {
    unsigned s;
    for (s = 0; s < 64; ++s)
      assert(fabs(output[s * 2]) < 0.2f &&
             output[s * 2] == output[s * 2 + 1]);
    assert(fabs(output[126]) < 1e-3f);
  }
  put16(device.control_rom + 0x14f3e, 0);
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 7, 100));
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 64, 127));
  assert(EmuSC::Xp::device_midi(&device, 0, 0x80, 60, 64));
  EmuSC::Xp::device_render(&device, output, 257);
  assert(EmuSC::Xp::engine_active_slots(&device.engine) == 1);
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 64, 0));
  /* Two control periods: one for the release to run out and compose
     amplitude 0, one for the chip's register to glide to it. */
  EmuSC::Xp::device_render(&device, output, 257);
  EmuSC::Xp::device_render(&device, output, 257);
  assert(EmuSC::Xp::engine_active_slots(&device.engine) == 0);
  assert(EmuSC::Xp::device_midi(&device, 1, 0xc0, 0, 0));
  assert(!EmuSC::Xp::device_midi(&device, 2, 0x90, 60, 100));
  /* The tone map reaches a melodic part, not only a rhythm one: this image
     fills both rows of the variation lookup with the same bank, so a part
     forced onto the SC-55 row sounds its tone rather than dropping the
     note. Thirteen corpus files send exactly this. */
  assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 32, 1));
  assert(device.engine.parts[0].tone_map == XP_TONE_MAP_SC55);
  assert(EmuSC::Xp::device_midi(&device, 0, 0x90, 62, 100));
  assert(EmuSC::Xp::engine_active_slots(&device.engine) == 1);
  assert(EmuSC::Xp::device_midi(&device, 0, 0x80, 62, 64));
  /* `40 4x 00` writes the byte CC32 writes and `40 4x 01` the part's own
     map, which the forcing byte defers to when it is zero. Block 1 is
     part 1; the last byte of each packet is its checksum. */
  {
    static const uint8_t forced_sc88[] = {
      0x41, 0x10, 0x42, 0x12, 0x40, 0x41, 0x00, 0x02, 0x7d };
    static const uint8_t forced_off[] = {
      0x41, 0x10, 0x42, 0x12, 0x40, 0x41, 0x00, 0x00, 0x7f };
    static const uint8_t selected_sc55[] = {
      0x41, 0x10, 0x42, 0x12, 0x40, 0x41, 0x01, 0x01, 0x7d };
    static const uint8_t selected_bad[] = {
      0x41, 0x10, 0x42, 0x12, 0x40, 0x41, 0x01, 0x00, 0x7e };
    assert(EmuSC::Xp::device_sysex(&device, 0, forced_sc88,
                                    sizeof forced_sc88));
    assert(device.engine.parts[0].tone_map == XP_TONE_MAP_SC88);
    assert(EmuSC::Xp::device_sysex(&device, 0, selected_sc55,
                                    sizeof selected_sc55));
    assert(device.engine.parts[0].tone_map == XP_TONE_MAP_SC88);
    assert(EmuSC::Xp::device_sysex(&device, 0, forced_off,
                                    sizeof forced_off));
    assert(device.engine.parts[0].tone_map == XP_TONE_MAP_SC55);
    /* `46e9` carries the range 01..02 and refuses anything else, so the
       selected map stays where it was. */
    assert(EmuSC::Xp::device_sysex(&device, 0, selected_bad,
                                    sizeof selected_bad));
    assert(device.channels[0].tone_map_selected == XP_TONE_MAP_SC55);
  }
  /* GM System On: bank select and NRPN stop being received, a GS Reset
     brings both back, and the part's selected tone map - SC-55 since the
     block above - survives every one of these resets. */
  {
    static const uint8_t gm_on[] = { 0xf0, 0x7e, 0x7f, 0x09, 0x01, 0xf7 };
    static const uint8_t gm_on_own_id[] = { 0x7e, 0x10, 0x09, 0x01 };
    static const uint8_t gm_off[] = { 0x7e, 0x7f, 0x09, 0x02 };
    EmuSC::Xp::ChannelState *ch = &device.channels[0];
    const uint8_t cutoff_reset = 64;

    /* What a part receives before: CC0, CC32 and an NRPN cutoff. */
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 0, 5));
    assert(ch->variation == 5);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 99, 0x01));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 98, 0x20));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 6, 10));
    assert(ch->cutoff == 10);

    /* `0x0e70` compares the device ID with 7F only. */
    assert(!EmuSC::Xp::device_sysex(&device, 0, gm_on_own_id,
                                     sizeof gm_on_own_id));
    assert(ch->rx_bank_select && ch->variation == 5);

    assert(EmuSC::Xp::device_sysex(&device, 0, gm_on, sizeof gm_on));
    assert(!ch->rx_bank_select && ch->rx_bank_select_lsb && !ch->rx_nrpn);
    assert(!device.channels[31].rx_bank_select && !device.channels[31].rx_nrpn);
    assert(ch->variation == 0 && ch->cutoff == cutoff_reset);
    assert(ch->tone_map_selected == XP_TONE_MAP_SC55);
    assert(device.engine.parts[0].tone_map == XP_TONE_MAP_SC55);

    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 0, 8));
    assert(ch->variation == 0);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 32, 2));
    assert(ch->tone_map_forced == 0);
    assert(device.engine.parts[0].tone_map == XP_TONE_MAP_SC55);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 99, 0x01));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 98, 0x20));
    assert(ch->nrpn_msb == 127 && ch->nrpn_lsb == 127);
    EmuSC::Xp::device_midi(&device, 0, 0xb0, 6, 10);
    assert(ch->cutoff == cutoff_reset);

    /* GS Reset turns both switches back on. */
    assert(dt1(&device, 0x40, 0x00, 0x7f, 0x00));
    assert(ch->rx_bank_select && ch->rx_bank_select_lsb && ch->rx_nrpn);
    assert(ch->tone_map_selected == XP_TONE_MAP_SC55);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 0, 8));
    assert(ch->variation == 8);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 32, 2));
    assert(device.engine.parts[0].tone_map == XP_TONE_MAP_SC88);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 99, 0x01));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 98, 0x20));
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 6, 10));
    assert(ch->cutoff == 10);

    /* The same mode reached without the message, and left by sub-ID 02,
       which `0x17e2` sends to the GS Reset's own module reset. */
    assert(EmuSC::Xp::device_gm_system_on(&device));
    assert(!ch->rx_bank_select && !ch->rx_nrpn);
    assert(EmuSC::Xp::device_sysex(&device, 0, gm_off, sizeof gm_off));
    assert(ch->rx_bank_select && ch->rx_nrpn);

    /* The switches themselves, `40 1x 23`, `24` and `0a`; block 1 is
       part 1. Turning a bank switch off also drops the stored CC0. */
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 0, 8));
    assert(dt1(&device, 0x40, 0x11, 0x24, 0x00));
    assert(ch->rx_bank_select && !ch->rx_bank_select_lsb);
    assert(ch->variation == 0);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 0, 8));
    assert(ch->variation == 8);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 32, 1));
    assert(ch->tone_map_forced == 0);
    assert(dt1(&device, 0x40, 0x11, 0x24, 0x01));
    assert(dt1(&device, 0x40, 0x11, 0x23, 0x00));
    assert(!ch->rx_bank_select && ch->variation == 0);
    assert(dt1(&device, 0x40, 0x11, 0x23, 0x01));
    assert(ch->rx_bank_select);
    assert(EmuSC::Xp::device_midi(&device, 0, 0xb0, 99, 0x01));
    assert(dt1(&device, 0x40, 0x11, 0x0a, 0x00));
    assert(!ch->rx_nrpn && ch->nrpn_msb == 127);
    assert(dt1(&device, 0x40, 0x11, 0x0a, 0x01));
    assert(ch->rx_nrpn);

    /* System Mode Set turns the bank switches back on. */
    assert(EmuSC::Xp::device_gm_system_on(&device));
    assert(dt1(&device, 0x00, 0x00, 0x7f, 0x00));
    assert(ch->rx_bank_select && ch->rx_bank_select_lsb);
    assert(dt1(&device, 0x40, 0x00, 0x7f, 0x00));
  }
  EmuSC::Xp::device_destroy(&device);
  free(chip);
  free(control);

  {
    const char *control_path = getenv("SC88_CONTROL_ROM");
    const char *wave_csv = getenv("SC88_WAVE_ROMS");
    char wave_paths[4][512];
    char *held_paths[5];
    FILE *probe;
    unsigned i;
    if (!control_path || !wave_csv)
      return 77;
    probe = fopen(control_path, "rb");
    if (!probe)
      return 77;
    fclose(probe);
    if (!split_wave_roms(wave_csv, wave_paths))
      return 77;
    for (i = 0; i < 4; ++i) {
      probe = fopen(wave_paths[i], "rb");
      if (!probe)
        return 77;
      fclose(probe);
    }
    held_paths[0] = (char *)control_path;
    for (i = 0; i < 4; ++i)
      held_paths[i + 1] = wave_paths[i];
    test_held_raw(held_paths);
  }
  return 0;
}
