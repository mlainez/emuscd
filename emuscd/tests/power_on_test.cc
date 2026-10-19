/* SPDX-License-Identifier: CC0-1.0 */

/* power_on(), what emuscd and emusc-winmidi do with --reset, on the SC-88:
 * --reset gm locks it in GM and enters GM, --reset gs locks it in GS. In GM
 * mode Rx.NRPN is off, so an NRPN no longer changes the sound.
 *
 *   SC88_CONTROL_ROM  the control ROM
 *   SC88_WAVE_ROMS    the four wave chips, comma separated, in chip order
 *
 * Without them the test reports skipped.
 */
#include "../common.h"
#include "../../libemusc/src/control_rom.h"
#include "../../libemusc/src/wave_rom.h"

#include <cassert>
#ifdef NDEBUG
#error "this test is assertion-driven; NDEBUG compiles it away"
#endif
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace emuscd;

namespace {

const uint32_t kRate = 32000;

void run(EmuSC::Synth &synth, unsigned frames)
{
  float l, r;
  for (unsigned i = 0; i < frames; i++)
    synth.get_next_frame(l, r);
}

/* Energy of a note on channel 1 over its first 8192 frames. */
double note_energy(EmuSC::Synth &synth)
{
  float l, r;
  double e = 0.0;
  synth.midi_input(0x90, 60, 100);
  for (unsigned i = 0; i < 8192; i++) {
    synth.get_next_frame(l, r);
    e += std::fabs(l) + std::fabs(r);
  }
  synth.midi_input(0x80, 60, 0);
  run(synth, kRate);
  return e;
}

/* NRPN 01 63, the part's attack time, on channel 1. */
void nrpn_attack(EmuSC::Synth &synth, uint8_t value)
{
  synth.midi_input(0xb0, 99, 0x01);
  synth.midi_input(0xb0, 98, 0x63);
  synth.midi_input(0xb0, 6, value);
  synth.midi_input(0xb0, 101, 127);
  synth.midi_input(0xb0, 100, 127);
  run(synth, kRate / 10);
}

/* Whether part 1 receives NRPN: the slowest attack leaves the note's opening
   well below what it is at the default. */
bool rx_nrpn(EmuSC::Synth &synth)
{
  const double normal = note_energy(synth);
  assert(normal > 0.0);
  nrpn_attack(synth, 127);
  const double slow = note_energy(synth);
  nrpn_attack(synth, 64);
  return slow < 0.5 * normal;
}

std::vector<std::string> split_commas(const std::string &s)
{
  std::vector<std::string> out;
  std::string::size_type start = 0;
  for (;;) {
    std::string::size_type comma = s.find(',', start);
    out.push_back(s.substr(start, comma == std::string::npos
                                  ? std::string::npos : comma - start));
    if (comma == std::string::npos)
      return out;
    start = comma + 1;
  }
}

}  // namespace

int main(void)
{
  const char *control = getenv("SC88_CONTROL_ROM");
  const char *wave = getenv("SC88_WAVE_ROMS");
  if (!control || !wave)
    return 77;
  EmuSC::ControlRom ctrl(control, "");
  EmuSC::WaveRom waves(split_commas(wave), ctrl);

  {
    EmuSC::Synth synth(ctrl, waves);
    synth.set_audio_format(kRate, 2);
    const PowerOnState st = power_on(synth, ResetMode::GM);
    assert(st.locked == ResetMode::GM);
    assert(synth.mode_lock() == EmuSC::Synth::ModeLock::GM);
    assert(power_on_message(st, "emuscd", "sc88") ==
           "emuscd: sc88 stays in GM mode while emuscd runs, whatever SysEx "
           "it is sent");
    run(synth, kRate / 10);
    assert(!rx_nrpn(synth));
  }

  {
    EmuSC::Synth synth(ctrl, waves);
    synth.set_audio_format(kRate, 2);
    const PowerOnState st = power_on(synth, ResetMode::GS);
    assert(st.locked == ResetMode::GS);
    assert(synth.mode_lock() == EmuSC::Synth::ModeLock::GS);
    run(synth, kRate / 10);
    assert(rx_nrpn(synth));
  }

  std::cout << "--reset gm and --reset gs checked on the SC-88" << std::endl;
  return 0;
}
