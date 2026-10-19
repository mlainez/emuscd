/* SPDX-License-Identifier: CC0-1.0 */

/* Synth::has_gs_mode(), Synth::has_gm_mode() and Synth::set_mode_lock().
 *
 * On the SC-55mkII, GM mode is GS with Rx.BankSelect and Rx.NRPN off on every
 * part (Settings::set_gm_mode()), which shows as a bank select that no longer
 * reaches the tone. A GM System On enters it; a GS Reset leaves it, and so
 * can a GS DT1 to a part's Rx.BankSelect. Locked in GS, the GM System On may
 * not enter it; locked in GM, neither of the others may leave it. Each locked
 * case is run unlocked first, so the test shows the lock, not the message, is
 * what holds the mode.
 *
 * On the SC-88 GM mode is the same two switches off, held in its own engine,
 * where a parameter read cannot reach them; an NRPN that no longer changes
 * the sound shows Rx.NRPN off. The same messages and locks are run there,
 * with a DT1 to Rx.NRPN in place of the one to Rx.BankSelect.
 *
 * Both queries are checked on every device whose ROMs are given:
 * has_gs_mode() true on the SC-55, SC-55mkII and SC-88; has_gm_mode() true on
 * the SC-55mkII, SC-88 and JV-1080.
 *
 *   SC55MKII_CONTROL_ROM, SC55MKII_CPU_ROM, SC55MKII_WAVE_ROMS
 *   SC55_CONTROL_ROM,     SC55_CPU_ROM,     SC55_WAVE_ROMS
 *   JV880_CONTROL_ROM,                      JV880_WAVE_ROMS
 *   SC88_CONTROL_ROM,                       SC88_WAVE_ROMS
 *   JV1080_CONTROL_ROM,                     JV1080_WAVE_ROMS
 *
 * The wave ROMs are comma separated, in chip order. Without the SC-55mkII's
 * and the SC-88's the test reports skipped.
 */

#include "synth.h"
#include "control_rom.h"
#include "wave_rom.h"

#include <cassert>
#include <cmath>
#ifdef NDEBUG
#error "this test is assertion-driven; NDEBUG compiles it away"
#endif
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

using SoundMap = EmuSC::Synth::SoundMap;
using ModeLock = EmuSC::Synth::ModeLock;
const uint32_t kRate = 32000;

std::vector<std::string> split_commas(const std::string &s)
{
  std::vector<std::string> out;
  std::string::size_type start = 0;
  while (start <= s.size()) {
    std::string::size_type comma = s.find(',', start);
    if (comma == std::string::npos) {
      out.push_back(s.substr(start));
      break;
    }
    out.push_back(s.substr(start, comma - start));
    start = comma + 1;
  }
  return out;
}

struct Roms {
  std::unique_ptr<EmuSC::ControlRom> ctrl;
  std::unique_ptr<EmuSC::WaveRom> wave;
};

/* Loads <PREFIX>_CONTROL_ROM, _CPU_ROM (if set) and _WAVE_ROMS; false when
   they are not given. */
bool load(const std::string &prefix, Roms &r)
{
  const char *control = getenv((prefix + "_CONTROL_ROM").c_str());
  const char *cpu = getenv((prefix + "_CPU_ROM").c_str());
  const char *wave = getenv((prefix + "_WAVE_ROMS").c_str());
  if (!control || !wave)
    return false;
  try {
    r.ctrl.reset(new EmuSC::ControlRom(control, cpu ? cpu : ""));
    r.wave.reset(new EmuSC::WaveRom(split_commas(wave), *r.ctrl));
  } catch (const std::string &e) {
    std::cerr << prefix << " ROM load failed: " << e << std::endl;
    return false;
  }
  return true;
}

/* Events are queued and applied as frames are rendered. */
void run(EmuSC::Synth &synth, unsigned frames = kRate / 10)
{
  float l, r;
  for (unsigned i = 0; i < frames; i++)
    synth.get_next_frame(l, r);
}

void sysex(EmuSC::Synth &synth, std::vector<uint8_t> msg)
{
  synth.midi_input_sysex(msg.data(), (uint16_t) msg.size());
  run(synth);
}

/* A GS (model 0x42) DT1 to address a0 a1 a2, checksum appended. */
std::vector<uint8_t> gs_dt1(uint8_t a0, uint8_t a1, uint8_t a2,
                            const std::vector<uint8_t> &data)
{
  std::vector<uint8_t> m = { 0xf0, 0x41, 0x10, 0x42, 0x12, a0, a1, a2 };
  int sum = a0 + a1 + a2;
  for (uint8_t d : data) {
    m.push_back(d);
    sum += d;
  }
  m.push_back((uint8_t) ((128 - (sum & 0x7f)) & 0x7f));
  m.push_back(0xf7);
  return m;
}

const std::vector<uint8_t> kGsReset =
  { 0xf0, 0x41, 0x10, 0x42, 0x12, 0x40, 0x00, 0x7f, 0x00, 0x41, 0xf7 };
const std::vector<uint8_t> kGmOn = { 0xf0, 0x7e, 0x7f, 0x09, 0x01, 0xf7 };
/* Rx.BankSelect on, part 1 (40 11 23). */
const std::vector<uint8_t> kRxBankSelectOn = gs_dt1(0x40, 0x11, 0x23, { 0x01 });

/* CC0 8, PC 0 on channel 1, then the bank the part ended up on. */
uint8_t bank_after_select(EmuSC::Synth &synth)
{
  synth.midi_input(0xb0, 0, 8);
  synth.midi_input(0xc0, 0, 0);
  run(synth);
  return synth.get_param(EmuSC::PatchParam::ToneNumber, 0);
}

bool rx_on(EmuSC::Synth &s, EmuSC::PatchParam p)
{
  for (int part = 0; part < 16; part++)
    if (s.get_param(p, part) == 0)
      return false;
  return true;
}

bool rx_off(EmuSC::Synth &s, EmuSC::PatchParam p)
{
  for (int part = 0; part < 16; part++)
    if (s.get_param(p, part) != 0)
      return false;
  return true;
}

bool in_gm(EmuSC::Synth &s)
{
  return rx_off(s, EmuSC::PatchParam::RxBankSelect) &&
         rx_off(s, EmuSC::PatchParam::RxNRPN);
}

bool in_gs(EmuSC::Synth &s)
{
  return rx_on(s, EmuSC::PatchParam::RxBankSelect) &&
         rx_on(s, EmuSC::PatchParam::RxNRPN);
}

std::unique_ptr<EmuSC::Synth> power_on(Roms &r, SoundMap map, ModeLock lock)
{
  std::unique_ptr<EmuSC::Synth> s(new EmuSC::Synth(*r.ctrl, *r.wave));
  s->set_audio_format(kRate, 2);
  s->reset(map, true);
  s->set_mode_lock(lock);
  run(*s);
  return s;
}

void test_sc55mkii(Roms &r)
{
  {
    std::unique_ptr<EmuSC::Synth> s = power_on(r, SoundMap::GS, ModeLock::None);
    assert(s->has_gs_mode() && s->has_gm_mode());
    assert(s->mode_lock() == ModeLock::None);
    assert(in_gs(*s));
    assert(bank_after_select(*s) == 8);

    /* Unlocked: GM System On enters GM, a GS Reset or a DT1 leaves it. */
    sysex(*s, kGmOn);
    assert(in_gm(*s));
    assert(bank_after_select(*s) == 0);

    sysex(*s, kGsReset);
    assert(in_gs(*s));
    assert(bank_after_select(*s) == 8);

    sysex(*s, kGmOn);
    assert(in_gm(*s));
    sysex(*s, kRxBankSelectOn);
    assert(s->get_param(EmuSC::PatchParam::RxBankSelect, 0) == 1);
    assert(bank_after_select(*s) == 8);
  }

  /* Locked in GS: GM System On resets, but into GS. */
  {
    std::unique_ptr<EmuSC::Synth> s = power_on(r, SoundMap::GS, ModeLock::GS);
    assert(s->mode_lock() == ModeLock::GS);

    const uint8_t level = s->get_param(EmuSC::PatchParam::PartLevel, 0);
    sysex(*s, gs_dt1(0x40, 0x11, 0x19, { (uint8_t) (level ^ 0x40) }));
    assert(s->get_param(EmuSC::PatchParam::PartLevel, 0) == (level ^ 0x40));
    sysex(*s, kGmOn);
    assert(s->get_param(EmuSC::PatchParam::PartLevel, 0) == level);
    assert(in_gs(*s));
    assert(bank_after_select(*s) == 8);

    /* Unlocking hands the mode back to the stream. */
    s->set_mode_lock(ModeLock::None);
    sysex(*s, kGmOn);
    assert(in_gm(*s));
  }

  /* Locked in GM: neither the GS Reset nor the DT1 leaves it. */
  {
    std::unique_ptr<EmuSC::Synth> s = power_on(r, SoundMap::GS_GM, ModeLock::GM);
    assert(s->mode_lock() == ModeLock::GM);
    assert(in_gm(*s));

    sysex(*s, kGsReset);
    assert(in_gm(*s));
    assert(bank_after_select(*s) == 0);

    sysex(*s, kRxBankSelectOn);
    assert(in_gm(*s));
    assert(bank_after_select(*s) == 0);

    /* The GS Reset still resets: a part level written before it is back at
       its default after it. */
    const uint8_t level = s->get_param(EmuSC::PatchParam::PartLevel, 0);
    sysex(*s, gs_dt1(0x40, 0x11, 0x19, { (uint8_t) (level ^ 0x40) }));
    assert(s->get_param(EmuSC::PatchParam::PartLevel, 0) == (level ^ 0x40));
    sysex(*s, kGsReset);
    assert(s->get_param(EmuSC::PatchParam::PartLevel, 0) == level);
    assert(in_gm(*s));

    s->set_mode_lock(ModeLock::None);
    sysex(*s, kGsReset);
    assert(in_gs(*s));
    assert(bank_after_select(*s) == 8);
  }
}

/* Energy of a note on channel 1 over its first 8192 frames, released and
   rendered out. */
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
  run(synth);
}

/* Whether the SC-88 receives NRPN on part 1: the slowest attack leaves the
   note's opening well below what it is at the default. The attack is put
   back afterwards. */
bool sc88_rx_nrpn(EmuSC::Synth &synth)
{
  const double normal = note_energy(synth);
  assert(normal > 0.0);
  nrpn_attack(synth, 127);
  const double slow = note_energy(synth);
  nrpn_attack(synth, 64);
  return slow < 0.5 * normal;
}

void test_sc88(Roms &r)
{
  {
    std::unique_ptr<EmuSC::Synth> s = power_on(r, SoundMap::GS, ModeLock::None);
    assert(s->has_gs_mode() && s->has_gm_mode());
    assert(sc88_rx_nrpn(*s));

    /* Unlocked: GM System On enters GM, a GS Reset leaves it. */
    sysex(*s, kGmOn);
    assert(!sc88_rx_nrpn(*s));
    sysex(*s, kGsReset);
    assert(sc88_rx_nrpn(*s));
  }

  /* A GM reset from the host enters it, which is what --reset gm does. */
  {
    std::unique_ptr<EmuSC::Synth> s = power_on(r, SoundMap::GS_GM, ModeLock::None);
    assert(!sc88_rx_nrpn(*s));
  }

  /* Locked in GS: GM System On resets, but into GS. */
  {
    std::unique_ptr<EmuSC::Synth> s = power_on(r, SoundMap::GS, ModeLock::GS);
    sysex(*s, kGmOn);
    assert(sc88_rx_nrpn(*s));

    s->set_mode_lock(ModeLock::None);
    sysex(*s, kGmOn);
    assert(!sc88_rx_nrpn(*s));
  }

  /* Locked in GM: neither a GS Reset nor a DT1 to Rx.NRPN (40 11 0A) turns
     it back on. */
  {
    std::unique_ptr<EmuSC::Synth> s = power_on(r, SoundMap::GS_GM, ModeLock::GM);
    assert(!sc88_rx_nrpn(*s));
    sysex(*s, kGsReset);
    assert(!sc88_rx_nrpn(*s));
    sysex(*s, gs_dt1(0x40, 0x11, 0x0a, { 0x01 }));
    assert(!sc88_rx_nrpn(*s));

    s->set_mode_lock(ModeLock::None);
    sysex(*s, kGsReset);
    assert(sc88_rx_nrpn(*s));
  }
}

void check_modes(const char *prefix, bool gs, bool gm, int &checked)
{
  Roms r;
  if (!load(prefix, r))
    return;
  EmuSC::Synth s(*r.ctrl, *r.wave);
  s.set_audio_format(kRate, 2);
  s.reset(SoundMap::GS, true);
  if (s.has_gs_mode() != gs || s.has_gm_mode() != gm) {
    std::cerr << prefix << ": has_gs_mode() " << s.has_gs_mode()
              << ", has_gm_mode() " << s.has_gm_mode() << "; expected "
              << gs << ", " << gm << std::endl;
    assert(false);
  }
  checked++;
}

}  // namespace

int main(void)
{
  Roms mk2, sc88;
  const bool haveMk2 = load("SC55MKII", mk2);
  const bool haveSc88 = load("SC88", sc88);
  if (!haveMk2 && !haveSc88)
    return 77;
  if (haveMk2)
    test_sc55mkii(mk2);
  if (haveSc88)
    test_sc88(sc88);

  int checked = 0;
  check_modes("SC55", true, false, checked);
  check_modes("JV880", false, false, checked);
  check_modes("SC88", true, true, checked);
  check_modes("JV1080", false, true, checked);
  std::cout << "has_gs_mode(), has_gm_mode() checked on " << checked
            << " more device(s)" << std::endl;
  return 0;
}
