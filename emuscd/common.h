// Pieces of the Roland Sound Canvas daemon shared between its ALSA backend
// (main.cc, Linux) and its WinMM backend (main_winmidi.cc, Windows): ROM
// filename resolution, which touches neither platform's audio/MIDI API.

#pragma once

#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>

#include "../libemusc/src/synth.h"
#include "../libemusc/tools/rom_paths.h"
#include "../libemusc/tools/gain.h"

namespace emuscd {

using emusc_tools::SUPPORTED_DEVICES;
using emusc_tools::default_rom_dir;
using emusc_tools::gain_db_to_linear;
using emusc_tools::apply_gain;

// --reset: the mode every device this daemon loads is powered on in and
// locked to, for as long as it runs, across device switches. None, without
// --reset, powers it on in GS and leaves the mode to the stream.
enum class ResetMode { None, GS, GM };

inline bool parse_reset_mode(const std::string &s, ResetMode &out) {
  if (s == "gs") { out = ResetMode::GS; return true; }
  if (s == "gm") { out = ResetMode::GM; return true; }
  return false;
}

// What power_on() did: which mode the device is locked in, if any, and
// whether that is the one --reset asked for.
struct PowerOnState {
  ResetMode asked = ResetMode::None;
  ResetMode locked = ResetMode::None;
};

// The power-on reset, which is what the hardware does when it is switched on.
// It must follow set_audio_format(), which is where the parts are
// instantiated and the SC-88 and JV-1080 engines are opened: before that the
// reset has nothing to act on, and has_gs_mode()/has_gm_mode() nothing to ask.
//
// A device without the mode asked for is locked in the other one instead, and
// a device with neither runs unlocked in the state a GS reset leaves it in -
// its own default.
inline PowerOnState power_on(EmuSC::Synth &synth, ResetMode mode) {
  using SoundMap = EmuSC::Synth::SoundMap;
  using ModeLock = EmuSC::Synth::ModeLock;
  PowerOnState st;
  st.asked = mode;
  if (mode != ResetMode::None) {
    const bool gs = synth.has_gs_mode(), gm = synth.has_gm_mode();
    if (mode == ResetMode::GM)
      st.locked = gm ? ResetMode::GM : gs ? ResetMode::GS : ResetMode::None;
    else
      st.locked = gs ? ResetMode::GS : gm ? ResetMode::GM : ResetMode::None;
  }
  synth.reset(st.locked == ResetMode::GM ? SoundMap::GS_GM : SoundMap::GS, true);
  synth.set_mode_lock(st.locked == ResetMode::GM ? ModeLock::GM
                      : st.locked == ResetMode::GS ? ModeLock::GS
                      : ModeLock::None);
  return st;
}

// The notice a daemon prints after power_on(), empty when --reset was not
// given. `prog` names the daemon, `dev` the device.
inline std::string power_on_message(const PowerOnState &st,
                                    const std::string &prog,
                                    const std::string &dev) {
  auto name = [](ResetMode m) { return m == ResetMode::GM ? "GM" : "GS"; };
  if (st.asked == ResetMode::None)
    return "";
  if (st.locked == ResetMode::None)
    return prog + ": neither GS nor GM mode is available on " + dev +
           ", so it runs in its own default mode";
  const std::string stays = " stays in " + std::string(name(st.locked)) +
                            " mode while " + prog +
                            " runs, whatever SysEx it is sent";
  if (st.locked == st.asked)
    return prog + ": " + dev + stays;
  return prog + ": no " + name(st.asked) + " mode available on " + dev +
         ", so " + name(st.locked) + " was selected as the default; it" + stays;
}

inline bool device_supported(const std::string &dev) {
  for (const char *d : SUPPORTED_DEVICES)
    if (dev == d) return true;
  return false;
}

struct DeviceRoms {
  std::string control_rom, cpu_rom;
  std::vector<std::string> wave_roms;
};

// One naming convention for every device, shared with emusc-render's
// --device preset (libemusc/tools/main.cc): <device>_control.bin is always
// the control/program ROM, <device>_cpu.bin is the internal CPU ROM that
// only SC-55 and SC-55mkII have. See README.md for exact file hashes.
inline DeviceRoms resolve_device_roms(const std::string &device, const std::string &rom_dir) {
  DeviceRoms r;
  r.control_rom = rom_dir + "/" + device + "_control.bin";
  if (device == "sc55" || device == "sc55mkii")
    r.cpu_rom = rom_dir + "/" + device + "_cpu.bin";

  // Wave chip counts: SC-55 3, the two XP devices 4, the rest 2.
  int n = (device == "sc55") ? 3
          : (device == "sc88" || device == "jv1080") ? 4 : 2;
  for (int k = 1; k <= n; k++)
    r.wave_roms.push_back(rom_dir + "/" + device + "_waverom" + std::to_string(k) + ".bin");
  return r;
}

inline int16_t to_i16(float x) {
  if (x > 1.0f) x = 1.0f;
  if (x < -1.0f) x = -1.0f;
  long v = std::lrintf(x * 32767.0f);
  if (v > 32767) v = 32767;
  if (v < -32767) v = -32767;
  return static_cast<int16_t>(v);
}

}  // namespace emuscd
