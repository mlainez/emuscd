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
 *  You should have received a copy of the GNU General Public License
 *  along with libEmuSC. If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef __SETTINGS_H__
#define __SETTINGS_H__


#include "control_rom.h"
#include "params.h"

#include <cstdint>

#include <algorithm>
#include <array>
#include <atomic>
#include <functional>
#include <string>


namespace EmuSC {

class Settings
{
public:
  Settings(ControlRom & ctrlRom);
  ~Settings();

  // Sound Canvas modes
  enum class Mode {
    GS,                  // Standard GS mode. Default.
    MT32,                // MT32 mode
    SC55                 // SC-55 mode for SC-88     TODO

    // TODO: What about GM mode?
  };

  enum class Controller {
    Modulation      = 0,
    PitchBend       = 1,
    ChannelPressure = 2,
    PolyKeyPressure = 3,
    CC1             = 4,
    CC2             = 5
  };

  enum class ControllerParam {
    Pitch          =  0,
    TVFCutoff      =  1,
    Amplitude      =  2,
    LFO1Rate       =  3,
    LFO1PitchDepth =  4,
    LFO1TVFDepth   =  5,
    LFO1TVADepth   =  6,
    LFO2Rate       =  7,
    LFO2PitchDepth =  8,
    LFO2TVFDepth   =  9,
    LFO2TVADepth   = 10
  };

  // Retrieve settings from Config paramters
  uint8_t  get_param(enum SystemParam sp);
  uint32_t get_param_uint32(enum SystemParam sp);
  uint16_t get_param_32nib(enum SystemParam sp);
  uint8_t  get_param(enum PatchParam pp, int8_t part = -1);
  uint16_t get_param_uint14(enum PatchParam pp, int8_t part = -1);
  uint16_t get_param_uint16(enum PatchParam pp, int8_t part = -1);
  uint8_t  get_param_nib16(enum PatchParam pp, int8_t part = -1);
  uint8_t  get_patch_param(uint16_t address, int8_t part = -1);

  // Voice reserve of a part, in partials. The sixteen reserve bytes are one
  // block addressed in Roland's part order, not in MIDI channel order
  // (SC-55 OM p.79 "PARTIAL RESERVE", SC-55mkII OM p.98 "VOICE RESERVE").
  uint8_t  get_partial_reserve(int8_t part);
  uint8_t  get_param(enum DrumParam, uint8_t map, uint8_t key);

  std::string get_drum_map_name(uint8_t map);

  // Set settings from Config paramters
  void set_param(enum SystemParam sp, uint8_t value);
  void set_param(enum SystemParam sp, uint8_t *data, uint8_t size);
  void set_param_uint32(enum SystemParam sp, uint32_t value);
  void set_param_32nib(enum SystemParam sp, uint16_t value);
  void set_system_param(uint16_t address, uint8_t *data, uint8_t size = 1);

  void set_param(enum PatchParam pp, uint8_t value, int8_t part = -1);
  void set_param(enum PatchParam pp, uint8_t *data, uint8_t size = 1,
		       int8_t part = -1);
  void set_param_uint14(enum PatchParam pp, uint16_t value, int8_t part = -1);
  void set_param_nib16(enum PatchParam pp, uint8_t value, int8_t part = -1);
  void set_patch_param(uint16_t address, uint8_t *data, uint8_t size = 1);
  void set_patch_param(uint16_t address, uint8_t value, int8_t part = -1);
  void set_param(enum DrumParam dp, uint8_t map, uint8_t key, uint8_t value);
  void set_param(enum DrumParam dp, uint8_t map, uint8_t *data, uint8_t length);
  void set_drum_param(uint16_t address, uint8_t *data, uint8_t size = 1);

  int update_drum_set(uint8_t map, uint8_t bank);

  // Store settings paramters to file (aka battery backup)
  bool load(std::string filePath);
  bool save(std::string filePath);

  // Reset all settings to default GS mode
  void reset();
  void set_gm_mode(void);
  // Whether set_gm_mode() leaves anything different from a GS reset.
  bool has_gm_mode(void) const;
  void set_map_mt32(void);

  // Temporary solution
  // Figure out the need for a common way with all controllers
  void update_pitchBend_factor(int8_t part);
  float get_pitchBend_factor(int8_t part) { return _PBController[part]; }

  // The same factor for a voice whose bend range is its own rather than the
  // part's - the JV's rhythm notes - from the part's current 14-bit bend.
  float get_pitchBend_factor(int8_t part, int semitones);

  static int8_t convert_from_roland_part_id(int8_t part);

  void set_sample_rate(int sampleRate) { _sampleRate = sampleRate; }
  inline int sample_rate(void) { return _sampleRate; }

  void set_channels(int channels) { _channels = channels; }
  inline int channels(void) { return _channels; }

  // Which generation of the hardware the loaded control ROM is; a few
  // parameter responses differ between them
  inline enum ControlRom::SynthGen generation(void)
  { return _ctrlRom.generation(); }

  // The loaded device's profile, never null. Everything that differs between
  // devices is read from here rather than branched on by generation.
  inline const DeviceProfile *device(void)
  { return _ctrlRom.device(); }

  // What Expression resets to on this device: 127 where it is a part volume,
  // 0 where it is a modulation source that rests at nothing (scdb D-79).
  inline int expression_reset(void)
  { const DeviceProfile *d = _ctrlRom.device();
    return (d && d->ctrlJv.enabled) ? d->ctrlJv.expressionRest : 0x7f; }

  int get_acc_control_param(enum ControllerParam cp, int part)
  { part = std::clamp(part, 0, 15);
    return _accControlParams[part][static_cast<int>(cp)].load(std::memory_order_relaxed); }

  void set_part_callback(std::function<void(const int)> cb);
  void clear_part_callback(void);

  // The engine's control-period clock, in samples of the internal 32 kHz rate,
  // advanced once per period by Synth. A voice that starts needs to know what
  // TIME it starts at, because the JV's LFO phase accumulators run whether a
  // note is sounding or not (scdb devices/jv880 D-26): a tone with Synchro OFF
  // inherits the phase its slot has reached, so two identical notes are not
  // identical. Nothing else reads it.
  inline void set_block_start(uint64_t s) { _blockStart = s; }
  inline uint64_t block_start(void) const { return _blockStart; }

private:
  uint64_t _blockStart = 0;

  // Both SysEx and non-SysEx data
  std::array<std::atomic<uint8_t>, 0x0100> _systemParams{};
  std::array<std::atomic<uint8_t>, 0x4000> _patchParams{};
  std::array<std::atomic<uint8_t>, 0x2000> _drumParams{};

  // Controller paramter values generated when controllers change - all parts
  std::array<std::array<std::array<int, 6>, 11>, 16> _controlParams{{{}}};
  // Accumulated controller parameter values per value category - all parts
  std::array<std::array<std::atomic<int16_t>, 11>, 16> _accControlParams{{}};

  ControlRom &_ctrlRom;

  // Non-native parameters
  int _sampleRate;
  int _channels;                        // 1 => mono or 2 => stereo

  std::function<void(const int)> _partCallback = NULL;

  void _initialize_system_params(enum Mode = Mode::GS);
  void _initialize_patch_params(enum Mode = Mode::GS);
  void _initialize_drumSet_params();
  void _apply_device_performance();
  bool _chorus_macro_sets_parameters(void);
  int  _copy_drum_set(uint8_t map, int index);

  // Counts device performance loads, so an effect can tell a load from a
  // parameter that happens to hold the same value afterwards.
  unsigned _devicePerformanceLoads = 0;
public:
  unsigned device_performance_loads(void) const { return _devicePerformanceLoads; }

  // A control-channel program change selected a Performance (D-46): reload it
  // and re-apply every part. False when the selection was rejected.
  bool select_performance(int selector);

  // Select a rhythm set by memory bank, for a JV rhythm program change.
  int  update_drum_set_bank(uint8_t map, int selector);

  // A JV rhythm-note DT1 (00 07 4n pp): edit the temporary rhythm setup, which
  // is the set the rhythm part currently holds, and re-read the drum
  // parameters of the key it touched. True when a key changed.
  bool rhythm_note_dt1(int note, uint8_t param, uint8_t value);
private:

  // Which rhythm set each of the two drum maps holds, or -1 for none. The JV
  // needs it because a DT1 edits the LOADED set and nothing else records which
  // that is.
  int _drumSetIndex[2] = { -1, -1 };
  void _copy_drum_key(uint8_t map, int index, int key);

  // BE / LE conversion
  inline bool _le_native(void) { uint16_t n = 1; return (*(uint8_t *) & n); }
  uint8_t  _to_native_endian_nib16(const std::atomic<uint8_t> *ptr);
  uint16_t _to_native_endian_uint14(const std::atomic<uint8_t> *ptr);
  uint16_t _to_native_endian_uint16(const std::atomic<uint8_t> *ptr);
  uint32_t _to_native_endian_uint32(const std::atomic<uint8_t> *ptr);

  // Macros for certain settings
  void _run_macro_chorus(uint8_t value);
  void _run_macro_reverb(uint8_t value);

  void _update_controller_input(enum PatchParam pp, int value, int part);
  void _update_controller_input_acc(int8_t part);
  int16_t _calc_controller_value(ControllerParam cp, int part, int max,int mul);

  // Temporary storage for pitchbend
  float _PBController[16];

  static constexpr std::array<uint8_t, 16> _convert_to_roland_part_id_LUT =
    { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 10, 11, 12, 13, 14, 15 };
  static constexpr std::array<uint8_t, 16> _convert_from_roland_part_id_LUT =
    { 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15 };

};

}

#endif  // __SETTINGS_H__
