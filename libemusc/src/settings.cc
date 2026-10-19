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


#include "settings.h"
#include "config.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <vector>
#include <algorithm>
#include <iterator>


namespace EmuSC {

// Make Clang compiler happy
constexpr std::array<uint8_t, 16> Settings::_convert_to_roland_part_id_LUT;
constexpr std::array<uint8_t, 16> Settings::_convert_from_roland_part_id_LUT;


Settings::Settings(ControlRom &ctrlRom)
  : _ctrlRom(ctrlRom),
    _sampleRate(44100),
    _channels(2)
{
  // TODO: Add SC-55/88 to master settings
  _initialize_system_params();
  _initialize_patch_params();
  _initialize_drumSet_params();
  _apply_device_performance();

  // TODO: Find a proper way to handle calculated controller values
  for (int i = 0; i < 16; i ++)
    _PBController[i] = 1;
}


Settings::~Settings()
{}


uint8_t Settings::get_param(enum SystemParam sp)
{
  return (uint8_t) _systemParams[(int) sp];
}


uint32_t Settings::get_param_uint32(enum SystemParam sp)
{
  return _to_native_endian_uint32(&_systemParams[(int) sp]);
}


uint16_t Settings::get_param_32nib(enum SystemParam sp)
{
  const std::atomic<uint8_t> *vPtr = &_systemParams[(int) sp];

  const uint8_t v0 = vPtr[0].load(std::memory_order_relaxed);
  const uint8_t v1 = vPtr[1].load(std::memory_order_relaxed);
  const uint8_t v2 = vPtr[2].load(std::memory_order_relaxed);
  const uint8_t v3 = vPtr[3].load(std::memory_order_relaxed);

  if (_le_native())
    return (uint16_t) v3 | v2 << 4 | v1 << 8 | v0 << 12;

  return (uint16_t) v0 | v1 << 4 | v2 << 8 | v3 << 12;
}


uint8_t Settings::get_param(enum PatchParam pp, int8_t part)
{
  if (part < 0 || part > 15)
    return _patchParams[(int) pp].load(std::memory_order_relaxed);

  int8_t rolandPart = _convert_to_roland_part_id_LUT[part];

  return _patchParams[(((int) pp) |
                       (rolandPart << 8))].load(std::memory_order_relaxed);
}


uint8_t Settings::get_partial_reserve(int8_t part)
{
  if (part < 0 || part > 15)
    return 0;

  return _patchParams[(int) PatchParam::PartialReserve +
                      _convert_to_roland_part_id_LUT[part]]
    .load(std::memory_order_relaxed);
}


uint16_t Settings::get_param_uint14(enum PatchParam pp, int8_t part)
{
  int address = (int) pp;

  if (part < 0 || part > 15)
    return  _to_native_endian_uint14(&_patchParams[address]);

  int8_t rolandPart = _convert_to_roland_part_id_LUT[part];

  return _to_native_endian_uint14(&_patchParams[(address | (rolandPart << 8))]);
}


uint16_t Settings::get_param_uint16(enum PatchParam pp, int8_t part)
{
  int address = (int) pp;

  if (part < 0 || part > 15)
    return  _to_native_endian_uint16(&_patchParams[address]);

  int8_t rolandPart = _convert_to_roland_part_id_LUT[part];

  return _to_native_endian_uint16(&_patchParams[(address | (rolandPart << 8))]);
}


uint8_t Settings::get_param_nib16(enum PatchParam pp, int8_t part)
{
  int8_t rolandPart;
  int address = (int) pp;

  if (part < 0 || part > 15)
    rolandPart = 0;
  else
    rolandPart = _convert_to_roland_part_id_LUT[part];

  return _to_native_endian_nib16(&_patchParams[(address | (rolandPart << 8))]);
}


uint8_t Settings::get_patch_param(uint16_t address, int8_t part)
{
  if (part < 0 || part > 15)
    return (uint8_t) _patchParams[address];

  int8_t rolandPart = _convert_to_roland_part_id_LUT[part];

  return (uint8_t) _patchParams[(address | (rolandPart << 8))];
}


uint8_t Settings::get_param(enum DrumParam dp, uint8_t map, uint8_t key)
{
  return (uint8_t) _drumParams[(int) dp | (map << 12) | key];
}


std::string Settings::get_drum_map_name(uint8_t map)
{
  std::string name;
  const int base = (int) DrumParam::DrumsMapName | (map << 12);

  for (int i = 0; i < 12; i++)
    name += (char) _drumParams[base + i].load(std::memory_order_relaxed);

  return name;
}


void Settings::set_param(enum SystemParam sp, uint8_t value)
{
  _systemParams[(int) sp] = value;
}


void Settings::set_param(enum SystemParam sp, uint8_t *value, uint8_t size)
{
  for (int i = 0; i < size; i++)
    _systemParams[(int) sp + i] = value[i];
}


void Settings::set_param_uint32(enum SystemParam sp, uint32_t value)
{
  if (_le_native()) {
    _systemParams[(int) sp + 0] = (value >> 24) & 0xff;
    _systemParams[(int) sp + 1] = (value >> 16) & 0xff;
    _systemParams[(int) sp + 2] = (value >>  8) & 0xff;
    _systemParams[(int) sp + 3] = (value >>  0) & 0xff;

  } else {
    _systemParams[(int) sp + 0] = (value >>  0) & 0xff;
    _systemParams[(int) sp + 1] = (value >>  8) & 0xff;
    _systemParams[(int) sp + 2] = (value >> 16) & 0xff;
    _systemParams[(int) sp + 3] = (value >> 24) & 0xff;
  }
}


void Settings::set_param_32nib(enum SystemParam sp, uint16_t value)
{
  if (_le_native()) {
    _systemParams[(int) sp + 0] = ((value >> 8) & 0xf0) >> 4;
    _systemParams[(int) sp + 1] = ((value >> 8) & 0x0f) >> 0;
    _systemParams[(int) sp + 2] = ((value >> 0) & 0xf0) >> 4;
    _systemParams[(int) sp + 3] = ((value >> 0) & 0x0f) >> 0;
    
  } else {
    _systemParams[(int) sp + 3] = ((value >> 8) & 0xf0) >> 4;
    _systemParams[(int) sp + 2] = ((value >> 8) & 0x0f) >> 0;
    _systemParams[(int) sp + 1] = ((value >> 0) & 0xf0) >> 4;
    _systemParams[(int) sp + 0] = ((value >> 0) & 0x0f) >> 0;
  }
}


void Settings::set_system_param(uint16_t address, uint8_t *value, uint8_t size)
{
  for (int i = 0; i < size; i++)
    _systemParams[address + i] = value[i];
}


void Settings::set_param(enum PatchParam pp, uint8_t value, int8_t part)
{
  if (part < 0 || part > 15) {
    _patchParams[(int) pp] = value;
  } else {
    int8_t rolandPart = _convert_to_roland_part_id_LUT[part];
    _patchParams[(((int) pp) | (rolandPart << 8))] = value;
  }

  if (pp == EmuSC::PatchParam::ChorusMacro) {
    if (_chorus_macro_sets_parameters())
      _run_macro_chorus(value);
  } else if (pp == EmuSC::PatchParam::ReverbMacro) {
    _run_macro_reverb(value);

  // Changes to one of the 6 defined controller groups triggers an update
  // across all controller paramters for that part
  } else if ((int) pp >= 0x1080 && (int) pp <= 0x1086) {
    _update_controller_input(pp, value, part);
  }

  // Send part updates for frontends
  // To avoid double update on instrument updates we can ignore ToneNumber2
  if (_partCallback && pp != PatchParam::ToneNumber2)
    _partCallback(part);
}


void Settings::set_param(enum PatchParam pp, uint8_t *data, uint8_t size,
			 int8_t part)
{
  int8_t rolandPart = _convert_to_roland_part_id_LUT[part];

  for (int i = 0; i < size; i++) {
    if (part < 0 || part > 15) {
      _patchParams[(int) pp + i] = data[i];
    } else {
      int8_t rolandPart = _convert_to_roland_part_id_LUT[part];
      _patchParams[(((int) pp) | (rolandPart << 8)) + i] = data[i];   
    }
  }
}


void Settings::set_param_uint14(enum PatchParam pp, uint16_t value, int8_t part)
{
  int8_t rolandPart = 0;

  if (part >= 0 && part <= 15)
    rolandPart = _convert_to_roland_part_id_LUT[part];

  if (_le_native()) {
    _patchParams[(int) pp | (rolandPart << 8) + 0] = (value >> 7) & 0x7f;
    _patchParams[(int) pp | (rolandPart << 8) + 1] = (value >> 0) & 0x7f;

  } else {
    _patchParams[(int) pp | (rolandPart << 8) + 0] = (value >> 0) & 0x7f;
    _patchParams[(int) pp | (rolandPart << 8) + 1] = (value >> 7) & 0x7f;
  }
}


void Settings::set_param_nib16(enum PatchParam pp, uint8_t value, int8_t part)
{
  int8_t rolandPart = 0;

  if (part >= 0 && part <= 15)
    rolandPart = _convert_to_roland_part_id_LUT[part];

  int address = (int) pp | (rolandPart << 8);
  if (_le_native()) {
    _patchParams[address + 0] = ((value & 0xf0) >> 4) & 0x0f;
    _patchParams[address + 1] = (value & 0x0f);

  } else { // TODO: Verify!
    _patchParams[address + 0] = (value & 0x0f);
    _patchParams[address + 1] = ((value & 0xf0) >> 4) & 0x0f;
  }
}


void Settings::set_patch_param(uint16_t address, uint8_t *data, uint8_t size)
{
  for (int i = 0; i < size; i++)
    _patchParams[address + i] = data[i];

  if (address == 0x138 && size >= 1) {
    if (_chorus_macro_sets_parameters())
      _run_macro_chorus(data[0]);
  } else if (address == 0x130 && size >= 1) {
    _run_macro_reverb(data[0]);
  }
}


void Settings::set_patch_param(uint16_t address, uint8_t value, int8_t part)
{
  if (part < 0 || part > 15) {
    _patchParams[address] = value;
  } else {
    int8_t rolandPart = _convert_to_roland_part_id_LUT[part];
    _patchParams[(address | (rolandPart << 8))] = value;
  }
}


void Settings::set_param(enum DrumParam dp, uint8_t map, uint8_t key, uint8_t value)
{
  if (map > 1 || key > 127)
    return;

  _drumParams[(int) dp | (map << 12) | key] = value;
}


void Settings::set_param(enum DrumParam dp, uint8_t map, uint8_t *data,
			 uint8_t length)
{
  if (map > 1)
    return;

  length = (length > 12) ? 12 : length;

  for (int i = 0; i < length; i++)
    _drumParams[(int) DrumParam::DrumsMapName + i | (map << 12)] = data[i];
}


void Settings::set_drum_param(uint16_t address, uint8_t *data, uint8_t size)
{
  if (address + size > _drumParams.size())
    return;

  for (int i = 0; i < size; i++)
    _drumParams[address + i] = data[i];
}


int8_t Settings::convert_from_roland_part_id(int8_t part)
{
  if (part < 0 || part > 15)  // TODO: Check for SC-88 and adjust max parts
    return -1;
  
  if (part == 0)
    return 9;
  else if (part <= 9)
    return part - 1;

  return part; 
}


// TODO: Evaluate this solution with other controllers
void Settings::update_pitchBend_factor(int8_t part)
{
  if (part < 0 || part > 15)
    return;

  // TODO: Convert to roland part?

  uint8_t pbRng = get_param(PatchParam::PB_PitchControl, part) - 0x40;
  uint16_t pbIn = get_param_uint16(PatchParam::PitchBend, part);
  _PBController[part] = exp(((pbIn - 8192) / 8192.0) * pbRng * (log(2) / 12));
}


float Settings::get_pitchBend_factor(int8_t part, int semitones)
{
  if (part < 0 || part > 15 || semitones <= 0)
    return 1.0f;

  uint16_t pbIn = get_param_uint16(PatchParam::PitchBend, part);
  return exp(((pbIn - 8192) / 8192.0) * semitones * (log(2) / 12));
}


void Settings::_initialize_system_params(enum Mode m)
{
  // SysEx messages
  _systemParams[(int) SystemParam::Tune + 0] = 0x00;
  _systemParams[(int) SystemParam::Tune + 1] = 0x04;
  _systemParams[(int) SystemParam::Tune + 2] = 0x00;
  _systemParams[(int) SystemParam::Tune + 3] = 0x00;
  _systemParams[(int) SystemParam::Volume] = 0x7f;
  _systemParams[(int) SystemParam::KeyShift] = 0x40;
  _systemParams[(int) SystemParam::Pan] = 0x40;

  // Non-SysEx configuration settings
  _systemParams[(int) SystemParam::RxSysEx] = 1;
  _systemParams[(int) SystemParam::RxGMOn] = 1;
  _systemParams[(int) SystemParam::RxGSReset] = 1;
  _systemParams[(int) SystemParam::RxInstrumentChange] = 1;
  _systemParams[(int) SystemParam::RxFunctionControl] = 1;
  _systemParams[(int) SystemParam::DeviceID] = 17;
  _systemParams[(int) SystemParam::Mute] = 0;
}


void Settings::_initialize_patch_params(enum Mode m)
{
  _patchParams[(int) PatchParam::PartialReserve +  0] = 0x02;
  _patchParams[(int) PatchParam::PartialReserve +  1] = 0x06;
  _patchParams[(int) PatchParam::PartialReserve +  2] = 0x02;
  _patchParams[(int) PatchParam::PartialReserve +  3] = 0x02;
  _patchParams[(int) PatchParam::PartialReserve +  4] = 0x02;
  _patchParams[(int) PatchParam::PartialReserve +  5] = 0x02;
  _patchParams[(int) PatchParam::PartialReserve +  6] = 0x02;
  _patchParams[(int) PatchParam::PartialReserve +  7] = 0x02;
  _patchParams[(int) PatchParam::PartialReserve +  8] = 0x02;
  _patchParams[(int) PatchParam::PartialReserve +  9] = 0x02;
  _patchParams[(int) PatchParam::PartialReserve + 10] = 0x00;
  _patchParams[(int) PatchParam::PartialReserve + 11] = 0x00;
  _patchParams[(int) PatchParam::PartialReserve + 12] = 0x00;
  _patchParams[(int) PatchParam::PartialReserve + 13] = 0x00;
  _patchParams[(int) PatchParam::PartialReserve + 14] = 0x00;
  _patchParams[(int) PatchParam::PartialReserve + 15] = 0x00;

  _patchParams[(int) PatchParam::ReverbMacro] =         0x04;
  _patchParams[(int) PatchParam::ReverbCharacter] =     0x04;
  _patchParams[(int) PatchParam::ReverbPreLPF] =        0x00;
  _patchParams[(int) PatchParam::ReverbLevel] =         0x40;
  _patchParams[(int) PatchParam::ReverbTime] =          0x40;
  _patchParams[(int) PatchParam::ReverbDelayFeedback] = 0x00;

  _patchParams[(int) PatchParam::ChorusMacro] =         0x02;
  _patchParams[(int) PatchParam::ChorusPreLPF] =        0x00;
  _patchParams[(int) PatchParam::ChorusLevel] =         0x40;
  _patchParams[(int) PatchParam::ChorusFeedback] =      0x08;
  _patchParams[(int) PatchParam::ChorusDelay] =         0x50;
  _patchParams[(int) PatchParam::ChorusRate] =          0x03;
  _patchParams[(int) PatchParam::ChorusDepth] =         0x13;
  _patchParams[(int) PatchParam::ChorusSendToReverb] =  0x00;

  // Remaining parameters are separate for each part
  for (int p = 0; p < 16; p ++) {           // TODO: Support SC-88 with 32 parts
    uint8_t partAddr = _convert_to_roland_part_id_LUT[p];

    _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = 0;
    _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0;
    _patchParams[(int) PatchParam::RxChannel       | (partAddr << 8)] = p;
    _patchParams[(int) PatchParam::RxPitchBend     | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxChPressure    | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxProgramChange | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxControlChange | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxPolyPressure  | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxNoteMessage   | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxRPN           | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxNRPN          | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxModulation    | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxVolume        | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxPanpot        | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxExpression    | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxHold1         | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxPortamento    | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxSostenuto     | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::RxSoft          | (partAddr << 8)] = 1;

    
    // Mono/Poly defaults to POLY (SC-55 owner's manual p.39). It was the one
    // part parameter this routine never wrote, so Part::add_note's mono-mode
    // test read whatever the heap held where Settings was allocated: a render
    // could change from adding a byte to an unrelated struct, and the JV demo
    // did (scdb devices/jv880 D-27, 2026-09-03).
    _patchParams[(int) PatchParam::PolyMode        | (partAddr << 8)] = 1;

    // MIDI channel 10 defaults to rhythm mode 1 (Drum1) in GS mode
    if (_patchParams[(int) PatchParam::RxChannel  | (partAddr << 8)] == 9) {
      _patchParams[(int) PatchParam::AssignMode   | (partAddr << 8)] = 0;
      _patchParams[(int) PatchParam::UseForRhythm | (partAddr << 8)] = 1;
    } else {
      _patchParams[(int) PatchParam::AssignMode   | (partAddr << 8)] = 1;
      _patchParams[(int) PatchParam::UseForRhythm | (partAddr << 8)] = 0;
    }

    _patchParams[(int) PatchParam::PitchKeyShift      | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PitchOffsetFine    | (partAddr << 8)] = 0x08;
    _patchParams[(int) PatchParam::PitchOffsetFine + 1| (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PartLevel          | (partAddr << 8)] = 0x64;
    _patchParams[(int) PatchParam::VelocitySenseDepth | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::VelocitySenseOffset| (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PartPanpot         | (partAddr << 8)] = 0x40;

    _patchParams[(int) PatchParam::KeyRangeLow        | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::KeyRangeHigh       | (partAddr << 8)] = 0x7f;
    _patchParams[(int) PatchParam::CC1ControllerNumber| (partAddr << 8)] = 0x10;
    _patchParams[(int) PatchParam::CC2ControllerNumber| (partAddr << 8)] = 0x11;
    _patchParams[(int) PatchParam::ChorusSendLevel    | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::ReverbSendLevel    | (partAddr << 8)] = 0x28;

    _patchParams[(int) PatchParam::RxBankSelect       | (partAddr << 8)] = 0x01;

    _patchParams[(int) PatchParam::PitchFineTune      | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PitchFineTune + 1  | (partAddr << 8)] = 0x00;

    // Tone mofify
    _patchParams[(int) PatchParam::VibratoRate    | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::VibratoDepth   | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::TVFCutoffFreq  | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::TVFResonance   | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::TVFAEnvAttack  | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::TVFAEnvDecay   | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::TVFAEnvRelease | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::VibratoDelay   | (partAddr << 8)] = 0x40;

    // Scale tuning
    _patchParams[(int) PatchParam::ScaleTuningC  | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningC_ | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningD  | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningD_ | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningE  | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningF  | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningF_ | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningG  | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningG_ | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningA  | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningA_ | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::ScaleTuningB  | (partAddr << 8)] = 0x40;

    // Controller settings
    _patchParams[(int) PatchParam::MOD_PitchControl    |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::MOD_TVFCutoffControl|(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::MOD_AmplitudeControl|(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::MOD_LFO1RateControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::MOD_LFO1PitchDepth  |(partAddr << 8)] = 0x0a;
    _patchParams[(int) PatchParam::MOD_LFO1TVFDepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::MOD_LFO1TVADepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::MOD_LFO2RateControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::MOD_LFO2PitchDepth  |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::MOD_LFO2TVFDepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::MOD_LFO2TVADepth    |(partAddr << 8)] = 0x00;

    _patchParams[(int) PatchParam::PB_PitchControl     |(partAddr << 8)] = 0x42;
    _patchParams[(int) PatchParam::PB_TVFCutoffControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PB_AmplitudeControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PB_LFO1RateControl  |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PB_LFO1PitchDepth   |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PB_LFO1TVFDepth     |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PB_LFO1TVADepth     |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PB_LFO2RateControl  |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PB_LFO2PitchDepth   |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PB_LFO2TVFDepth     |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PB_LFO2TVADepth     |(partAddr << 8)] = 0x00;

    _patchParams[(int) PatchParam::CAf_PitchControl    |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CAf_TVFCutoffControl|(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CAf_AmplitudeControl|(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CAf_LFO1RateControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CAf_LFO1PitchDepth  |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CAf_LFO1TVFDepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CAf_LFO1TVADepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CAf_LFO2RateControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CAf_LFO2PitchDepth  |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CAf_LFO2TVFDepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CAf_LFO2TVADepth    |(partAddr << 8)] = 0x00;

    _patchParams[(int) PatchParam::PAf_PitchControl    |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PAf_TVFCutoffControl|(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PAf_AmplitudeControl|(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PAf_LFO1RateControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PAf_LFO1PitchDepth  |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PAf_LFO1TVFDepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PAf_LFO1TVADepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PAf_LFO2RateControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::PAf_LFO2PitchDepth  |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PAf_LFO2TVFDepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PAf_LFO2TVADepth    |(partAddr << 8)] = 0x00;

    _patchParams[(int) PatchParam::CC1_PitchControl    |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CC1_TVFCutoffControl|(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CC1_AmplitudeControl|(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CC1_LFO1RateControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CC1_LFO1PitchDepth  |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC1_LFO1TVFDepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC1_LFO1TVADepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC1_LFO2RateControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CC1_LFO2PitchDepth  |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC1_LFO2TVFDepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC1_LFO2TVADepth    |(partAddr << 8)] = 0x00;

    _patchParams[(int) PatchParam::CC2_PitchControl    |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CC2_TVFCutoffControl|(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CC2_AmplitudeControl|(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CC2_LFO1RateControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CC2_LFO1PitchDepth  |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC2_LFO1TVFDepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC2_LFO1TVADepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC2_LFO2RateControl |(partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::CC2_LFO2PitchDepth  |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC2_LFO2TVFDepth    |(partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC2_LFO2TVADepth    |(partAddr << 8)] = 0x00;

    // Controller values
    _patchParams[(int) PatchParam::PitchBend       | (partAddr << 8)] = 0x20;
    _patchParams[(int) PatchParam::PitchBend + 1   | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::Modulation      | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC1Controller   | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::CC2Controller   | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::ChannelPressure | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PolyKeyPressure | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::Hold1           | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::Sostenuto       | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::Soft            | (partAddr << 8)] = 0x00;
    // Expression rests at 127 on a Sound Canvas, where it IS a part volume, and
    // at 0 on the JV, where it is one of the three sources of the per-tone
    // controller matrix and reaches nothing else. Measured: with no CC11 at all
    // the machine renders exactly its CC11 = 0 render, and a tone routing
    // expression to LEVEL at -63 is silenced by CC11 = 127. scdb D-79.
    _patchParams[(int) PatchParam::Expression      | (partAddr << 8)] =
      expression_reset();
    _patchParams[(int) PatchParam::Portamento      | (partAddr << 8)] = 0x00;
    _patchParams[(int) PatchParam::PortamentoTime  | (partAddr << 8)] = 0x00;

    // The JV keeps CC7 apart from the part level and its reset writes 127 there
    // (scdb D-28, ROM1 0x67b0). Inert under GS, where CC7 IS the part level and
    // nothing reads this slot - but it must be 127 rather than 0, or a JV part
    // that never receives CC7 would be silent.
    _patchParams[(int) PatchParam::PartVolume      | (partAddr << 8)] = 0x7f;

    _patchParams[(int) PatchParam::RPN_LSB         | (partAddr << 8)] = 0x7f;
    _patchParams[(int) PatchParam::RPN_MSB         | (partAddr << 8)] = 0x7f;
    _patchParams[(int) PatchParam::NRPN_LSB        | (partAddr << 8)] = 0x7f;
    _patchParams[(int) PatchParam::NRPN_MSB        | (partAddr << 8)] = 0x7f;

    _patchParams[(int) PatchParam::PitchCoarseTune | (partAddr << 8)] = 0x40;
    _patchParams[(int) PatchParam::Mute            | (partAddr << 8)] = 0x00;
  }
}


void Settings::_initialize_drumSet_params(void)
{
  // Both MAP0 & MAP1 to Standard drum set (0)
  update_drum_set(0, 0);
  update_drum_set(1, 0);
}


bool Settings::has_gm_mode(void) const
{
  // Why the SC-55 and a Roland-only receiver have none: see set_gm_mode().
  return _ctrlRom.generation() != ControlRom::SynthGen::SC55 &&
         !_ctrlRom.device()->rolandSysExOnly;
}


void Settings::set_gm_mode(void)
{
  // "Turn General MIDI System On" sets every internal parameter to the
  // General MIDI System Level 1 defaults. Those are the same defaults the GS
  // reset establishes, with one documented exception: "( Rx.NRPN SW and
  // Rx.Bank sel SW will be turned OFF by this message.)" -- SC-55mkII Owner's
  // Manual p.93, where the corresponding note under "GS reset" on the same
  // page says both switches are turned ON instead.
  //
  // That exception belongs to the mkII. The original SC-55's manual documents
  // neither this message nor an Rx.Bank-Select switch (its receive chart on
  // p.76 lists only manufacturer ID 41H), and measurement agrees: on the
  // SC-55 a bank-selected variation tone and an NRPN both still take effect
  // after the message, while on the SC-55mkII neither does (P-0092). So the
  // SC-55 reaches plain GS defaults here and nothing more.
  //
  // A device that cannot receive GM System On has no GM mode either: a GS_GM
  // host reset leaves it exactly as a GS one does, receive switches included.
  if (!has_gm_mode())
    return;

  for (int p = 0; p < 16; p ++) {           // TODO: Support SC-88 with 32 parts
    uint8_t partAddr = _convert_to_roland_part_id_LUT[p];
    _patchParams[(int) PatchParam::RxNRPN       | (partAddr << 8)] = 0x0;
    _patchParams[(int) PatchParam::RxBankSelect | (partAddr << 8)] = 0x0;
  }
}


void Settings::set_map_mt32(void)
{
  uint8_t partAddr = _convert_to_roland_part_id_LUT[0];
  _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0x00;
  _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = 0x40;
  _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = 0x40;

  partAddr = _convert_to_roland_part_id_LUT[1];
  _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0x44;
  _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = 0x36;
  _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = 0x40;

  partAddr = _convert_to_roland_part_id_LUT[2];
  _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0x30;
  _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = 0x36;
  _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = 0x40;

  partAddr = _convert_to_roland_part_id_LUT[3];
  _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0x5f;
  _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = 0x36;
  _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = 0x40;

  partAddr = _convert_to_roland_part_id_LUT[4];
  _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0x4e;
  _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = 0x36;
  _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = 0x40;

  partAddr = _convert_to_roland_part_id_LUT[5];
  _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0x29;
  _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = 0x12;
  _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = 0x40;

  partAddr = _convert_to_roland_part_id_LUT[6];
  _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0x03;
  _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = 0x5b;
  _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = 0x40;

  partAddr = _convert_to_roland_part_id_LUT[7];
  _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0x6e;
  _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = 0x01;
  _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = 0x40;

  partAddr = _convert_to_roland_part_id_LUT[8];
  _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0x7a;
  _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = 0x40;

  int dsIndex = update_drum_set(0, 127);
  partAddr = _convert_to_roland_part_id_LUT[9];
  _patchParams[(int) PatchParam::ToneNumber      | (partAddr << 8)] = dsIndex;
  _patchParams[(int) PatchParam::ToneNumber + 1  | (partAddr << 8)] = 0x7f;
  _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = 0x40;
  _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = 0x40;
}


int Settings::update_drum_set(uint8_t map, uint8_t bank)
{
  if (map > 1 || bank > 127)
    return -2;

  // Find index for drum set in given bank
  const std::array<uint8_t, 128> &drumSetsLUT = _ctrlRom.get_drum_sets_LUT();

  // Invalid entries in the drum sets lookup table have the value 0xff
  if (drumSetsLUT[bank] == 0xff)
    return -1;

  int index = 0;
  for (int it = 0; it < bank; it ++) {
    if (drumSetsLUT[it] != 0xff)
      index++;
  }

  // The index is counted out of the drum-set LOOKUP TABLE, which has 128 slots,
  // but it then subscripts the device's actual drum-set array. Those two are not
  // the same size on every device: the JV-880 has THREE rhythm sets while the
  // table it inherits admits far more, so a General MIDI file selecting a high
  // drum kit indexed past the end and the render died with SIGSEGV - two of the
  // first five corpus files (D-42). Treat an out-of-range kit exactly as the
  // function already treats an invalid table entry.
  if (index >= _ctrlRom.numDrumSets())
    return -1;

  return _copy_drum_set(map, index);
}


// A JV rhythm program change selects the SET BY BANK: the top two bits of the
// selector name the memory bank and the program number within it is ignored
// (ROM2 0x30446, devices/jv880/04_protocol/program_bank.md). Internal is set 0,
// Preset A set 1, Preset B set 2; the Card bank is absent on a bare machine and
// is rejected exactly as an absent patch bank is (D-43). Returns the set index,
// or -1 when nothing was selected and the part keeps the kit it has.
int Settings::update_drum_set_bank(uint8_t map, int selector)
{
  if (map > 1) return -2;

  int set;
  switch ((selector >> 6) & 0x03) {
    case 0:  set = 0; break;          // Internal
    case 1:  return -1;               // Card: not present
    case 2:  set = 1; break;          // Preset A
    default: set = 2; break;          // Preset B
  }
  if (set >= _ctrlRom.numDrumSets())
    return -1;

  return _copy_drum_set(map, set);
}


int Settings::_copy_drum_set(uint8_t map, int index)
{
  // On the original hardware both the active drum set configurations are
  // copied from ROM to RAM where they can be modified by the user.
  //
  // On the JV that copy is the temporary rhythm setup, and selecting a set is
  // also what THROWS AWAY any DT1 edits the previous selection had collected -
  // so the records go back to what the ROM holds before anything is read out
  // of them (scdb D-32). A no-op when nothing was edited.
  _ctrlRom.device_rhythm_reload(index);

  for (unsigned int i = 0; i < 12; i ++) {
    if (i < _ctrlRom.drumSet(index).name.length()) {
      _drumParams[(int) DrumParam::DrumsMapName + i |(map << 12)] =
	_ctrlRom.drumSet(index).name[i];
    } else {
      _drumParams[(int) DrumParam::DrumsMapName + i |(map << 12)] = ' ';
    }
  }
  for (int r = 0; r < 128; r++)
    _copy_drum_key(map, index, r);

  if (map < 2)
    _drumSetIndex[map] = index;

  return index;
}


// One key of a drum set, from the control ROM's copy into the map's RAM. Split
// out of _copy_drum_set() because a JV rhythm-note DT1 changes one key.
void Settings::_copy_drum_key(uint8_t map, int index, int r)
{
  _drumParams[(int) DrumParam::PlayKeyNumber     | (map << 12) | r] =
    _ctrlRom.drumSet(index).key[r];
  _drumParams[(int) DrumParam::Level             | (map << 12) | r] =
    _ctrlRom.drumSet(index).volume[r];
  _drumParams[(int) DrumParam::AssignGroupNumber | (map << 12) | r] =
    _ctrlRom.drumSet(index).assignGroup[r];
  _drumParams[(int) DrumParam::Panpot            | (map << 12) | r] =
    _ctrlRom.drumSet(index).panpot[r];
  _drumParams[(int) DrumParam::ReverbDepth       | (map << 12) | r] =
    _ctrlRom.drumSet(index).reverb[r];
  _drumParams[(int) DrumParam::ChorusDepth       | (map << 12) | r] =
    _ctrlRom.drumSet(index).chorus[r];
  _drumParams[(int) DrumParam::RxNoteOff         | (map << 12) | r] =
    _ctrlRom.drumSet(index).flags[r] & 0x01;
  _drumParams[(int) DrumParam::RxNoteOn          | (map << 12) | r] =
    _ctrlRom.drumSet(index).flags[r] & 0x10;
}


// A JV rhythm-note DT1. The device has ONE temporary rhythm setup, so the edit
// lands on whichever set the drum maps hold - both, when they hold the same
// one, which is the only case the JV reaches.
bool Settings::rhythm_note_dt1(int note, uint8_t param, uint8_t value)
{
  if (!_ctrlRom.has_device_rhythm_dt1())
    return false;

  bool acted = false;
  int done = -1;
  for (uint8_t map = 0; map < 2; map++) {
    const int index = _drumSetIndex[map];
    if (index < 0 || index == done)
      continue;

    const int key = _ctrlRom.device_rhythm_dt1(index, note, param, value);
    done = index;
    if (key < 0 || key > 127)
      continue;

    for (uint8_t m = 0; m < 2; m++)
      if (_drumSetIndex[m] == index)
        _copy_drum_key(m, index, key);
    acted = true;
  }

  return acted;
}


bool Settings::load(std::string filePath)
{
  return 0;
}


bool Settings::save(std::string filePath)
{
  return 0;
}

void Settings::reset(void)
{
  _initialize_system_params();
  _initialize_patch_params();
  _initialize_drumSet_params();
  _apply_device_performance();
}


uint8_t Settings::_to_native_endian_nib16(const std::atomic<uint8_t> *ptr)
{
  const uint8_t b0 = ptr[0].load(std::memory_order_relaxed);
  const uint8_t b1 = ptr[1].load(std::memory_order_relaxed);

  if (_le_native())
    return ((b0 << 4) | (b1 & 0x0f));

  return ((b1 << 4) | (b0 & 0x0f));
}


uint16_t Settings::_to_native_endian_uint14(const std::atomic<uint8_t> *ptr)
{
  const uint8_t b0 = ptr[0].load(std::memory_order_relaxed);
  const uint8_t b1 = ptr[1].load(std::memory_order_relaxed);

  if (_le_native())
    return ((b0 & 0x7f) << 7 | b1);

  return ((b1 & 0x7f) << 7 | b0);
}


uint16_t Settings::_to_native_endian_uint16(const std::atomic<uint8_t> *ptr)
{
  const uint8_t b0 = ptr[0].load(std::memory_order_relaxed);
  const uint8_t b1 = ptr[1].load(std::memory_order_relaxed);

  if (_le_native())
    return ((b0 << 8) | b1);

  return ((b1 << 8) | b0);
}


uint32_t Settings::_to_native_endian_uint32(const std::atomic<uint8_t> *ptr)
{
  const uint8_t b0 = ptr[0].load(std::memory_order_relaxed);
  const uint8_t b1 = ptr[1].load(std::memory_order_relaxed);
  const uint8_t b2 = ptr[2].load(std::memory_order_relaxed);
  const uint8_t b3 = ptr[3].load(std::memory_order_relaxed);

  if (_le_native())
    return (b0 << 24 | b1 << 16 | b2 << 8 | b3);

  return (b3 << 24 | b2 << 16 | b1 << 8 | b0);
}


// On a Sound Canvas the chorus macro is a preset: selecting one also writes the
// feedback, delay, rate and depth it stands for. On the JV-880 the same
// parameter slot carries Chorus Type, which selects a ROM record and touches
// nothing else - CHORUS3 with Depth 80 and Rate 60 sweeps 0.25-8.6 ms in 0.43 s
// on the machine, and the macro table would have made it Depth 19 and Rate 3,
// a 5.5 ms window in 0.75 s (scdb devices/jv880 D-20, M-083).
bool Settings::_chorus_macro_sets_parameters(void)
{
  const DeviceProfile *dev = device();
  return !dev || dev->chorusLawKind != ChorusLawKind::JVSweptPointer;
}


// Macro based on table from SC-8820 owner's manual and verified on SC-55MkII
void Settings::_run_macro_chorus(uint8_t value)
{
  switch(value)
    {
    case 0: // Chorus 1
      _patchParams[(int) PatchParam::ChorusFeedback] = 0x00;
      _patchParams[(int) PatchParam::ChorusDelay] =    0x70;
      _patchParams[(int) PatchParam::ChorusRate] =     0x03;
      _patchParams[(int) PatchParam::ChorusDepth] =    0x06;
      break;

    case 1: // Chorus 2
      _patchParams[(int) PatchParam::ChorusFeedback] = 0x08;
      _patchParams[(int) PatchParam::ChorusDelay] =    0x50;
      _patchParams[(int) PatchParam::ChorusRate] =     0x09;
      _patchParams[(int) PatchParam::ChorusDepth] =    0x13;
      break;

    case 2: // Chorus 3
      _patchParams[(int) PatchParam::ChorusFeedback] = 0x08;
      _patchParams[(int) PatchParam::ChorusDelay] =    0x50;
      _patchParams[(int) PatchParam::ChorusRate] =     0x03;
      _patchParams[(int) PatchParam::ChorusDepth] =    0x13;
      break;

    case 3: // Chorus 4
      _patchParams[(int) PatchParam::ChorusFeedback] = 0x08;
      _patchParams[(int) PatchParam::ChorusDelay] =    0x40;
      _patchParams[(int) PatchParam::ChorusRate] =     0x09;
      _patchParams[(int) PatchParam::ChorusDepth] =    0x10;
      break;

    case 4: // Feedback Chorus
      _patchParams[(int) PatchParam::ChorusFeedback] = 0x40;
      _patchParams[(int) PatchParam::ChorusDelay] =    0x7f;
      _patchParams[(int) PatchParam::ChorusRate] =     0x02;
      _patchParams[(int) PatchParam::ChorusDepth] =    0x18;
      break;

    case 5: // Flanger
      _patchParams[(int) PatchParam::ChorusFeedback] = 0x70;
      _patchParams[(int) PatchParam::ChorusDelay] =    0x7f;
      _patchParams[(int) PatchParam::ChorusRate] =     0x01;
      _patchParams[(int) PatchParam::ChorusDepth] =    0x05;
      break;

    case 6: // Short Delay
      _patchParams[(int) PatchParam::ChorusFeedback] = 0x00;
      _patchParams[(int) PatchParam::ChorusDelay] =    0x7f;
      _patchParams[(int) PatchParam::ChorusRate] =     0x00;
      _patchParams[(int) PatchParam::ChorusDepth] =    0x7f;
      break;

    case 7: // Short Delay (FB)
      _patchParams[(int) PatchParam::ChorusFeedback] = 0x50;
      _patchParams[(int) PatchParam::ChorusDelay] =    0x7f;
      _patchParams[(int) PatchParam::ChorusRate] =     0x00;
      _patchParams[(int) PatchParam::ChorusDepth] =    0x7f;
      break;
    }

  // These params are equal for all macro values
  _patchParams[(int) PatchParam::ChorusLevel] =        0x40;
  _patchParams[(int) PatchParam::ChorusPreLPF] =       0x00;
  _patchParams[(int) PatchParam::ChorusSendToReverb] = 0x00;
}


// Macro based on table from SC-8820 owner's manual and verified on SC-55MkII
void Settings::_run_macro_reverb(uint8_t value)
{
  switch(value)
    {
    case 0: // Room 1
      _patchParams[(int) PatchParam::ReverbCharacter] =     0x00;
      _patchParams[(int) PatchParam::ReverbPreLPF] =        0x03;
      _patchParams[(int) PatchParam::ReverbTime] =          0x50;
      _patchParams[(int) PatchParam::ReverbDelayFeedback] = 0x00;
      break;

    case 1: // Room 2
      _patchParams[(int) PatchParam::ReverbCharacter] =     0x01;
      _patchParams[(int) PatchParam::ReverbPreLPF] =        0x04;
      _patchParams[(int) PatchParam::ReverbTime] =          0x38;
      _patchParams[(int) PatchParam::ReverbDelayFeedback] = 0x00;
      break;

    case 2: // Room 3
      _patchParams[(int) PatchParam::ReverbCharacter] =     0x02;
      _patchParams[(int) PatchParam::ReverbPreLPF] =        0x00;
      _patchParams[(int) PatchParam::ReverbTime] =          0x40;
      _patchParams[(int) PatchParam::ReverbDelayFeedback] = 0x00;
      break;

    case 3: // Hall 1
      _patchParams[(int) PatchParam::ReverbCharacter] =     0x03;
      _patchParams[(int) PatchParam::ReverbPreLPF] =        0x04;
      _patchParams[(int) PatchParam::ReverbTime] =          0x48;
      _patchParams[(int) PatchParam::ReverbDelayFeedback] = 0x00;
      break;

    case 4: // Hall 2
      _patchParams[(int) PatchParam::ReverbCharacter] =     0x04;
      _patchParams[(int) PatchParam::ReverbPreLPF] =        0x00;
      _patchParams[(int) PatchParam::ReverbTime] =          0x40;
      _patchParams[(int) PatchParam::ReverbDelayFeedback] = 0x00;
      break;

    case 5: // Plate
      _patchParams[(int) PatchParam::ReverbCharacter] =     0x05;
      _patchParams[(int) PatchParam::ReverbPreLPF] =        0x00;
      _patchParams[(int) PatchParam::ReverbTime] =          0x58;
      _patchParams[(int) PatchParam::ReverbDelayFeedback] = 0x00;
      break;

    case 6: // Delay
      _patchParams[(int) PatchParam::ReverbCharacter] =     0x06;
      _patchParams[(int) PatchParam::ReverbPreLPF] =        0x00;
      _patchParams[(int) PatchParam::ReverbTime] =          0x20;
      _patchParams[(int) PatchParam::ReverbDelayFeedback] = 0x28;
      break;

    case 7: // Panning Delay
      _patchParams[(int) PatchParam::ReverbCharacter] =     0x07;
      _patchParams[(int) PatchParam::ReverbPreLPF] =        0x00;
      _patchParams[(int) PatchParam::ReverbTime] =          0x40;
      _patchParams[(int) PatchParam::ReverbDelayFeedback] = 0x20;
      break;
    }

  // These params are equal for all macro values
  _patchParams[(int) PatchParam::ReverbLevel] =         0x40;
}


// The SC-55+ have 6 independent controllers that each controls 11 parameters.
// These parameters are accumulated from each of the controllers. Note that
// accumulated values are only updated when a controller changes value - and
// not when a controller parameter is changed.
// This function updates all 11 parameters for a specific controller
void Settings::_update_controller_input(enum PatchParam pp, int value, int part)
{
  int ctrlInt;

  switch ((int) pp) {
    case ((int) PatchParam::Modulation):
      ctrlInt = static_cast<int>(Controller::Modulation);
      break;
    case ((int) PatchParam::PitchBend):
      ctrlInt = static_cast<int>(Controller::PitchBend);
      break;
    case ((int) PatchParam::ChannelPressure):
      ctrlInt = static_cast<int>(Controller::ChannelPressure);
      break;
    case ((int) PatchParam::PolyKeyPressure):
      ctrlInt = static_cast<int>(Controller::PolyKeyPressure);
      break;
    case ((int) PatchParam::CC1Controller):
      ctrlInt = static_cast<int>(Controller::CC1);
      break;
    case ((int) PatchParam::CC2Controller):
      ctrlInt = static_cast<int>(Controller::CC2);
      break;
    default:
      std::fprintf(stderr, "libEmuSC: Internal error (unkown controller)\n");
      return;
  }

  int ppMemOffset = ctrlInt *  0x10;
  uint8_t partAddr = _convert_to_roland_part_id_LUT[part];

  _controlParams[part][static_cast<int>(ControllerParam::Pitch)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_PitchControl | (partAddr << 8))
                    + ppMemOffset] - 0x40) * value) / 1);

  _controlParams[part][static_cast<int>(ControllerParam::TVFCutoff)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_TVFCutoffControl | (partAddr << 8))
                    + ppMemOffset] - 0x40) * value) / 2);

  _controlParams[part][static_cast<int>(ControllerParam::Amplitude)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_AmplitudeControl | (partAddr << 8))
                    + ppMemOffset] - 0x40) * value) / 2);

  _controlParams[part][static_cast<int>(ControllerParam::LFO1Rate)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_LFO1RateControl | (partAddr << 8))
                    + ppMemOffset] - 0x40) * value) / 2);

  _controlParams[part][static_cast<int>(ControllerParam::LFO1PitchDepth)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_LFO1PitchDepth | (partAddr << 8))
                    + ppMemOffset]) * value) / 4);

  _controlParams[part][static_cast<int>(ControllerParam::LFO1TVFDepth)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_LFO1TVFDepth | (partAddr << 8))
                    + ppMemOffset]) * value) / 4);

  _controlParams[part][static_cast<int>(ControllerParam::LFO1TVADepth)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_LFO1TVADepth | (partAddr << 8))
                    + ppMemOffset]) * value) / 4);

  _controlParams[part][static_cast<int>(ControllerParam::LFO2Rate)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_LFO2RateControl | (partAddr << 8))
                    + ppMemOffset] - 0x40) * value) / 2);

  _controlParams[part][static_cast<int>(ControllerParam::LFO2PitchDepth)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_LFO2PitchDepth | (partAddr << 8))
                    + ppMemOffset]) * value) / 4);

  _controlParams[part][static_cast<int>(ControllerParam::LFO2TVFDepth)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_LFO2TVFDepth | (partAddr << 8))
                    + ppMemOffset]) * value) / 4);

  _controlParams[part][static_cast<int>(ControllerParam::LFO2TVADepth)][ctrlInt] =
    (((_patchParams[((int) PatchParam::MOD_LFO2TVADepth | (partAddr << 8))
                    + ppMemOffset]) * value) / 4);


  // FIXME: Should this one run once at 125Hz for each part? Move to Part calss?
  _update_controller_input_acc(part);
}


void Settings::_update_controller_input_acc(int8_t part)
{
  _accControlParams[part][static_cast<int>(ControllerParam::Pitch)] =
    _calc_controller_value(ControllerParam::Pitch, part, 0xbe8, 8 * 0xfbf8);

  _accControlParams[part][static_cast<int>(ControllerParam::TVFCutoff)] =
    _calc_controller_value(ControllerParam::TVFCutoff, part, 0xfa0, 8 * 0xc49c);

  _accControlParams[part][static_cast<int>(ControllerParam::Amplitude)] =
    _calc_controller_value(ControllerParam::Amplitude, part, 0xfa0, 16 * 0x820d);

  _accControlParams[part][static_cast<int>(ControllerParam::LFO1Rate)] =
    _calc_controller_value(ControllerParam::LFO1Rate, part, 0xfa0, 2 * 0xa7c7);

  _accControlParams[part][static_cast<int>(ControllerParam::LFO1PitchDepth)] =
    _calc_controller_value(ControllerParam::LFO1PitchDepth, part, 0xfc0, 2 * 0xbe7a);

  _accControlParams[part][static_cast<int>(ControllerParam::LFO1TVFDepth)] =
    _calc_controller_value(ControllerParam::LFO1TVFDepth, part, 0xfc0, 2 * 0xc30d);

  _accControlParams[part][static_cast<int>(ControllerParam::LFO1TVADepth)] =
    _calc_controller_value(ControllerParam::LFO1TVADepth, part, 0xfc0, 16 * 0x8105);

  _accControlParams[part][static_cast<int>(ControllerParam::LFO2Rate)] =
  _calc_controller_value(ControllerParam::LFO2Rate, part, 0xfa0, 2 * 0xa7c7);

  _accControlParams[part][static_cast<int>(ControllerParam::LFO2PitchDepth)] =
    _calc_controller_value(ControllerParam::LFO2PitchDepth, part, 0xfc0, 2 * 0xbe7a);

  _accControlParams[part][static_cast<int>(ControllerParam::LFO2TVFDepth)] =
    _calc_controller_value(ControllerParam::LFO2TVFDepth, part, 0xfc0, 2 * 0xc30d);

  _accControlParams[part][static_cast<int>(ControllerParam::LFO2TVADepth)] =
    _calc_controller_value(ControllerParam::LFO2TVADepth, part, 0xfc0, 16 * 0x8105);


}


int16_t Settings::_calc_controller_value(ControllerParam cp, int part, int max, int mul)
{
  int accValue = 0;
  for (const auto& cValue : _controlParams[part][static_cast<int>(cp)]) {
    accValue += cValue;
  }

  int output = accValue;
  if (accValue < 0) output = -output;
  if (output > max) output = max;
  output *= mul;
  output >>= 16;
  if (accValue < 0) output = -output;

  return output;
}

void Settings::set_part_callback(std::function<void(const int)> cb)
{
  _partCallback = cb;
}


void Settings::clear_part_callback(void)
{
  _partCallback = NULL;
}




// Everything the JV takes from its PERFORMANCE, applied as a final pass.
//
// It is a separate pass on purpose. libEmuSC's initialisation was written for
// the Sound Canvas and is full of values hardcoded for it, several of which are
// written to parts AFTER _initialize_patch_params returns -
// _initialize_drumSet_params and set_gm_mode both do. Applied inside that
// function, these settings were silently undone twice: once by the generic
// per-part defaults resetting PitchKeyShift, which cost an octave on the demo's
// melody, and again by the drum-set pass rewriting part 10's tone number, pan
// and reverb send. Running last is the only placement that survives both.
bool Settings::select_performance(int selector)
{
  if (!_ctrlRom.select_performance(selector))
    return false;
  _apply_device_performance();
  return true;
}


void Settings::_apply_device_performance(void)
{
  if (_ctrlRom.generation() != ControlRom::SynthGen::JV880)
    return;

  _devicePerformanceLoads++;

  // The performance's effect settings, applied before the parts.
  const auto &fx = _ctrlRom.device_effects();
  _patchParams[(int) PatchParam::ReverbCharacter]     = fx.reverbType;
  _patchParams[(int) PatchParam::ReverbLevel]         = fx.reverbLevel;
  _patchParams[(int) PatchParam::ReverbTime]          = fx.reverbTime;
  _patchParams[(int) PatchParam::ReverbDelayFeedback] = fx.reverbFeedback;
  _patchParams[(int) PatchParam::ChorusMacro]         = fx.chorusType;
  _patchParams[(int) PatchParam::ChorusLevel]         = fx.chorusLevel;
  _patchParams[(int) PatchParam::ChorusRate]          = fx.chorusRate;
  _patchParams[(int) PatchParam::ChorusDepth]         = fx.chorusDepth;
  _patchParams[(int) PatchParam::ChorusFeedback]      = fx.chorusFeedback;
  // Chorus Output: the JV's switch is on or off, so it maps to a full send or
  // none rather than a level.
  _patchParams[(int) PatchParam::ChorusSendToReverb]  = fx.chorusToReverb ? 0x7f : 0;

  const auto &jp = _ctrlRom.device_parts();
  for (int p = 0; p < 16; p++) {
    uint8_t partAddr = _convert_to_roland_part_id_LUT[p];

    if (p >= (int) jp.size()) {
      // The JV has eight parts and libEmuSC sixteen. Silence the rest rather
      // than leave them answering on channels the performance never assigned.
      _patchParams[(int) PatchParam::RxNoteMessage | (partAddr << 8)] = 0;
      continue;
    }

    const auto &j = jp[p];


    _patchParams[(int) PatchParam::RxChannel       | (partAddr << 8)] = j.channel;
    _patchParams[(int) PatchParam::RxNoteMessage   | (partAddr << 8)] = 1;
    _patchParams[(int) PatchParam::UseForRhythm    | (partAddr << 8)] = j.rhythm ? 1 : 0;
    _patchParams[(int) PatchParam::AssignMode      | (partAddr << 8)] = j.rhythm ? 0 : 1;

    // Key Assign. A SOLO patch plays one note at a time on its part: the
    // firmware's note-on path for such a part re-uses the sounding note's
    // voices for the new key (ROM1 0x9D6 -> 0xBCD -> 0xC84, selected by patch
    // common +0x18 bit 7 through @0x8C50). The boot performance's SAW Lead is
    // SOLO, on parts 1-6 (scdb D-44).
    _patchParams[(int) PatchParam::PolyMode        | (partAddr << 8)] =
      (!j.rhythm && j.patch >= 0 && _ctrlRom.device_patch_solo(j.patch)) ? 0 : 1;
    if (j.rhythm) {
      // The rhythm part takes a Rhythm SET, and which one is named by the bank
      // of the performance's own patch byte: below 64 Internal, 64-127 Preset A
      // (the translation in control_rom.cc puts raw 128-191 there), 128 and up
      // Preset B. "for CompuMix", which every demo song selects, carries 128 -
      // Preset A - and loading the Internal kit instead was worth +20 dB on
      // song 3's rhythm channel alone. scdb D-52.
      const int set = (j.patch < 0) ? 0
                    : (j.patch < 64) ? 0 : (j.patch < 128) ? 1 : 2;
      const int idx = _copy_drum_set(0, set);
      _patchParams[(int) PatchParam::ToneNumber    | (partAddr << 8)] =
        (idx >= 0) ? idx : 0;
      _patchParams[(int) PatchParam::ToneNumber + 1| (partAddr << 8)] = 0;
    } else if (j.patch >= 0) {
      // A part's patch has to be written as the (bank, program) pair the engine
      // resolves through _variations, not as a raw instrument index. On the JV
      // the table is built as _variations[0][0..63] = Internal and
      // _variations[81][0..127] = Preset A then Preset B, so an instrument at
      // or above 64 written into bank 0 lands on 0xffff and the part is SILENT.
      //
      // That never showed while the port only ever loaded the boot performance:
      // "Syn Lead" carries Internal patches throughout. Every other performance
      // has preset patches - "for CompuMix", the one all seven demo songs
      // select, carries 129/183/195/172/140/166/155, all Preset - and each of
      // those parts went quiet. scdb D-46.
      const auto &P = _ctrlRom.device()->records->patch;
      int bank = P.midiBankFirst, prog = j.patch;
      if (j.patch >= P.perBank) {
        bank = P.midiBankPresets;
        prog = j.patch - P.perBank;      // Preset A at 0-63, Preset B at 64-127
      }
      _patchParams[(int) PatchParam::ToneNumber    | (partAddr << 8)] = bank;
      _patchParams[(int) PatchParam::ToneNumber + 1| (partAddr << 8)] = prog;
    }
    _patchParams[(int) PatchParam::PartLevel       | (partAddr << 8)] = j.level;
    _patchParams[(int) PatchParam::PartPanpot      | (partAddr << 8)] = j.pan;
    _patchParams[(int) PatchParam::PitchKeyShift   | (partAddr << 8)] =
      (uint8_t) std::clamp(0x40 + j.keyShift, 0, 127);
    _patchParams[(int) PatchParam::ReverbSendLevel | (partAddr << 8)] = j.reverb;
    _patchParams[(int) PatchParam::ChorusSendLevel | (partAddr << 8)] = j.chorus;
  }
}

}
