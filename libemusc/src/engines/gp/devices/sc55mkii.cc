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
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with libEmuSC. If not, see <http://www.gnu.org/licenses/>.
 *
 *  Roland SC-55mkII device profile (firmware 1.01).
 *
 *  Data only. See device_profile.h for what each field means. The ROM
 *  identification signature lives in src/devices/sc55mkii.cc instead: that
 *  part is read before any engine is selected, by code this profile has no
 *  part in.
 */
#include "../../../device_profile.h"

namespace EmuSC
{

// Instrument bank offsets in the control ROM. The same eight as the SC-55's,
// repeated here rather than shared so each device file stands on its own.
static const std::vector<uint32_t> SC55MKII_BANKS =
  { 0x10000, 0x1BD00, 0x1DEC0, 0x20000, 0x2BD00, 0x2DEC0, 0x30000, 0x38000 };

static const ProgramRomMap SC55MKII_PROGRAM_MAP = {
  0x3d1e8,      // VelocityCurves
  0x3dd7c,      // KeyMapperIndex
  0x3de8c,      // KeyMapper
  0x3f814       // TVAPanKeyFollow
};

static const CpuRomMap SC55MKII_CPU_MAP = {
  // LFORate was 0x6486, which breaks the 0x100 ladder its neighbours sit on and
  // points at bytes whose values all exceed the rate clamp, so every tone's LFO
  // ran pinned at 20 Hz instead of its own rate. The table at 0x6d86 is monotone,
  // its top entry is the clamp constant itself, and it sits the same distance
  // past the envelope-time table as the SC-55's does (P-0112, P-0113).
  0x1310, 0x650e, 0x653a, 0x6c86, 0x6d86, 0x6e86, 0x6f86, 0x7086,
  0x7186, 0x7246, 0x7270, 0x7286, 0x7386, 0x7488, 0x758a, 0x763a,
  0x7650, 0x7666, 0x7766, 0x77a6, 0x652e, 0x687a, 0x6a84, 0x673a,
  0x6a03, 0x6883, 0x6903, 0x78ee, 0x7aee
};

static const SoundCanvasLayout SC55MKII_LAYOUT = {
  &SC55MKII_BANKS,
  &SC55MKII_PROGRAM_MAP,
  &SC55MKII_CPU_MAP,
  12,           // velocity curves

  0x03c028,     // drum sets run to here, in 1164-byte blocks
  1164,

  0x30000,      // the key mapper offset is relative to this bank

  0x03fff0,     // the demo songs sit late in the ROM
  true,         // and run to its end

  // Two animations laid end to end: the SC-55mkII's at 0x70000 and the
  // SC-155mkII's at 0x71280, each 0x1280 long.
  0x70000,
  0x1280,
  2
};

const DeviceProfile SC55MKII_PROFILE = {
  "SC-55mkII",

  0, 2,        // identified by signature, not by size

  28,           // max polyphony

  // 8.4 dB/ms, measured (PROVENANCE.md P-0080).
  8.4f,

  nullptr,      // not a record-addressed ROM
  &SC55MKII_LAYOUT,

  nullptr, 0,

  // The Sound Canvas accumulates attenuations in a log index domain and does not
  // use this law; left zero so a misuse is obvious rather than plausible. Its
  // envelope table is already in engine ticks, and segments of 8 ticks or fewer
  // snap - the behaviour tva.cc has always had for this generation.
  { 0, 0, 0, 0, 0, 0, 8, 1 },

  LevelLawKind::SoundCanvasLogIndex,

  {
    1.829f, -11.9f, 187,

    // Return level, pre-LPF pair and delay taps. Shared across the family
    // because the reverb DSP is: see ReverbLaw. levelDivisor is not the
    // firmware's return law and ReverbLaw says why it still stands here.
    64.0f,
    ReverbReturnLaw::LevelOverDivisor,
    8, 0x3f, 4,
    0x16, 112,
    ReverbDelayTapLaw::LinearPerTime,

    // The Sound Canvas program and its own Time -> loop-gain line, unchanged.
    ReverbNetworkKind::SoundCanvasProgram,
    ReverbFeedbackLaw::SoundCanvasTimeLine
  },

  { true,  0x3f, 0x7f },

  // This generation carries a 2 MB wave ROM, so bank id 2 lands a megabyte
  // further up than on the mk1.
  { false, 0x200000 },

  // Written out only to reach the protocol flags below; each is the value an
  // omitted member gets.
  {},     // tvfLawKind
  {},     // tvfJv
  {},     // chorusLawKind
  {},     // chorusJv
  {},     // pitchJv
  {},     // analog
  {},     // ctrlJv
  false,  // variationFallback
  false,  // rolandSysExOnly
  false,  // gsScaleTuningOnly
  false,  // ignoresAllSoundOff

  // @provenance class=DOCUMENTED devices=SC-55mkII
  // Owner's manual p.92: "MONO is recognized as 'all sounds off'" and "POLY
  // is recognized as 'all sounds off'". The mk1's manual (p.76) has both as
  // "all notes off", so this is the mkII's own.
  true    // monoPolyCutSound
};

}
