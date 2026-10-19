/*
 * SPDX-License-Identifier: CC0-1.0
 *
 * This file consists entirely of AI-generated code without direct human
 * authorship and is dedicated to the public domain under CC0 1.0.
 *
 *  Resolves a --midi-in argument against the names of the MIDI input ports a
 *  system has. Kept free of any platform MIDI API so the matching rules can be
 *  tested on every platform, not only the one whose ports they pick from.
 */

#pragma once

#include <cctype>
#include <string>
#include <vector>

namespace emuscd {

struct MidiPortMatch {
  int index = -1;              // the chosen port, or -1 when none is chosen
  bool isIndex = false;        // the argument was a number, not a name
  std::vector<int> candidates; // every port a partial name matched, when
                               // more than one did
};

// Lower-cased, with every run of whitespace collapsed to one space and none
// at either end. Port names from virtual cable drivers are not consistent
// about either (MIDI Yoke's are "In From MIDI Yoke:  1", two spaces), and
// neither is something a user should have to reproduce exactly.
inline std::string normalize_port_name(const std::string &s) {
  std::string out;
  bool pendingSpace = false;
  for (unsigned char c : s) {
    if (std::isspace(c)) { pendingSpace = !out.empty(); continue; }
    if (pendingSpace) { out += ' '; pendingSpace = false; }
    out += static_cast<char>(std::tolower(c));
  }
  return out;
}

// An argument made only of digits is a port index, whether or not that index
// exists - the caller range-checks it, so a typo in a number is reported as
// one rather than searched for as a name. Anything else is a name: a port
// whose whole name matches is chosen first, then the one port whose name
// contains it. A partial name that several ports contain chooses none, and
// lists them, rather than silently taking the first.
inline MidiPortMatch match_midi_port(const std::string &want,
                                     const std::vector<std::string> &names) {
  MidiPortMatch m;
  if (want.empty()) return m;

  bool digits = true;
  for (unsigned char c : want)
    if (!std::isdigit(c)) { digits = false; break; }
  if (digits) {
    m.isIndex = true;
    if (want.size() <= 9) m.index = std::stoi(want);
    return m;
  }

  const std::string key = normalize_port_name(want);
  if (key.empty()) return m;
  for (size_t i = 0; i < names.size(); i++)
    if (normalize_port_name(names[i]) == key) { m.index = static_cast<int>(i); return m; }

  for (size_t i = 0; i < names.size(); i++)
    if (normalize_port_name(names[i]).find(key) != std::string::npos)
      m.candidates.push_back(static_cast<int>(i));
  if (m.candidates.size() == 1) {
    m.index = m.candidates[0];
    m.candidates.clear();
  }
  return m;
}

}  // namespace emuscd
