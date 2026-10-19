/* SPDX-License-Identifier: CC0-1.0 */

/* match_midi_port() decides which port emusc-winmidi opens for --midi-in.
 * The names below are the ones real virtual cable drivers report.
 */
#include "../midi_port_match.h"

#include <cassert>
#ifdef NDEBUG
#error "this test is assertion-driven; NDEBUG compiles it away"
#endif
#include <string>
#include <vector>

using namespace emuscd;

namespace {

const std::vector<std::string> PORTS = {
  "In From MIDI Yoke:  1",
  "In From MIDI Yoke:  2",
  "loopMIDI Port",
  "loopMIDI Port 1",
  "USB MIDI Interface",
};

int pick(const std::string &want) { return match_midi_port(want, PORTS).index; }

void test_index()
{
  assert(match_midi_port("0", PORTS).isIndex);
  assert(!match_midi_port("loopMIDI Port", PORTS).isIndex);
  assert(pick("0") == 0);
  assert(pick("4") == 4);
  // A number is never searched for as a name: an index past the end comes
  // back as that index, for the caller's range check to report.
  assert(pick("7") == 7);
  assert(match_midi_port("7", PORTS).candidates.empty());
  assert(pick("99999999999") == -1);
  assert(match_midi_port("99999999999", PORTS).isIndex);
}

void test_exact_name_wins_over_a_longer_one()
{
  // "loopMIDI Port" is also a part of "loopMIDI Port 1"; the whole name
  // must still pick the port it names, not report both.
  MidiPortMatch m = match_midi_port("loopMIDI Port", PORTS);
  assert(m.index == 2);
  assert(m.candidates.empty());
  assert(pick("loopMIDI Port 1") == 3);
}

void test_normalize()
{
  assert(normalize_port_name("  In From  MIDI Yoke:\t 1 ") == "in from midi yoke: 1");
  assert(normalize_port_name("") == "");
  assert(normalize_port_name(" \t ") == "");
  // Spaces between words are kept, so they still separate them.
  assert(pick("loopMIDI Port1") == -1);
}

void test_case_and_whitespace()
{
  assert(pick("loopmidi port") == 2);
  assert(pick("  LOOPMIDI   PORT ") == 2);
  // MIDI Yoke's own names carry two spaces before the number.
  assert(pick("In From MIDI Yoke: 2") == 1);
  assert(pick("in from midi yoke:  1") == 0);
}

void test_partial_name()
{
  assert(pick("usb") == 4);
  assert(pick("Yoke:  2") == 1);
}

void test_ambiguous_partial_name()
{
  MidiPortMatch m = match_midi_port("MIDI Yoke", PORTS);
  assert(m.index == -1);
  assert((m.candidates == std::vector<int>{0, 1}));

  m = match_midi_port("midi", PORTS);
  assert(m.index == -1);
  assert((m.candidates == std::vector<int>{0, 1, 2, 3, 4}));
}

void test_no_match()
{
  MidiPortMatch m = match_midi_port("Maple", PORTS);
  assert(m.index == -1);
  assert(m.candidates.empty());
  assert(pick("") == -1);
  assert(pick("   ") == -1);
  assert(match_midi_port("loopMIDI", {}).index == -1);
}

}  // namespace

int main()
{
  test_index();
  test_normalize();
  test_exact_name_wins_over_a_longer_one();
  test_case_and_whitespace();
  test_partial_name();
  test_ambiguous_partial_name();
  test_no_match();
  return 0;
}
