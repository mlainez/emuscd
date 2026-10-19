# emuscd — Roland Sound Canvas Daemon

This directory builds two realtime MIDI-in/audio-out front ends for libEmuSC:
**emuscd** on Linux (ALSA) and **emusc-winmidi** on Windows (WinMM). Only one
of the two is built for a given target - see the [top-level
README](../README.md#building) for build and cross-compilation instructions.

## Supported Devices

- `sc55` — Roland Sound Canvas SC-55
- `sc55mkii` — Roland Sound Canvas SC-55mkII
- `sc88` — Roland Sound Canvas SC-88 (default)
- `jv880` — Roland JV-880

## ROM Files

Both binaries need Roland ROM files for the devices they emulate. Set
`$EMUSCD_ROM_DIR` to the directory holding them, or pass `--rom-dir` instead
(it overrides the environment variable when both are given):

- `sc55_control.bin`, `sc55_cpu.bin`, `sc55_waverom{1,2,3}.bin` (SC-55)
- `sc55mkii_control.bin`, `sc55mkii_cpu.bin`, `sc55mkii_waverom{1,2}.bin` (SC-55mkII)
- `sc88_control.bin`, `sc88_waverom{1,2,3,4}.bin` (SC-88; no separate CPU ROM)
- `jv880_control.bin`, `jv880_waverom{1,2}.bin` (JV-880)
- `jv1080_control.bin`, `jv1080_waverom{1,2,3,4}.bin` (JV-1080)

See the top-level [README.md](../README.md) for exact file sizes and
SHA1/MD5 hashes of known-good ROM dumps.

If `$EMUSCD_ROM_DIR` is unset, both use `roms` relative to the working
directory if it exists. Otherwise `emuscd` uses `/usr/share/emuscd/roms`, and
`emusc-winmidi`, which has no such location on Windows, stays with `.\roms`.
`emusc-render` follows the same rule.

---

## Linux (emuscd)

emuscd creates its own ALSA sequencer MIDI port.

### Usage

```bash
emuscd --device sc88 --name "My Synth"
```

### Options

```
--device NAME       Device to emulate (default: sc88); sc55, sc55mkii, sc88, jv880, jv1080
--name NAME         ALSA MIDI port name (default: emuscd)
--pcm DEVICE        ALSA PCM output device (default: default)
--list-pcm          List ALSA PCM devices and exit
--rom-dir DIR       Directory holding device ROM files (default:
                    $EMUSCD_ROM_DIR, else ./roms if it exists,
                    else /usr/share/emuscd/roms)
--rate HZ           Requested audio sample rate (default: 48000)
--latency MS        Requested output buffer size in ms (default: 20)
--block N           Audio frames per ALSA write (default: 256)
--threads N         Threads the SC-88 and JV-1080 render their voices on,
                    this one included; 0 is one per CPU the process may use
                    (default: 0). The audio is the same at every count
--cpus LIST         Pin the render threads one per CPU in LIST (e.g. 0-3):
                    this thread on the first, the pool's from the last
                    down, so a core kept out of scheduling (isolcpus)
                    still gets one. Without --threads, one per CPU listed
--stats             Every 5 s on stderr: render time per block as a
                    share of its playing time (avg, p99, max)
--gain-db DB        Output gain in dB, linear multiplier 10^(DB/20) applied
                    to the final samples (default: 0, no change - a
                    listening-convenience knob only, see emusc-render's
                    own README for details)
--help              Show full help
```

### Connecting MIDI

```bash
aconnect -l                          # List ALSA ports
aconnect "Your Keyboard" "emuscd"    # Connect your keyboard to emuscd
aplaymidi -p 'emuscd' song.mid       # Play a MIDI file
```

If JACK is running, emuscd uses ALSA's JACK bridge automatically - connect it
from Qjackctl or your JACK patchbay the same way.

### Audio output

Audio goes to `--pcm`'s device, `default` if not given - usually your
speakers, or wherever ALSA/PipeWire/JACK routes `default`. Route to a
specific device instead:

```bash
emuscd --pcm hw:1
```

`--list-pcm` shows what's available; ALSA's own `ALSA_CARD` environment
variable still works too.

### Audio output API

DirectSound is the default (`--audio-api auto`): one looping secondary
buffer, kept `--latency` ms ahead of the play cursor (more if the driver or
the scheduler's timer granularity needs it), with the primary buffer set to
the stream's own rate so the mixer does not resample. If DirectSound cannot be
opened, the reason is printed and WinMM `waveOut` is used instead.
`--audio-api winmm` forces WinMM; `--audio-api dsound` forces DirectSound and
exits if it fails. `dsound.dll` is loaded at runtime rather than linked, so
the executable's DLL imports and its Windows 98 floor are unchanged. Underruns
are reported on the console; raising `--latency` is the remedy.

### Runtime device switching

While emuscd is running, type a device name at its stdin to switch, or
`quit`/`exit` to stop it:

```
$ emuscd --device sc55
ALSA MIDI input opened: emuscd
ALSA audio output 'default' initialized at 48000 Hz, ~20 ms buffer
Loaded device: sc55 (SC-55 v1.21)
emuscd daemon running (device: sc55). Type a device name to switch, or 'quit' to exit.
sc88
Loaded device: sc88 (SC-88 v...)
```

---

## Windows (emusc-winmidi)

emusc-winmidi is emuscd's Windows counterpart, built on WinMM instead of
ALSA. Unlike emuscd it does not create its own MIDI port - WinMM's `midiIn`
API only opens an *existing* one - so route MIDI into it with a virtual MIDI
cable (loopMIDI on Windows 7+, Maple Virtual MIDI Cable on 95/98/ME/2000/XP),
then point `--midi-in` at that port.

### Usage

```bat
emusc-winmidi.exe --list-midi-in
emusc-winmidi.exe --device sc88 --midi-in 1
emusc-winmidi.exe --device sc88 --midi-in "loopMIDI Port"
```

### Options

```
--device NAME       Device to emulate (default: sc88); sc55, sc55mkii, sc88, jv880, jv1080
--midi-in PORT       MIDI input device: index, or name or part of one (default: 0)
--audio-api API      Audio output API: auto, winmm or dsound (default: auto,
                     DirectSound with WinMM fallback)
--wave-out N         WinMM wave output device index (default: system default)
--dsound-out N       DirectSound output device index (default: system default)
--list-midi-in       List MIDI input devices and exit
--list-wave-out      List WinMM wave output devices and exit
--list-dsound-out    List DirectSound output devices and exit
--rom-dir DIR        Directory holding device ROM files (default:
                      %EMUSCD_ROM_DIR%, or .\roms if unset)
--rate HZ            Audio sample rate (default: 48000)
--block N            Audio frames per wave buffer (default: 256)
--latency MS         Requested output buffer size in ms (default: 20)
--gain-db DB         Output gain in dB, linear multiplier 10^(DB/20) applied
                     to the final samples (default: 0, no change - a
                     listening-convenience knob only, see emusc-render's
                     own README for details)
--help               Show full help
```

### Audio output API

DirectSound is the default (`--audio-api auto`): one looping secondary
buffer, kept `--latency` ms ahead of the play cursor (more if the driver or
the scheduler's timer granularity needs it), with the primary buffer set to
the stream's own rate so the mixer does not resample. If DirectSound cannot be
opened, the reason is printed and WinMM `waveOut` is used instead.
`--audio-api winmm` forces WinMM; `--audio-api dsound` forces DirectSound and
exits if it fails. `dsound.dll` is loaded at runtime rather than linked, so
the executable's DLL imports and its Windows 98 floor are unchanged. Underruns
are reported on the console; raising `--latency` is the remedy.

### Choosing --latency

`--latency` trades response time for robustness. A deeper buffer rides out
transient OS contention - dragging a window, disk I/O, a game loading a level -
without an audible dropout, but every incoming MIDI event also sounds that much
later, which matters for live playing and game audio. There is no universally
right value: it depends on the CPU, the sound card and its driver's own
buffering, and how much else competes for the CPU during a session. Start at
the default, raise it until dropouts stop on your machine, and stop at the
lowest value that plays cleanly.

Every ~2 seconds a `CPU utilization N%` line reports the share of wall-clock
time spent synthesizing audio, excluding time spent waiting on the audio
device. Lower means more headroom; a figure approaching 100% means the machine
is close to not keeping up.

### Runtime device switching

Same mechanism as emuscd: type a device name at the console it's running in
to switch, or `quit`/`exit` to stop it (Ctrl+C also works).
