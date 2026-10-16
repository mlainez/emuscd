# XP engines: block rendering and voice threads

Follow-up to `rpi2-headroom-2026-09-25.md`, which found SC-88 and JV-1080
saturating one core of a Raspberry Pi 2 while the others sat idle. All the
changes below are lossless: every render is byte-identical to the previous
main at every thread count.

## What changed

1. **Block rendering** (`Synth::get_next_frames`). `get_next_frame()` drove
   the XP engines one frame per call: one MIDI-lock round trip, one pass of
   every effect and, on the JV-1080, one re-entry of the whole voice loop
   per sample. The block call renders up to the next queued MIDI event, so
   every event still lands on its frame. emusc-render, emuscd and
   emusc-winmidi use it.
2. **SC-88 voices a segment at a time.** Between two serviced control
   periods each voice renders its whole segment before the next voice; the
   scheduler stays serial, the frames are summed in active-slot order and
   the slots that ran out are freed in the frame loop's order. Only the
   stage-tap probe still renders frame by frame.
3. **Voice threads** (`Synth::set_render_threads`, `--threads` in emuscd and
   emusc-render, default one per usable CPU). Each SC-88 voice segment or
   JV-1080 voice block renders on a worker pool into storage of its own;
   the join sums them in the single-threaded order. `EMUSC_RENDER_THREADS`
   is off on Windows (the Win98 floor has no `std::thread`), where every
   count renders on the calling thread.
4. **Startup**: the wave-ROM line unscramble uses per-byte tables (it was a
   fifth of a 40 s SC-88 render's instructions).

## Raspberry Pi 2, offline render

Pi 2 B v1.1, 4x Cortex-A7 @ 900 MHz, `isolcpus=3` (3 usable cores),
`--rate 32000 --max-seconds 60 --tail 0`, real-time ratio as printed by
emusc-render, final build. Same compiler and flags for both binaries.

| device / song    | main  | 1 thread | 3 threads |
|------------------|-------|----------|-----------|
| SC-88 J_LAGOON   | 0.79x | 1.00x    | 2.20x     |
| SC-88 ARCADIA    | 0.60x | 0.76x    | 1.72x     |
| JV-1080 RISE     | 0.76x | 1.15x    | 2.57x     |
| JV-1080 ARCADIA  | 0.50x | 0.70x    | 1.81x     |

At 3 threads user time stays at the 1-thread figure while wall time falls,
and the 3-thread ratios did not move when the 1-thread SC-88 loop got
faster: they are now bound by the serial part (effects, scheduler, join).

md5 of every render matched main's on the Pi.

## Raspberry Pi 2, live emuscd

`emuscd --cpus 3,0,1,2 --rt 70 --stats --rate 32000 --block 512
--latency 48`, MIDI from `aplaymidi` on the Pi itself, din5 stopped. Worst
5-second window of render time per 16 ms block, as a share of the block:

| device / song           | avg | p99 | max | blocks over 100% |
|-------------------------|-----|-----|-----|------------------|
| SC-88 mystic (150 s)    | 53% | 70% | 83% | 0                |
| JV-1080 RISE (whole)    | 58% | 76% | 78% | 0                |

Three things got it there beyond the threads:
- Live MIDI lands at the start of a block, so a block's note-ons are one
  serial burst. JV-1080 note-ons decode waves; with a 32-entry cache RISE
  decoded 1408 times for 190 distinct waves. The cache now holds 512
  within a 64 MiB budget, and a SCHED_IDLE helper decodes a newly loaded
  patch's or kit's waves before its notes ask (JV-1080 max 113% -> 78%).
- `--rt` (SCHED_FIFO, memory locked): an SC-88 run waits for its slowest
  worker, and on the non-isolated cores workers were preempted ~2500 times
  a minute (SC-88 max 101-186% -> 83%).
- `--cpus 3,0,1,2` puts the main thread, which carries the serial work, on
  the core din5 keeps out of scheduling.

The SC-88 oscillator reads its four taps without per-tap cycle arithmetic
(two integer divisions a tap, library calls on an ARMv7-A core without a
divider), and the JV-1080 interpolator reads its taps directly away from
loop seams: -14% and -11% instructions on x86-64.

## Single-thread instruction counts (callgrind, x86-64, 40 s at 32 kHz)

| device / song  | main   | now, 1 thread |
|----------------|--------|---------------|
| SC-88 88jazz   | 14.0 G | 9.9 G         |
| JV-1080 RISE   | 19.6 G | 12.8 G        |

About 2.5 G of the SC-88's saving is the startup unscramble; the rest is the
segment loop, worth ~12% native wall time on one thread. The 32-bit Windows
build under Wine renders SC-88 and JV-1080 3-5% faster than main; its
executables import exactly what main's do, and libemusc.dll adds only three
libstdc++ operator new/delete entries.

## Verification

- Byte-identical WAVs against main: 8 SC-88 and 3 JV-1080 demos at 32, 44.1
  and 48 kHz at 1, 2 and 4 threads, plus SC-55 and JV-880 (103 renders),
  native x86-64; armhf under qemu-arm at 1 and 3 threads; mingw32 under Wine
  (SC-88, JV-1080, SC-55 at 32 and 44.1 kHz); on the Pi itself.
- ThreadSanitizer: no reports over threaded SC-88 and JV-1080 renders.
- `device_test` renders the SC-88 frame loop, the segment loop and a 4-thread
  pool side by side and compares bytes; it fails if the join's summation
  order is reversed. Reversing the order in which same-frame slot frees are
  replayed is not caught by it or by the corpus: it only becomes audible
  once the free list is exhausted down to those slots. The order is kept by
  construction.

## Not done

- The SC-88 effects chain (output FIR, reverb, chorus, delay; ~15% of a
  render) and its control-period scheduler stay serial, which bounds the
  speedup.
- At host rates other than 32 kHz the JV-1080 resampler still asks for one
  native frame at a time, so its voices render serially there.
- GP devices (SC-55, SC-55mkII, JV-880) are unchanged; they already run in
  real time on the Pi 2.
