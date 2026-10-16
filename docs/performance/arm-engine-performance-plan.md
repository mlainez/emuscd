# Portable ARM32/ARM64 Engine Performance Plan

## Goal and policy

Optimize the existing portable ARM32 and ARM64 builds without introducing
processor-specific artifacts or changing rendered audio.

- ARM32 retains the project's ARMv7 hard-float baseline. Optional NEON code
  must use runtime feature detection so the standard binary still runs on
  ARMv7 systems without NEON.
- ARM64 retains the baseline ARMv8-A target, where Advanced SIMD is mandatory.
- Every lossless change must preserve each architecture's existing
  floating-point behavior and produce byte-identical output before and after
  the change.
- This work stays independent of the oscillator, resampler, stereo-filter,
  effects and LTO work in `sse1-engine-performance-plan.md`.

Do not use blanket `-ffast-math`, reassociation, reciprocal estimates,
flush-to-zero, reduced precision or a processor-specific `-mcpu` setting.

## Build and toolchain changes

Strengthen the existing ARM toolchains rather than adding new build variants:

- ARM32: explicitly use `-march=armv7-a -mfpu=vfpv3-d16
  -mfloat-abi=hard -mtune=generic-armv7-a`.
- ARM64: explicitly use the portable `-march=armv8-a` baseline and generic
  tuning, retaining outlined atomics.
- Use `-O3 -pipe` and independently verify the value-preserving flags
  `-fno-math-errno -fno-trapping-math -fno-signaling-nans` on both targets.

Preserve the current contraction behavior on each architecture. The ARM32
baseline has no FMA, while the existing baseline ARM64 build can use FMA; do
not globally add `-ffp-contract=off`, because that would change ARM64's current
results rather than merely optimize them.

Add a build-time object inspection that confirms:

- ARM32 output does not accidentally require NEON, VFPv4 or a particular CPU.
- ARM64 output stays within the ARMv8-A baseline.
- No unsafe floating-point transformations have been enabled.

## Lossless engine improvements

### 1. Remove redundant buffer initialization

For GP, avoid clearing an active partial's temporary stereo and send blocks
when the oscillator, TVF and TVA overwrite every element. Explicitly clear
only finished or drain paths that actually require silence. Preserve the
current sample and voice accumulation order.

For XP, remove the three send-bus `memset` calls before
`engine_render_with_send()`: that function assigns every requested reverb,
chorus and delay frame before the effects read them.

### 2. Profile-guided optimization

Add an optional PGO workflow for both existing ARM builds. Train one combined
profile with representative dense GP and XP renders so neither engine is
optimized at the other's expense.

Evaluate PGO without LTO first. This keeps it independent of the LTO item in
the SSE1 plan and makes its contribution measurable by itself.

### 3. Cache XP decoded waveforms if profiling justifies it

If note-on decoding or allocation enters callback-critical time, add a lazy,
renderer-owned cache of immutable decoded PCM:

- Key entries by wave-bank identity and exact decoded address extent, never by
  a device name.
- Let voices reference cached storage instead of allocating, decoding and
  freeing identical PCM repeatedly.
- Give cached data renderer/device lifetime and release it during device
  destruction.
- Keep all layout and size facts in the injected `XpDeviceProfile`.

This is a generic XP improvement motivated by ARM cache and allocation costs;
it must not add SC-88 knowledge to common renderer code.

## SIMD policy

### ARM32

Keep scalar VFPv3 as the universal DSP implementation. Add runtime NEON
dispatch only for an exact integer or memory kernel that demonstrates a
material improvement. Do not use ARMv7 NEON for floating-point DSP in the
lossless tier: its subnormal handling can differ from scalar VFP and therefore
cannot be assumed byte-identical.

### ARM64

Allow the compiler to use mandatory Advanced SIMD wherever strict semantics
permit. Add explicit intrinsics only when profiling, vectorization reports and
assembly together show a missed hot loop, and only when the implementation
preserves arithmetic and accumulation order.

Do not duplicate the SSE1 plan's GP resampler, oscillator interpolation,
delay/chorus or paired stereo-filter work. Do not vectorize across voices when
that changes their accumulation order.

## Verification and acceptance gates

Cross-build and run the complete synthetic test suite under `qemu-arm` and
`qemu-aarch64`. Use physical ARM hardware for performance and live-audio
acceptance.

For every lossless change require:

- Byte-identical WAV output for SC-55, SC-55mkII, JV-880 and SC-88 reference
  material at 32, 44.1 and 48 kHz.
- Identical deterministic output, frame counts, event timing, voice allocation
  and voice lifetimes.
- All existing unit, integration, determinism and regression tests passing.
- No regression in the other engine path.
- A repeatable improvement in cycles per frame, callback duration or cache
  behavior on physical ARM hardware.

Profile GP and XP separately with `perf`, recording cycles per frame,
instructions, cache misses, branch misses, worst callback duration and
underruns. Test silence, long decays and near-zero filter tails explicitly to
detect subnormal-handling differences.

Measure each candidate independently before combining it with the others.
Retain only changes whose benefit survives repeated runs.

## Delivery order

1. Make the ARM32 and ARM64 baseline flags explicit and validate their output.
2. Remove the redundant GP and XP buffer initialization independently.
3. Add and measure the combined-engine PGO workflow.
4. Profile XP note-on decoding and add the decoded-wave cache only if it is a
   demonstrated callback or throughput cost.
5. Consider exact runtime-dispatched ARM32 NEON or ARM64 Advanced SIMD kernels
   only for remaining measured hotspots outside the SSE1 plan.

## Multi-core (out of scope here, but measured and worth its own plan)

Done for the XP engines; see `xp-block-and-threads-2026-10-16.md`. The text
below is the measurement that motivated it.

Real hardware measurement on a Raspberry Pi 2 v1.1 (4x Cortex-A7 @ 900 MHz),
recorded in `rpi2-headroom-2026-09-25.md`: both `emuscd` and `emusc-render`
are single-threaded (`Threads: 1` for the whole process lifetime, confirmed
via `/proc/<pid>/status` during a live run), and SC-88 and JV-1080 each
saturate their one core (measured 99-100% and 93-100% busy respectively on
real demo songs) while 2-3 other cores on the same board sit idle throughout.

This is a different kind of change from everything else in this plan: it is
not lossless in the sense above (accumulation order across threads has to be
fixed and documented to keep byte-identical output, not merely preserved by
construction), it is not ARM-specific, and it is not a small item alongside
the toolchain/PGO/cache work here. It deserves its own plan rather than a
line item in this one. What is worth carrying over from the measurement:

- The natural split is per-voice/per-partial (GP and XP already look
  structured that way from their symbol names), not per-effect: the global
  reverb/chorus bus is a small, inherently serial stage after mixdown.
- Nobody has profiled *where* SC-88/JV-1080 time actually goes yet. Amdahl's
  law means the achievable speedup depends entirely on how much of the
  measured cost is per-voice DSP versus the serial effects bus — that
  profiling has to happen before threading work is designed, not after.
- Whatever reduction order is chosen (e.g. per-thread partial buffers summed
  in a fixed index order) becomes a new invariant this plan's byte-identical
  gate would also need to cover once it lands.
- The payoff is real, though: SC-88 and JV-1080 are exactly the two devices
  this plan's own baseline hardware cannot hold realtime on today, and they
  are exactly the two a voice-parallel split would help most.

## Assumptions

- `cmake/toolchain-linux-armhf.cmake` and
  `cmake/toolchain-linux-arm64.cmake` remain the only ARM build interfaces.
- ARM32 remains compatible with ARMv7 hard-float systems without NEON.
- ARM64 remains compatible with baseline ARMv8-A rather than a particular
  Cortex implementation.
- No public synthesis API, runtime device selection or device-profile shape
  needs to change for the initial toolchain, buffer or PGO work.
