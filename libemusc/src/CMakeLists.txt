cmake_minimum_required(VERSION 3.8...3.30)

configure_file(config.h.in config.h)

# Only worker_pool.cc reads EMUSC_RENDER_THREADS, and the pool's class layout
# does not depend on it, so it is set on that one file rather than carried by
# every target that compiles it. Every target this directory builds links the
# thread library, since every XP target compiles the pool.
if(EMUSC_RENDER_THREADS)
  set(THREADS_PREFER_PTHREAD_FLAG ON)
  find_package(Threads REQUIRED)
  set_source_files_properties(engines/xp/common/worker_pool.cc PROPERTIES
    COMPILE_DEFINITIONS EMUSC_RENDER_THREADS=1)
  link_libraries(Threads::Threads)
endif()

set(EMUSC_SOURCES
  control_rom.cc
  control_rom.h
  devices/common/rom_signature.h
  devices/jv1080.cc
  devices/jv880.cc
  devices/registry.cc
  devices/sc55.cc
  devices/sc55mkii.cc
  devices/sc88.cc
  params.h
  settings.cc
  settings.h
  slew_calc.h
  synth.cc
  synth.h
  wave_rom.cc
  wave_rom.h
  engines/gp/analog_stage.cc
  engines/gp/analog_stage.h
  engines/gp/chorus.cc
  engines/gp/chorus.h
  engines/gp/devices/jv880.cc
  engines/gp/devices/sc55.cc
  engines/gp/devices/sc55mkii.cc
  engines/gp/devices/sound_canvas_default.cc
  engines/gp/envelope.cc
  engines/gp/envelope.h
  engines/gp/note.cc
  engines/gp/note.h
  engines/gp/part.cc
  engines/gp/part.h
  engines/gp/partial.cc
  engines/gp/partial.h
  engines/gp/pitch.cc
  engines/gp/pitch.h
  engines/gp/random.cc
  engines/gp/random.h
  engines/gp/resampler.cc
  engines/gp/resampler.h
  engines/gp/reverb.cc
  engines/gp/reverb.h
  engines/gp/svf.cc
  engines/gp/svf.h
  engines/gp/system_effects.cc
  engines/gp/system_effects.h
  engines/gp/tva.cc
  engines/gp/tva.h
  engines/gp/tvf.cc
  engines/gp/tvf.h
  engines/gp/tvf_cents_ratio_law.cc
  engines/gp/tvf_cents_ratio_law.h
  engines/gp/tvf_indexed_law.cc
  engines/gp/tvf_indexed_law.h
  engines/gp/tvf_law.cc
  engines/gp/tvf_law.h
  engines/gp/wave_generator.cc
  engines/gp/wave_generator.h
  engines/gp/wave_oscillator.cc
  engines/gp/wave_oscillator.h
  engines/xp/device.cc
  engines/xp/device.h
  engines/xp/devices/jv1080.cc
  engines/xp/devices/jv1080.h
  engines/xp/devices/jv1080_engine.cc
  engines/xp/devices/jv1080_resample.cc
  engines/xp/devices/jv1080_resample.h
  engines/xp/devices/jv1080_voice.cc
  engines/xp/devices/profile.h
  ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc
  engines/xp/devices/sc88.h
  engines/xp/engine.cc
  engines/xp/engine.h
  engines/xp/oscillator.cc
  engines/xp/oscillator.h
  engines/xp/output.cc
  engines/xp/output.h
  engines/xp/lfo.cc
  engines/xp/lfo.h
  engines/xp/packed_rom.cc
  engines/xp/packed_rom.h
  engines/xp/pan.cc
  engines/xp/pan.h
  engines/xp/pitch.cc
  engines/xp/pitch.h
  engines/xp/chorus.cc
  engines/xp/chorus.h
  engines/xp/delay.cc
  engines/xp/delay.h
  engines/xp/drive.cc
  engines/xp/drive.h
  engines/xp/dynamics.cc
  engines/xp/dynamics.h
  engines/xp/enhancer.cc
  engines/xp/enhancer.h
  engines/xp/spectrum.cc
  engines/xp/spectrum.h
  engines/xp/phaser.cc
  engines/xp/phaser.h
  engines/xp/rotary.cc
  engines/xp/rotary.h
  engines/xp/stereo_eq.cc
  engines/xp/stereo_eq.h
  engines/xp/efx.cc
  engines/xp/efx.h
  engines/xp/eq.cc
  engines/xp/eq.h
  engines/xp/renderer.cc
  engines/xp/renderer.h
  engines/xp/reverb.cc
  engines/xp/reverb.h
  engines/xp/rom.cc
  engines/xp/rom.h
  engines/xp/tva.cc
  engines/xp/tva.h
  engines/xp/tvf.cc
  engines/xp/tvf.h
  engines/xp/wave.cc
  engines/xp/wave.h
  engines/xp/wave_cache.cc
  engines/xp/wave_cache.h
  engines/xp/common/worker_pool.cc
  engines/xp/common/worker_pool.h)

# A plain OBJECT library, not folded into EMUSC_SOURCES: a static archive
# only pulls in a member when something already needs one of its symbols at
# the point the archive is scanned, and nothing does until libstdc++'s own
# guard.o is linked in afterwards - by which point ld has already moved past
# emusc_static.a and won't come back for it. Object libraries are always
# passed to the final link as loose .o files instead, sidestepping that.
#
# Only emusc-render and emusc-winmidi (see their own CMakeLists.txt) link
# this in: emusc/emusc_static's own sources need none of it, confirmed by
# checking the built libemusc.dll's own import table directly.
add_library(win9x_compat OBJECT win9x_compat.cc)
set_target_properties(win9x_compat PROPERTIES CXX_EXTENSIONS OFF)

add_library(emusc SHARED ${EMUSC_SOURCES})
add_library(emusc_static STATIC ${EMUSC_SOURCES})

target_compile_features(emusc PUBLIC cxx_std_17)
target_compile_features(emusc PRIVATE c_std_99)
target_compile_features(emusc_static PUBLIC cxx_std_17)
target_compile_features(emusc_static PRIVATE c_std_99)

# The DSP-tier macro (emusc_dsp_tier, ../CMakeLists.txt) goes on every target
# that compiles engine sources, not just the libraries: the tools and tests
# below build the same .cc files directly.
target_link_libraries(emusc PRIVATE emusc_dsp_tier)
target_link_libraries(emusc_static PRIVATE emusc_dsp_tier)
if(EMUSC_RENDER_THREADS)
  target_link_libraries(emusc_static PUBLIC Threads::Threads)
endif()

target_include_directories(emusc PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")
target_include_directories(emusc_static PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")

set_target_properties(emusc PROPERTIES CXX_EXTENSIONS OFF VERSION ${CMAKE_PROJECT_VERSION} SOVERSION ${CMAKE_PROJECT_VERSION_MAJOR})
set_target_properties(emusc_static PROPERTIES CXX_EXTENSIONS OFF VERSION ${CMAKE_PROJECT_VERSION})

# A toolchain whose output has to stay inside an ISA or floating-point
# baseline names the check and the binutils that can read it in
# EMUSC_BASELINE_CHECK/EMUSC_BASELINE_CHECK_ARGS; this runs it over the
# library's own objects as part of the build, so a flag that widens the ISA
# or relaxes the floating-point model fails the build instead of shipping.
# Objects rather than the linked library, because a statically linked libgcc
# carries runtime-guarded instructions of its own that say nothing about what
# this project compiled. Nothing here knows which architecture it is.
if(EMUSC_BASELINE_CHECK)
  add_custom_command(TARGET emusc_static POST_BUILD
    COMMAND "${EMUSC_BASELINE_CHECK}" ${EMUSC_BASELINE_CHECK_ARGS}
            $<TARGET_OBJECTS:emusc_static>
    COMMAND_EXPAND_LISTS VERBATIM
    COMMENT "Checking objects against the toolchain's ISA/FP baseline")
endif()

# The JV-1080's profile names its own voice engine, so any target that
# links the profile - which is every target that links engines/xp/rom.cc,
# since that is where the engine identifies its devices - links the engine
# and the descriptor-packed reader behind it too. One list, so a new file
# is wired in once.
set(XP_JV1080_SOURCES
  engines/xp/devices/jv1080.cc
  engines/xp/devices/jv1080_engine.cc
  engines/xp/devices/jv1080_resample.cc
  engines/xp/devices/jv1080_voice.cc
  engines/xp/chorus.cc
  engines/xp/drive.cc
  engines/xp/dynamics.cc
  engines/xp/efx.cc
  engines/xp/packed_rom.cc
  engines/xp/reverb.cc
  engines/xp/enhancer.cc
  engines/xp/phaser.cc
  engines/xp/rotary.cc
  engines/xp/spectrum.cc
  engines/xp/stereo_eq.cc
  engines/xp/wave.cc
  engines/xp/wave_cache.cc
  engines/xp/common/worker_pool.cc)

# XP-engine specific debug/analysis tools. Distinct from the multi-device
# emusc-render in libemusc/tools/: that one renders through EmuSC::Synth's
# public API, these link the xp_* engine sources directly, because the
# values they report are internal to the engine and no measurement of the
# rendered audio can separate them.
add_executable(xp_tvf_probe ../tools/xp_tvf_probe.cc
  ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc
  engines/xp/rom.cc engines/xp/tvf.cc engines/xp/tva.cc)
target_include_directories(xp_tvf_probe PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(xp_tvf_probe PRIVATE emusc_dsp_tier)
if(NOT MSVC)
  target_link_libraries(xp_tvf_probe PRIVATE m)
endif()

# Probe for the descriptor-packed ROM layout: the descramble's own output,
# a factory patch's whole resolution chain, and a song whose SysEx programs
# the machine itself. Links the engine sources directly for the same reason
# the two above do - what it reports is internal to the engine.
add_executable(xp_packed_probe ../tools/xp_packed_probe.cc ../tools/smf.cc
  ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc
  engines/xp/rom.cc)
target_include_directories(xp_packed_probe PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/../tools")
if(NOT MSVC)
  target_link_libraries(xp_packed_probe PRIVATE m)
endif()

add_executable(xp_dump_samples ../tools/xp_dump_samples.cc
  ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc
  engines/xp/oscillator.cc engines/xp/rom.cc)
target_include_directories(xp_dump_samples PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(xp_dump_samples PRIVATE emusc_dsp_tier)
if(NOT MSVC)
  target_link_libraries(xp_dump_samples PRIVATE m)
endif()

if(BUILD_TESTING)
  # Every test here is assertion-driven: the assertions are the test, not a
  # decoration on it. NDEBUG therefore compiles the bodies away and leaves a
  # suite that passes while testing nothing - and two of the eight went
  # further, freeing an engine that the stripped assertions never initialised.
  # -UNDEBUG is applied last, so it survives whatever the build type asks for.
  set(XP_VOICE_SOURCES
    ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc
    engines/xp/engine.cc engines/xp/lfo.cc engines/xp/oscillator.cc
    engines/xp/pan.cc engines/xp/pitch.cc
    engines/xp/renderer.cc engines/xp/reverb.cc
    engines/xp/rom.cc engines/xp/tva.cc engines/xp/tvf.cc
    )

  function(xp_test name)
    add_executable(${name} ../tests/${name}.cc ${ARGN})
    target_include_directories(${name} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${name} PRIVATE emusc_dsp_tier)
    if(NOT MSVC)
      target_link_libraries(${name} PRIVATE m)
    endif()
    if(MSVC)
      target_compile_options(${name} PRIVATE /UNDEBUG)
    else()
      target_compile_options(${name} PRIVATE -UNDEBUG)
    endif()
    add_test(NAME ${name} COMMAND ${name})
  endfunction()

  xp_test(xp_wave_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc)
  xp_test(xp_wave_cache_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc)
  xp_test(xp_oscillator_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/oscillator.cc)
  xp_test(xp_pan_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/pan.cc)
  xp_test(xp_reverb_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/reverb.cc engines/xp/rom.cc)
  xp_test(xp_chorus_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/chorus.cc engines/xp/reverb.cc engines/xp/rom.cc)
  xp_test(xp_delay_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/delay.cc engines/xp/reverb.cc engines/xp/rom.cc)
  xp_test(xp_eq_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/eq.cc engines/xp/rom.cc)
  xp_test(xp_output_test engines/xp/output.cc)
  xp_test(xp_worker_pool_test engines/xp/common/worker_pool.cc)

  add_executable(gp_random_test ../tests/gp_random_test.cc engines/gp/random.cc)
  target_include_directories(gp_random_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  if(MSVC)
    target_compile_options(gp_random_test PRIVATE /UNDEBUG)
  else()
    target_compile_options(gp_random_test PRIVATE -UNDEBUG)
  endif()
  add_test(NAME gp_random_test COMMAND gp_random_test)

  # The filter laws take their tables, profile data and part parameters as
  # arguments, so the test builds them itself and needs no ROM.
  add_executable(gp_tvf_test ../tests/gp_tvf_test.cc
    engines/gp/tvf_law.cc engines/gp/tvf_indexed_law.cc
    engines/gp/tvf_cents_ratio_law.cc engines/gp/envelope.cc engines/gp/svf.cc
    engines/gp/devices/jv880.cc engines/gp/devices/sc55.cc
    engines/gp/devices/sc55mkii.cc engines/gp/devices/sound_canvas_default.cc)
  target_include_directories(gp_tvf_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(gp_tvf_test PRIVATE emusc_dsp_tier)
  if(MSVC)
    target_compile_options(gp_tvf_test PRIVATE /UNDEBUG)
  else()
    target_compile_options(gp_tvf_test PRIVATE -UNDEBUG)
  endif()
  add_test(NAME gp_tvf_test COMMAND gp_tvf_test)
  # This one checks the held ROM's own coefficient records, so it needs
  # the ROM; without it the test reports skipped rather than passing
  # while checking nothing. The path is read from SC88_CONTROL_ROM by the
  # test itself, at run time, so ctest's own environment carries it
  # whatever the tree was configured with.
  set_property(TEST xp_eq_test PROPERTY SKIP_RETURN_CODE 77)
  xp_test(xp_drum_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/rom.cc)
  # The descriptor-packed ROM layout, checked against the held image of a
  # device that uses it; skipped without JV1080_CONTROL_ROM, since a
  # synthetic image would only re-check this test's own arithmetic.
  xp_test(xp_packed_rom_test ${XP_JV1080_SOURCES}
    engines/xp/devices/sc88.cc engines/xp/rom.cc)
  set_property(TEST xp_packed_rom_test PROPERTY SKIP_RETURN_CODE 77)
  # The insert effect's program bank: geometry, the type table and where
  # each program touches delay memory. No opcode is interpreted - U-R5-02
  # is silicon - so this checks shape, not sound.
  xp_test(xp_efx_test ${XP_JV1080_SOURCES}
    engines/xp/devices/sc88.cc engines/xp/rom.cc)
  set_property(TEST xp_efx_test PROPERTY SKIP_RETURN_CODE 77)
  # COMPRESSOR and LIMITER's level law against the machine's own steady
  # levels, run on the effect alone; skipped without JV1080_CONTROL_ROM.
  xp_test(xp_dynamics_test ${XP_JV1080_SOURCES}
    engines/xp/devices/sc88.cc engines/xp/rom.cc)
  set_property(TEST xp_dynamics_test PROPERTY SKIP_RETURN_CODE 77)
  # ROTARY on its own: binding, crossover, speed law and ramp, tap sides.
  xp_test(xp_rotary_test ${XP_JV1080_SOURCES}
    engines/xp/devices/sc88.cc engines/xp/rom.cc)
  set_property(TEST xp_rotary_test PROPERTY SKIP_RETURN_CODE 77)
  # The filter envelope's measured tables and laws, checked as arithmetic:
  # no ROM is involved, so this one never reports skipped.
  xp_test(xp_filter_env_test ${XP_JV1080_SOURCES}
    engines/xp/devices/sc88.cc engines/xp/rom.cc)
  xp_test(xp_lfo_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/lfo.cc engines/xp/rom.cc)
  xp_test(xp_pitch_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/pitch.cc engines/xp/rom.cc)
  xp_test(xp_tvf_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/rom.cc engines/xp/tvf.cc engines/xp/tva.cc)
  xp_test(xp_tva_test ${XP_JV1080_SOURCES} engines/xp/devices/sc88.cc engines/xp/tva.cc engines/xp/rom.cc)
  # The interpolation word's identity is a property of the SC-88's own
  # curve and rate tables, so the sweep needs the ROM; without it the test
  # reports skipped rather than passing while checking only the layout.
  # The path is read from SC88_CONTROL_ROM by the test itself, at run time.
  set_property(TEST xp_tva_test PROPERTY SKIP_RETURN_CODE 77)
  # The four that build a voice share one source list, so a new module is
  # wired in once instead of four times.
  xp_test(xp_rom_test ${XP_VOICE_SOURCES})
  # Each also sweeps the held ROM(s) end to end (every melodic tone for
  # xp_rom_test; a real note-on and render for xp_renderer_test and
  # device_test), gated on the same environment variables as
  # xp_tva_test above; without them the sweep is skipped and only the
  # synthetic fixtures run.
  set_property(TEST xp_rom_test PROPERTY SKIP_RETURN_CODE 77)
  xp_test(xp_renderer_test ${XP_VOICE_SOURCES})
  set_property(TEST xp_renderer_test PROPERTY SKIP_RETURN_CODE 77)
  xp_test(xp_engine_test ${XP_VOICE_SOURCES})

  # EmuSC::Xp::Device and EmuSC::Xp::ChannelState (engines/xp/device.h) never
  # had a plain C compat struct or extern "C" shim layer - synth.cc was their
  # only caller outside this test, and it has always held them as an opaque
  # pointer - so device_test needs its own source list rather than
  # XP_VOICE_SOURCES plus xp_test()'s single shared-name convention.
  add_executable(device_test ../tests/device_test.cc engines/xp/device.cc
    engines/xp/chorus.cc engines/xp/delay.cc engines/xp/eq.cc
    engines/xp/output.cc ${XP_VOICE_SOURCES})
  target_include_directories(device_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(device_test PRIVATE emusc_dsp_tier)
  if(NOT MSVC)
    target_link_libraries(device_test PRIVATE m)
  endif()
  if(MSVC)
    target_compile_options(device_test PRIVATE /UNDEBUG)
  else()
    target_compile_options(device_test PRIVATE -UNDEBUG)
  endif()
  add_test(NAME device_test COMMAND device_test)

  # Which patch a JV-1080 program change loads, and what GM System On
  # leaves, checked by rendering the same note through two routes. Needs
  # JV1080_CONTROL_ROM and JV1080_WAVE_ROMS; skipped without them.
  add_executable(xp_jv1080_program_test ../tests/xp_jv1080_program_test.cc
    engines/xp/device.cc engines/xp/chorus.cc engines/xp/delay.cc
    engines/xp/eq.cc engines/xp/output.cc ${XP_VOICE_SOURCES})
  target_include_directories(xp_jv1080_program_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}")
  if(NOT MSVC)
    target_link_libraries(xp_jv1080_program_test PRIVATE m)
  endif()
  if(MSVC)
    target_compile_options(xp_jv1080_program_test PRIVATE /UNDEBUG)
  else()
    target_compile_options(xp_jv1080_program_test PRIVATE -UNDEBUG)
  endif()
  add_test(NAME xp_jv1080_program_test COMMAND xp_jv1080_program_test)
  set_property(TEST xp_jv1080_program_test PROPERTY SKIP_RETURN_CODE 77)
  set_property(TEST device_test PROPERTY SKIP_RETURN_CODE 77)

  # Synth::reset() is reached through the public C++ API, so this one links the
  # library rather than the engine sources. ControlRom and WaveRom read files
  # and have no public constructor that takes bytes, so it needs the SC-88's
  # own ROM images and reports skipped without them. The paths are read from
  # the environment by the test itself, at run time, so ctest's own environment
  # carries them whatever the tree was configured with.
  add_executable(synth_reset_test ../tests/synth_reset_test.cc)
  target_include_directories(synth_reset_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(synth_reset_test PRIVATE emusc_static)
  if(MSVC)
    target_compile_options(synth_reset_test PRIVATE /UNDEBUG)
  else()
    target_compile_options(synth_reset_test PRIVATE -UNDEBUG)
  endif()
  set_target_properties(synth_reset_test PROPERTIES CXX_EXTENSIONS OFF)
  add_test(NAME synth_reset_test COMMAND synth_reset_test)
  set_property(TEST synth_reset_test PROPERTY SKIP_RETURN_CODE 77)

  # The same public API, for the branch of reset() that puts a device into the
  # Performance its own control channel would select. It needs that device's
  # ROMs, read from the environment at run time, and reports skipped without
  # them.
  add_executable(synth_reset_performance_test ../tests/synth_reset_performance_test.cc)
  target_include_directories(synth_reset_performance_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(synth_reset_performance_test PRIVATE emusc_static)
  if(MSVC)
    target_compile_options(synth_reset_performance_test PRIVATE /UNDEBUG)
  else()
    target_compile_options(synth_reset_performance_test PRIVATE -UNDEBUG)
  endif()
  set_target_properties(synth_reset_performance_test PROPERTIES CXX_EXTENSIONS OFF)
  add_test(NAME synth_reset_performance_test COMMAND synth_reset_performance_test)
  set_property(TEST synth_reset_performance_test PROPERTY SKIP_RETURN_CODE 77)

  # A receiver that takes Roland SysEx only ignores GM System On, universal
  # Master Volume and the GS Reset. Needs the JV-880 ROMs, as above.
  add_executable(synth_rx_roland_only_test ../tests/synth_rx_roland_only_test.cc)
  target_include_directories(synth_rx_roland_only_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(synth_rx_roland_only_test PRIVATE emusc_static)
  if(MSVC)
    target_compile_options(synth_rx_roland_only_test PRIVATE /UNDEBUG)
  else()
    target_compile_options(synth_rx_roland_only_test PRIVATE -UNDEBUG)
  endif()
  set_target_properties(synth_rx_roland_only_test PROPERTIES CXX_EXTENSIONS OFF)
  add_test(NAME synth_rx_roland_only_test COMMAND synth_rx_roland_only_test)
  set_property(TEST synth_rx_roland_only_test PROPERTY SKIP_RETURN_CODE 77)
endif()
