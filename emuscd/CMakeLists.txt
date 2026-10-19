cmake_minimum_required(VERSION 3.12)

project(emuscd LANGUAGES CXX)

if(WIN32)
  add_executable(emusc-winmidi main_winmidi.cc audio_out_dsound.cc audio_out_dsound.h)
  target_link_libraries(emusc-winmidi PRIVATE emusc_static win9x_compat winmm)
  set_target_properties(emusc-winmidi PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF)
  install(TARGETS emusc-winmidi DESTINATION bin)
else()
  find_package(ALSA REQUIRED)

  add_executable(emuscd main.cc)
  target_link_libraries(emuscd PRIVATE emusc_static ${ALSA_LIBRARIES})
  target_include_directories(emuscd PRIVATE ${ALSA_INCLUDE_DIRS})
  set_target_properties(emuscd PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF)
  install(TARGETS emuscd DESTINATION bin)
endif()

if(BUILD_TESTING)
  add_executable(midi_port_match_test tests/midi_port_match_test.cc)
  set_target_properties(midi_port_match_test PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF)
  target_compile_options(midi_port_match_test PRIVATE -UNDEBUG)
  add_test(NAME midi_port_match_test COMMAND midi_port_match_test)

  # power_on() on the SC-88: --reset gm enters GM mode. Needs the SC-88 ROMs.
  add_executable(power_on_test tests/power_on_test.cc)
  target_link_libraries(power_on_test PRIVATE emusc_static)
  set_target_properties(power_on_test PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF)
  target_compile_options(power_on_test PRIVATE -UNDEBUG)
  add_test(NAME power_on_test COMMAND power_on_test)
  set_property(TEST power_on_test PROPERTY SKIP_RETURN_CODE 77)
endif()
