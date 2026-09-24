cmake_minimum_required(VERSION 3.18)
project(eposlib VERSION 0.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# eposlib - a C++ API for maxon EPOS4 controllers over CANopen.
#
# Plain CMake, on purpose. The library does not use ROS, so building it must
# not need ROS either: `cmake -S . -B build && cmake --build build` works on
# any Linux with Lely CANopen installed. It still builds under colcon, like
# any CMake package with a package.xml, and a sourced ROS environment is
# searched for its dependencies - but nothing requires one.
#
# Two libraries, and the split is the architecture:
#
#   eposlib::core      pure CiA 402 logic: state machine, object dictionary,
#                      units, configs, error tables, the cyclic state. No
#                      Lely, no CAN. Tested with no bus and no hardware.
#   eposlib::eposlib   the device API: CanBus, Epos4, Encoder. Adds the Lely
#                      transport on top of core.
# ---------------------------------------------------------------------------

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/eposlibDependencies.cmake)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Installed executables live in lib/eposlib; let them find the libraries next
# door and the Lely they were linked against, wherever it came from.
set(CMAKE_INSTALL_RPATH "$ORIGIN/..;$ORIGIN")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ---------------------------------------------------------------------------
# The libraries
# ---------------------------------------------------------------------------
add_library(eposlib_core SHARED
  src/core/Cia402StateMachine.cpp
  src/core/CyclicState.cpp
  src/Version.cpp
  src/signals/Enums.cpp
  src/signals/Errors.cpp
  src/signals/Identity.cpp
  src/configs/Configs.cpp
  src/configs/EncoderConfigs.cpp
)
target_include_directories(eposlib_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${EPOSLIB_UNITS_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(eposlib_core PUBLIC cxx_std_17)
set_target_properties(eposlib_core PROPERTIES EXPORT_NAME core)

add_library(eposlib SHARED
  src/CanBus.cpp
  src/hardware/Epos4.cpp
  src/hardware/Encoder.cpp
)
target_include_directories(eposlib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(eposlib PUBLIC cxx_std_17)
target_link_libraries(eposlib PUBLIC eposlib_core PkgConfig::LELY)

add_library(eposlib::core ALIAS eposlib_core)
add_library(eposlib::eposlib ALIAS eposlib)

# ---------------------------------------------------------------------------
# Tools and examples. Not part of the library: nothing links against them.
# ---------------------------------------------------------------------------

# A simulated EPOS4 that implements the CiA 402 transitions properly, so the
# whole stack can be developed and tested without hardware.
add_executable(epos4_sim tools/epos4_sim.cpp)
target_link_libraries(epos4_sim eposlib_core PkgConfig::LELY)

# 01-03 build up the Lely stack one piece at a time; api_demo uses the API.
foreach(example 01_event_loop 02_can_listen 03_master_boot)
  add_executable(${example} examples/${example}.cpp)
  target_link_libraries(${example} PkgConfig::LELY)
endforeach()
add_executable(api_demo examples/api_demo.cpp)
target_link_libraries(api_demo eposlib)

# ---------------------------------------------------------------------------
# The example CANopen network, turned into a master DCF at build time. The
# same function is installed for users to generate their own network's.
# ---------------------------------------------------------------------------
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/eposlibGenerateDcf.cmake)
eposlib_generate_dcf(epos4_network
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config)
eposlib_generate_dcf(sim_network
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/config)

# ---------------------------------------------------------------------------
# Install, and a package config so `find_package(eposlib)` works
# ---------------------------------------------------------------------------
install(TARGETS eposlib_core eposlib
  EXPORT eposlibTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS epos4_sim api_demo 01_event_loop 02_can_listen 03_master_boot
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

set(EPOSLIB_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
install(EXPORT eposlibTargets
  NAMESPACE eposlib::
  DESTINATION ${EPOSLIB_CMAKE_DIR}
)
configure_package_config_file(
  cmake/eposlibConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/eposlibConfig.cmake
  INSTALL_DESTINATION ${EPOSLIB_CMAKE_DIR}
)
# Pre-1.0: a minor version may break the API, so only the same minor matches.
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/eposlibConfigVersion.cmake
  COMPATIBILITY SameMinorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/eposlibConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/eposlibConfigVersion.cmake
  cmake/eposlibDependencies.cmake
  cmake/eposlibGenerateDcf.cmake
  DESTINATION ${EPOSLIB_CMAKE_DIR}
)

# Harmless without ROS; with it, `ros2 run eposlib epos4_sim` and rosdep can
# see the package.
install(FILES package.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/ament_index_marker "")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ament_index_marker
  DESTINATION ${CMAKE_INSTALL_DATADIR}/ament_index/resource_index/packages
  RENAME ${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Tests: no bus, no hardware, no ROS.
# ---------------------------------------------------------------------------
include(CTest)
if(BUILD_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)

  function(eposlib_add_test name library)
    add_executable(${name} test/${name}.cpp)
    target_link_libraries(${name} ${library} GTest::gtest_main)
    gtest_discover_tests(${name})
  endfunction()

  eposlib_add_test(test_cia402_state_machine eposlib_core)
  eposlib_add_test(test_errors eposlib_core)
  eposlib_add_test(test_configs eposlib_core)
  eposlib_add_test(test_unit_conversion eposlib_core)
  eposlib_add_test(test_digital_inputs eposlib_core)
  eposlib_add_test(test_status_signal eposlib_core)
  eposlib_add_test(test_identity eposlib_core)
  eposlib_add_test(test_cyclic_state eposlib_core)
  # These two link the device library but open no socket: they exercise
  # what a device allows before CanBus::Start().
  eposlib_add_test(test_encoder_configs eposlib)
  eposlib_add_test(test_device_lifecycle eposlib)

  # Code style, when the formatter is available (it comes with ROS). Pointed
  # at the source directories explicitly: run over the build tree it reports
  # CMake's own generated files.
  find_program(AMENT_UNCRUSTIFY ament_uncrustify)
  if(AMENT_UNCRUSTIFY)
    add_test(NAME uncrustify
      COMMAND ${AMENT_UNCRUSTIFY} include src test tools examples
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()
endif()
