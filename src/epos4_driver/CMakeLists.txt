cmake_minimum_required(VERSION 3.8)
project(epos4_driver VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(robot_units REQUIRED)

# ---------------------------------------------------------------------------
# The library.
#
# Split in two on purpose:
#
#   epos4_core   pure CiA 402 logic. No Lely, no CAN, no ROS. Testable on a
#                laptop with no hardware, which is why the state machine has
#                18 tests that need no bus.
#   epos4        the device API. Adds the Lely transport on top of the core.
#
# Nothing here depends on ROS. That boundary is what makes this a library
# rather than a node with functions in it.
# ---------------------------------------------------------------------------
add_library(epos4_core SHARED
  src/core/Cia402StateMachine.cpp
  src/core/CyclicState.cpp
  src/Version.cpp
  src/signals/Enums.cpp
  src/signals/Errors.cpp
  src/configs/Configs.cpp
  src/configs/EncoderConfigs.cpp
)
target_include_directories(epos4_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(epos4_core PUBLIC cxx_std_17)
ament_target_dependencies(epos4_core robot_units)

add_library(${PROJECT_NAME} SHARED
  src/CanBus.cpp
  src/hardware/Epos4.cpp
  src/hardware/Encoder.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} epos4_core)
ament_target_dependencies(${PROJECT_NAME} lely_core_libraries robot_units)

# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------

# A simulated EPOS4 that implements the CiA 402 transitions properly, so the
# whole stack can be developed and tested without hardware.
add_executable(epos4_sim tools/epos4_sim.cpp)
target_link_libraries(epos4_sim epos4_core)
ament_target_dependencies(epos4_sim lely_core_libraries)

# ---------------------------------------------------------------------------
# Examples. Not part of the library: nothing links against them.
# 01-03 build up the Lely stack one piece at a time; api_demo uses the API.
# ---------------------------------------------------------------------------
add_executable(01_event_loop examples/01_event_loop.cpp)
ament_target_dependencies(01_event_loop lely_core_libraries)

add_executable(02_can_listen examples/02_can_listen.cpp)
ament_target_dependencies(02_can_listen lely_core_libraries)

add_executable(03_master_boot examples/03_master_boot.cpp)
ament_target_dependencies(03_master_boot lely_core_libraries)

add_executable(api_demo examples/api_demo.cpp)
target_link_libraries(api_demo ${PROJECT_NAME})

install(TARGETS
  epos4_core ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  epos4_sim 01_event_loop 02_can_listen 03_master_boot api_demo
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(lely_core_libraries robot_units)

if(BUILD_TESTING)
  # Linters are pointed at the source directories explicitly rather than
  # through ament_lint_auto. The automatic variant runs from the build
  # directory and recurses into it, so it ends up reporting CMake's own
  # generated CTestTestfile.cmake and CompilerId probes as style violations -
  # 192 failures in files nobody wrote.
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_uncrustify(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${CMAKE_CURRENT_SOURCE_DIR}/tools
    ${CMAKE_CURRENT_SOURCE_DIR}/examples
  )
  ament_lint_cmake(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)
  ament_xmllint(${CMAKE_CURRENT_SOURCE_DIR}/package.xml)

  # These run with NO bus and NO hardware, which is the whole point of keeping
  # the state machine pure logic. The 16 transitions of Table 2-6 are verified
  # in milliseconds.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cia402_state_machine
    test/test_cia402_state_machine.cpp
  )
  target_link_libraries(test_cia402_state_machine epos4_core)

  ament_add_gtest(test_errors test/test_errors.cpp)
  target_link_libraries(test_errors epos4_core)

  ament_add_gtest(test_configs test/test_configs.cpp)
  target_link_libraries(test_configs epos4_core)

  ament_add_gtest(test_unit_conversion test/test_unit_conversion.cpp)
  target_link_libraries(test_unit_conversion epos4_core)

  ament_add_gtest(test_digital_inputs test/test_digital_inputs.cpp)
  target_link_libraries(test_digital_inputs epos4_core)

  ament_add_gtest(test_encoder_configs test/test_encoder_configs.cpp)
  target_link_libraries(test_encoder_configs ${PROJECT_NAME})

  # Links the full library, not epos4_core, but opens no socket: it only
  # exercises what a device allows before CanBus::Start().
  ament_add_gtest(test_cyclic_state test/test_cyclic_state.cpp)
  target_link_libraries(test_cyclic_state epos4_core)

  ament_add_gtest(test_device_lifecycle test/test_device_lifecycle.cpp)
  target_link_libraries(test_device_lifecycle ${PROJECT_NAME})
endif()

ament_package()
