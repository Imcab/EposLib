cmake_minimum_required(VERSION 3.8)
project(robot_units VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

# Header-only. The vendored units.h is installed as-is so that the usual
# `#include <units.h>` works, exactly as it would with an upstream install.
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # uncrustify is deliberately not run: include/units.h is vendored upstream
  # code and reformatting it would make every future update a merge conflict.
  ament_lint_cmake(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)
  ament_xmllint(${CMAKE_CURRENT_SOURCE_DIR}/package.xml)

  ament_add_gtest(test_units test/test_units.cpp)
  target_link_libraries(test_units ${PROJECT_NAME})
endif()

ament_package()
