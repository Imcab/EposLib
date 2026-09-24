cmake_minimum_required(VERSION 3.8)
project(epos4_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(epos4_driver REQUIRED)

# --------------------------------------------------------------------------
# The plugin. Built as a shared library and installed into lib/ (not
# lib/${PROJECT_NAME}/) because pluginlib dlopens from the library path, not
# from the package path.
add_library(${PROJECT_NAME} SHARED
  src/Epos4System.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  epos4_driver
)
pluginlib_export_plugin_description_file(hardware_interface
  epos4_ros2_control_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  epos4_driver
)

if(BUILD_TESTING)
  # Linters scoped to the source directories, not run from the build tree:
  # the automatic variant recurses into it and reports CMake's own generated
  # files as style violations.
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_uncrustify(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_lint_cmake(${CMAKE_CURRENT_SOURCE_DIR}/CMakeLists.txt)
  # One call, not one per file: ament_xmllint names the test after itself,
  # so two calls in the same directory collide on the test name "xmllint".
  ament_xmllint(
    ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/epos4_ros2_control_plugin.xml
  )

  ament_add_gtest(test_epos4_system test/test_epos4_system.cpp)
  target_link_libraries(test_epos4_system ${PROJECT_NAME})
endif()

ament_package()
