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
# PHASE 4 - The plugin, written once PPM works over SDO.
#
#   include/epos4_ros2_control/epos4_system.hpp   (derives SystemInterface)
#   src/epos4_system.cpp                          (+ PLUGINLIB_EXPORT_CLASS)
#   epos4_ros2_control_plugin.xml                 (plugin description)
#
# Note: it installs into lib/ (not lib/${PROJECT_NAME}/) because pluginlib
# dlopens from the library path, not from the package path.
# --------------------------------------------------------------------------
# add_library(${PROJECT_NAME} SHARED
#   src/epos4_system.cpp
# )
# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# ament_target_dependencies(${PROJECT_NAME}
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   epos4_driver
# )
# pluginlib_export_plugin_description_file(hardware_interface
#   epos4_ros2_control_plugin.xml)
#
# install(TARGETS ${PROJECT_NAME}
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
# install(DIRECTORY include/ DESTINATION include)
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  epos4_driver
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
