cmake_minimum_required(VERSION 3.8)
project(epos4_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# --------------------------------------------------------------------------
# Uncomment and add files as they are needed. rosidl fails when invoked with
# an empty list, which is why this is off for now.
#
# Candidates (do not create them before they are actually needed):
#   msg/Epos4State.msg       decoded CiA 402 state + active mode
#   msg/AxisDiagnostics.msg  error code, following error, temperature
#   srv/FaultReset.srv       phase 5
#   srv/SetOperationMode.srv phase 3
#   action/Home.action       phase 6 (an action: homing takes time and can be
#                            cancelled)
# --------------------------------------------------------------------------
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Epos4State.msg"
#   DEPENDENCIES std_msgs
# )
# ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
