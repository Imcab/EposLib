# ---------------------------------------------------------------------------
# Finds what eposlib needs. Used by the build and, installed, by
# eposlibConfig.cmake, so a project that does find_package(eposlib) finds the
# same dependencies the same way.
#
#   Lely CANopen   through pkg-config (liblely-coapp). Lely installs .pc files
#                  whichever way it was installed: from source, from its PPA,
#                  or as ros-<distro>-lely-core-libraries.
#   ros2units      header-only; only <ros2units/units.h> is needed. A plain
#                  git clone works: point CMAKE_PREFIX_PATH at it.
#
# A sourced ROS or colcon environment exports its prefixes in
# AMENT_PREFIX_PATH, not in CMAKE_PREFIX_PATH or PKG_CONFIG_PATH. Those
# prefixes are added to the search, so under ROS nothing has to be passed by
# hand. Without ROS the variable is simply absent.
# ---------------------------------------------------------------------------

if(DEFINED ENV{AMENT_PREFIX_PATH})
  string(REPLACE ":" ";" _eposlib_ament_prefixes "$ENV{AMENT_PREFIX_PATH}")
  list(APPEND CMAKE_PREFIX_PATH ${_eposlib_ament_prefixes})
endif()

find_package(PkgConfig REQUIRED)
if(NOT TARGET PkgConfig::LELY)
  pkg_check_modules(LELY REQUIRED IMPORTED_TARGET GLOBAL liblely-coapp)
endif()

find_path(EPOSLIB_UNITS_INCLUDE_DIR ros2units/units.h
  PATH_SUFFIXES include
  DOC "Directory containing ros2units/units.h (https://github.com/Imcab/ros2units)")
if(NOT EPOSLIB_UNITS_INCLUDE_DIR)
  message(FATAL_ERROR
    "ros2units not found. Clone https://github.com/Imcab/ros2units and add it "
    "to CMAKE_PREFIX_PATH, or build it in the same colcon workspace.")
endif()
