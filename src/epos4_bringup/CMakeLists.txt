cmake_minimum_required(VERSION 3.8)
project(epos4_bringup)

find_package(ament_cmake REQUIRED)
find_package(lely_core_libraries REQUIRED)

# --------------------------------------------------------------------------
# PHASE 2 - generate_dcf() comes from lely_core_libraries. It takes the maxon
# EDS plus a YAML describing the network (node-IDs, bitrate, which axis is
# which) and produces the master DCF + master.bin that Lely reads at startup.
#
#   config/epos4_network/bus.yml
#   config/epos4_network/epos4.eds   (download from epos.maxongroup.com)
# --------------------------------------------------------------------------
# Real hardware: maxon EPOS4 EDS exported from EPOS Studio.
generate_dcf(epos4_network)

# Simulation: canopen_fake_slaves mock on vcan0. Separate because the master
# checks slave identity and the mock is not a maxon drive.
generate_dcf(sim_network)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
