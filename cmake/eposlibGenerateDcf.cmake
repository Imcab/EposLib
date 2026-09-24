# ---------------------------------------------------------------------------
# eposlib_generate_dcf(<network> [DESTINATION <dir>] [SOURCE_DIR <dir>])
#
# Turns config/<network>/bus.yml plus the EDS files next to it into the files
# the CANopen master reads: master.dcf and one concise node_<id>.bin per
# slave, which the master pushes to each drive at boot.
#
# dcfgen, from Lely, does the work. This is the plain-CMake equivalent of
# the generate_dcf() macro that ros-<distro>-lely-core-libraries ships, so
# a network can be generated with or without ROS. Installed with eposlib:
# a robot's own package calls it for its own bus.yml after
# find_package(eposlib).
#
# SOURCE_DIR defaults to ${CMAKE_CURRENT_SOURCE_DIR}/config/<network>. The
# generated files land in ${CMAKE_CURRENT_BINARY_DIR}/config/<network> and,
# with DESTINATION, are installed with the source files to
# <DESTINATION>/<network>.
# ---------------------------------------------------------------------------

function(eposlib_generate_dcf network)
  cmake_parse_arguments(ARG "" "DESTINATION;SOURCE_DIR" "" ${ARGN})
  if(NOT ARG_SOURCE_DIR)
    set(ARG_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config/${network})
  endif()
  set(out ${CMAKE_CURRENT_BINARY_DIR}/config/${network})

  find_program(EPOSLIB_DCFGEN dcfgen)
  if(NOT EPOSLIB_DCFGEN)
    message(WARNING
      "dcfgen not found: the master DCF for '${network}' will not be generated. "
      "It comes with Lely CANopen (python3-dcf-tools, or "
      "ros-<distro>-lely-core-libraries).")
    return()
  endif()

  file(GLOB eds_files ${ARG_SOURCE_DIR}/*.eds)
  add_custom_command(
    OUTPUT ${out}/master.dcf
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${out}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${out}
    COMMAND ${EPOSLIB_DCFGEN} -d ${out} -rS ${ARG_SOURCE_DIR}/bus.yml
    # dcfgen resolves the dcf: entries of bus.yml relative to where it runs.
    WORKING_DIRECTORY ${ARG_SOURCE_DIR}
    DEPENDS ${ARG_SOURCE_DIR}/bus.yml ${eds_files}
    COMMENT "Generating the master DCF for ${network}"
    VERBATIM
  )
  add_custom_target(${PROJECT_NAME}_dcf_${network} ALL DEPENDS ${out}/master.dcf)

  if(ARG_DESTINATION)
    install(DIRECTORY ${ARG_SOURCE_DIR}/ DESTINATION ${ARG_DESTINATION}/${network})
    install(DIRECTORY ${out}/ DESTINATION ${ARG_DESTINATION}/${network})
  endif()
endfunction()
