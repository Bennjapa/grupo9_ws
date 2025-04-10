# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_very_simple_robot_simulator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED very_simple_robot_simulator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(very_simple_robot_simulator_FOUND FALSE)
  elseif(NOT very_simple_robot_simulator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(very_simple_robot_simulator_FOUND FALSE)
  endif()
  return()
endif()
set(_very_simple_robot_simulator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT very_simple_robot_simulator_FIND_QUIETLY)
  message(STATUS "Found very_simple_robot_simulator: 0.0.0 (${very_simple_robot_simulator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'very_simple_robot_simulator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${very_simple_robot_simulator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(very_simple_robot_simulator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${very_simple_robot_simulator_DIR}/${_extra}")
endforeach()
