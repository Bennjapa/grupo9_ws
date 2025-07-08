# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_proyecto_grupo_9_2025_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED proyecto_grupo_9_2025_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(proyecto_grupo_9_2025_FOUND FALSE)
  elseif(NOT proyecto_grupo_9_2025_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(proyecto_grupo_9_2025_FOUND FALSE)
  endif()
  return()
endif()
set(_proyecto_grupo_9_2025_CONFIG_INCLUDED TRUE)

# output package information
if(NOT proyecto_grupo_9_2025_FIND_QUIETLY)
  message(STATUS "Found proyecto_grupo_9_2025: 0.0.0 (${proyecto_grupo_9_2025_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'proyecto_grupo_9_2025' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${proyecto_grupo_9_2025_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(proyecto_grupo_9_2025_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${proyecto_grupo_9_2025_DIR}/${_extra}")
endforeach()
