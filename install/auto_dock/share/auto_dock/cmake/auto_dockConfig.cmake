# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_auto_dock_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED auto_dock_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(auto_dock_FOUND FALSE)
  elseif(NOT auto_dock_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(auto_dock_FOUND FALSE)
  endif()
  return()
endif()
set(_auto_dock_CONFIG_INCLUDED TRUE)

# output package information
if(NOT auto_dock_FIND_QUIETLY)
  message(STATUS "Found auto_dock: 0.0.0 (${auto_dock_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'auto_dock' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${auto_dock_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(auto_dock_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${auto_dock_DIR}/${_extra}")
endforeach()
