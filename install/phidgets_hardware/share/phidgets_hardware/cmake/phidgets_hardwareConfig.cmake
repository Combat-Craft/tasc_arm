# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_phidgets_hardware_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED phidgets_hardware_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(phidgets_hardware_FOUND FALSE)
  elseif(NOT phidgets_hardware_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(phidgets_hardware_FOUND FALSE)
  endif()
  return()
endif()
set(_phidgets_hardware_CONFIG_INCLUDED TRUE)

# output package information
if(NOT phidgets_hardware_FIND_QUIETLY)
  message(STATUS "Found phidgets_hardware: 0.0.1 (${phidgets_hardware_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'phidgets_hardware' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${phidgets_hardware_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(phidgets_hardware_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${phidgets_hardware_DIR}/${_extra}")
endforeach()
