# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_db_tsdf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED db_tsdf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(db_tsdf_FOUND FALSE)
  elseif(NOT db_tsdf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(db_tsdf_FOUND FALSE)
  endif()
  return()
endif()
set(_db_tsdf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT db_tsdf_FIND_QUIETLY)
  message(STATUS "Found db_tsdf: 0.0.0 (${db_tsdf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'db_tsdf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT db_tsdf_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(db_tsdf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${db_tsdf_DIR}/${_extra}")
endforeach()
