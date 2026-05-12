#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "opengv" for configuration "Release"
set_property(TARGET opengv APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(opengv PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libopengv.so.1.0"
  IMPORTED_SONAME_RELEASE "libopengv.so.1.0"
  )

list(APPEND _cmake_import_check_targets opengv )
list(APPEND _cmake_import_check_files_for_opengv "${_IMPORT_PREFIX}/lib/libopengv.so.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
