#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ctraj" for configuration "Release"
set_property(TARGET ctraj APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ctraj PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libctraj.so"
  IMPORTED_SONAME_RELEASE "libctraj.so"
  )

list(APPEND _cmake_import_check_targets ctraj )
list(APPEND _cmake_import_check_files_for_ctraj "${_IMPORT_PREFIX}/lib/libctraj.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
