#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "BLK360G2" for configuration "Release"
set_property(TARGET BLK360G2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(BLK360G2 PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libBLK360G2.so"
  IMPORTED_SONAME_RELEASE "libBLK360G2.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS BLK360G2 )
list(APPEND _IMPORT_CHECK_FILES_FOR_BLK360G2 "${_IMPORT_PREFIX}/lib/libBLK360G2.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
