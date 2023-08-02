#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ortools::ortools" for configuration "Release"
set_property(TARGET ortools::ortools APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ortools::ortools PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libortools.so.9.6.2534"
  IMPORTED_SONAME_RELEASE "libortools.so.9"
  )

list(APPEND _cmake_import_check_targets ortools::ortools )
list(APPEND _cmake_import_check_files_for_ortools::ortools "${_IMPORT_PREFIX}/lib/libortools.so.9.6.2534" )

# Import target "ortools::flatzinc" for configuration "Release"
set_property(TARGET ortools::flatzinc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ortools::flatzinc PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libortools_flatzinc.so.9.6.2534"
  IMPORTED_SONAME_RELEASE "libortools_flatzinc.so.9"
  )

list(APPEND _cmake_import_check_targets ortools::flatzinc )
list(APPEND _cmake_import_check_files_for_ortools::flatzinc "${_IMPORT_PREFIX}/lib/libortools_flatzinc.so.9.6.2534" )

# Import target "ortools::fzn" for configuration "Release"
set_property(TARGET ortools::fzn APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ortools::fzn PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/fzn-ortools"
  )

list(APPEND _cmake_import_check_targets ortools::fzn )
list(APPEND _cmake_import_check_files_for_ortools::fzn "${_IMPORT_PREFIX}/bin/fzn-ortools" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
