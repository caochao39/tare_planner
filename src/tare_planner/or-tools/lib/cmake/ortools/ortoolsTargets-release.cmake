#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ortools::solve" for configuration "Release"
set_property(TARGET ortools::solve APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ortools::solve PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/solve"
  )

list(APPEND _IMPORT_CHECK_TARGETS ortools::solve )
list(APPEND _IMPORT_CHECK_FILES_FOR_ortools::solve "${_IMPORT_PREFIX}/bin/solve" )

# Import target "ortools::ortools" for configuration "Release"
set_property(TARGET ortools::ortools APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ortools::ortools PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libortools.so.9.9.3963"
  IMPORTED_SONAME_RELEASE "libortools.so.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS ortools::ortools )
list(APPEND _IMPORT_CHECK_FILES_FOR_ortools::ortools "${_IMPORT_PREFIX}/lib/libortools.so.9.9.3963" )

# Import target "ortools::flatzinc" for configuration "Release"
set_property(TARGET ortools::flatzinc APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ortools::flatzinc PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libortools_flatzinc.so.9.9.3963"
  IMPORTED_SONAME_RELEASE "libortools_flatzinc.so.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS ortools::flatzinc )
list(APPEND _IMPORT_CHECK_FILES_FOR_ortools::flatzinc "${_IMPORT_PREFIX}/lib/libortools_flatzinc.so.9.9.3963" )

# Import target "ortools::fzn" for configuration "Release"
set_property(TARGET ortools::fzn APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ortools::fzn PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/fzn-cp-sat"
  )

list(APPEND _IMPORT_CHECK_TARGETS ortools::fzn )
list(APPEND _IMPORT_CHECK_FILES_FOR_ortools::fzn "${_IMPORT_PREFIX}/bin/fzn-cp-sat" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
