#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Coin::Clp" for configuration "Release"
set_property(TARGET Coin::Clp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Coin::Clp PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libClp.a"
  )

list(APPEND _cmake_import_check_targets Coin::Clp )
list(APPEND _cmake_import_check_files_for_Coin::Clp "${_IMPORT_PREFIX}/lib/libClp.a" )

# Import target "Coin::OsiClp" for configuration "Release"
set_property(TARGET Coin::OsiClp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Coin::OsiClp PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libOsiClp.a"
  )

list(APPEND _cmake_import_check_targets Coin::OsiClp )
list(APPEND _cmake_import_check_files_for_Coin::OsiClp "${_IMPORT_PREFIX}/lib/libOsiClp.a" )

# Import target "Coin::ClpSolver" for configuration "Release"
set_property(TARGET Coin::ClpSolver APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Coin::ClpSolver PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libClpSolver.a"
  )

list(APPEND _cmake_import_check_targets Coin::ClpSolver )
list(APPEND _cmake_import_check_files_for_Coin::ClpSolver "${_IMPORT_PREFIX}/lib/libClpSolver.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
