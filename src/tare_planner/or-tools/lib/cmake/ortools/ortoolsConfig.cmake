## ortools CMake configuration file

set(ORTOOLS_VERSION 9.6.2534)


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was ortoolsConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

####################################################################################
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/modules")

include(CMakeFindDependencyMacro)
# Kitware CMake provide a FindZLIB.cmake module
if(NOT ZLIB_FOUND AND NOT TARGET ZLIB::ZLIB)
  find_dependency(ZLIB REQUIRED)
endif()

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.9.6")
  set(CONFIG_FLAG CONFIG)
endif()

if(NOT absl_FOUND)
  find_dependency(absl REQUIRED ${CONFIG_FLAG})
endif()

if(NOT Protobuf_FOUND AND NOT PROTOBUF_FOUND AND NOT TARGET protobuf::libprotobuf)
  # CMake provide a FindProtobuf module so we don't want to use the CONFIG_FLAG
  # also most distro still use the autotools based build for Protobuf.
  # ref: https://cmake.org/cmake/help/latest/module/FindProtobuf.html
  if(ON)
    find_dependency(Protobuf CONFIG REQUIRED)
  else()
    find_dependency(Protobuf REQUIRED)
  endif()
endif()

if(ON)
  # re2 may not provide a CMake config files
  if(NOT re2_FOUND AND NOT TARGET re2::re2)
    find_dependency(re2 REQUIRED)
  endif()
endif()

if(ON)
  # COIN-OR packages don't provide CMake config files
  if(NOT Clp_FOUND AND NOT TARGET Coin::ClpSolver)
    find_dependency(Clp REQUIRED)
  endif()
  if(NOT Cbc_FOUND AND NOT TARGET Coin::CbcSolver)
    find_dependency(Cbc REQUIRED)
  endif()
endif()

if(OFF)
  if(NOT GLPK_FOUND AND NOT TARGET GLPK::GLPK)
    find_dependency(GLPK REQUIRED)
  endif()
endif()

if(OFF)
  if(NOT HIGHS_FOUND AND NOT TARGET HIGHS::HIGHS)
    find_dependency(HIGHS REQUIRED ${CONFIG_FLAG})
  endif()
endif()

if(ON)
  if(NOT Eigen3_FOUND AND NOT TARGET Eigen3::Eigen)
    find_dependency(Eigen3 REQUIRED)
  endif()
endif()

if(ON)
  if(NOT scip_FOUND AND NOT TARGET libscip)
    find_dependency(SCIP REQUIRED)
  endif()
endif()

include("${CMAKE_CURRENT_LIST_DIR}/ortoolsTargets.cmake")
