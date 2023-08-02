if(NOT TARGET libscip)
  include("${CMAKE_CURRENT_LIST_DIR}/scip-targets.cmake")
endif()

if()
   set(ZIMPL_DIR "")
   find_package(ZIMPL QUIET CONFIG)
endif()

if()
   set(SOPLEX_DIR "")
   find_package(SOPLEX QUIET CONFIG)
endif()

set(SCIP_LIBRARIES libscip)
set(SCIP_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include")
set(SCIP_FOUND TRUE)
