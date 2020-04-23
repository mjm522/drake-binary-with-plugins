
# Generated by cps2cmake
# https://github.com/mwoehlke/pycps

if(CMAKE_MAJOR_VERSION LESS 3)
  message(FATAL_ERROR "CMake >= 3.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 3.0)
set(CMAKE_IMPORT_FILE_VERSION 1)

if(CMAKE_VERSION VERSION_LESS 3.9.0)
  # Imported from CMake 3.9.0
  macro(find_dependency dep)
    if (NOT ${dep}_FOUND)
      set(cmake_fd_quiet_arg)
      if(${CMAKE_FIND_PACKAGE_NAME}_FIND_QUIETLY)
        set(cmake_fd_quiet_arg QUIET)
      endif()
      set(cmake_fd_required_arg)
      if(${CMAKE_FIND_PACKAGE_NAME}_FIND_REQUIRED)
        set(cmake_fd_required_arg REQUIRED)
      endif()

      get_property(cmake_fd_alreadyTransitive GLOBAL PROPERTY
        _CMAKE_${dep}_TRANSITIVE_DEPENDENCY
      )

      find_package(${dep} ${ARGN}
        ${cmake_fd_quiet_arg}
        ${cmake_fd_required_arg}
      )

      if(NOT DEFINED cmake_fd_alreadyTransitive OR cmake_fd_alreadyTransitive)
        set_property(GLOBAL PROPERTY _CMAKE_${dep}_TRANSITIVE_DEPENDENCY TRUE)
      endif()

      if (NOT ${dep}_FOUND)
        set(${CMAKE_FIND_PACKAGE_NAME}_NOT_FOUND_MESSAGE "${CMAKE_FIND_PACKAGE_NAME} could not be found because dependency ${dep} could not be found.")
        set(${CMAKE_FIND_PACKAGE_NAME}_FOUND False)
        return()
      endif()
      set(cmake_fd_required_arg)
      set(cmake_fd_quiet_arg)
      set(cmake_fd_exact_arg)
    endif()
  endmacro()
else()
  include(CMakeFindDependencyMacro)
endif()


get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)

if(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX STREQUAL "/")
  set(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/modules;${CMAKE_MODULE_PATH}")
find_dependency(bot2-core-lcmtypes CONFIG HINTS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/cmake/bot2-core-lcmtypes")
find_dependency(Eigen3 3.3.3 CONFIG HINTS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/cmake/eigen3")
find_dependency(fmt 6.0 CONFIG HINTS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/cmake/fmt")
find_dependency(ignition-math6 6.4 CONFIG HINTS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/cmake/ignition-math6")
find_dependency(lcm 1.4 CONFIG HINTS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/cmake/lcm")
find_dependency(optitrack CONFIG HINTS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/cmake/optitrack")
find_dependency(robotlocomotion-lcmtypes CONFIG HINTS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/cmake/robotlocomotion-lcmtypes")
find_dependency(spdlog 1.3 CONFIG HINTS "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/cmake/spdlog")
find_dependency(TinyXML2 2.2 MODULE)
set(_expectedTargets drake::drake drake::drake-lcmtypes-cpp drake::drake-lcmtypes-java drake::drake-marker)

set(_targetsDefined)
set(_targetsNotDefined)

foreach(_expectedTarget ${_expectedTargets})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)

add_library(drake::drake SHARED IMPORTED)
set_target_properties(drake::drake PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/libdrake.so"
  IMPORTED_SONAME "libdrake.so"
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include"
  INTERFACE_LINK_LIBRARIES "drake::drake-lcmtypes-cpp;drake::drake-marker;bot2-core-lcmtypes::lcmtypes_bot2-core-cpp;Eigen3::Eigen;fmt::fmt-header-only;ignition-math6::ignition-math6;lcm::lcm;optitrack::optitrack-lcmtypes-cpp;robotlocomotion-lcmtypes::robotlocomotion-lcmtypes-cpp;spdlog::spdlog;tinyxml2::tinyxml2;${yaml-cpp_LIBRARIES}"
  INTERFACE_COMPILE_FEATURES "cxx_std_17"
)

add_library(drake::drake-lcmtypes-cpp INTERFACE IMPORTED)
set_target_properties(drake::drake-lcmtypes-cpp PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/drake_lcmtypes"
  INTERFACE_LINK_LIBRARIES "lcm::lcm-coretypes"
)

add_library(drake::drake-lcmtypes-java STATIC IMPORTED)
set_target_properties(drake::drake-lcmtypes-java PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/share/java/lcmtypes_drake.jar"
  JAR_FILE "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/share/java/lcmtypes_drake.jar"
)

add_library(drake::drake-marker SHARED IMPORTED)
set_target_properties(drake::drake-marker PROPERTIES
  IMPORTED_LOCATION "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib/libdrake_marker.so"
  IMPORTED_SONAME "libdrake_marker.so"
)

set(drake_LIBRARIES "drake::drake")
set(drake_INCLUDE_DIRS "")


unset(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX)
unset(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)

