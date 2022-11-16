cmake_minimum_required(VERSION 3.9)
project(MyYouBotDriver
    DESCRIPTION "My YouBot Driver"
    VERSION 0.0.1
    LANGUAGES CXX)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

SET(3RDPARTY_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/third_parties)

## ---------------------------------------------------------------------------------
## Generally used variables for third parties
include(cmake/ThirdPartyHandling.cmake)
set(THIRD_PARTY_CONFIG Debug;Release)
set(ONLY_RELEASE Release)
set(GIT_NAMES GIT Git)
set(HG_NAMES HG Hg)

if (WIN32)
  ## ---------------------------------------------------------------------------------
  ## Use folders in IDEs supporting it
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "")
  ## ---------------------------------------------------------------------------------
  ## Set /MT or /MD compiler flags
  set(RUNTIME_LIBRARY_SETTINGS "/MT*" CACHE STRING "Code Generation: multi-threaded (/MT*) or multi threaded DLL (/MD*)")
  set_property(CACHE RUNTIME_LIBRARY_SETTINGS PROPERTY STRINGS "/MT*" "/MD*")
  set(CompilerFlags
	  CMAKE_CXX_FLAGS
	  CMAKE_CXX_FLAGS_DEBUG
	  CMAKE_CXX_FLAGS_RELEASE
	  CMAKE_CXX_FLAGS_RELWITHDEBINFO
	  CMAKE_CXX_FLAGS_MINSIZEREL
	  CMAKE_C_FLAGS
	  CMAKE_C_FLAGS_DEBUG
	  CMAKE_C_FLAGS_RELEASE
	  CMAKE_C_FLAGS_RELWITHDEBINFO
	  CMAKE_C_FLAGS_MINSIZEREL)
  set(EXTERNAL_FLAG_SETTER "")
  if (RUNTIME_LIBRARY_SETTINGS STREQUAL "/MT*")
	set(REPLACE_ARGS "/MD" "/MT")
	set(RUNTIME_INFO "mt")
  else()
	set(REPLACE_ARGS "/MT" "/MD")
	set(RUNTIME_INFO "md")
  endif()
  foreach(CompilerFlag ${CompilerFlags})
	string(REPLACE ${REPLACE_ARGS} ${CompilerFlag} "${${CompilerFlag}}")
	list(APPEND EXTERNAL_FLAG_SETTER -D${CompilerFlag}="${${CompilerFlag}}" ) #produces a list like -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
  endforeach()
  ## ---------------------------------------------------------------------------------
  ## Set configurations to be used
  set (CMAKE_CONFIGURATION_TYPES Debug;Release;MinSizeRel;RelWithDebInfo CACHE STRING "Configuration types" FORCE)
endif()

## Find or get SOEM
set(SOEM_COMMAND "git clone https://github.com/kutij/SOEM.git")
set(SOEM_CACHE_SETTINGS -DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}")

third_party_policy(soem ${SOEM_COMMAND} SOEM_CACHE_SETTINGS THIRD_PARTY_CONFIG true GIT_NAMES)
macro(requires_soem NAME)
	target_link_libraries (${NAME} soem)
endmacro(requires_soem)

## Find or get SPDLOG
set(SPDLOG_COMMAND "git clone https://github.com/gabime/spdlog.git -bv1.5.0")
set(SPDLOG_CACHE_SETTINGS -DSPDLOG_BUILD_BENCH=FALSE;-DSPDLOG_BUILD_EXAMPLE=FALSE;
	-DSPDLOG_BUILD_EXAMPLE_HO=FALSE;-DSPDLOG_BUILD_TESTS=FALSE;-DSPDLOG_BUILD_TESTS_HO=FALSE;
	-DSPDLOG_INSTALL=TRUE;-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";${EXTERNAL_FLAG_SETTER})
third_party_policy(spdlog ${SPDLOG_COMMAND} SPDLOG_CACHE_SETTINGS THIRD_PARTY_CONFIG TRUE GIT_NAMES)
if (spdlog_FOUND)
	set_target_properties(spdlog::spdlog PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE) #By default the debug libs are tried to use...
	set_target_properties(spdlog::spdlog PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE) #By default the debug libs are tried to use...
endif()
macro(requires_spdlog NAME)
	target_link_libraries (${NAME} spdlog::spdlog)
endmacro(requires_spdlog)

## ---------------------------------------------------------------
add_subdirectory(slaveinfo)
add_subdirectory(lowlevelcontrol)

set(Boost_USE_STATIC_LIBS true)
FIND_PACKAGE(Boost COMPONENTS thread date_time filesystem system regex REQUIRED)
IF (Boost_FOUND)
  MESSAGE(STATUS "SUCCESSFUL: Boost found")
ELSE (Boost_FOUND)
  MESSAGE(STATUS "ERROR: Boost not found")
ENDIF (Boost_FOUND)


add_subdirectory(fromscratch)
add_subdirectory(oldshit)

