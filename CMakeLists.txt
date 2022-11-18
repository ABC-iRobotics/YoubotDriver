cmake_minimum_required(VERSION 3.15)
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

  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
  ## ---------------------------------------------------------------------------------
  ## Set configurations to be used
  set (CMAKE_CONFIGURATION_TYPES Debug;Release;MinSizeRel;RelWithDebInfo CACHE STRING "Configuration types" FORCE)
endif()

## Find or get SOEM
set(SOEM_COMMAND "git clone https://github.com/kutij/SOEM.git")
set(SOEM_CACHE_SETTINGS -DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";-DCMAKE_MSVC_RUNTIME_LIBRARY="MultiThreaded$<$<CONFIG:Debug>:Debug>")
third_party_policy(soem ${SOEM_COMMAND} SOEM_CACHE_SETTINGS THIRD_PARTY_CONFIG true GIT_NAMES)
macro(requires_soem NAME)
	target_link_libraries (${NAME} soem)
endmacro(requires_soem)

## Find or get SPDLOG
set(SPDLOG_COMMAND "git clone https://github.com/gabime/spdlog.git -bv1.11.0")
set(SPDLOG_CACHE_SETTINGS -DSPDLOG_BUILD_BENCH=FALSE;-DSPDLOG_BUILD_EXAMPLE=FALSE;
	-DSPDLOG_BUILD_EXAMPLE_HO=FALSE;-DSPDLOG_BUILD_TESTS=FALSE;-DSPDLOG_BUILD_TESTS_HO=FALSE;
	-DSPDLOG_INSTALL=TRUE;-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";-DCMAKE_MSVC_RUNTIME_LIBRARY="MultiThreaded$<$<CONFIG:Debug>:Debug>")
third_party_policy(spdlog ${SPDLOG_COMMAND} SPDLOG_CACHE_SETTINGS THIRD_PARTY_CONFIG TRUE GIT_NAMES)
if (spdlog_FOUND)
	set_target_properties(spdlog::spdlog PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE) #By default the debug libs are tried to use...
	set_target_properties(spdlog::spdlog PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE) #By default the debug libs are tried to use...
endif()
macro(requires_spdlog NAME)
	target_link_libraries (${NAME} spdlog::spdlog)
endmacro(requires_spdlog)

## Find or get Eigen
set(EIGEN3_COMMAND "git clone https://gitlab.com/libeigen/eigen.git -b3.3.7")
set(EIGEN3_CACHE_SETTINGS -DBUILD_TESTING=FALSE;-DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}";
	-DCMAKE_INSTALL_INCLUDEDIR="include";${EXTERNAL_FLAG_SETTER})
third_party_policy(Eigen3 ${EIGEN3_COMMAND} EIGEN3_CACHE_SETTINGS ONLY_RELEASE false GIT_NAMES)
macro(requires_eigen NAME)
	target_link_libraries (${NAME} Eigen3::Eigen)
endmacro(requires_eigen)

## ---------------------------------------------------------------
add_subdirectory(slaveinfo)
add_subdirectory(lowlevelcontrol)
add_subdirectory(tester)
add_subdirectory(oldshit)

