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

set(SOEM_COMMAND "git clone https://github.com/kutij/SOEM.git")
set(SOEM_CACHE_SETTINGS -DCMAKE_INSTALL_PREFIX="${3RDPARTY_INSTALL_PREFIX}")

#if (WIN32)
#	list(APPEND ZMQ_CACHE_SETTINGS ${EXTERNAL_FLAG_SETTER})
#endif()
third_party_policy(soem ${SOEM_COMMAND} SOEM_CACHE_SETTINGS THIRD_PARTY_CONFIG true GIT_NAMES)
macro(requires_soem NAME)
	target_link_libraries (${NAME} soem)
endmacro(requires_soem)


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

