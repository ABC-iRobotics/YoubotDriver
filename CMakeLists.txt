cmake_minimum_required(VERSION 3.9)
project(SOEMAPP
    DESCRIPTION "My YouBot Driver"
    VERSION 0.0.1
    LANGUAGES C)

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

