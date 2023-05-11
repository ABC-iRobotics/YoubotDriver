cmake_minimum_required(VERSION 3.15)
project(Example
    DESCRIPTION "Example"
    VERSION 0.0.1
    LANGUAGES CXX)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

## ---------------------------------------------------------------------------------
## Set folders
set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "" FORCE) #Nem convention instead of program files/SensorFusion...
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Bin) #Or the program puts the application into different folders...

## ---------------------------------------------------------------------------------
## Generally used variables for third parties
set (3RDPARTY_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/ThirdParties/package")
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

## Find or get youBotDriver
find_package(youBotDriver)
if (0)
# TODO fetchcontent
  set(youBotDriver_COMMAND "git clone https://github.com/kutij/myYouBotDriver.git")
  set(youBotDriver_CACHE_SETTINGS -DCMAKE_MSVC_RUNTIME_LIBRARY="MultiThreaded$<$<CONFIG:Debug>:Debug>";-DPolicy_ALL_3RD_PARTIES="Download all";-DCMAKE_GENERATOR_PLATFORM=${CMAKE_GENERATOR_PLATFORM})
  third_party_policy(youBotDriver ${youBotDriver_COMMAND} youBotDriver_CACHE_SETTINGS THIRD_PARTY_CONFIG true GIT_NAMES)
endif()
macro(requires_youBotDriver NAME)
	target_link_libraries (${NAME} youBotDriver::manipulatormodul)
endmacro(requires_youBotDriver)

add_executable(tester tester.cpp)
set_target_properties(tester PROPERTIES LINKER_LANGUAGE CXX)
requires_youBotDriver(tester)
