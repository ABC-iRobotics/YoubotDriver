cmake_minimum_required(VERSION 3.15)
project(MyYouBotDriver
    DESCRIPTION "My YouBot Driver"
    VERSION 0.0.1
    LANGUAGES CXX)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

## ---------------------------------------------------------------------------------
## Set folders
#set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install CACHE PATH "" FORCE) #Nem convention instead of program files/SensorFusion...
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/Bin) #Or the program puts the application into different folders...

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

## Fetch third parties
include(FetchContent)
set(FETCHCONTENT_QUIET TRUE)

## Fetch SOEM 
FetchContent_Declare(SOEM
                     GIT_REPOSITORY https://github.com/kutij/SOEM.git
                     )
FetchContent_MakeAvailable(SOEM)
set_target_properties(soem PROPERTIES FOLDER "3rd parties")
## Fetch SPDLOG
FetchContent_Declare(SPDLOG
                     GIT_REPOSITORY https://github.com/gabime/spdlog.git
                     GIT_TAG        v1.11.0
                     )
set(SPDLOG_BUILD_BENCH FALSE)
set(SPDLOG_BUILD_EXAMPLE FALSE)
set(SPDLOG_BUILD_EXAMPLE_HO FALSE)
set(SPDLOG_BUILD_TESTS FALSE)
set(SPDLOG_BUILD_TESTS_HO FALSE)
set(SPDLOG_INSTALL TRUE)
set(SPDLOG_BUILD_BENCH FALSE)
set(SPDLOG_BUILD_EXAMPLE FALSE)
if(WIN32)
    SET(ADD_INFO -${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO})
endif()
FetchContent_MakeAvailable(SPDLOG)
set_target_properties(spdlog PROPERTIES
			RELEASE_POSTFIX "${ADD_INFO}"
			RELWITHDEBINFO_POSTFIX "${ADD_INFO}-relwithdebinfo"
			MINSIZEREL_POSTFIX "${ADD_INFO}-minsizerel"
			DEBUG_POSTFIX "${ADD_INFO}-debug")
set_target_properties(spdlog PROPERTIES FOLDER "3rd parties")
#set_target_properties(spdlog::spdlog PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE) #By default the debug libs are tried to use...
#set_target_properties(spdlog::spdlog PROPERTIES MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE) #By default the debug libs are tried to use...
  
## Fetch Eigen3
FetchContent_Declare(Eigen3
                     GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
                     GIT_TAG        3.3.7
                     )
set(BUILD_TESTING FALSE)
set(CMAKE_INSTALL_INCLUDEDIR "include")
FetchContent_MakeAvailable(Eigen3)
set_target_properties(uninstall PROPERTIES FOLDER "3rd parties")

## Fetch Googletest
FetchContent_Declare(  googletest
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/5376968f6948923e2411081fd9372e71a59d8e77.zip
  )
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF)
set(INSTALL_GTEST OFF)
FetchContent_MakeAvailable(googletest)
set_target_properties(gtest PROPERTIES FOLDER "3rd parties")
set_target_properties(gtest_main PROPERTIES FOLDER "3rd parties")

## ---------------------------------------------------------------------------------
## Names used in the CMake package
set(export_config_name "youBotDriver")
set(export_targets ${export_config_name}Targets)
if(WIN32)
	set(export_targets_name ${export_config_name}Targets-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO})
else()
	set(export_targets_name ${export_config_name}Targets)
endif()

## ---------------------------------------------------------------
## Macro to setup archive targets
macro(install_lib NAME HEADERS SOURCEDIR)
	set(LIBS_TO_INSTALL ${LIBS_TO_INSTALL} "${NAME}" PARENT_SCOPE) #for configuring Config.cmake in installing.cmake
	set_target_properties(${NAME} PROPERTIES FOLDER "Lib" IMPORTED_LOCATION lib)
	if(WIN32)
		set_target_properties(${NAME} PROPERTIES
			RELEASE_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO}"
			RELWITHDEBINFO_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO}-relwithdebinfo"
			MINSIZEREL_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO}-minsizerel"
			DEBUG_POSTFIX "-${CMAKE_VS_PLATFORM_TOOLSET}-${CMAKE_GENERATOR_PLATFORM}-${RUNTIME_INFO}-debug")
	endif()
	target_include_directories(${NAME} INTERFACE
		$<BUILD_INTERFACE:${SOURCEDIR}/>
		$<INSTALL_INTERFACE:include>)
	install(TARGETS ${NAME} EXPORT ${export_targets}
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		RUNTIME DESTINATION bin
		INCLUDES DESTINATION include)
	install(FILES ${HEADERS} DESTINATION include)
endmacro(install_lib)

## ---------------------------------------------------------------
add_subdirectory(slaveinfo)
add_subdirectory(lowlevelcontrol)
add_subdirectory(manipulatormodul)
add_subdirectory(tester)
add_subdirectory(modultester)
add_subdirectory(matlabexporter)
#add_subdirectory(old)

## ---------------------------------------------------------------------------------
## Additional install methods
install(EXPORT ${export_targets}
  FILE ${export_targets_name}.cmake
  DESTINATION "cmake"
  NAMESPACE ${export_config_name}::)
# Howto install third parties:
if(WIN32)
	install(CODE "
		set(LIBS_TO_INSTALL ${LIBS_TO_INSTALL})
		set(RUNTIME_INFO ${RUNTIME_INFO})
		set(export_config_name ${export_config_name})
		set(export_targets ${export_targets})

		configure_file(\"${CMAKE_CURRENT_LIST_DIR}/cmake/ConfigWindows.cmake.in\"
		\"${CMAKE_INSTALL_PREFIX}/${export_config_name}Config.cmake\" @ONLY)
	")
else() #TODO!!
	install(CODE "
		include(\"${CMAKE_SOURCE_DIR}/cmake/GetGitVersion.cmake\")

		set(LIBS_TO_INSTALL ${LIBS_TO_INSTALL})
		set(export_config_name ${export_config_name})
		set(export_targets ${export_targets})
		set(export_targets_name ${export_targets_name})

		#------------------------------------------------------------------------------
		# Configure <export_config_name>ConfigVersion.cmake common to build and install tree
		include(CMakePackageConfigHelpers)
		write_basic_package_version_file(
			\"${CMAKE_INSTALL_PREFIX}/${export_config_name}ConfigVersion.cmake\"
			VERSION \${GIT_TAG_VERSION}
			COMPATIBILITY ExactVersion)

		configure_file(\"${CMAKE_CURRENT_LIST_DIR}/cmake/sf_version.h.in\"
			\"${CMAKE_INSTALL_PREFIX}/include/sf_version.h\" @ONLY)

		set(ADD_PLOTTER ${ADD_PLOTTER})
		configure_file(\"${CMAKE_CURRENT_LIST_DIR}/cmake/ConfigLinux.cmake.in\"
		\"${CMAKE_INSTALL_PREFIX}/${export_config_name}Config.cmake\" @ONLY)
	")
endif()

# Copy config files
file(GLOB config_files "config/*.*" )
install(FILES ${config_files} DESTINATION "${CMAKE_INSTALL_PREFIX}/config")