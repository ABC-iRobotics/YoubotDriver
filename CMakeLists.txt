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
SET(3RDPARTY_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/third_parties)

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
set(SOEM_CACHE_SETTINGS -DCMAKE_MSVC_RUNTIME_LIBRARY="MultiThreaded$<$<CONFIG:Debug>:Debug>")
third_party_policy(soem ${SOEM_COMMAND} SOEM_CACHE_SETTINGS THIRD_PARTY_CONFIG true GIT_NAMES)
macro(requires_soem NAME)
	target_link_libraries (${NAME} soem)
endmacro(requires_soem)

## Find or get SPDLOG
set(SPDLOG_COMMAND "git clone https://github.com/gabime/spdlog.git -bv1.11.0")
set(SPDLOG_CACHE_SETTINGS -DSPDLOG_BUILD_BENCH=FALSE;-DSPDLOG_BUILD_EXAMPLE=FALSE;
	-DSPDLOG_BUILD_EXAMPLE_HO=FALSE;-DSPDLOG_BUILD_TESTS=FALSE;-DSPDLOG_BUILD_TESTS_HO=FALSE;
	-DSPDLOG_INSTALL=TRUE;-DCMAKE_MSVC_RUNTIME_LIBRARY="MultiThreaded$<$<CONFIG:Debug>:Debug>")
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
set(EIGEN3_CACHE_SETTINGS -DBUILD_TESTING=FALSE;-DCMAKE_INSTALL_INCLUDEDIR="include";${EXTERNAL_FLAG_SETTER})
third_party_policy(Eigen3 ${EIGEN3_COMMAND} EIGEN3_CACHE_SETTINGS ONLY_RELEASE false GIT_NAMES)
macro(requires_eigen NAME)
	target_link_libraries (${NAME} Eigen3::Eigen)
endmacro(requires_eigen)

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
add_subdirectory(tester)
add_subdirectory(matlabexporter)
#add_subdirectory(oldshit)

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