# - this module looks for Matlab
# Defines:
#  MATLAB_INCLUDE_DIR: include path for mex.h
#  MATLAB_LIBRARIES:   required libraries: libmex, libmx
#  MATLAB_MEX_LIBRARY: path to libmex
#  MATLAB_MX_LIBRARY:  path to libmx
set(MATLAB_PATH "E.g. D:/Program Files/MATLAB/R2011a" CACHE STRING "Path to installed MATLAB")

SET(MATLAB_FOUND 0)

FIND_PATH(MATLAB_INCLUDE_DIR NO_CACHE mex.h ${MATLAB_PATH}/extern/include)

IF(NOT MATLAB_INCLUDE_DIR)
  MESSAGE(STATUS "MATLAB has not been found." )
  MESSAGE(STATUS "In Linux this can be done in your user .bashrc file by appending the corresponding line, e.g:" )
  MESSAGE(STATUS "MATLAB_PATH=/usr/local/MATLAB/R2012b" )
  MESSAGE(STATUS "In Windows MATLAB_PATH=D:\\Program Files\\MATLAB\\R2011a" )
ELSE()
  INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR})

  FIND_LIBRARY( MATLAB_MEX_LIBRARY
                NAMES libmex mex
                PATHS ${MATLAB_PATH}/bin ${MATLAB_PATH}/extern/lib 
                PATH_SUFFIXES glnxa64 glnx86 win64/microsoft win32/microsoft)

  FIND_LIBRARY( MATLAB_MX_LIBRARY
                NAMES libmx mx
                PATHS ${MATLAB_PATH}/bin ${MATLAB_PATH}/extern/lib 
                PATH_SUFFIXES glnxa64 glnx86 win64/microsoft win32/microsoft)

  # This is common to UNIX and Win32:
  SET(MATLAB_LIBRARIES
    ${MATLAB_MEX_LIBRARY}
    ${MATLAB_MX_LIBRARY}
  )

  IF(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)
    SET(MATLAB_FOUND 1)
    MESSAGE(STATUS "Matlab libraries will be used")
  ENDIF(MATLAB_INCLUDE_DIR AND MATLAB_LIBRARIES)

  MARK_AS_ADVANCED(
    MATLAB_LIBRARIES
    MATLAB_MEX_LIBRARY
    MATLAB_MX_LIBRARY
    MATLAB_INCLUDE_DIR
    MATLAB_FOUND
    MATLAB_PATH
  )
  SET(MATLAB_FOUND 1)
ENDIF()

macro(add_matlabexporter NAME files)
  IF(MATLAB_FOUND)
    message(STATUS "MATLAB Found, MATLAB MEX will be compiled.")

    # set up matlab libraries
    INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIR})
    add_library(${NAME} SHARED ${files} ${CMAKE_SOURCE_DIR}/CMake/Matlabdef.def)
    target_link_libraries(${NAME} ${MATLAB_LIBRARIES})

    # 32-bit or 64-bit mex
    if(WIN32)
      if (CMAKE_CL_64)
          SET_TARGET_PROPERTIES(${NAME} PROPERTIES SUFFIX .mexw64)
      else(CMAKE_CL_64)
          SET_TARGET_PROPERTIES(${NAME} PROPERTIES SUFFIX .mexw32)
      endif(CMAKE_CL_64)
    else(WIN32)
      if (CMAKE_SIZEOF_VOID_P MATCHES "8")
          SET_TARGET_PROPERTIES(${NAME} PROPERTIES SUFFIX .mexa64 PREFIX "")
      else(CMAKE_SIZEOF_VOID_P MATCHES "8")
          SET_TARGET_PROPERTIES(${NAME} PROPERTIES SUFFIX .mexglx PREFIX "")
      endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
    endif(WIN32)
    target_compile_definitions(${NAME} PUBLIC MATLAB_MEX_FILE MX_COMPAT_32)
    
    # install to /bin by default
    install(TARGETS ${NAME} DESTINATION ../bin/MATLAB)
    
    # Copy config files
    file(GLOB matlab_files "${CMAKE_CURRENT_SOURCE_DIR}/*.m" )
    install(FILES ${matlab_files} DESTINATION "${CMAKE_BINARY_DIR}/bin/MATLAB")
    set_target_properties(${NAME} PROPERTIES FOLDER "App")
  ELSE(MATLAB_FOUND)
      MESSAGE("MATLAB not found...nothing will be built.")
  ENDIF(MATLAB_FOUND)
endmacro(add_matlabexporter)