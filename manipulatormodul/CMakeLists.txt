set(SOURCES
    Manager.cpp
    MotionLayer.cpp
    MTask.cpp
    MTaskRawConstantJointSpeed.cpp
    MTaskCalibration.cpp
    MTaskCommutation.cpp
    MTaskStop.cpp
    MTaskZeroCurrent.cpp
)

set(HEADERS
    Manager.hpp
    MotionLayer.hpp
    MTask.hpp
    MTaskRawConstantJointSpeed.hpp
    MTaskCalibration.hpp
    MTaskCommutation.hpp
    MTaskStop.hpp
    MTaskZeroCurrent.hpp
)

add_library(manipulatormodul STATIC ${SOURCES} ${HEADERS} ${INTRINSIC_HEADERS})

source_group("Intrinsic headers" FILES ${INTRINSIC_HEADERS} )

target_link_libraries(manipulatormodul lowlevelcontrol)
target_include_directories(manipulatormodul PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/_deps/eigen3-src>
  $<INSTALL_INTERFACE:include/eigen3>)

set_target_properties(manipulatormodul PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(manipulatormodul PROPERTIES FOLDER "Lib")

install_lib(manipulatormodul "${HEADERS}" ${CMAKE_CURRENT_SOURCE_DIR})
