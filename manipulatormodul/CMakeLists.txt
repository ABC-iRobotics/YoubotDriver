
set(SOURCES
    Manager.cpp
    MotionLayer.cpp
    MTask.cpp
    MTaskRawConstantJointSpeed.cpp
    MTaskRawConstantJointPosition.cpp
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
    MTaskRawConstantJointPosition.hpp
    MTaskCalibration.hpp
    MTaskCommutation.hpp
    MTaskStop.hpp
    MTaskZeroCurrent.hpp
)

set(INTRINSIC_HEADERS
)

add_library(manipulatormodul STATIC ${SOURCES} ${HEADERS} ${INTRINSIC_HEADERS})

source_group("Intrinsic headers" FILES ${INTRINSIC_HEADERS} )

target_link_libraries(manipulatormodul lowlevelcontrol)
requires_eigen(manipulatormodul)

set_target_properties(manipulatormodul PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(manipulatormodul PROPERTIES FOLDER "Lib")

install_lib(manipulatormodul "${HEADERS}" ${CMAKE_CURRENT_SOURCE_DIR})
