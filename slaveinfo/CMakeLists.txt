
set(SOURCES slaveinfo.cpp)
add_executable(slaveinfo ${SOURCES})
target_link_libraries(slaveinfo lowlevelcontrol)


set_target_properties(slaveinfo PROPERTIES LINKER_LANGUAGE CPP)

set_target_properties(slaveinfo PROPERTIES FOLDER "App")