
set(SOURCES slaveinfo.c)
add_executable(slaveinfo ${SOURCES})
target_link_libraries(slaveinfo soem)


set_target_properties(slaveinfo PROPERTIES LINKER_LANGUAGE CXX)

set_target_properties(slaveinfo PROPERTIES FOLDER "App")