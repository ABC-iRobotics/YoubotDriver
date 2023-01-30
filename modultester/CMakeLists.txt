
set(SOURCES
		modultester.cpp
	  )

set(HEADERS
)

add_executable(modultester ${SOURCES} ${HEADERS})

target_link_libraries(modultester manipulatormodul)
set_target_properties(modultester PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(modultester PROPERTIES FOLDER "App")

target_compile_definitions(modultester PUBLIC CONFIG_FOLDER="${CMAKE_SOURCE_DIR}/config/")