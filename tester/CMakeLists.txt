
set(SOURCES
		tester.cpp
	  )

set(HEADERS
)

add_executable(tester ${SOURCES} ${HEADERS})

target_link_libraries(tester lowlevelcontrol)
set_target_properties(tester PROPERTIES LINKER_LANGUAGE CXX)

set_target_properties(tester PROPERTIES FOLDER "App")