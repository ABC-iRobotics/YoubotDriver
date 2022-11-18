
set(SOURCES
		tester.cpp
	  )

set(HEADERS
)

add_executable(tester ${SOURCES} ${HEADERS})

#requires_soem(lowlevelcontrol)
#requires_spdlog(lowlevelcontrol)
#requires_eigen(lowlevelcontrol)

#target_include_directories(lowlevelcontrol PUBLIC ${CMAKE_SOURCE_DIR}/lowlevelcontrol)

target_link_libraries(tester lowlevelcontrol)
set_target_properties(tester PROPERTIES LINKER_LANGUAGE CXX)

