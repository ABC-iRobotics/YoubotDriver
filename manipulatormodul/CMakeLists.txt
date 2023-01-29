
set(SOURCES
		YoubotManipulatorModul.cpp
	  )

set(HEADERS
		YoubotManipulatorModul.hpp
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
