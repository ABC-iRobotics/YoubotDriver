
set(SOURCES findcica.cpp
      adapters.cpp
	  )

set(HEADERS
      adapters.hpp
      DataObjectLockFree.hpp
)

add_library(lowlevelcontrol STATIC ${SOURCES} ${HEADERS})

target_link_libraries(lowlevelcontrol soem)

set_target_properties(lowlevelcontrol PROPERTIES LINKER_LANGUAGE CXX)
