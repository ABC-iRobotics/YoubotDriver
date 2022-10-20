
set(SOURCES findcica.c
      adapters.c)

set(HEADERS
      adapters.h)

add_executable(lowlevelcontrol ${SOURCES} ${HEADERS})

target_link_libraries(lowlevelcontrol soem)

set_target_properties(lowlevelcontrol PROPERTIES LINKER_LANGUAGE CXX)
