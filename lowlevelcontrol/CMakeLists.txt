
set(SOURCES
      adapters.cpp
      PidController.cpp
	  Logger.cpp
	  )

set(HEADERS
      adapters.hpp
      DataObjectLockFree.hpp
      PidController.hpp
      Exceptions.hpp
      Time.hpp
	  Logger.hpp
)

add_library(lowlevelcontrol STATIC ${SOURCES} ${HEADERS})

target_link_libraries(lowlevelcontrol soem)

set_target_properties(lowlevelcontrol PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(lowlevelcontrol PUBLIC WIN32
  __STDC_LIMIT_MACROS
  __STDC_WANT_SECURE_LIB__=1
    _USE_MATH_DEFINES
  _CRT_SECURE_NO_WARNINGS
  )