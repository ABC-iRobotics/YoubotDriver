
set(SOURCES 
    findcica.cpp
	)

set(HEADERS
	)

add_executable(fromscratch ${SOURCES} ${HEADERS})

target_link_libraries(fromscratch soem lowlevelcontrol)

target_include_directories(fromscratch PUBLIC ${CMAKE_SOURCE_DIR}/lowlevelcontrol)

set_target_properties(fromscratch PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(fromscratch PUBLIC
  _USE_MATH_DEFINES
  _CRT_SECURE_NO_WARNINGS
  __PRETTY_FUNCTION__=__FUNCSIG__ #only on Windows
  _ALLOW_KEYWORD_MACROS #macroised inline...
  )

IF(MSVC)
    ADD_DEFINITIONS("/EHsc")
ENDIF(MSVC)