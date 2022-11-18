
set(SOURCES
		adapters.cpp
		Logger.cpp
		SOEMMessageCenter.cpp
		TMCLMailboxMessage.cpp
		TMCLDefinitions.cpp
		VMessageCenter.cpp
		YoubotJoint.cpp
		YoubotConfig.cpp
		YoubotManipulator.cpp
		ProcessBuffer.cpp
	  )

set(HEADERS
		adapters.hpp
		Time.hpp
		Logger.hpp
		VMessageCenter.hpp
		MailboxMessage.hpp
		YoubotJoint.hpp
		YoubotConfig.hpp
		YoubotManipulator.hpp
		ProcessBuffer.hpp
)

set(INTRINSIC_HEADERS
		DataObjectLockFree.hpp
		SOEMMessageCenter.hpp
		TMCLMailboxMessage.hpp
		TMCLDefinitions.hpp
)

add_library(lowlevelcontrol STATIC ${SOURCES} ${HEADERS} ${INTRINSIC_HEADERS})

source_group("Intrinsic headers" FILES ${INTRINSIC_HEADERS} )

requires_soem(lowlevelcontrol)
requires_spdlog(lowlevelcontrol)
requires_eigen(lowlevelcontrol)

target_include_directories(lowlevelcontrol PUBLIC ${CMAKE_SOURCE_DIR}/lowlevelcontrol)

set_target_properties(lowlevelcontrol PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(lowlevelcontrol PUBLIC WIN32
  __STDC_LIMIT_MACROS
  __STDC_WANT_SECURE_LIB__=1
  _USE_MATH_DEFINES
  _CRT_SECURE_NO_WARNINGS
  __PRETTY_FUNCTION__=__FUNCSIG__ #only on Windows
  _ALLOW_KEYWORD_MACROS #macroised inline...
  )

IF(MSVC)
    ADD_DEFINITIONS("/EHsc")
ENDIF(MSVC)
