
set(SOURCES
		adapters.cpp
		Logger.cpp
		SimpleOpenEtherCATMaster.cpp
		TMCLMailboxMessage.cpp
		TMCLDefinitions.cpp
		EtherCATMaster.cpp
		YoubotJoint.cpp
		YoubotJointPhysical.cpp
		YoubotJointVirtual.cpp
		YoubotConfig.cpp
		YoubotManipulator.cpp
		ProcessBuffer.cpp
	  )

set(HEADERS
		adapters.hpp
		Time.hpp
		Logger.hpp
		EtherCATMaster.hpp
		MailboxMessage.hpp
		YoubotJoint.hpp
		YoubotJointPhysical.hpp
		YoubotJointVirtual.hpp
		YoubotConfig.hpp
		YoubotManipulator.hpp
		ProcessBuffer.hpp
)

set(INTRINSIC_HEADERS
		DataObjectLockFree.hpp
		SimpleOpenEtherCATMaster.hpp
		TMCLMailboxMessage.hpp
		TMCLDefinitions.hpp
)

add_library(lowlevelcontrol STATIC ${SOURCES} ${HEADERS} ${INTRINSIC_HEADERS})

source_group("Intrinsic headers" FILES ${INTRINSIC_HEADERS} )

requires_soem(lowlevelcontrol)
requires_spdlog(lowlevelcontrol)
requires_eigen(lowlevelcontrol)

set_target_properties(lowlevelcontrol PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(lowlevelcontrol PUBLIC WIN32 #only on Windows
  __STDC_LIMIT_MACROS # for INT_MAX
  _USE_MATH_DEFINES # for M_PI
  __PRETTY_FUNCTION__=__FUNCSIG__ #only on Windows
  _ALLOW_KEYWORD_MACROS #macroised inline...
  )

IF(MSVC)
    ADD_DEFINITIONS("/EHsc")
ENDIF(MSVC)

install_lib(lowlevelcontrol "${HEADERS}" ${CMAKE_CURRENT_SOURCE_DIR})
