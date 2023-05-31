
set(SOURCES
		Logger.cpp
		VirtualEtherCATMaster.cpp
		TMCLMailboxMessage.cpp
		TMCLDefinitions.cpp
		EtherCATMaster.cpp
		Joint.cpp
		JointVirtual.cpp
		Config.cpp
		Manipulator.cpp
		JointState.cpp
		ProcessBuffer.cpp
	  )
if (NOT ONLY_VIRTUAL_ROBOT)
  list(APPEND SOURCES
		SimpleOpenEtherCATMaster.cpp
		JointPhysical.cpp
		adapters.cpp
  )
endif()

set(HEADERS
		Time.hpp
		Logger.hpp
		EtherCATMaster.hpp
		MailboxMessage.hpp
		Joint.hpp
		Config.hpp
		Manipulator.hpp
		ProcessBuffer.hpp
		JointState.hpp
		Data.hpp
)
if (NOT ONLY_VIRTUAL_ROBOT)
  list(APPEND HEADERS
		adapters.hpp
  )
endif()

set(INTRINSIC_HEADERS
		VirtualEtherCATMaster.hpp
		JointVirtual.hpp
		TMCLMailboxMessage.hpp
		TMCLDefinitions.hpp
)
if (NOT ONLY_VIRTUAL_ROBOT)
  list(APPEND INTRINSIC_HEADERS
		SimpleOpenEtherCATMaster.hpp
		JointPhysical.hpp
		adapters.hpp
  )
endif()

add_library(lowlevelcontrol STATIC ${SOURCES} ${HEADERS} ${INTRINSIC_HEADERS})

source_group("Intrinsic headers" FILES ${INTRINSIC_HEADERS} )
if (NOT ONLY_VIRTUAL_ROBOT)
  requires_soem(lowlevelcontrol)
endif()
requires_spdlog(lowlevelcontrol)

set_target_properties(lowlevelcontrol PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(lowlevelcontrol PUBLIC WIN32 #only on Windows
  __STDC_LIMIT_MACROS # for INT_MAX
  _USE_MATH_DEFINES # for M_PI
  __PRETTY_FUNCTION__=__FUNCSIG__ #only on Windows
  _ALLOW_KEYWORD_MACROS #macroised inline...
  )

if (ONLY_VIRTUAL_ROBOT)
  target_compile_definitions(lowlevelcontrol PUBLIC 
  _ONLY_VIRTUAL_ROBOT
  )
endif()

IF(MSVC)
    ADD_DEFINITIONS("/EHsc")
ENDIF(MSVC)

install_lib(lowlevelcontrol "${HEADERS}" ${CMAKE_CURRENT_SOURCE_DIR})
