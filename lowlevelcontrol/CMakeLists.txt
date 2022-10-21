
set(SOURCES findcica.cpp
      adapters.cpp
	  YouBotJointParameter.cpp
	  JointData.cpp
	  YouBotJointParameterReadOnly.cpp
	  YouBotJoint.cpp
	  YouBotJointParameterPasswordProtected.cpp
	  )

set(HEADERS
      adapters.hpp
      DataObjectLockFree.hpp
	  YouBotSlaveMsg.hpp
	  YouBotJointParameter.hpp
	  JointData.hpp
	  YouBotSlaveMailboxMsg.hpp
	  YouBotJointStorage.hpp
	  TMCLProtocolDefinitions.hpp
	  YouBotJointParameterReadOnly.hpp
	  YouBotJoint.hpp
	  Joint.hpp
	  YouBotJointParameterPasswordProtected.hpp
)

add_executable(lowlevelcontrol ${SOURCES} ${HEADERS})

target_link_libraries(lowlevelcontrol soem)

set_target_properties(lowlevelcontrol PROPERTIES LINKER_LANGUAGE CXX)
