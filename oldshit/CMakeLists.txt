
set(SOURCES 
	  YouBotJointParameter.cpp
	  JointData.cpp
	  YouBotJointParameterReadOnly.cpp
	  YouBotJoint.cpp
	  YouBotJointParameterPasswordProtected.cpp
    ConfigFile.cpp
    Logger.cpp
    PidController.cpp
    DataTrace.cpp
    EtherCatMaster.cpp
    EtherCatMasterWithoutThread.cpp
    EtherCatMasterWithThread.cpp
    GripperDataTrace.cpp
    JointLimitMonitor.cpp
    JointTrajectoryController.cpp
    YouBotBase.cpp
    YouBotGripper.cpp
    YouBotGripperBar.cpp
    YouBotGripperParameter.cpp
    YouBotManipulator.cpp
    JointTrajectory.cpp
    FourSwedishWheelOmniBaseKinematic.cpp
    FourSwedishWheelOmniBaseKinematicConfiguration.cpp
    WheeledBaseKinematic.cpp
	  )

set(HEADERS
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
    ConfigFile.hpp
    Exceptions.hpp
    Logger.hpp
    PidController.hpp
    Time.hpp
    Units.hpp
    Gripper.hpp
    GripperData.hpp
    GripperParameter.hpp
    DataTrace.hpp
    EtherCatMaster.hpp
    EtherCatMasterInterface.hpp
    EtherCatMasterWithoutThread.hpp
    EtherCatMasterWithThread.hpp
    GripperDataTrace.hpp
    JointLimitMonitor.hpp
    JointTrajectoryController.hpp
    YouBotBase.hpp
    YouBotGripper.hpp
    YouBotGripperBar.hpp
    YouBotGripperParameter.hpp
    YouBotManipulator.hpp
    OneDOFGripperData.hpp
    OneDOFGripper.hpp
    JointTrajectory.hpp
    FourSwedishWheelOmniBaseKinematic.hpp
    FourSwedishWheelOmniBaseKinematicConfiguration.hpp
    WheeledBaseKinematic.hpp
)

add_executable(oldshit ${SOURCES} ${HEADERS})

target_link_libraries(oldshit soem lowlevelcontrol ${Boost_LIBRARIES})

target_include_directories(oldshit PUBLIC ${CMAKE_SOURCE_DIR}/lowlevelcontrol ${Boost_INCLUDE_DIRS})

set_target_properties(oldshit PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(oldshit PUBLIC
  __STDC_LIMIT_MACROS
  __STDC_WANT_SECURE_LIB__=1
  _USE_MATH_DEFINES
  _CRT_SECURE_NO_WARNINGS
  __PRETTY_FUNCTION__=__FUNCSIG__ #only on Windows
  )
# __STDC_WANT_SECURE_LIB__ __STDC_CONSTANT_MACROS)

#later:
#if !defined(__PRETTY_FUNCTION__) && !defined(__GNUC__)
#define __PRETTY_FUNCTION__ __FUNCSIG__
#endif