
set(SOURCES 
	  YouBotJointParameter.cpp
	  JointData.cpp
	  YouBotJointParameterReadOnly.cpp
	  YouBotJoint.cpp
	  YouBotJointParameterPasswordProtected.cpp
      TMCLProtocolDefinitions.hpp
    ConfigFile.cpp
    DataTrace.cpp
    EthercatMaster.cpp
    EthercatMasterWithoutThread.cpp
    EthercatMasterWithThread.cpp
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
    findcica.cpp
    EthercatMasterInterface.cpp
	Logger.cpp
	PidController.cpp
	  )

set(HEADERS
	  YouBotSlaveMsg.hpp
	  Exceptions.hpp
	  PidController.hpp
	  Time.hpp
	  Logger.hpp
	  DataObjectLockFree.hpp
	  YouBotJointParameter.hpp
	  JointData.hpp
	  YouBotSlaveMailboxMsg.hpp
	  YouBotJointInfo.hpp
	  YouBotJointParameterReadOnly.hpp
	  YouBotJoint.hpp
	  YouBotJointParameterPasswordProtected.hpp
    ConfigFile.hpp
    Units.hpp
    DataTrace.hpp
    EthercatMaster.hpp
    EthercatMasterInterface.hpp
    EthercatMasterWithoutThread.hpp
    EthercatMasterWithThread.hpp
    GripperDataTrace.hpp
    JointLimitMonitor.hpp
    JointTrajectoryController.hpp
    YouBotBase.hpp
    YouBotGripper.hpp
    YouBotGripperBar.hpp
    YouBotGripperParameter.hpp
    YouBotManipulator.hpp
    OneDOFGripperData.hpp
    JointTrajectory.hpp
    FourSwedishWheelOmniBaseKinematic.hpp
    FourSwedishWheelOmniBaseKinematicConfiguration.hpp
)

add_executable(oldshit ${SOURCES} ${HEADERS})

target_link_libraries(oldshit soem Boost::thread Boost::date_time
  Boost::filesystem Boost::system Boost::regex)

target_include_directories(oldshit PUBLIC ${Boost_INCLUDE_DIRS})

set_target_properties(oldshit PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(oldshit PUBLIC
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
# __STDC_WANT_SECURE_LIB__ __STDC_CONSTANT_MACROS)
#TODO: set toolset and MT
#later:
#if !defined(__PRETTY_FUNCTION__) && !defined(__GNUC__)
#define __PRETTY_FUNCTION__ __FUNCSIG__
#endif