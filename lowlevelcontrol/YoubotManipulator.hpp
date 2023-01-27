#ifndef YOUBOT_MANIPULATOR_HPP
#define YOUBOT_MANIPULATOR_HPP

#include <vector>
#include "YoubotJoint.hpp"
#include "YoubotConfig.hpp"
#include "EtherCATMaster.hpp"

namespace youbot {

  class YoubotManipulator {
  public:
	// Unavailable constructors
	YoubotManipulator() = delete;
	YoubotManipulator(YoubotManipulator&) = delete;
	YoubotManipulator(const YoubotManipulator&) = delete;

	// Constructor
	YoubotManipulator(const YoubotConfig& config, EtherCATMaster::Ptr center);
	// Destructor
	~YoubotManipulator() {};

	// Main initialization of the robot joints
	void InitializeManipulator(bool forceConfiguration = false);
	// Submethods of joint initialization
	void CollectBasicJointParameters();
	void ConfigJointControlParameters(bool forceConfiguration = false);
	bool CheckJointControlParameters();
	void InitJointCommutation();

	// Calibrate the manipulator
	void Calibrate(bool forceCalibration = false);
	bool IsAllJointsCalibratedViaMailbox();

	// To handle timeout/I2t error flags
	void CheckAndResetErrorFlags(); // to check and reset i2t and timeout
	void CheckAndResetI2tFlagsViaMailbox(); // to check and reset i2t and timeout

	// To get joint operations
	YoubotJoint::Ptr GetJoint(int i);

	// Process message related functions
	void ReqManipulatorStop();
	void ReqJointPositionRad(double q0, double q1, double q2, double q3, double q4);
	void ReqJointSpeedRadPerSec(double dq0, double dq1, double dq2, double dq3, double dq4);
	void ReqJointTorqueNm(double tau0, double tau1, double tau2, double tau3, double tau4);
	void ReqZeroVoltage();

	void GetQLatest(double& q0, double& q1, double& q2, double& q3, double& q4);
	void GetDQLatest(double& dq0, double& dq1, double& dq2, double& dq3, double& dq4);
	void GetTauLatest(double& tau0, double& tau1, double& tau2, double& tau3, double& tau4);

	void CheckI2tAndTimeoutErrorProcess(); // runs to error if it finds sg in process msgs
	void LogStatusProcess();

	struct State {
	  struct Joint {
		Data<double> q, dq, tau;
		Data<YoubotJoint::JointStatus> status;
	  } joint[5];
	};

	State GetStateLatest() const;;

  private:
	const YoubotConfig config;
	EtherCATMaster::Ptr center;
	std::vector<YoubotJoint::Ptr> joints;
  };
}
#endif
