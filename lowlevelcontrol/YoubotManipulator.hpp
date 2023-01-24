#ifndef YOUBOT_MANIPULATOR_HPP
#define YOUBOT_MANIPULATOR_HPP

#include <vector>
#include "YoubotJointReal.hpp"
#include "YoubotConfig.hpp"

namespace youbot {

  class YoubotManipulator {
	const YoubotConfig config;
	EtherCATMaster* center;
	std::vector<YoubotJointReal::Ptr> joints;

  public:
	YoubotManipulator() = delete;

	YoubotManipulator(YoubotManipulator&) = delete;

	YoubotManipulator(const YoubotManipulator&) = delete;

	YoubotManipulator(const YoubotConfig& config, EtherCATMaster* center);

	~YoubotManipulator() {};

	YoubotJointReal::Ptr GetJoint(int i);

	void ConfigJoints(bool forceConfiguration = false);

	bool CheckJointConfigs();

	void SetParameters();

	void InitCommutationAllJoints();

	void Calibrate(bool forceCalibration = false);

	void ResetErrorFlags();

	bool IsAllJointsCalibratedViaMailbox();

	void CheckAndResetI2tFlagsViaMailbox();

	// Process message related functions
	void ReqManipulatorStop();

	void ReqJointPositionRad(double q0, double q1, double q2, double q3, double q4);

	void ReqJointSpeedRadPerSec(double dq0, double dq1, double dq2, double dq3, double dq4);

	void ReqJointTorqueNm(double tau0, double tau1, double tau2, double tau3, double tau4);

	void ReqZeroVoltage();

	void GetJointPositionRad(double& q0, double& q1, double& q2, double& q3, double& q4);

	void GetJointSpeedRadPerSec(double& dq0, double& dq1, double& dq2, double& dq3, double& dq4);

	void GetJointTorqueNm(double& tau0, double& tau1, double& tau2, double& tau3, double& tau4);

	void CheckI2tAndTimeoutErrorProcess();

	void LogStatusProcess();
  };
}
#endif
