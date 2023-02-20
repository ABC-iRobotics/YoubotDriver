#ifndef MANIPULATOR_HPP
#define MANIPULATOR_HPP

#include <vector>
#include "Joint.hpp"
#include "Config.hpp"
#include "EtherCATMaster.hpp"

namespace youbot {

  /// <summary>
  /// Describes the state of the 5 joints
  /// </summary>
  struct JointsState {
	JointState joint[5];
  };

  /// <summary>
  /// Class to cover the low-level functionality (initialization and communication) of the youbot manipulator arm
  /// 
  /// Currently only the 5 joints
  /// </summary>
  class Manipulator {
  public:
	Manipulator() = delete; ///<  Unavailable constructor
	Manipulator(Manipulator&) = delete; ///<  Unavailable constructor
	Manipulator(const Manipulator&) = delete; ///<  Unavailable constructor

	/// <summary>
	/// Constructor of the class. Only saves the config instance, the pointer to the ethercat master and calls the constructors of the joints.
	/// </summary>
	/// <param name="config"></param>
	/// <param name="center"></param>
	Manipulator(const Config& config, EtherCATMaster::Ptr center);
	~Manipulator() {}; ///< Destructor

	/// <summary>
	/// Main initialization of the robot joints. Calls their Initialize methods (collect parameters, config and init commutation)
	/// </summary>
	/// <param name="forceConfiguration"> forwarded to Joint::Config </param>
	void InitializeManipulator(bool forceConfiguration = false);
	void CollectBasicJointParameters(); ///< Calls Joint::CollectBasicParameters
	void ConfigJointControlParameters(bool forceConfiguration = false); ///< Calls Joint::ConfigControlParameters
	bool CheckJointControlParameters(); ///< Calls Joint::CheckControlParameters
	void InitJointCommutation(); ///< Calls Joint::InitializeJoint

	// Calibrate the manipulator
	void Calibrate(bool forceCalibration = false); ///< Calibration subroutine
	bool IsAllJointsCalibratedViaMailbox();  ///< Calls Joint::IsCalibratedViaMailbox

	// To handle timeout/I2t error flags
	void CheckAndResetErrorFlagsViaMailbox(); ///< To check and reset i2t and timeout
	void CheckAndResetI2tFlagsViaMailbox(); ///< To check and reset i2t

	// To get joint operations
	Joint::Ptr GetJoint(int i); ///< Get a Joint::Ptr for joint-wise operations. NOT thread-safe.

	// Process message related functions
	void ReqManipulatorStop(); ///< Process buffer: Set stop joint commands into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
	void ReqJointPositionRad(double q0, double q1, double q2, double q3, double q4); ///< Process buffer: Set joint position commands into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
	void ReqJointSpeedRadPerSec(double dq0, double dq1, double dq2, double dq3, double dq4); ///< Process buffer: Set joint speed commands into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
	void ReqJointTorqueNm(double tau0, double tau1, double tau2, double tau3, double tau4); ///< Process buffer: Set joint torque commands into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
	void ReqZeroVoltage(); ///< Process buffer: Set zero motor voltage commands into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)

	void GetQLatest(double& q0, double& q1, double& q2, double& q3, double& q4); ///< Thread-safe get latest joint positions, returns the values from the latest mailbox or process messages
	void GetDQLatest(double& dq0, double& dq1, double& dq2, double& dq3, double& dq4); ///< Thread-safe get latest joint speeds, returns the values from the latest mailbox or process messages
	void GetTauLatest(double& tau0, double& tau1, double& tau2, double& tau3, double& tau4); ///< Thread-safe get latest joint torques, returns the values from the latest mailbox or process messages

	void CheckI2tAndTimeoutErrorProcess(); ///< Throw std::runtime_error if timeout or I2t error occured
	void LogStatusProcess(); ///< Log the current state of the manipulator

	JointsState GetStateLatest() const; ///< Thread-safe get the whole (latest) state of the manipulator

  private:
	const Config config;
	EtherCATMaster::Ptr center;
	std::vector<Joint::Ptr> joints = {NULL, NULL, NULL, NULL, NULL};
  };
}
#endif
