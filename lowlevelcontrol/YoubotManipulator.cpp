#include "YoubotManipulator.hpp"
#include "YoubotJointPhysical.hpp"
#include "YoubotJointVirtual.hpp"
#include <stdexcept>
#include "Logger.hpp"
#include "Time.hpp"

using namespace youbot;

YoubotManipulator::YoubotManipulator(const YoubotConfig& config, EtherCATMaster::Ptr center)
  : config(config), center(center) {
  // Construct joints
  for (int i = 0; i < 5; i++)
	if (center->GetType() == EtherCATMaster::PHYSICAL) {
	  auto ptr = std::make_shared<YoubotJointPhysical>(config.jointIndices[i], config.jointConfigs[i], center);
	  ptr->Init();
	  joints[i] = ptr;
	}
	else
	  joints[i] = std::make_shared<YoubotJointVirtual>(config.jointIndices[i], config.jointConfigs[i], center);
}

YoubotJointPhysical::Ptr YoubotManipulator::GetJoint(int i) {
  return joints[i];
}

void YoubotManipulator::CollectBasicJointParameters() {
  for (int i = 0; i < 5; i++)
	joints[i]->ConfigControlParameters();
}

void YoubotManipulator::ConfigJointControlParameters(bool forceConfiguration) {
  for (int i = 0; i < 5; i++)
	joints[i]->ConfigControlParameters(forceConfiguration);
}

bool YoubotManipulator::CheckJointControlParameters() {
  for (int i = 0; i < 5; i++)
	if (!joints[i]->CheckControlParameters())
	  return false;
  return true;
}

void YoubotManipulator::InitializeManipulator(bool forceConfiguration) {
  for (int i = 0; i < 5; i++)
	joints[i]->InitializeJoint(forceConfiguration);
}

void YoubotManipulator::InitJointCommutation() {
  for (auto& it : joints)
	it->InitCommutation();
}

void YoubotManipulator::Calibrate(bool forceCalibration) {
  // Check if all joints are calibrated
  if (!forceCalibration)
	if (IsAllJointsCalibratedViaMailbox())
	  return; // if all of them are calibrated, then calibration is not necessary, return
  // otherwise perform the full calibration
  const double calJointRadPerSec = 0.35;
  // RESET I2t flags
  CheckAndResetI2tFlagsViaMailbox();
  // Req calibration velocity
  for (int i = 0; i < 5; i++) {
	bool backward = bool(config.jointConfigs[i].at("CalibrationDirection"))
	  ^ bool(config.jointConfigs[i].at("qDirectionSameAsEnc"));
	joints[i]->ReqJointSpeedRadPerSec(backward ? -calJointRadPerSec : calJointRadPerSec);
	log(Log::info, "Calibration of joint " + std::to_string(i) + "started");
  }
  // Reset timeouts (after all possible i2t reset or it can go to timeout again)
  for (int i = 0; i < 5; i++)
	joints[i]->ResetTimeoutViaMailbox();
  // Main calibration loop: move with constant speed and check if it has stopped
  {
	auto start = std::chrono::steady_clock::now();
	int reached_since = 0; // reached the limit position for x cycles 
	bool still_moving[5] = { true,true,true,true,true };
	int cycles_in_zero_speed[5] = { 0,0,0,0,0 }; //counters for ~0 speed, if achieves 5->still moving to zero
	center->ExchangeProcessMsg(); // do sg to avoid a new timout
	SLEEP_MILLISEC(2); // Wait until the status flag of process messages will be refreshed (without timeout flag)
	do {
	  center->ExchangeProcessMsg();
	  // Check status
	  CheckI2tAndTimeoutErrorProcess();
	  // Check if enough time elapsed (in the first 200ms, the joints can start to move)
	  SLEEP_MILLISEC(3);
	  if (std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - start).count() < 200)
		continue; // in the first 200[ms] let the joints start moving
	  // if dt>200ms, check if the joint has already stopped
	  std::string str = "Calibration vel: ";
	  for (int i = 0; i < 5; i++)
		if (still_moving[i]) {
		  int vel = joints[i]->GetRPMLatest().value;
		  str = str + std::to_string(vel) + "RPM ";
		  if (vel<5 && vel>-5) // if it is ~stopped, increase its counter
			cycles_in_zero_speed[i]++;
		  else
			cycles_in_zero_speed[i] = 0; // if it is moving, null the counter
		  // if it has stopped for 5 cycles, hold via small current
		  if (cycles_in_zero_speed[i] >= 5) {
			log(Log::info, "Joint " + std::to_string(i) + " at endposition");
			bool sign = bool(config.jointConfigs[i].at("CalibrationDirection"));
			int holding_current = 30; //[mA]
			joints[i]->ReqMotorCurrentmA(sign ? holding_current : -holding_current);
			still_moving[i] = false; // change state
		  }
		}
		else { // if it has reached the holding state do nothing
		  int ticks = joints[i]->GetTicksLatest().value;
		  str = str + std::to_string(ticks) + "ticks ";
		}
	  log(Log::info, str);
	  // if all joints just holding increase a counter - to let the hold current command start working
	  if (still_moving[0] || still_moving[1] || still_moving[2]
		|| still_moving[3] || still_moving[4])
		reached_since = 0;
	  else
		reached_since++;
	} while (reached_since <= 5);
  }
  // Now all joints are holding the endposition with 30mA
  // Start reference setting
  // (it does need a few milliseconds, we wait here still the returned positions become ~0)
  {
	auto start = std::chrono::steady_clock::now();
	for (int i = 0; i < 5; i++)
	  joints[i]->ReqEncoderReference(0);
	bool allSet;
	do {
	  center->ExchangeProcessMsg(); // do sg to avoid a new timout
	  allSet = true;
	  std::string str;
	  for (int i = 0; i < 5; i++) {
		int ticks = joints[i]->GetTicksLatest().value;
		if (abs(ticks) > 3)
		  allSet = false;
		int mA = joints[i]->GetMALatest().value;
		str = str + std::to_string(ticks) + "ticks (" + std::to_string(mA) + "mA) ";
	  }
	  log(Log::info, str);
	  int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - start).count();
	  if (elapsed_ms > 2000) {
		// Stop the manipulator
		ReqManipulatorStop();
		center->ExchangeProcessMsg();
		// Save/send error messages
		log(Log::fatal, "During calibration unsuccessful reference setting (>2000[ms])");
		SLEEP_MILLISEC(2) // leave time to log...
		throw std::runtime_error("During calibration unsuccessful reference setting (>2000[ms])");
	  }
	  SLEEP_MILLISEC(2)
	} while (!allSet);
	int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
	  std::chrono::steady_clock::now() - start).count();
	log(Log::info, "Encoders nulled, elapsed time: " + std::to_string(elapsed_ms) + "[ms]");
  }
  // Set to idle:
  for (int i = 0; i < 5; i++)
	joints[i]->ReqVoltagePWM(0);
  center->ExchangeProcessMsg();
  // Save that the joints are calibrated now
  for (int i = 0; i < 5; i++)
	joints[i]->SetCalibratedViaMailbox();
  // Print the after calibration status
  log(Log::info, "After calibration:");
  for (auto& it : joints) {
	it->LogLatestState();
  }
}

void YoubotManipulator::ReqJointPositionRad(double q0,
  double q1, double q2, double q3, double q4) {
  joints[0]->ReqJointPositionRad(q0);
  joints[1]->ReqJointPositionRad(q1);
  joints[2]->ReqJointPositionRad(q2);
  joints[3]->ReqJointPositionRad(q3);
  joints[4]->ReqJointPositionRad(q4);
}

void YoubotManipulator::GetQLatest(double& q0,
  double& q1, double& q2, double& q3, double& q4) {
  q0 = joints[0]->GetQLatestRad().value;
  q1 = joints[1]->GetQLatestRad().value;
  q2 = joints[2]->GetQLatestRad().value;
  q3 = joints[3]->GetQLatestRad().value;
  q4 = joints[4]->GetQLatestRad().value;
}

void youbot::YoubotManipulator::ReqJointSpeedRadPerSec(double dq0, double dq1, double dq2, double dq3, double dq4) {
  joints[0]->ReqJointSpeedRadPerSec(dq0);
  joints[1]->ReqJointSpeedRadPerSec(dq1);
  joints[2]->ReqJointSpeedRadPerSec(dq2);
  joints[3]->ReqJointSpeedRadPerSec(dq3);
  joints[4]->ReqJointSpeedRadPerSec(dq4);
}

void youbot::YoubotManipulator::GetDQLatest(double& dq0, double& dq1, double& dq2, double& dq3, double& dq4) {
  dq0 = joints[0]->GetDQLatestRad().value;
  dq1 = joints[1]->GetDQLatestRad().value;
  dq2 = joints[2]->GetDQLatestRad().value;
  dq3 = joints[3]->GetDQLatestRad().value;
  dq4 = joints[4]->GetDQLatestRad().value;
}

void youbot::YoubotManipulator::ReqJointTorqueNm(double tau0, double tau1, double tau2, double tau3, double tau4) {
  joints[0]->ReqJointTorqueNm(tau0);
  joints[1]->ReqJointTorqueNm(tau1);
  joints[2]->ReqJointTorqueNm(tau2);
  joints[3]->ReqJointTorqueNm(tau3);
  joints[4]->ReqJointTorqueNm(tau4);
}

void youbot::YoubotManipulator::ReqZeroVoltage() {
  joints[0]->ReqVoltagePWM(0);
  joints[1]->ReqVoltagePWM(0);
  joints[2]->ReqVoltagePWM(0);
  joints[3]->ReqVoltagePWM(0);
  joints[4]->ReqVoltagePWM(0);
}

void youbot::YoubotManipulator::GetTauLatest(double& tau0, double& tau1, double& tau2, double& tau3, double& tau4) {
  tau0 = joints[0]->GetTauLatestNm().value;
  tau1 = joints[1]->GetTauLatestNm().value;
  tau2 = joints[2]->GetTauLatestNm().value;
  tau3 = joints[3]->GetTauLatestNm().value;
  tau4 = joints[4]->GetTauLatestNm().value;
}

void youbot::YoubotManipulator::CheckAndResetI2tFlagsViaMailbox() {
  for (int i = 0; i < 5; i++) {
	// RESET I2t flags
	auto status = joints[i]->GetJointStatusViaMailbox();
	log(Log::info, status.toString());
	if (status.I2TExceeded())
	  joints[i]->ResetI2TExceededViaMailbox();
  }
}

void youbot::YoubotManipulator::CheckI2tAndTimeoutErrorProcess() {
  for (int i = 0; i < 5; i++) {
	auto status = joints[i]->GetStatusLatest().value;
	joints[i]->CheckI2tAndTimeoutError(status);
  }
}

void youbot::YoubotManipulator::LogStatusProcess() {
  log(Log::info, "Manipulator status: ");
  for (int i = 0; i < 5; i++)
	joints[i]->LogLatestState();
}

youbot::JointsState youbot::YoubotManipulator::GetStateLatest() const {
  JointsState out;
  for (int i = 0; i < 5; i++)
	out.joint[i] = joints[i]->GetLatestState();
  return out;
}

void YoubotManipulator::ReqManipulatorStop() {
  for (auto& it : joints)
	it->ReqStop();
}
void YoubotManipulator::CheckAndResetErrorFlags() {
  for (int i = 0; i < 5; i++) {
	auto status = joints[i]->GetJointStatusViaMailbox();
	if (status.I2TExceeded())
	  joints[i]->ResetI2TExceededViaMailbox();
  }
  for (int i = 0; i < 5; i++)
	joints[i]->ResetTimeoutViaMailbox();
}

bool youbot::YoubotManipulator::IsAllJointsCalibratedViaMailbox() {
  for (int i = 0; i < 5; i++)
	if (!joints[i]->IsCalibratedViaMailbox())
	  return false;
  return true;
}
