#include "YoubotManipulator.hpp"
#include <stdexcept>
#include "Logger.hpp"
#include "Time.hpp"

YoubotManipulator::YoubotManipulator(const YoubotConfig& config, VMessageCenter* center)
  : config(config), center(center) {
  // Check slavenames
  if (center->getSlaveNum() < 6)
	throw std::runtime_error("Less than 6 ethercat slaves");
  for (int i = 0; i < 5; i++) {
	if (config.jointIndices[i] >= center->getSlaveNum())
	  throw std::runtime_error("Slave index " + std::to_string(config.jointIndices[i]) + " not found");
	if (center->getSlaveName(config.jointIndices[i]).compare("TMCM-1610") != 0)
	  throw std::runtime_error("Unknown module name " + center->getSlaveName(config.jointIndices[i]));
	joints.push_back(std::make_shared<YoubotJoint>(config.jointIndices[i], config.jointConfigs[i], center));
  }

  // Set ProcessMsgFromSlave sizes
  for (int i = 0; i < 5; i++)
	center->SetProcessFromSlaveSize(20, config.jointIndices[i]);
}

YoubotJoint::Ptr YoubotManipulator::GetJoint(int i) {
  return joints[i];
}

void YoubotManipulator::ConfigJoints(bool forceConfiguration) {
  for (int i = 0; i < 5; i++)
	joints[i]->ConfigParameters(forceConfiguration);
}

bool YoubotManipulator::CheckJointConfigs() {
  for (int i = 0; i < 5; i++)
	if (!joints[i]->CheckConfig())
	  return false;
  return true;
}

void YoubotManipulator::InitializeAllJoints() {
  for (auto& it : joints) {
	it->Initialize();
	SLEEP_MILLISEC(200); // Wait to finish the moves - controller likes it
  }
}

enum CalibState : uint8_t {
  TO_CALIBRATE = 0,
  ENCODER_SETTING = 1,
  PEACE = 2,
  IDLE = 3
};

void YoubotManipulator::Calibrate(bool forceCalibration) {
  CalibState jointcalstate[5];

  const double calJointRadPerSec = 0.2;
  for (int i = 0; i < 5; i++)
	if (joints[i]->IsCalibratedViaMailbox() && !forceCalibration)
	  jointcalstate[i] = IDLE;
	else {
	  jointcalstate[i] = TO_CALIBRATE;
	  bool forward = config.jointConfigs[i].at("CalibrationDirection");
	  joints[i]->ResetI2TExceededViaMailbox();
	  joints[i]->ReqVelocityJointRadPerSec(forward ? calJointRadPerSec : -calJointRadPerSec);
	  log(Log::info, "Calibration of joint " + std::to_string(i) + "started");
	}
  for (int i = 0; i < 5; i++)
	if (jointcalstate[i] != IDLE)
	  joints[i]->ResetTimeoutViaMailbox();

  do {
	center->ExchangeProcessMsg();
	std::string str = "Calibration currents: ";
	for (int i = 0; i < 5; i++)
	  switch (jointcalstate[i]) {
	  case TO_CALIBRATE: {
		int curr = joints[i]->GetProcessReturnData().currentmA;
		str = str + std::to_string(curr) + " ";
		if (abs(curr) >= config.jointConfigs[i].at("CalibrationCurrentmA")) {
		  log(Log::info, "Joint " + std::to_string(i) + " calibrated");
		  joints[i]->ReqEncoderReference(0);
		  jointcalstate[i] = ENCODER_SETTING;
		}
		break;
	  }
	  case ENCODER_SETTING: {
		int enc = joints[i]->GetProcessReturnData().encoderPosition;
		if (enc == 0) {
		  str = str + " to_set ";
		  joints[i]->ReqVoltagePWM(0);
		  jointcalstate[i] = PEACE;
		}
		else str = str + " under_set ";
	  }
	  case PEACE:
		str = str + " set ";
		break;
	  case IDLE:
		str = str + " - ";
		break;
	  }
	log(Log::info, str);
	SLEEP_MILLISEC(3);
  } while (jointcalstate[0] < PEACE || jointcalstate[1] < PEACE ||
	jointcalstate[2] < PEACE || jointcalstate[3] < PEACE || jointcalstate[4] < PEACE);
  for (int i = 0; i < 5; i++)
	if (jointcalstate[i] == PEACE)
	  joints[i]->SetCalibratedViaMailbox();
  for (int i = 0; i < 5; i++)
	joints[i]->IsCalibratedViaMailbox();
  log(Log::info, "After calibration:");
  for (auto& it : joints)
	it->GetProcessReturnData().Print();
}

void YoubotManipulator::ReqJointPosition(double q0, double q1, double q2, double q3, double q4) {
  joints[0]->ReqJointPositionDeg(q0);
  joints[1]->ReqJointPositionDeg(q1);
  joints[2]->ReqJointPositionDeg(q2);
  joints[3]->ReqJointPositionDeg(q3);
  joints[4]->ReqJointPositionDeg(q4);
}

void YoubotManipulator::_processThreadFunc(int sleepMS) {
  isRunning = true;
  for (int i = 0; i < 5; i++)
	joints[i]->ResetI2TExceededViaMailbox();
  for (int i = 0; i < 5; i++)
	joints[i]->ResetTimeoutViaMailbox();

  while (!toStopThread) {
	center->ExchangeProcessMsg();
	log(Log::info, "Thread running");
	SLEEP_MILLISEC(sleepMS);
  }
  isRunning = false;
}

YoubotManipulator::~YoubotManipulator() {
  StopProcessThread();
}

void YoubotManipulator::StartProcessThread(int sleepMS) {
  toStopThread = false;
  thread = std::thread([this, sleepMS] { _processThreadFunc(sleepMS); });
  thread.detach();
}

void YoubotManipulator::StopProcessThread() {
  toStopThread = true;
  while (isRunning)
	SLEEP_MILLISEC(1);
  if (thread.joinable())
	thread.join();
}