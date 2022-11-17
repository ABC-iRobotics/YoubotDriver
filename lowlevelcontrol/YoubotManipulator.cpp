#include "YoubotManipulator.hpp"
#include <iostream>
#include <stdexcept>

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
	joints.push_back({ config.jointIndices[i], config.jointConfigs[i], center });
  }

  // Set ProcessMsgFromSlave sizes
  for (int i = 0; i < 5; i++)
	center->SetProcessFromSlaveSize(20, config.jointIndices[i]);
}

YoubotJoint& YoubotManipulator::GetJoint(int i) {
  return joints[i];
}

void YoubotManipulator::ConfigJoints() {
  for (int i = 0; i < 5; i++)
	joints[i].ConfigParameters();
}

bool YoubotManipulator::CheckJointConfigs() {
  for (int i = 0; i < 5; i++)
	if (!joints[i].CheckConfig())
	  return false;
  return true;
}

void YoubotManipulator::InitializeAllJoints() {
	for (auto& it : joints)
	  if (!it.IsInitialized()) {
		auto status = it.GetJointStatusViaMailbox();
		std::cout << status.toString() << std::endl;
		it.ResetTimeoutViaMailbox();
		it.ResetI2TExceededViaMailbox();
		status = it.GetJointStatusViaMailbox();
		std::cout << status.toString() << std::endl;
		it.StartInitialization();

		for (int i = 0; i < 300; i++)
		  if (it.IsInitialized()) {
			std::cout << " joint isInitialized" << std::endl;
			break;
		  }
		if (!it.IsInitialized())
		  throw std::runtime_error("One joint is not initialized and cannot be done it...");
	}
}

enum CalibState : uint8_t {
  MOVED = 0,
  ENCODER_SET = 1,
  PEACE = 2
};

void YoubotManipulator::Calibrate() {
  const double calJointRadPerSec = 0.2;
  for (int i = 0; i < 5; i++) {
	bool forward = config.jointConfigs[i].at("CalibrationDirection");
	joints[i].ResetI2TExceededViaMailbox();
	joints[i].ReqVelocityJointRadPerSec(forward ? calJointRadPerSec : -calJointRadPerSec);
	log(Log::info, "Calibration of joint " + std::to_string(i) + "started");
  }
  for (int i = 0; i < 5; i++)
	joints[i].ResetTimeoutViaMailbox();

  CalibState jointcalstate[5] = { MOVED,MOVED,MOVED,MOVED,MOVED };

  do {
	center->ExchangeProcessMsg();
	std::string str = "Calibration currents: ";
	for (int i = 0; i < 5; i++)
	  switch (jointcalstate[i]) {
	  case MOVED: {
		int curr = joints[i].GetProcessReturnData().currentmA;
		str = str + std::to_string(curr) + " ";
		if (abs(curr) >= config.jointConfigs[i].at("CalibrationCurrentmA")) {
		  log(Log::info, "Joint " + std::to_string(i) + " calibrated");
		  joints[i].ReqEncoderReference(0);
		  jointcalstate[i] = ENCODER_SET;
		}
		break;
	  }
	  case ENCODER_SET:
		str = str + " under_set ";
		joints[i].ReqVoltagePWM(0);
		jointcalstate[i] = PEACE;
		break;
	  case PEACE:
		str = str + " set ";
		break;
	  }
	log(Log::info, str);
	//joints[4].GetProcessReturnData().Print();
  } while (jointcalstate[0] != PEACE || jointcalstate[1] != PEACE ||
	jointcalstate[2] != PEACE || jointcalstate[3] != PEACE || jointcalstate[4] != PEACE);
  log(Log::info, "After calibration:");
  for (auto& it : joints)
	it.GetProcessReturnData().Print();
