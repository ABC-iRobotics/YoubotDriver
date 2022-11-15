#include "YoubotManipulator.hpp"

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
}

void YoubotManipulator::InitializeAllJoints() {
	for (auto& it : joints) {
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
	}
}
