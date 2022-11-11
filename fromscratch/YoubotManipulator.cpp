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
