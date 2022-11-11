#ifndef YOUBOT_MANIPULATOR_HPP
#define YOUBOT_MANIPULATOR_HPP

#include <vector>
#include "YoubotJoint.hpp"
#include "YoubotConfig.hpp"

class YoubotManipulator {
  const YoubotConfig config;
  VMessageCenter* center;
  std::vector<YoubotJoint> joints;

public:
  YoubotManipulator(const YoubotConfig& config, VMessageCenter* center);;

  YoubotJoint& GetJoint(int i);

  void ConfigJoints();

};

#endif
