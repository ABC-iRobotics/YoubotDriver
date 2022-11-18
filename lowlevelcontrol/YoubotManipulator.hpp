#ifndef YOUBOT_MANIPULATOR_HPP
#define YOUBOT_MANIPULATOR_HPP

#include <vector>
#include "YoubotJoint.hpp"
#include "YoubotConfig.hpp"

class YoubotManipulator {
  const YoubotConfig config;
  VMessageCenter* center;
  std::vector<YoubotJoint::Ptr> joints;

public:
  YoubotManipulator() = delete;

  YoubotManipulator(YoubotManipulator&) = delete;

  YoubotManipulator(const YoubotManipulator&) = delete;

  YoubotManipulator(const YoubotConfig& config, VMessageCenter* center);

  ~YoubotManipulator() {};

  YoubotJoint::Ptr GetJoint(int i);

  void ConfigJoints(bool forceConfiguration = false);

  bool CheckJointConfigs();

  void InitializeAllJoints();

  void Calibrate(bool forceCalibration=false);

  void ReqJointPosition(double q0, double q1, double q2, double q3, double q4);

  void ResetErrorFlags();
};

#endif
