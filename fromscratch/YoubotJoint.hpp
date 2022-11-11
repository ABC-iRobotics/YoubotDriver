#ifndef YOUBOT_JOINT_HPP
#define YOUBOT_JOINT_HPP

#include "VMessageCenter.hpp"
#include "TMCLMailboxMessage.hpp"
#include "Time.hpp"
#include <iostream>

#include "YoubotConfig.hpp"


class YoubotJoint {
  int slaveIndex;
  VMessageCenter* center;
  const NameValueMap config;

  // Read during configuration
  uint32_t ticksperround = -1;
  int firmwareversion = -1, controllerNum = -1;
  // Maybe modfied during configuration
  bool directionreverted = false;


  bool calibratedposition = -1;

public:
  YoubotJoint(int slaveIndex, const NameValueMap& config, VMessageCenter* center);;

  void ConfigParameters();

};

#endif
