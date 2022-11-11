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

  uint32_t ticksperround = -1;
  bool directionreverted = -1;
  bool calibratedposition = -1;
  int firmwareversion = -1;

public:
  YoubotJoint(int slaveIndex, const NameValueMap& config, VMessageCenter* center);;

  void ConfigParameters() {
    // 1. Stop the motor
    {
      auto ptr = MotorStop::InitSharedPtr(slaveIndex);
      center->SendMessage_(ptr);
      std::cout << " MotorStop: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // Get Status
    uint32_t status;
    {
      auto ptr = GetErrorStatusFlag::InitSharedPtr(slaveIndex);
      center->SendMessage_(ptr);
      status = ptr->GetReplyValue();
      std::cout << "GetErrorStatusFlag: " <<
        TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
        << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // If timeout clear it..
    if (status & (uint32_t)TMCL::StatusErrorFlags::TIMEOUT) {
      auto ptr = ClearMotorControllerTimeoutFlag::InitSharedPtr(slaveIndex);
      center->SendMessage_(ptr);
      std::cout << "  ClearMotorControllerTimeoutFlag: " << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
    }
    {
      auto ptr = GetErrorStatusFlag::InitSharedPtr(slaveIndex);
      center->SendMessage_(ptr);
      status = ptr->GetReplyValue();
      std::cout << "GetErrorStatusFlag: " <<
        TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
        << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // If I2T exceeded
    if (status & (uint32_t)TMCL::StatusErrorFlags::I2T_EXCEEDED) {
      auto ptr = ClearI2TFlag::InitSharedPtr(slaveIndex);
      center->SendMessage_(ptr);
      std::cout << "  ClearI2TFlag: " << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
    }
    {
      auto ptr = GetErrorStatusFlag::InitSharedPtr(slaveIndex);
      center->SendMessage_(ptr);
      status = ptr->GetReplyValue();
      std::cout << "GetErrorStatusFlag: " <<
        TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
        << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
  }

};

#endif
