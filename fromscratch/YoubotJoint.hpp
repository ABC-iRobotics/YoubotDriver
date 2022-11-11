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

  void ConfigParameters() {
    // 0. Get FirmwareVersion
    {
      auto ptr = GetFirmware::InitSharedPtr(slaveIndex);
      center->SendMessage_(ptr);
      ptr->GetOutput(controllerNum, firmwareversion);
      if (controllerNum != 1610 || firmwareversion != 148)
        throw std::runtime_error("Not supported joint controller/firmware type");
      std::cout << "Joint with slaveindex " << slaveIndex << " initialized. Controller: " << controllerNum
        << " Firmware: " << firmwareversion << std::endl;
    }
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
    // If I2T exceeded
    if (status & (uint32_t)TMCL::StatusErrorFlags::I2T_EXCEEDED) {
      auto ptr = ClearI2TFlag::InitSharedPtr(slaveIndex);
      center->SendMessage_(ptr);
      std::cout << "  ClearI2TFlag: " << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
    }
    SLEEP_MILLISEC(6)
    {
      auto ptr = GetErrorStatusFlag::InitSharedPtr(slaveIndex);
      center->SendMessage_(ptr);
      status = ptr->GetReplyValue();
      std::cout << "GetErrorStatusFlag: " <<
        TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
        << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // GetTickPerRounds
    {
      auto ptr = GetEncoderStepsPerRotation::InitSharedPtr(slaveIndex);
      center->SendMessage_(ptr);
      ticksperround = ptr->GetReplyValue();
      std::cout << " GetEncoderStepsPerRotation: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetMaxRampVelocity
    {
      auto ptr = SetMaxRampVelocityRPM::InitSharedPtr(slaveIndex,
        int32_t(config.at("MaximumPositioningVelocityRadPerSec") / 2. / M_PI * 60));
      center->SendMessage_(ptr);
      std::cout << " SetMaxRampVelocityRPM: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetAccelerationParam
    {
      auto ptr = SetAccelerationParamRPMPSec::InitSharedPtr(slaveIndex,
        int32_t(config.at("MotorAccelerationRadPerSec2") / 2. / M_PI * 60));
      center->SendMessage_(ptr);
      std::cout << " SetAccelerationParamRPMPSec: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetTresholdSpeedForPosPIDRPM
    {
      auto ptr = GetTresholdSpeedForPosPIDRPM::InitSharedPtr(slaveIndex,
        int32_t(config.at("PositionControlSwitchingThresholdRadPerSec") / 2. / M_PI * 60));
      center->SendMessage_(ptr);
      std::cout << " SetTresholdSpeedForPosPIDRPM: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetTresholdSpeedForVelPIDRPM
    {
      auto ptr = GetTresholdSpeedForVelPIDRPM::InitSharedPtr(slaveIndex,
        int32_t(config.at("SpeedControlSwitchingThresholdRadPerSec") / 2. / M_PI * 60));
      center->SendMessage_(ptr);
      std::cout << " SetTresholdSpeedForVelPIDRPM: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetP1ParameterPositionControl
    {
      auto ptr = SetP1ParameterPositionControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("PParameterFirstParametersPositionControl")));
      center->SendMessage_(ptr);
      std::cout << " SetP1ParameterPositionControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetI1ParameterPositionControl
    {
      auto ptr = SetI1ParameterPositionControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("IParameterFirstParametersPositionControl")));
      center->SendMessage_(ptr);
      std::cout << " SetI1ParameterPositionControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetD1ParameterPositionControl
    {
      auto ptr = SetD1ParameterPositionControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("DParameterFirstParametersPositionControl")));
      center->SendMessage_(ptr);
      std::cout << " SetD1ParameterPositionControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetClipping1ParameterPositionControl
    {
      auto ptr = SetClipping1ParameterPositionControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("IClippingParameterFirstParametersPositionControl")));
      center->SendMessage_(ptr);
      std::cout << " SetClipping1ParameterPositionControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetP1ParameterVelocityControl
    {
      auto ptr = SetP1ParameterVelocityControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("PParameterFirstParametersSpeedControl")));
      center->SendMessage_(ptr);
      std::cout << " SetP1ParameterVelocityControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetD1ParameterVelocityControl
    {
      auto ptr = SetD1ParameterVelocityControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("DParameterFirstParametersSpeedControl")));
      center->SendMessage_(ptr);
      std::cout << " SetD1ParameterVelocityControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetI1ParameterVelocityControl
    {
      auto ptr = SetI1ParameterVelocityControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("IParameterFirstParametersSpeedControl")));
      center->SendMessage_(ptr);
      std::cout << " SetI1ParameterVelocityControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetClipping1ParameterVelocityControl
    {
      auto ptr = SetClipping1ParameterVelocityControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("IClippingParameterFirstParametersSpeedControl")));
      center->SendMessage_(ptr);
      std::cout << " SetClipping1ParameterVelocityControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetP2ParameterPositionControl
    {
      auto ptr = SetP2ParameterPositionControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("PParameterSecondParametersPositionControl")));
      center->SendMessage_(ptr);
      std::cout << " SetP2ParameterPositionControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetI2ParameterPositionControl
    {
      auto ptr = SetI2ParameterPositionControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("IParameterSecondParametersPositionControl")));
      center->SendMessage_(ptr);
      std::cout << " SetI2ParameterPositionControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetD2ParameterPositionControl
    {
      auto ptr = SetD2ParameterPositionControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("DParameterSecondParametersPositionControl")));
      center->SendMessage_(ptr);
      std::cout << " SetD2ParameterPositionControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetClipping2ParameterPositionControl
    {
      auto ptr = SetClipping2ParameterPositionControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("IClippingParameterSecondParametersPositionControl")));
      center->SendMessage_(ptr);
      std::cout << " SetClipping2ParameterPositionControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetP2ParameterVelocityControl
    {
      auto ptr = SetP2ParameterVelocityControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("PParameterSecondParametersSpeedControl")));
      center->SendMessage_(ptr);
      std::cout << " SetP2ParameterVelocityControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetI2ParameterVelocityControl
    {
      auto ptr = SetI2ParameterVelocityControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("IParameterSecondParametersSpeedControl")));
      center->SendMessage_(ptr);
      std::cout << " SetI2ParameterVelocityControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetD2ParameterVelocityControl
    {
      auto ptr = SetD2ParameterVelocityControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("DParameterSecondParametersSpeedControl")));
      center->SendMessage_(ptr);
      std::cout << " SetD2ParameterVelocityControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetClipping2ParameterVelocityControl
    {
      auto ptr = SetClipping2ParameterVelocityControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("IClippingParameterSecondParametersSpeedControl")));
      center->SendMessage_(ptr);
      std::cout << " SetClipping2ParameterVelocityControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetP2ParameterCurrentControl
    {
      auto ptr = SetP2ParameterCurrentControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("PParameterCurrentControl")));
      center->SendMessage_(ptr);
      std::cout << " SetP2ParameterCurrentControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetI2ParameterCurrentControl
    {
      auto ptr = SetI2ParameterCurrentControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("IParameterCurrentControl")));
      center->SendMessage_(ptr);
      std::cout << " SetI2ParameterCurrentControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetD2ParameterCurrentControl
    {
      auto ptr = SetD2ParameterCurrentControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("DParameterCurrentControl")));
      center->SendMessage_(ptr);
      std::cout << " SetD2ParameterCurrentControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetClipping2ParameterCurrentControl
    {
      auto ptr = SetClipping2ParameterCurrentControl::InitSharedPtr(slaveIndex,
        int32_t(config.at("IClippingParameterCurrentControl")));
      center->SendMessage_(ptr);
      std::cout << " SetClipping2ParameterCurrentControl: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetMaxVelocityToReachTargetRPM
    {
      auto ptr = SetMaxVelocityToReachTargetRPM::InitSharedPtr(slaveIndex,
        int32_t(config.at("MaximumVelocityToSetPositionRadPerSec") / 2. / M_PI * 60));
      center->SendMessage_(ptr);
      std::cout << " SetMaxVelocityToReachTargetRPM: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetMaxDistanceToReachTarget
    {
      auto ptr = SetMaxDistanceToReachTarget::InitSharedPtr(slaveIndex,
        int32_t(config.at("PositionTargetReachedDistance")));
      center->SendMessage_(ptr);
      std::cout << " SetMaxDistanceToReachTarget: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    // SetVelThresholdHallFXRPM
    {
      auto ptr = SetVelThresholdHallFXRPM::InitSharedPtr(slaveIndex,
        int32_t(config.at("VelocityThresholdForHallFX") / 2. / M_PI * 60.));
      center->SendMessage_(ptr);
      std::cout << " SetVelThresholdHallFXRPM: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    

  }

};

#endif
