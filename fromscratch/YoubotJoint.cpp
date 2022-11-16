#include "YoubotJoint.hpp"
#include "TMCLMailboxMessage.hpp"
#include "Time.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>

void YoubotJoint::_getFirmwareVersion() {
  auto ptr = GetFirmware::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  ptr->GetOutput(controllerNum, firmwareversion);
  std::cout << "Joint with slaveindex " << slaveIndex << " initialized. Controller: " << controllerNum
    << " Firmware: " << firmwareversion << std::endl;
  if (controllerNum != 1610 || firmwareversion != 148)
    throw std::runtime_error("Not supported joint controller/firmware type");
}

YoubotJoint::YoubotJoint(int slaveIndex, const NameValueMap& config, VMessageCenter* center)
    : slaveIndex(slaveIndex), center(center), config(config) {
  _getFirmwareVersion();
}

void YoubotJoint::ConfigParameters() {
  {
    gearRatio = config.at("GearRatio");
    std::cout << " GearRatio: " << gearRatio << std::endl;
    torqueconstant = config.at("TorqueConstant_NmPerAmpere");
    std::cout << " TorqueConstant_NmPerAmpere: " << torqueconstant << std::endl;
    directionreversed = config.at("InverseMovement");
    std::cout << " InverseMovement: " << directionreversed << std::endl;
    calibrationDirection = config.at("CalibrationDirection");
    std::cout << " CalibrationDirection: " << calibrationDirection << std::endl;
    calibrationmaxAmpere = config.at("CalibrationMaxCurrentAmpere");
    std::cout << " CalibrationMaxCurrentAmpere: " << calibrationmaxAmpere << std::endl;
  }
  // 1. Stop the motor
  StopViaMailbox();
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
      int32_t(config.at("MaximumPositioningVelocityMotorRPM")));
    center->SendMessage_(ptr);
    std::cout << " SetMaxRampVelocityRPM: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  // SetAccelerationParam
  {
    auto ptr = SetAccelerationParamRPMPSec::InitSharedPtr(slaveIndex,
      int32_t(config.at("MotorAccelerationMotorRPMPerSec")));
    center->SendMessage_(ptr);
    std::cout << " SetAccelerationParamRPMPSec: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  // SetTresholdSpeedForPosPIDRPM
  {
    auto ptr = GetTresholdSpeedForPosPIDRPM::InitSharedPtr(slaveIndex,
      int32_t(config.at("PositionControlSwitchingThresholdMotorRPM")));
    center->SendMessage_(ptr);
    std::cout << " SetTresholdSpeedForPosPIDRPM: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  // SetTresholdSpeedForVelPIDRPM
  {
    auto ptr = GetTresholdSpeedForVelPIDRPM::InitSharedPtr(slaveIndex,
      int32_t(config.at("SpeedControlSwitchingThresholdMotorRPM")));
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
      int32_t(config.at("MaximumVelocityToSetPositionMotorRPM")));
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
      int32_t(config.at("VelocityThresholdForHallFXMotorRPM")));
    center->SendMessage_(ptr);
    std::cout << " SetVelThresholdHallFXRPM: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
}

bool YoubotJoint::CheckConfig() {
  // GetMaxRampVelocityRPM
  {
    auto ptr = GetMaxRampVelocityRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("MaximumPositioningVelocityMotorRPM"));
    std::cout << " GetMaxRampVelocityRPM: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetAccelerationParamRPMPSec
  {
    auto ptr = GetAccelerationParamRPMPSec::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("MotorAccelerationMotorRPMPerSec"));
    std::cout << " GetAccelerationParamRPMPSec: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetTresholdSpeedForPosPIDRPM
  {
    auto ptr = GetTresholdSpeedForPosPIDRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PositionControlSwitchingThresholdMotorRPM"));
    std::cout << " GetTresholdSpeedForPosPIDRPM: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetTresholdSpeedForVelPIDRPM
  {
    auto ptr = GetTresholdSpeedForVelPIDRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("SpeedControlSwitchingThresholdMotorRPM"));
    std::cout << " GetTresholdSpeedForVelPIDRPM: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP1ParameterPositionControl
  {
    auto ptr = GetP1ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PParameterFirstParametersPositionControl"));
    std::cout << " GetP1ParameterPositionControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI1ParameterPositionControl
  {
    auto ptr = GetI1ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IParameterFirstParametersPositionControl"));
    std::cout << " GetI1ParameterPositionControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD1ParameterPositionControl
  {
    auto ptr = GetD1ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("DParameterFirstParametersPositionControl"));
    std::cout << " GetD1ParameterPositionControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping1ParameterPositionControl
  {
    auto ptr = GetClipping1ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IClippingParameterFirstParametersPositionControl"));
    std::cout << " GetClipping1ParameterPositionControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP1ParameterVelocityControl
  {
    auto ptr = GetP1ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PParameterFirstParametersSpeedControl"));
    std::cout << " GetP1ParameterVelocityControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD1ParameterVelocityControl
  {
    auto ptr = GetD1ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("DParameterFirstParametersSpeedControl"));
    std::cout << " GetD1ParameterVelocityControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI1ParameterVelocityControl
  {
    auto ptr = GetI1ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IParameterFirstParametersSpeedControl"));
    std::cout << " GetI1ParameterVelocityControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping1ParameterVelocityControl
  {
    auto ptr = GetClipping1ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IClippingParameterFirstParametersSpeedControl"));
    std::cout << " GetClipping1ParameterVelocityControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP2ParameterPositionControl
  {
    auto ptr = GetP2ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PParameterSecondParametersPositionControl"));
    std::cout << " GetP2ParameterPositionControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI2ParameterPositionControl
  {
    auto ptr = GetI2ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IParameterSecondParametersPositionControl"));
    std::cout << " GetI2ParameterPositionControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD2ParameterPositionControl
  {
    auto ptr = GetD2ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("DParameterSecondParametersPositionControl"));
    std::cout << " GetD2ParameterPositionControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping2ParameterPositionControl
  {
    auto ptr = GetClipping2ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IClippingParameterSecondParametersPositionControl"));
    std::cout << " GetClipping2ParameterPositionControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP2ParameterVelocityControl
  {
    auto ptr = GetP2ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PParameterSecondParametersSpeedControl"));
    std::cout << " GetP2ParameterVelocityControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI2ParameterVelocityControl
  {
    auto ptr = GetI2ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IParameterSecondParametersSpeedControl"));
    std::cout << " GetI2ParameterVelocityControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD2ParameterVelocityControl
  {
    auto ptr = GetD2ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("DParameterSecondParametersSpeedControl"));
    std::cout << " GetD2ParameterVelocityControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping2ParameterVelocityControl
  {
    auto ptr = GetClipping2ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IClippingParameterSecondParametersSpeedControl"));
    std::cout << " GetClipping2ParameterVelocityControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP2ParameterCurrentControl
  {
    auto ptr = GetP2ParameterCurrentControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PParameterCurrentControl"));
    std::cout << " GetP2ParameterCurrentControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI2ParameterCurrentControl
  {
    auto ptr = GetI2ParameterCurrentControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IParameterCurrentControl"));
    std::cout << " GetI2ParameterCurrentControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD2ParameterCurrentControl
  {
    auto ptr = GetD2ParameterCurrentControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("DParameterCurrentControl"));
    std::cout << " GetD2ParameterCurrentControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping2ParameterCurrentControl
  {
    auto ptr = GetClipping2ParameterCurrentControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IClippingParameterCurrentControl"));
    std::cout << " GetClipping2ParameterCurrentControl: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetMaxVelocityToReachTargetRPM
  {
    auto ptr = GetMaxVelocityToReachTargetRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("MaximumVelocityToSetPositionMotorRPM"));
    std::cout << " GetMaxVelocityToReachTargetRPM: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetMaxDistanceToReachTarget
  {
    auto ptr = GetMaxDistanceToReachTarget::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PositionTargetReachedDistance"));
    std::cout << " GetMaxDistanceToReachTarget: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetVelThresholdHallFXRPM
  {
    auto ptr = GetVelThresholdHallFXRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("VelocityThresholdForHallFXMotorRPM"));
    std::cout << " GetVelThresholdHallFXRPM: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  return true;
}

void YoubotJoint::RotateJointRightViaMailbox(double speedJointRadPerSec) {
  auto ptr = RotateRightMotorRPM::InitSharedPtr(slaveIndex, int32_t(speedJointRadPerSec / gearRatio / 2. / M_PI * 60.));
  center->SendMessage_(ptr);
  std::cout << " RotateRightMotorRPM: " << ptr->GetReplyValue() <<
    " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
}

void YoubotJoint::RotateJointLeftViaMailbox(double speedJointRadPerSec) {
  auto ptr = RotateLeftMotorRPM::InitSharedPtr(slaveIndex, int32_t(speedJointRadPerSec / gearRatio / 2. / M_PI * 60.));
  center->SendMessage_(ptr);
  std::cout << " RotateLeftMotorRPM: " << ptr->GetReplyValue() <<
    " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
}

void YoubotJoint::StopViaMailbox() {
  auto ptr = MotorStop::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  std::cout << " MotorStop: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
}

YoubotJoint::JointStatus YoubotJoint::GetJointStatusViaMailbox() {
  auto ptr = GetErrorStatusFlag::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  std::cout << "GetErrorStatusFlag: " <<
    TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
    << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  return ptr->GetReplyValue();
}

void YoubotJoint::ResetTimeoutViaMailbox() {
  auto ptr = ClearMotorControllerTimeoutFlag::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  std::cout << "  ClearMotorControllerTimeoutFlag: " << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  SLEEP_MILLISEC(6)
}

void YoubotJoint::ResetI2TExceededViaMailbox() {
  auto ptr = ClearI2TFlag::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  std::cout << "  ClearI2TFlag: " << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  SLEEP_MILLISEC(6)
}

void YoubotJoint::StartInitialization() {
  StopViaMailbox();
  auto status = GetJointStatusViaMailbox();
  std::cout << status.toString() << std::endl;
  if (status.Timeout())
    ResetTimeoutViaMailbox();
  if (status.I2TExceeded())
    ResetI2TExceededViaMailbox();
  status = GetJointStatusViaMailbox();
  std::cout << status.toString() << std::endl;
  auto ptr = SetInitialize::InitSharedPtr(slaveIndex, 1);
  center->SendMessage_(ptr);
  std::cout << "  SetInitialize: " << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
}

bool YoubotJoint::IsInitialized() {
  auto ptr = GetInitialized::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  std::cout << "  GetInitialized: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  return ptr->GetReplyValue();
}

bool YoubotJoint::JointStatus::OverCurrent() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::OVER_CURRENT;
}

bool YoubotJoint::JointStatus::UnderVoltage() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::UNDER_VOLTAGE;
};

bool YoubotJoint::JointStatus::OverVoltage() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::OVER_VOLTAGE;
};

bool YoubotJoint::JointStatus::OverTemperature() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::OVER_TEMPERATURE;
};

bool YoubotJoint::JointStatus::MotorHalted() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::MOTOR_HALTED;
};

bool YoubotJoint::JointStatus::HallSensorError() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::HALL_SENSOR_ERROR;
};

bool YoubotJoint::JointStatus::EncoderError() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::ENCODER_ERROR;
};

bool YoubotJoint::JointStatus::InitializationError() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::INITIALIZATION_ERROR;
};

bool YoubotJoint::JointStatus::PWMMode() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::PWM_MODE_ACTIVE;
};

bool YoubotJoint::JointStatus::VelocityMode() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::VELOCITY_MODE_ACTIVE;
};

bool YoubotJoint::JointStatus::PositionMode() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::POSITION_MODE_ACTIVE;
};

bool YoubotJoint::JointStatus::TorqueMode() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::TORQUE_MODE_ACTIVE;
};

bool YoubotJoint::JointStatus::EmergencyStop() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::EMERGENCY_STOP;
};

bool YoubotJoint::JointStatus::FreeRunning() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::FREERUNNING;
};

bool YoubotJoint::JointStatus::PosiitonReached() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::POSITION_REACHED;
};

bool YoubotJoint::JointStatus::Initialized() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::INITIALIZED;
};

bool YoubotJoint::JointStatus::Timeout() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::TIMEOUT;
};

bool YoubotJoint::JointStatus::I2TExceeded() const {
  return value & (uint32_t)TMCL::StatusErrorFlags::I2T_EXCEEDED;
};

std::string YoubotJoint::JointStatus::toString() const {
  std::stringstream ss;
  if (value & (uint32_t)TMCL::StatusErrorFlags::OVER_CURRENT)
    ss << " OVER_CURRENT";
  if (value & (uint32_t)TMCL::StatusErrorFlags::UNDER_VOLTAGE)
    ss << " UNDER_VOLTAGE";
  if (value & (uint32_t)TMCL::StatusErrorFlags::OVER_VOLTAGE)
    ss << " OVER_VOLTAGE";
  if (value & (uint32_t)TMCL::StatusErrorFlags::OVER_TEMPERATURE)
    ss << " OVER_TEMPERATURE";
  if (value & (uint32_t)TMCL::StatusErrorFlags::MOTOR_HALTED)
    ss << " MOTOR_HALTED";
  if (value & (uint32_t)TMCL::StatusErrorFlags::HALL_SENSOR_ERROR)
    ss << " HALL_SENSOR_ERROR";
  if (value & (uint32_t)TMCL::StatusErrorFlags::PWM_MODE_ACTIVE)
    ss << " PWM_MODE_ACTIVE";
  if (value & (uint32_t)TMCL::StatusErrorFlags::VELOCITY_MODE_ACTIVE)
    ss << " VELOCITY_MODE_ACTIVE";
  if (value & (uint32_t)TMCL::StatusErrorFlags::POSITION_MODE_ACTIVE)
    ss << " POSITION_MODE_ACTIVE";
  if (value & (uint32_t)TMCL::StatusErrorFlags::TORQUE_MODE_ACTIVE)
    ss << " TORQUE_MODE_ACTIVE";
  if (value & (uint32_t)TMCL::StatusErrorFlags::POSITION_REACHED)
    ss << " POSITION_REACHED";
  if (value & (uint32_t)TMCL::StatusErrorFlags::INITIALIZED)
    ss << " INITIALIZED";
  if (value & (uint32_t)TMCL::StatusErrorFlags::TIMEOUT)
    ss << " TIMEOUT";
  if (value & (uint32_t)TMCL::StatusErrorFlags::I2T_EXCEEDED)
    ss << " I2T_EXCEEDED";
  return ss.str();
}

void YoubotJoint::ProcessReturn::Print() const {
  std::cout << "encoderPosition: " << (int)encoderPosition << " currentmA: " << (int)currentmA << " motorVelocityRPM: " << (int)motorVelocityRPM << " status: " << status.toString() << " motorPWM: " << (int)motorPWM << std::endl;
}

YoubotJoint::ProcessReturn::ProcessReturn() : status(0), encoderPosition(-1),
currentmA(-1), motorVelocityRPM(-1), motorPWM(-1) {};

const YoubotJoint::ProcessReturn& YoubotJoint::GetProcessReturnData() {
  static ProcessBuffer buffer;
  center->GetProcessMsg(buffer, slaveIndex);
  processReturn.encoderPosition = _toInt32(&buffer.buffer[0]);
  processReturn.currentmA = _toInt32(&buffer.buffer[4]);
  processReturn.motorVelocityRPM = _toInt32(&buffer.buffer[8]);
  processReturn.status = _toInt32(&buffer.buffer[12]);
  processReturn.motorPWM = _toInt32(&buffer.buffer[16]);
  return processReturn;
}

void YoubotJoint::ReqVelocityMotorRPM(int32_t value) {
  static ProcessBuffer toSet(5);
  toSet.buffer[3] = value >> 24;
  toSet.buffer[2] = value >> 16;
  toSet.buffer[1] = value >> 8;
  toSet.buffer[0] = value & 0xff;
  toSet.buffer[4] = TMCL::ControllerMode::VELOCITY_CONTROL;
  center->SetProcessMsg(toSet, slaveIndex);
}

void YoubotJoint::ReqMotorStopViaProcess() {
  static ProcessBuffer toSet(5);
  for (int i = 0; i < 4; i++)
    toSet.buffer[i] = 0;
  toSet.buffer[4] = TMCL::ControllerMode::MOTOR_STOP;
  center->SetProcessMsg(toSet, slaveIndex);
}

void YoubotJoint::ReqSetPositionToReferenceViaProcess() {
  static ProcessBuffer toSet(5);
  for (int i = 0; i < 4; i++)
    toSet.buffer[i] = 0;
  toSet.buffer[4] = TMCL::ControllerMode::SET_POSITION_TO_REFERENCE;
  center->SetProcessMsg(toSet, slaveIndex);
}

void YoubotJoint::ReqInitializationViaProcess() {
  static ProcessBuffer toSet(5);
  for (int i = 0; i < 4; i++)
    toSet.buffer[i] = 0;
  toSet.buffer[4] = TMCL::ControllerMode::INITIALIZE;
  center->SetProcessMsg(toSet, slaveIndex);
}

double YoubotJoint::GetCurrentAViaMailbox() {
  auto ptr = GetCurrent::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  std::cout << " GetCurrent[mA]: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  return double(ptr->GetReplyValue()) / 1000.;
}

void YoubotJoint::RotateMotorRightViaMailbox(int32_t speedMotorRPM) {
  auto ptr = RotateRightMotorRPM::InitSharedPtr(slaveIndex, speedMotorRPM);
  center->SendMessage_(ptr);
  std::cout << " RotateRightMotorRPM: " << ptr->GetReplyValue() <<
    " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
}

void YoubotJoint::RotateMotorLeftViaMailbox(int32_t speedMotorRPM) {
  auto ptr = RotateLeftMotorRPM::InitSharedPtr(slaveIndex, speedMotorRPM);
  center->SendMessage_(ptr);
  std::cout << " RotateLeftMotorRPM: " << ptr->GetReplyValue() <<
    " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
}

double YoubotJoint::GetJointVelocityRadPerSec() {
  auto ptr = GetActualSpeedMotorRPM::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  std::cout << " GetActualSpeedMotorRPM: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  return double(ptr->GetReplyValue()) * 2. * M_PI / 60. * gearRatio;
}

void YoubotJoint::SetTargetCurrentA(double current) {
  auto ptr = SetTargetCurrentmA::InitSharedPtr(slaveIndex, int32_t(current * 1000.));
  center->SendMessage_(ptr);
  std::cout << " SetTargetCurrentmA[mA]: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
}