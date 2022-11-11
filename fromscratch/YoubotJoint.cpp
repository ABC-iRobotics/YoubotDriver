#include "YoubotJoint.hpp"

YoubotJoint::YoubotJoint(int slaveIndex, const NameValueMap& config, VMessageCenter* center)
    : slaveIndex(slaveIndex), center(center), config(config) {}

void YoubotJoint::ConfigParameters() {
  // 0. Get FirmwareVersion
  {
    auto ptr = GetFirmware::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    ptr->GetOutput(controllerNum, firmwareversion);
    std::cout << "Joint with slaveindex " << slaveIndex << " initialized. Controller: " << controllerNum
      << " Firmware: " << firmwareversion << std::endl;
    if (controllerNum != 1610 || firmwareversion != 148)
      throw std::runtime_error("Not supported joint controller/firmware type");
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

bool YoubotJoint::CheckConfig() {
  // 0. Get FirmwareVersion
  {
    auto ptr = GetFirmware::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    ptr->GetOutput(controllerNum, firmwareversion);
    std::cout << "Joint with slaveindex " << slaveIndex << " to be checked. Controller: " << controllerNum << " Firmware: " << firmwareversion << std::endl;
    if (controllerNum != 1610 || firmwareversion != 148)
      throw std::runtime_error("Not supported joint controller/firmware type");
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
  // GetMaxRampVelocityRPM
  {
    auto ptr = GetMaxRampVelocityRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("MaximumPositioningVelocityRadPerSec") / 2. / M_PI * 60);
    std::cout << " GetMaxRampVelocityRPM: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetAccelerationParamRPMPSec
  {
    auto ptr = GetAccelerationParamRPMPSec::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("MotorAccelerationRadPerSec2") / 2. / M_PI * 60);
    std::cout << " GetAccelerationParamRPMPSec: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetTresholdSpeedForPosPIDRPM
  {
    auto ptr = GetTresholdSpeedForPosPIDRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PositionControlSwitchingThresholdRadPerSec") / 2. / M_PI * 60);
    std::cout << " GetTresholdSpeedForPosPIDRPM: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetTresholdSpeedForVelPIDRPM
  {
    auto ptr = GetTresholdSpeedForVelPIDRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("SpeedControlSwitchingThresholdRadPerSec") / 2. / M_PI * 60);
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
    int32_t fromconfig = int32_t(config.at("MaximumVelocityToSetPositionRadPerSec") / 2. / M_PI * 60);
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
    int32_t fromconfig = int32_t(config.at("VelocityThresholdForHallFX") / 2. / M_PI * 60.);
    std::cout << " GetVelThresholdHallFXRPM: " << ptr->GetReplyValue() << "(=" << fromconfig << ")" << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  return true;
}
