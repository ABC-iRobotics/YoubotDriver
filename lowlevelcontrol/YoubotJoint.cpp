#include "YoubotJoint.hpp"
#include "TMCLMailboxMessage.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include <sstream>
#include <stdexcept>

using namespace youbot;
using namespace youbot::intrinsic;

void YoubotJoint::_getFirmwareVersionViaMailbox() {
  auto ptr = GetFirmware::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  ptr->GetOutput(controllerNum, firmwareversion);
  if (controllerNum != 1610 || firmwareversion != 148) {
    log(Log::fatal, "Not supported joint with slaveindex " + std::to_string(slaveIndex) + ". Controller: "
      + std::to_string(controllerNum) + " Firmware: " + std::to_string(firmwareversion));
    throw std::runtime_error("Not supported joint controller/firmware type");
  }
  else
    log(Log::info, "Joint with slaveindex " + std::to_string(slaveIndex) + " initialized. Controller: "
      + std::to_string(controllerNum) + " Firmware: " + std::to_string(firmwareversion));
}

YoubotJoint::YoubotJoint(int slaveIndex, const std::map<std::string,
  double>& config, EtherCATMaster* center)
    : slaveIndex(slaveIndex), center(center), config(config) {
  _getFirmwareVersionViaMailbox();
  // GetTickPerRounds
  {
    auto ptr = GetEncoderStepsPerRotation::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    ticksperround = ptr->GetReplyValue();
    log(Log::info, "Init joint " + std::to_string(slaveIndex) +
      " GetEncoderStepsPerRotation: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // Get cooldowntime_sec
  {
    cooldowntime_sec = GetThermalWindingTimeSec();
  }
  gearRatio = config.at("GearRatio");
  log(Log::info, "Init joint " + std::to_string(slaveIndex) + " GearRatio: "
    + std::to_string(gearRatio));
  qMinDeg = config.at("qMinDeg");
  log(Log::info, "Init joint " + std::to_string(slaveIndex) + " qMinDeg: "
    + std::to_string(qMinDeg));
  qMaxDeg = config.at("qMaxDeg");
  log(Log::info, "Init joint " + std::to_string(slaveIndex) + " qMaxDeg: "
    + std::to_string(qMaxDeg));
  torqueconstant = config.at("TorqueConstant_NmPerAmpere");
  log(Log::info, "Init joint " + std::to_string(slaveIndex) + " TorqueConstant_NmPerAmpere: "
    + std::to_string(torqueconstant));
  calibrationDirection = config.at("CalibrationDirection");
  log(Log::info, "Init joint " + std::to_string(slaveIndex) + " CalibrationDirection: "
    + std::to_string(calibrationDirection));
  bool qDirectionSameAsEnc = config.at("qDirectionSameAsEnc");
  log(Log::info, " qDirectionSameAsEnc: " + std::to_string(int(qDirectionSameAsEnc)));
  double qCalibrationDeg = config.at("qCalibrationDeg");
  log(Log::info, " qCalibrationDeg: " + std::to_string(qCalibrationDeg));
  conversion = Conversion(qDirectionSameAsEnc, ticksperround, gearRatio, qCalibrationDeg);
}

void YoubotJoint::ConfigParameters(bool forceConfiguration) {
  if (!forceConfiguration && IsConfiguratedViaMailbox()) {
    log(Log::info, "Joint " + std::to_string(slaveIndex) + " is already configurated");
    return;
  }
  // 1. Stop the motor
  StopViaMailbox();
  // SetMaxRampVelocity
  {
    auto ptr = SetMaxRampVelocityRPM::InitSharedPtr(slaveIndex,
      int32_t(config.at("MaximumPositioningVelocityMotorRPM")));
    center->SendMessage_(ptr);
    log(Log::info, " SetMaxRampVelocityRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetAccelerationParam
  {
    auto ptr = SetAccelerationParamRPMPSec::InitSharedPtr(slaveIndex,
      int32_t(config.at("MotorAccelerationMotorRPMPerSec")));
    center->SendMessage_(ptr);
    log(Log::info, " SetAccelerationParamRPMPSec: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetTresholdSpeedForPosPIDRPM
  {
    auto ptr = GetTresholdSpeedForPosPIDRPM::InitSharedPtr(slaveIndex,
      int32_t(config.at("PositionControlSwitchingThresholdMotorRPM")));
    center->SendMessage_(ptr);
    log(Log::info, " SetTresholdSpeedForPosPIDRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetTresholdSpeedForVelPIDRPM
  {
    auto ptr = GetTresholdSpeedForVelPIDRPM::InitSharedPtr(slaveIndex,
      int32_t(config.at("SpeedControlSwitchingThresholdMotorRPM")));
    center->SendMessage_(ptr);
    log(Log::info, " SetTresholdSpeedForVelPIDRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetP1ParameterPositionControl
  {
    auto ptr = SetP1ParameterPositionControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("PParameterFirstParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetP1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetI1ParameterPositionControl
  {
    auto ptr = SetI1ParameterPositionControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("IParameterFirstParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetI1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetD1ParameterPositionControl
  {
    auto ptr = SetD1ParameterPositionControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("DParameterFirstParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetD1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetClipping1ParameterPositionControl
  {
    auto ptr = SetClipping1ParameterPositionControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("IClippingParameterFirstParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetClipping1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetP1ParameterVelocityControl
  {
    auto ptr = SetP1ParameterVelocityControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("PParameterFirstParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetP1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetD1ParameterVelocityControl
  {
    auto ptr = SetD1ParameterVelocityControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("DParameterFirstParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetD1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetI1ParameterVelocityControl
  {
    auto ptr = SetI1ParameterVelocityControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("IParameterFirstParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetI1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetClipping1ParameterVelocityControl
  {
    auto ptr = SetClipping1ParameterVelocityControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("IClippingParameterFirstParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetClipping1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetP2ParameterPositionControl
  {
    auto ptr = SetP2ParameterPositionControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("PParameterSecondParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetP2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetI2ParameterPositionControl
  {
    auto ptr = SetI2ParameterPositionControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("IParameterSecondParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetI2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetD2ParameterPositionControl
  {
    auto ptr = SetD2ParameterPositionControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("DParameterSecondParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetD2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetClipping2ParameterPositionControl
  {
    auto ptr = SetClipping2ParameterPositionControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("IClippingParameterSecondParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetClipping2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetP2ParameterVelocityControl
  {
    auto ptr = SetP2ParameterVelocityControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("PParameterSecondParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetP2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetI2ParameterVelocityControl
  {
    auto ptr = SetI2ParameterVelocityControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("IParameterSecondParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetI2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetD2ParameterVelocityControl
  {
    auto ptr = SetD2ParameterVelocityControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("DParameterSecondParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetD2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetClipping2ParameterVelocityControl
  {
    auto ptr = SetClipping2ParameterVelocityControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("IClippingParameterSecondParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetClipping2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetP2ParameterCurrentControl
  {
    auto ptr = SetP2ParameterCurrentControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("PParameterCurrentControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetP2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetI2ParameterCurrentControl
  {
    auto ptr = SetI2ParameterCurrentControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("IParameterCurrentControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetI2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetD2ParameterCurrentControl
  {
    auto ptr = SetD2ParameterCurrentControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("DParameterCurrentControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetD2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetClipping2ParameterCurrentControl
  {
    auto ptr = SetClipping2ParameterCurrentControl::InitSharedPtr(slaveIndex,
      int32_t(config.at("IClippingParameterCurrentControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetClipping2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetMaxVelocityToReachTargetRPM
  {
    auto ptr = SetMaxVelocityToReachTargetRPM::InitSharedPtr(slaveIndex,
      int32_t(config.at("MaximumVelocityToSetPositionMotorRPM")));
    center->SendMessage_(ptr);
    log(Log::info, " SetMaxVelocityToReachTargetRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetMaxDistanceToReachTarget
  {
    auto ptr = SetMaxDistanceToReachTarget::InitSharedPtr(slaveIndex,
      int32_t(config.at("PositionTargetReachedDistance")));
    center->SendMessage_(ptr);
    log(Log::info, " SetMaxDistanceToReachTarget: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetVelThresholdHallFXRPM
  {
    auto ptr = SetVelThresholdHallFXRPM::InitSharedPtr(slaveIndex,
      int32_t(config.at("VelocityThresholdForHallFXMotorRPM")));
    center->SendMessage_(ptr);
    log(Log::info, " SetVelThresholdHallFXRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  SetConfiguratedViaMailbox();
}

bool YoubotJoint::CheckConfig() {
  // GetMaxRampVelocityRPM
  {
    auto ptr = GetMaxRampVelocityRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("MaximumPositioningVelocityMotorRPM"));
    log(Log::info, " GetMaxRampVelocityRPM: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetAccelerationParamRPMPSec
  {
    auto ptr = GetAccelerationParamRPMPSec::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("MotorAccelerationMotorRPMPerSec"));
    log(Log::info, " GetAccelerationParamRPMPSec: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetTresholdSpeedForPosPIDRPM
  {
    auto ptr = GetTresholdSpeedForPosPIDRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PositionControlSwitchingThresholdMotorRPM"));
    log(Log::info, " GetTresholdSpeedForPosPIDRPM: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetTresholdSpeedForVelPIDRPM
  {
    auto ptr = GetTresholdSpeedForVelPIDRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("SpeedControlSwitchingThresholdMotorRPM"));
    log(Log::info, " GetTresholdSpeedForVelPIDRPM: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP1ParameterPositionControl
  {
    auto ptr = GetP1ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PParameterFirstParametersPositionControl"));
    log(Log::info, " GetP1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI1ParameterPositionControl
  {
    auto ptr = GetI1ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IParameterFirstParametersPositionControl"));
    log(Log::info, " GetI1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD1ParameterPositionControl
  {
    auto ptr = GetD1ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("DParameterFirstParametersPositionControl"));
    log(Log::info, " GetD1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping1ParameterPositionControl
  {
    auto ptr = GetClipping1ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IClippingParameterFirstParametersPositionControl"));
    log(Log::info, " GetClipping1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP1ParameterVelocityControl
  {
    auto ptr = GetP1ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PParameterFirstParametersSpeedControl"));
    log(Log::info, " GetP1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD1ParameterVelocityControl
  {
    auto ptr = GetD1ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("DParameterFirstParametersSpeedControl"));
    log(Log::info, " GetD1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI1ParameterVelocityControl
  {
    auto ptr = GetI1ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IParameterFirstParametersSpeedControl"));
    log(Log::info, " GetI1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping1ParameterVelocityControl
  {
    auto ptr = GetClipping1ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IClippingParameterFirstParametersSpeedControl"));
    log(Log::info, " GetClipping1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP2ParameterPositionControl
  {
    auto ptr = GetP2ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PParameterSecondParametersPositionControl"));
    log(Log::info, " GetP2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI2ParameterPositionControl
  {
    auto ptr = GetI2ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IParameterSecondParametersPositionControl"));
    log(Log::info, " GetI2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD2ParameterPositionControl
  {
    auto ptr = GetD2ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("DParameterSecondParametersPositionControl"));
    log(Log::info, " GetD2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping2ParameterPositionControl
  {
    auto ptr = GetClipping2ParameterPositionControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IClippingParameterSecondParametersPositionControl"));
    log(Log::info, " GetClipping2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP2ParameterVelocityControl
  {
    auto ptr = GetP2ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PParameterSecondParametersSpeedControl"));
    log(Log::info, " GetP2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI2ParameterVelocityControl
  {
    auto ptr = GetI2ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IParameterSecondParametersSpeedControl"));
    log(Log::info, " GetI2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD2ParameterVelocityControl
  {
    auto ptr = GetD2ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("DParameterSecondParametersSpeedControl"));
    log(Log::info, " GetD2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping2ParameterVelocityControl
  {
    auto ptr = GetClipping2ParameterVelocityControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IClippingParameterSecondParametersSpeedControl"));
    log(Log::info, " GetClipping2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP2ParameterCurrentControl
  {
    auto ptr = GetP2ParameterCurrentControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PParameterCurrentControl"));
    log(Log::info, " GetP2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI2ParameterCurrentControl
  {
    auto ptr = GetI2ParameterCurrentControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IParameterCurrentControl"));
    log(Log::info, " GetI2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD2ParameterCurrentControl
  {
    auto ptr = GetD2ParameterCurrentControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("DParameterCurrentControl"));
    log(Log::info, " GetD2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping2ParameterCurrentControl
  {
    auto ptr = GetClipping2ParameterCurrentControl::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("IClippingParameterCurrentControl"));
    log(Log::info, " GetClipping2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetMaxVelocityToReachTargetRPM
  {
    auto ptr = GetMaxVelocityToReachTargetRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("MaximumVelocityToSetPositionMotorRPM"));
    log(Log::info, " GetMaxVelocityToReachTargetRPM: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetMaxDistanceToReachTarget
  {
    auto ptr = GetMaxDistanceToReachTarget::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("PositionTargetReachedDistance"));
    log(Log::info, " GetMaxDistanceToReachTarget: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetVelThresholdHallFXRPM
  {
    auto ptr = GetVelThresholdHallFXRPM::InitSharedPtr(slaveIndex);
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(config.at("VelocityThresholdForHallFXMotorRPM"));
    log(Log::info, " GetVelThresholdHallFXRPM: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  return true;
}

void YoubotJoint::RotateJointRightViaMailbox(double speedJointRadPerSec) {
  auto ptr = RotateRightMotorRPM::InitSharedPtr(slaveIndex, int32_t(speedJointRadPerSec / gearRatio / 2. / M_PI * 60.));
  center->SendMessage_(ptr);
  log(Log::info, " RotateRightMotorRPM: " + std::to_string(ptr->GetReplyValue()) +
    " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

void YoubotJoint::RotateJointLeftViaMailbox(double speedJointRadPerSec) {
  auto ptr = RotateLeftMotorRPM::InitSharedPtr(slaveIndex, int32_t(speedJointRadPerSec / gearRatio / 2. / M_PI * 60.));
  center->SendMessage_(ptr);
  log(Log::info, " RotateLeftMotorRPM: " + std::to_string(ptr->GetReplyValue()) +
    " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

void YoubotJoint::StopViaMailbox() {
  auto ptr = MotorStop::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, " MotorStop: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

YoubotJoint::JointStatus YoubotJoint::GetJointStatusViaMailbox() {
  auto ptr = GetErrorStatusFlag::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, "GetErrorStatusFlag: " +
    TMCL::StatusErrorFlagsToString(ptr->GetReplyValue())
    + "(" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return ptr->GetReplyValue();
}

void YoubotJoint::ResetTimeoutViaMailbox() {
  auto ptr = ClearMotorControllerTimeoutFlag::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, "  ClearMotorControllerTimeoutFlag: " + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()));
  SLEEP_MILLISEC(6)
}

void YoubotJoint::ResetI2TExceededViaMailbox() {
  auto ptr = ClearI2TFlag::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, "  ClearI2TFlag: " + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()));
  SLEEP_MILLISEC(6)
}

void YoubotJoint::StartInitialization() {
  StopViaMailbox();
  auto status = GetJointStatusViaMailbox();
  log(Log::info, status.toString());
  if (status.Timeout())
    ResetTimeoutViaMailbox();
  if (status.I2TExceeded())
    ResetI2TExceededViaMailbox();
  status = GetJointStatusViaMailbox();
  log(Log::info, status.toString());
  auto ptr = SetInitialize::InitSharedPtr(slaveIndex, 1);
  center->SendMessage_(ptr);
  log(Log::info, "  SetInitialize: " + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()));
}

bool YoubotJoint::IsInitialized() {
  auto ptr = GetInitialized::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, "  GetInitialized: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return ptr->GetReplyValue();
}

bool YoubotJoint::IsConfiguratedViaMailbox() {
  auto ptr = GetNeedConfiguration::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, " GetNeedConfiguration: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return !ptr->GetReplyValue();
}

void YoubotJoint::SetConfiguratedViaMailbox() {
  auto ptr = SetIsConfigurated::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, " SetIsConfigurated: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
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
    ss <<  " OVER_CURRENT";
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
  log(Log::info, "encoderPosition: " + std::to_string((int)encoderPosition) + " currentmA: " + std::to_string((int)currentmA) + " motorVelocityRPM: " + std::to_string((int)motorVelocityRPM) + " status: " + status.toString() + " motorPWM: " + std::to_string((int)motorPWM));
}

YoubotJoint::ProcessReturn::ProcessReturn() : status(0), encoderPosition(-1),
currentmA(-1), motorVelocityRPM(-1), motorPWM(-1) {};

const YoubotJoint::ProcessReturn& YoubotJoint::GetProcessReturnData() {
  static ProcessBuffer buffer;
  center->GetProcessMsg(buffer, slaveIndex);
  processReturn.encoderPosition = _toInt32(&buffer.buffer[0]);
  processReturn.qDeg = conversion.qDegFromTicks(processReturn.encoderPosition);
  processReturn.currentmA = _toInt32(&buffer.buffer[4]);
  processReturn.motorVelocityRPM = _toInt32(&buffer.buffer[8]);
  processReturn.status = _toInt32(&buffer.buffer[12]);
  processReturn.motorPWM = _toInt32(&buffer.buffer[16]);
  return processReturn;
}

void YoubotJoint::ReqVelocityJointRadPerSec(double value) {
  ReqVelocityMotorRPM(int32_t(value / 2. / M_PI * 60. / gearRatio));
}


void YoubotJoint::ReqJointPositionDeg(double deg) {
  static ProcessBuffer toSet(5);
  int32_t ticks = conversion.ticksFromqDeg(deg);
  toSet.buffer[3] = ticks >> 24;
  toSet.buffer[2] = ticks >> 16;
  toSet.buffer[1] = ticks >> 8;
  toSet.buffer[0] = ticks & 0xff;
  toSet.buffer[4] = TMCL::ControllerMode::POSITION_CONTROL;
  center->SetProcessMsg(toSet, slaveIndex);
}

double YoubotJoint::GetJointPositionDeg() {
  return GetProcessReturnData().qDeg;
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

void YoubotJoint::ReqEncoderReference(int32_t value) {
  static ProcessBuffer toSet(5);
  toSet.buffer[3] = value >> 24;
  toSet.buffer[2] = value >> 16;
  toSet.buffer[1] = value >> 8;
  toSet.buffer[0] = value & 0xff;
  toSet.buffer[4] = TMCL::ControllerMode::SET_POSITION_TO_REFERENCE;
  center->SetProcessMsg(toSet, slaveIndex);
}

void YoubotJoint::ReqMotorStopViaProcess() {
  static ProcessBuffer toSet(5);
  for (int i = 0; i < 4; i++)
    toSet.buffer[i] = 0;
  toSet.buffer[4] = TMCL::ControllerMode::MOTOR_STOP;
  center->SetProcessMsg(toSet, slaveIndex);
}

void YoubotJoint::ReqVoltagePWM(int32_t value) {
  static ProcessBuffer toSet(5);
  toSet.buffer[3] = value >> 24;
  toSet.buffer[2] = value >> 16;
  toSet.buffer[1] = value >> 8;
  toSet.buffer[0] = value & 0xff;
  toSet.buffer[4] = TMCL::ControllerMode::PWM_MODE;
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
  log(Log::info, " GetCurrent[mA]: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return double(ptr->GetReplyValue()) / 1000.;
}

void YoubotJoint::RotateMotorRightViaMailbox(int32_t speedMotorRPM) {
  auto ptr = RotateRightMotorRPM::InitSharedPtr(slaveIndex, speedMotorRPM);
  center->SendMessage_(ptr);
  log(Log::info, " RotateRightMotorRPM: " + std::to_string(ptr->GetReplyValue()) +
    " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

void YoubotJoint::RotateMotorLeftViaMailbox(int32_t speedMotorRPM) {
  auto ptr = RotateLeftMotorRPM::InitSharedPtr(slaveIndex, speedMotorRPM);
  center->SendMessage_(ptr);
  log(Log::info, " RotateLeftMotorRPM: " + std::to_string(ptr->GetReplyValue()) +
    " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

double YoubotJoint::GetJointVelocityRadPerSec() {
  auto ptr = GetActualSpeedMotorRPM::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, " GetActualSpeedMotorRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return double(ptr->GetReplyValue()) * 2. * M_PI / 60. * gearRatio;
}

void YoubotJoint::SetJointVelocityRadPerSec(double value) {
  auto ptr = RotateRightMotorRPM::InitSharedPtr(slaveIndex, value / gearRatio * 60. / M_PI / 2.);
  center->SendMessage_(ptr);
  log(Log::info, " GetActualSpeedMotorRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

double YoubotJoint::GetThermalWindingTimeSec() {
  auto ptr = GetThermalWindingTimeMs::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, " GetThermalWindingTimeMs: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return double(ptr->GetReplyValue()) / 1000.;
}

void YoubotJoint::SetTargetCurrentA(double current) {
  auto ptr = SetTargetCurrentmA::InitSharedPtr(slaveIndex, int32_t(current * 1000.));
  center->SendMessage_(ptr);
  log(Log::info, " SetTargetCurrentmA[mA]: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

bool YoubotJoint::IsCalibratedViaMailbox() {
  auto ptr = GetNeedCalibration::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, " GetNeedCalibration: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return !ptr->GetReplyValue();
}

void YoubotJoint::SetCalibratedViaMailbox() {
  auto ptr = SetIsCalibrated::InitSharedPtr(slaveIndex);
  center->SendMessage_(ptr);
  log(Log::info, " SetIsCalibrated: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

void YoubotJoint::Initialize() {
  if (!IsInitialized()) {
    auto status = GetJointStatusViaMailbox();
    ResetTimeoutViaMailbox();
    ResetI2TExceededViaMailbox();
    status = GetJointStatusViaMailbox();
    log(Log::info, "Joint " + std::to_string(slaveIndex) + " status before calibration: " + status.toString());
    StartInitialization();
    for (int i = 0; i < 300; i++)
      if (IsInitialized()) {
        log(Log::info, "Joint " + std::to_string(slaveIndex) + " is initialized");
        return;
      }
    if (!IsInitialized())
      throw std::runtime_error("One joint is not initialized and cannot be done it...");
  }
}

double YoubotJoint::Conversion::qDegFromTicks(int32_t ticks) const {
  return double(ticks) * c + qCalibrationDeg;
}

int32_t YoubotJoint::Conversion::ticksFromqDeg(double qDeg) const {
  return int32_t((qDeg - qCalibrationDeg) / c);
}

YoubotJoint::Conversion::Conversion(bool qDirectionSameAsEnc, int32_t ticksPerRound,
  double gearRatio, double qCalibrationDeg) : intialized(1),
  qCalibrationDeg(qCalibrationDeg), c(360. * gearRatio / double(ticksPerRound)) {
  if (!qDirectionSameAsEnc)
    c = -c;
}