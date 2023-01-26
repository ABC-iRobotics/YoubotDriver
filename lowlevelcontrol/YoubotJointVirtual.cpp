#include "YoubotJointVirtual.hpp"
#include "VirtualEtherCATMaster.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include <sstream>
#include <stdexcept>

using namespace youbot;

void YoubotJointVirtual::GetFirmwareVersionViaMailbox(int& controllernum,
  int& firmwareversion) {
  controllernum = 1610;
  firmwareversion = 148;
}

unsigned int YoubotJointVirtual::GetEncoderResolutionViaMailbox() {
  return 4000;
}

YoubotJointVirtual::JointStatus YoubotJointVirtual::_getStatus() {
  int32_t status = 0;
  if (timeout)
    status |= int32_t(JointStatus::StatusErrorFlags::INITIALIZED);
  if (I2terror)
    status |= int32_t(JointStatus::StatusErrorFlags::I2T_EXCEEDED);
  if (commutationState.initialized)
    status |= int32_t(JointStatus::StatusErrorFlags::INITIALIZED);
  switch (controlMode) {
  case POSITION:
    status |= int32_t(JointStatus::StatusErrorFlags::POSITION_MODE_ACTIVE);
    break;
  case VELOCITY:
    status |= int32_t(JointStatus::StatusErrorFlags::VELOCITY_MODE_ACTIVE);
    break;
  case CURRENT:
    status |= int32_t(JointStatus::StatusErrorFlags::TORQUE_MODE_ACTIVE);
    break;
  case PWM:
    status |= int32_t(JointStatus::StatusErrorFlags::PWM_MODE_ACTIVE);
    break;
  }
  if (abs(RPM) < 10) //TODO: 10?
    status |= int32_t(JointStatus::StatusErrorFlags::MOTOR_HALTED);
  if (controlMode == POSITION && abs(target - ticks) < 10) //TODO: 10?
    status |= int32_t(JointStatus::StatusErrorFlags::POSITION_REACHED);
  return status;
}

YoubotJointVirtual::YoubotJointVirtual(int slaveIndex, const std::map<std::string,
  double>& config, EtherCATMaster::Ptr center)
  : YoubotJoint(slaveIndex, config, center) {
  std::dynamic_pointer_cast<intrinsic::VirtualEtherCATMaster>(center)->RegisterSlave(
    std::bind(&YoubotJointVirtual::_calledAtExchange, this));
}

void YoubotJointVirtual::ConfigControlParameters(bool forceConfiguration) {
  configurated = true;
}

bool YoubotJointVirtual::CheckControlParameters() {
  return configurated;
}

void YoubotJointVirtual::RotateJointRightViaMailbox(double speedJointRadPerSec) {
  _update();
  controlMode = VELOCITY;
  target = qRadPerSec2RPM(speedJointRadPerSec);
  log(Log::info, " RotateRightMotorRPM: " + std::to_string(target));
}

void YoubotJointVirtual::RotateJointLeftViaMailbox(double speedJointRadPerSec) {
  _update();
  controlMode = VELOCITY;
  target = -qRadPerSec2RPM(speedJointRadPerSec);
  log(Log::info, " RotateLeftMotorRPM: " + std::to_string(-target));
}

void YoubotJointVirtual::StopViaMailbox() {
  _update();
  controlMode = VELOCITY;
  target = 0;
  log(Log::info, " MotorStop");
}

YoubotJointVirtual::JointStatus YoubotJointVirtual::GetJointStatusViaMailbox() {
  _update();
  return _getStatus();
}

void YoubotJointVirtual::ResetTimeoutViaMailbox() {
  _update();
  timeout = false;
  log(Log::info, " ResetTimeoutViaMailbox");
}

void YoubotJointVirtual::ResetI2TExceededViaMailbox() {
  _update();
  SLEEP_MILLISEC(long(GetParameters().cooldowntime_sec * 1000));
  I2terror = false;
  log(Log::info, "  ClearI2TFlag:  waiting done");
}

void YoubotJointVirtual::StartInitializationViaMailbox() {
  StopViaMailbox();
  auto status = GetJointStatusViaMailbox();
  log(Log::info, status.toString());
  if (status.I2TExceeded())
    ResetI2TExceededViaMailbox();
  if (status.Timeout())
    ResetTimeoutViaMailbox();
  status = GetJointStatusViaMailbox();
  log(Log::info, status.toString());

  _update();
  commutationState.started = true;
  commutationState.started_at = std::chrono::steady_clock::now();
  log(Log::info, "  SetInitialize: OK");
}

bool YoubotJointVirtual::IsInitializedViaMailbox() {
  _update();
  return commutationState.initialized;
}

bool YoubotJointVirtual::IsConfiguratedViaMailbox() {
  return configurated_flag;
}

void YoubotJointVirtual::SetConfiguratedViaMailbox() {
  configurated_flag = true;
}

const YoubotJointVirtual::ProcessReturn& YoubotJointVirtual::GetProcessReturnData() {
  return processReturnAfterExchange;
}

void YoubotJointVirtual::ReqNoAction() {
  processCommand.type = ProcessCommand::NO_MORE_ACTION;
}

void YoubotJointVirtual::ReqMotorPositionTick(int ticks) {
  processCommand.type = ProcessCommand::POSITION;
  processCommand.value = ticks;
}

void YoubotJointVirtual::ReqMotorSpeedRPM(int32_t value) {
  processCommand.type = ProcessCommand::VELOCITY;
  processCommand.value = value;
}

void YoubotJointVirtual::ReqEncoderReference(int32_t value) {
  processCommand.type = ProcessCommand::ENCODER_ZERO;
}

void YoubotJointVirtual::ReqStop() {
  processCommand.type = ProcessCommand::VELOCITY;
  processCommand.value = 0;
}

void YoubotJointVirtual::ReqVoltagePWM(int32_t value) {
  processCommand.type = ProcessCommand::PWM;
  processCommand.value = value;
}

void youbot::YoubotJointVirtual::ReqMotorCurrentmA(int32_t value) {
  processCommand.type = ProcessCommand::CURRENT;
  processCommand.value = value;
}

void YoubotJointVirtual::ReqInitializationViaProcess() {
  processCommand.type = ProcessCommand::INITIALIZE;
}

void youbot::YoubotJointVirtual::CheckI2tAndTimeoutError(JointStatus status) {
  _update();
  if (status.I2TExceeded()) {
    log(Log::fatal, "I2t exceeded in slave " + std::to_string(GetSlaveIndex()) + " (" + status.toString() + ")");
    SLEEP_MILLISEC(10);
    throw std::runtime_error("I2t exceeded");
  }
  if (status.Timeout()) {
    log(Log::fatal, "Timeout in slave " + std::to_string(GetSlaveIndex()) + " (" + status.toString() + ")");
    SLEEP_MILLISEC(10);
    throw std::runtime_error("Timeout");
  }
}

double YoubotJointVirtual::GetCurrentAViaMailbox() {
  _update();
  return double(current_mA) / 1000.;
}

void YoubotJointVirtual::RotateMotorRightViaMailbox(int32_t speedMotorRPM) {
  _update();
  controlMode = VELOCITY;
  target = speedMotorRPM;
  log(Log::info, " RotateRightMotorRPM: " + std::to_string(speedMotorRPM));
}

void YoubotJointVirtual::RotateMotorLeftViaMailbox(int32_t speedMotorRPM) {
  _update();
  controlMode = VELOCITY;
  target = -speedMotorRPM;
  log(Log::info, " RotateLeftMotorRPM: " + std::to_string(speedMotorRPM));
}

double YoubotJointVirtual::GetJointVelocityRadPerSecViaMailbox() {
  _update();
  return RPM2qRadPerSec(RPM);
}

long youbot::YoubotJointVirtual::GetI2tLimitValueViaMailbox() {
  throw std::runtime_error("Not implemented");
  return 10000;
}

long youbot::YoubotJointVirtual::GetCurrentI2tValueViaMailbox() {
  throw std::runtime_error("Not implemented");
  return 10000;
}

void YoubotJointVirtual::SetJointVelocityRadPerSecViaMailbox(double value) {
  _update();
  controlMode = VELOCITY;
  target = qRadPerSec2RPM(value);
}

double YoubotJointVirtual::GetThermalWindingTimeSecViaMailbox() {
  return 10.;
}

void YoubotJointVirtual::SetTargetCurrentAViaMailbox(double current) {
  _update();
  controlMode = CURRENT;
  target = current*1000.;
}

bool YoubotJointVirtual::IsCalibratedViaMailbox() {
  return calibrated_flag;
}

void YoubotJointVirtual::SetCalibratedViaMailbox() {
  calibrated_flag = true;
}