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

void YoubotJointVirtual::_calledAtExchange() {
  _update();
  // Update latest values
  ticksLatest.exchange(ticks());
  mALatest.exchange(current_mA);
  RPMLatest.exchange(RPM);
  statusLatest.exchange({ _getStatus() });
  UpwmLatest.exchange(0);
  // Update command
  switch (processCommand.type) {
  case ProcessCommand::POSITION:
    controlMode = POSITION;
    target = processCommand.value;
    break;
  case ProcessCommand::VELOCITY:
    controlMode = VELOCITY;
    target = processCommand.value;
    break;
  case ProcessCommand::CURRENT:
    controlMode = CURRENT;
    target = processCommand.value;
    break;
  case ProcessCommand::PWM:
    controlMode = PWM;
    target = processCommand.value;
    break;
  case ProcessCommand::ENCODER_ZERO:
    settickstozero();
    break;
  case ProcessCommand::INITIALIZE:
    throw std::runtime_error("not implemented"); // not used
    break;
  case ProcessCommand::NO_MORE_ACTION:
    throw std::runtime_error("not implemented"); // not understood
    break;
  }
}

unsigned int YoubotJointVirtual::GetEncoderResolutionViaMailbox() {
  return 4000;
}

std::string youbot::YoubotJointVirtual::GetCommutationModeViaMailbox() {
    return "virtual";
}

JointStatus YoubotJointVirtual::_getStatus() {
  int32_t status = 0;
  if (timeout)
    status |= int32_t(JointStatus::StatusErrorFlags::TIMEOUT);
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
  if (controlMode == POSITION && abs(target - ticks()) < 10) //TODO: 10?
    status |= int32_t(JointStatus::StatusErrorFlags::POSITION_REACHED);
  return status;
}

int64_t youbot::YoubotJointVirtual::ticks() const {
  return qRad2Ticks(qRad_true) + ticks_offset;
}

inline void youbot::YoubotJointVirtual::settickstozero() {
  ticks_offset = -qRad2Ticks(qRad_true);
}

void youbot::YoubotJointVirtual::_updateFor(double elapsedTime) {
  if (timeout || I2terror) { // forcestop
    RPM = 0;
    current_mA = 0;
  }
  else {
    switch (controlMode) {
    case CURRENT: {
      current_mA = target;
      double T = mA2Nm(current_mA);
      double phi0 = qRad_true, dphi0 = RPM2qRadPerSec(RPM);
      double lambda = -damping / theta;
      double A = (dphi0 - T / theta) / lambda;
      double C = phi0 - A;
      double newq = A * exp(lambda * elapsedTime) + T / theta * elapsedTime + C;
      double newdq = A * lambda * exp(lambda * elapsedTime) + T / theta;
      qRad_true = newq;
      RPM = qRadPerSec2RPM(newdq);
      break;
    }
    case VELOCITY: {// fucking fast setling
      RPM = target;
      double qRadPerSec = RPM2qRadPerSec(RPM);
      qRad_true += qRadPerSec * elapsedTime;
      current_mA = 0;
      break;
    }
    case POSITION: {// fucking fast setling
      double old = qRad_true;
      qRad_true = Ticks2qRad(target);
      RPM = qRadPerSec2RPM((qRad_true - old) / elapsedTime);
      break;
    }
    }
  }
  // Stop the joint at the limit
  if (qRad_true < (GetParameters().qMinDeg) / 180 * M_PI) {
    qRad_true = (GetParameters().qMinDeg) / 180 * M_PI;
    RPM = 0;
    return;
  }
  if (qRad_true >(GetParameters().qMaxDeg) / 180 * M_PI) {
    qRad_true = (GetParameters().qMaxDeg) / 180 * M_PI;
    RPM = 0;
    return;
  }
}

void youbot::YoubotJointVirtual::_update() {
  auto now = std::chrono::steady_clock::now();
  double dt = double(std::chrono::duration_cast<std::chrono::milliseconds>(now
    - updated_at).count()) / 1000.;
  // Update commutation state
  commutationState.Update();
  // if commutation is not initialized: do nothing
  if (!commutationState.initialized) {
    updated_at = now;
    return;
  }
  // if went to timeout
  if (dt >= 0.100) {
    _updateFor(0.1); // Update till updated_at+100ms
    timeout = true; // Set timeout
    _updateFor(dt - 0.1);
    updated_at = now;
    return;
  }
  // otherwise
  _updateFor(dt);
  updated_at = now;
}

YoubotJointVirtual::YoubotJointVirtual(int slaveIndex, const std::map<std::string,
  double>& config, EtherCATMaster::Ptr center)
  : YoubotJoint(slaveIndex, config, center) {
  center->RegisterAfterExchangeCallback(std::bind(&YoubotJointVirtual::_calledAtExchange, this));
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

JointStatus YoubotJointVirtual::GetJointStatusViaMailbox() {
  _update();
  auto status = _getStatus();
  statusLatest.exchange(status);
  return status;
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

bool YoubotJointVirtual::IsConfiguratedViaMailbox() {
  return configurated_flag;
}

void YoubotJointVirtual::SetConfiguratedViaMailbox() {
  configurated_flag = true;
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

// Cheat funciton

double youbot::YoubotJointVirtual::GetJointPositionTRUE() const {
  return qRad_true;
}

double YoubotJointVirtual::GetCurrentAViaMailbox() {
  _update();
  auto mA = current_mA;
  mALatest.exchange(mA);
  return double(mA) / 1000.;
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
  RPMLatest.exchange(RPM);
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

void youbot::YoubotJointVirtual::CommutationState::Update() {
  if (!initialized && started) {
    // TODO: move
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at).count() > 500)
      initialized = true;
  }
}
