#include "JointVirtual.hpp"
#include "VirtualEtherCATMaster.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include <sstream>
#include <stdexcept>

using namespace youbot;
using namespace youbot::intrinsic;

void JointVirtual::GetFirmwareVersionViaMailbox(int& controllernum,
  int& firmwareversion) {
  controllernum = 1610;
  firmwareversion = 148;
}

void JointVirtual::_calledAtExchange() {
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
  case ProcessCommand::STOP:
    controlMode = VELOCITY;
    target = 0;
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

unsigned int JointVirtual::GetEncoderResolutionViaMailbox() {
  return 4000;
}

std::string JointVirtual::GetCommutationModeViaMailbox() {
    return "virtual";
}

JointStatus JointVirtual::_getStatus() {
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

int64_t JointVirtual::ticks() const {
  return qRad2Ticks(qRad_true) + ticks_offset;
}

inline void JointVirtual::settickstozero() {
  ticks_offset = -qRad2Ticks(qRad_true);
}

void JointVirtual::_updateFor(double elapsedTime) {
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

void JointVirtual::_update() {
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

JointVirtual::JointVirtual(int slaveIndex, const std::map<std::string,
  double>& config, EtherCATMaster::Ptr center)
  : Joint(slaveIndex, config, center) {
  center->RegisterAfterExchangeCallback(std::bind(&JointVirtual::_calledAtExchange, this));
}

void JointVirtual::ConfigControlParameters(bool forceConfiguration) {
  configurated = true;
}

bool JointVirtual::CheckControlParameters() {
  return configurated;
}

void JointVirtual::RotateJointRightViaMailbox(double speedJointRadPerSec) {
  _update();
  controlMode = VELOCITY;
  target = qRadPerSec2RPM(speedJointRadPerSec);
  log(Log::info, " RotateRightMotorRPM: " + std::to_string(target));
}

void JointVirtual::RotateJointLeftViaMailbox(double speedJointRadPerSec) {
  _update();
  controlMode = VELOCITY;
  target = -qRadPerSec2RPM(speedJointRadPerSec);
  log(Log::info, " RotateLeftMotorRPM: " + std::to_string(-target));
}

void JointVirtual::StopViaMailbox() {
  _update();
  controlMode = VELOCITY;
  target = 0;
  log(Log::info, " MotorStop");
}

JointStatus JointVirtual::GetJointStatusViaMailbox() {
  _update();
  auto status = _getStatus();
  statusLatest.exchange(status);
  return status;
}

void JointVirtual::ResetTimeoutViaMailbox() {
  _update();
  timeout = false;
  log(Log::info, " ResetTimeoutViaMailbox");
}

void JointVirtual::ResetI2TExceededViaMailbox() {
  _update();
  SLEEP_MILLISEC(long(GetParameters().cooldowntime_sec * 1000));
  I2terror = false;
  log(Log::info, "  ClearI2TFlag:  waiting done");
}

void JointVirtual::StartInitializationViaMailbox() {
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

bool JointVirtual::IsConfiguratedViaMailbox() {
  return configurated_flag;
}

void JointVirtual::SetConfiguratedViaMailbox() {
  configurated_flag = true;
}

void JointVirtual::ReqNoAction() {
  processCommand.type = ProcessCommand::NO_MORE_ACTION;
}

void JointVirtual::ReqMotorPositionTick(int ticks) {
  processCommand.type = ProcessCommand::POSITION;
  processCommand.value = ticks;
}

void JointVirtual::ReqMotorSpeedRPM(int32_t value) {
  processCommand.type = ProcessCommand::VELOCITY;
  processCommand.value = value;
}

void JointVirtual::ReqEncoderReference(int32_t value) {
  processCommand.type = ProcessCommand::ENCODER_ZERO;
}

void JointVirtual::ReqStop() {
  processCommand.type = ProcessCommand::VELOCITY;
  processCommand.value = 0;
}

void JointVirtual::ReqVoltagePWM(int32_t value) {
  processCommand.type = ProcessCommand::PWM;
  processCommand.value = value;
}

void JointVirtual::ReqMotorCurrentmA(int32_t value) {
  processCommand.type = ProcessCommand::CURRENT;
  processCommand.value = value;
}

void JointVirtual::ReqInitializationViaProcess() {
  processCommand.type = ProcessCommand::INITIALIZE;
}

void JointVirtual::CheckI2tAndTimeoutError(JointStatus status) {
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

double JointVirtual::GetJointPositionTRUE() const {
  return qRad_true;
}

double JointVirtual::GetCurrentAViaMailbox() {
  _update();
  auto mA = current_mA;
  mALatest.exchange(mA);
  return double(mA) / 1000.;
}

void JointVirtual::RotateMotorRightViaMailbox(int32_t speedMotorRPM) {
  _update();
  controlMode = VELOCITY;
  target = speedMotorRPM;
  log(Log::info, " RotateRightMotorRPM: " + std::to_string(speedMotorRPM));
}

void JointVirtual::RotateMotorLeftViaMailbox(int32_t speedMotorRPM) {
  _update();
  controlMode = VELOCITY;
  target = -speedMotorRPM;
  log(Log::info, " RotateLeftMotorRPM: " + std::to_string(speedMotorRPM));
}

double JointVirtual::GetJointVelocityRadPerSecViaMailbox() {
  _update();
  RPMLatest.exchange(RPM);
  return RPM2qRadPerSec(RPM);
}

long JointVirtual::GetI2tLimitValueViaMailbox() {
  throw std::runtime_error("Not implemented");
  return 10000;
}

long JointVirtual::GetCurrentI2tValueViaMailbox() {
  throw std::runtime_error("Not implemented");
  return 10000;
}

void JointVirtual::SetJointVelocityRadPerSecViaMailbox(double value) {
  _update();
  controlMode = VELOCITY;
  target = qRadPerSec2RPM(value);
}

double JointVirtual::GetThermalWindingTimeSecViaMailbox() {
  return 10.;
}

void JointVirtual::SetTargetCurrentAViaMailbox(double current) {
  _update();
  controlMode = CURRENT;
  target = current*1000.;
}

bool JointVirtual::IsCalibratedViaMailbox() {
  return calibrated_flag;
}

void JointVirtual::SetCalibratedViaMailbox() {
  calibrated_flag = true;
}

void JointVirtual::CommutationState::Update() {
  if (!initialized && started) {
    // TODO: move
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at).count() > 500)
      initialized = true;
  }
}
