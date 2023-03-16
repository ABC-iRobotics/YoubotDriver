#include "JointState.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include <sstream>
#include <stdexcept>

using namespace youbot;

bool JointStatus::OverCurrent() const {
  return value & (uint32_t)StatusErrorFlags::OVER_CURRENT;
}

bool JointStatus::UnderVoltage() const {
  return value & (uint32_t)StatusErrorFlags::UNDER_VOLTAGE;
};

bool JointStatus::OverVoltage() const {
  return value & (uint32_t)StatusErrorFlags::OVER_VOLTAGE;
};

bool JointStatus::OverTemperature() const {
  return value & (uint32_t)StatusErrorFlags::OVER_TEMPERATURE;
};

bool JointStatus::MotorHalted() const {
  return value & (uint32_t)StatusErrorFlags::MOTOR_HALTED;
};

bool JointStatus::HallSensorError() const {
  return value & (uint32_t)StatusErrorFlags::HALL_SENSOR_ERROR;
};

bool JointStatus::EncoderError() const {
  return value & (uint32_t)StatusErrorFlags::ENCODER_ERROR;
};

bool JointStatus::InitializationError() const {
  return value & (uint32_t)StatusErrorFlags::INITIALIZATION_ERROR;
};

bool JointStatus::PWMMode() const {
  return value & (uint32_t)StatusErrorFlags::PWM_MODE_ACTIVE;
};

bool JointStatus::VelocityMode() const {
  return value & (uint32_t)StatusErrorFlags::VELOCITY_MODE_ACTIVE;
};

bool JointStatus::PositionMode() const {
  return value & (uint32_t)StatusErrorFlags::POSITION_MODE_ACTIVE;
};

bool JointStatus::TorqueMode() const {
  return value & (uint32_t)StatusErrorFlags::TORQUE_MODE_ACTIVE;
};

bool JointStatus::EmergencyStop() const {
  return value & (uint32_t)StatusErrorFlags::EMERGENCY_STOP;
};

bool JointStatus::FreeRunning() const {
  return value & (uint32_t)StatusErrorFlags::FREERUNNING;
};

bool JointStatus::PositionReached() const {
  return value & (uint32_t)StatusErrorFlags::POSITION_REACHED;
};

bool JointStatus::Initialized() const {
  return value & (uint32_t)StatusErrorFlags::INITIALIZED;
};

bool JointStatus::Timeout() const {
  return value & (uint32_t)StatusErrorFlags::TIMEOUT;
};

bool JointStatus::I2TExceeded() const {
  return value & (uint32_t)StatusErrorFlags::I2T_EXCEEDED;
};

std::string JointStatus::toString() const {
  std::stringstream ss;
  if (OverCurrent())
    ss <<  " OVER_CURRENT";
  if (UnderVoltage())
    ss << " UNDER_VOLTAGE";
  if (OverVoltage())
    ss << " OVER_VOLTAGE";
  if (OverTemperature())
    ss << " OVER_TEMPERATURE";
  if (MotorHalted())
    ss << " MOTOR_HALTED";
  if (HallSensorError())
    ss << " HALL_SENSOR_ERROR";
  if (EncoderError())
    ss << " ENCODER_ERROR";
  if (InitializationError())
    ss << " INITIALIZATION_ERROR";
  if (PWMMode())
    ss << " PWM_MODE_ACTIVE";
  if (VelocityMode())
    ss << " VELOCITY_MODE_ACTIVE";
  if (PositionMode())
    ss << " POSITION_MODE_ACTIVE";
  if (TorqueMode())
    ss << " TORQUE_MODE_ACTIVE";
  if (EmergencyStop())
    ss << " EMERGENCY_STOP";
  if (FreeRunning())
    ss << " FREERUNNING";
  if (PositionReached())
    ss << " POSITION_REACHED";
  if (Initialized())
    ss << " INITIALIZED";
  if (Timeout())
    ss << " TIMEOUT";
  if (I2TExceeded())
    ss << " I2T_EXCEEDED";
  return ss.str();
}

JointState::JointState() {};
JointState::JointState(const Data<double>& q, const Data<int32_t>& motorticks,
  const Data<double>& dq, const Data<int32_t>& motorRPM,
  const Data<double>& tau, const Data<JointStatus>& status) :
  q(q), dq(dq), tau(tau), status(status), motorticks(motorticks), motorRPM(motorRPM) {};

static std::chrono::steady_clock::time_point started_at = std::chrono::steady_clock::now() - std::chrono::minutes(1);
