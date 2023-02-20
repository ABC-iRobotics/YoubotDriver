#ifndef JOINT_STATE_HPP
#define JOINT_STATE_HPP

#include <string>
#include <atomic>
#include "Data.hpp"

namespace youbot {

  struct JointStatus {
    enum class StatusErrorFlags : uint32_t {
      OVER_CURRENT = 0x1,
      UNDER_VOLTAGE = 0x2,
      OVER_VOLTAGE = 0x4,
      OVER_TEMPERATURE = 0x8,
      MOTOR_HALTED = 0x10,
      HALL_SENSOR_ERROR = 0x20,
      ENCODER_ERROR = 0x40,
      INITIALIZATION_ERROR = 0x80,
      PWM_MODE_ACTIVE = 0x100,
      VELOCITY_MODE_ACTIVE = 0x200,
      POSITION_MODE_ACTIVE = 0x400,
      TORQUE_MODE_ACTIVE = 0x800,
      EMERGENCY_STOP = 0x1000,
      FREERUNNING = 0x2000,
      POSITION_REACHED = 0x4000,
      INITIALIZED = 0x8000,
      TIMEOUT = 0x10000,
      I2T_EXCEEDED = 0x20000
    };
    uint32_t value;

    bool OverCurrent() const;
    bool UnderVoltage() const;
    bool OverVoltage() const;
    bool OverTemperature() const;
    bool MotorHalted() const;
    bool HallSensorError() const;
    bool EncoderError() const;
    bool InitializationError() const;
    bool PWMMode() const;
    bool VelocityMode() const;
    bool PositionMode() const;
    bool TorqueMode() const;
    bool EmergencyStop() const;
    bool FreeRunning() const;
    bool PositionReached() const;
    bool Initialized() const;
    bool Timeout() const;
    bool I2TExceeded() const;
    std::string toString() const;

    JointStatus(uint32_t value) : value(value) {} // can be only used if the flags are the same
  };

  struct JointState {
    Data<double> q, dq, tau;
    Data<JointStatus> status;
    JointState();
    JointState(const Data<double>& q, const Data<double>& dq,
      const Data<double>& tau, const Data<JointStatus>& status);
  };
}
#endif
