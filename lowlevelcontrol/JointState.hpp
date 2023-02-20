#ifndef JOINT_STATE_HPP
#define JOINT_STATE_HPP

#include <string>
#include <atomic>
#include "Data.hpp"

namespace youbot {

  /// <summary>
  /// Struct to describe and handle the status of the motor: error flags, operation mode, etc.
  /// </summary>
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

    bool OverCurrent() const; ///> Check one flag
    bool UnderVoltage() const; ///> Check one flag
    bool OverVoltage() const; ///> Check one flag
    bool OverTemperature() const; ///> Check one flag
    bool MotorHalted() const; ///> Check one flag
    bool HallSensorError() const; ///> Check one flag
    bool EncoderError() const; ///> Check one flag
    bool InitializationError() const; ///> Check one flag
    bool PWMMode() const; ///> Check one flag
    bool VelocityMode() const; ///> Check one flag
    bool PositionMode() const; ///> Check one flag
    bool TorqueMode() const; ///> Check one flag
    bool EmergencyStop() const; ///> Check one flag
    bool FreeRunning() const; ///> Check one flag
    bool PositionReached() const; ///> Check one flag
    bool Initialized() const; ///> Check one flag
    bool Timeout() const; ///> Check one flag
    bool I2TExceeded() const; ///> Check one flag

    /// <summary>
    /// Create string from the flags of STATUS
    /// </summary>
    /// <returns> String that lists the true flags of the status </returns>
    std::string toString() const;

    /// <summary>
    /// Constructor
    /// </summary>
    /// <param name="value"> value created from the status flags </param>
    JointStatus(uint32_t value) : value(value) {} // can be only used if the flags are the same
  };

  /// <summary>
  /// Latest full state of a joint, including position, speed, torque and status
  /// 
  /// Each value describes the timestamp of the given value originated at
  /// </summary>
  struct JointState {
    Data<double> q, dq, tau;
    Data<JointStatus> status;

    /// <summary>
    /// Empty contructor
    /// </summary>
    JointState();

    /// <summary>
    /// Full constructor
    /// </summary>
    /// <param name="q"> joint position [rad] </param>
    /// <param name="dq"> joint speed [rad/s] </param>
    /// <param name="tau"> joint torque [Nm] </param>
    /// <param name="status"> joint status </param>
    JointState(const Data<double>& q, const Data<double>& dq,
      const Data<double>& tau, const Data<JointStatus>& status);
  };
}
#endif
