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
  double gearRatio = -1;
  bool directionreversed = false;
  bool calibrationDirection = false;
  double torqueconstant = false;
  double calibrationmaxAmpere = -1;


  bool calibratedposition = -1;

  void _getFirmwareVersion();

public:
  struct JointStatus {
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
    bool PosiitonReached() const;
    bool Initialized() const;
    bool Timeout() const;
    bool I2TExceeded() const;
    std::string toString() const;

    JointStatus(uint32_t value) : value(value) {}

  };

  YoubotJoint(int slaveIndex, const NameValueMap& config, VMessageCenter* center);

  void ConfigParameters();

  bool CheckConfig();

  void RotateRightViaMailbox(double speedJointRadPerSec);

  void RotateLeftViaMailbox(double speedJointRadPerSec);

  void StopViaMailbox();

  JointStatus GetJointStatusViaMailbox();

  void ResetTimeoutViaMailbox();

  void ResetI2TExceededViaMailbox();

  void StartInitialization();

  bool IsInitialized();

};

#endif
