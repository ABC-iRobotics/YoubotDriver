#ifndef YOUBOT_JOINT_HPP
#define YOUBOT_JOINT_HPP

#include "VMessageCenter.hpp"
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

  struct ProcessReturn {
    int32_t encoderPosition;
    int32_t currentmA;
    int32_t motorVelocityRPM;
    JointStatus status;
    int32_t motorPWM;
    // double jointAngle, jointVelocityRad/s, torque, ...

    ProcessReturn();
    void Print() const;
  };

  int32_t _toInt32(uint8_t* buff) {
    return buff[3] << 24 | buff[2] << 16 | buff[1] << 8 | buff[0];
  }

private:
  ProcessReturn processReturn;

public:
  YoubotJoint(int slaveIndex, const NameValueMap& config, VMessageCenter* center);

  void ConfigParameters();

  bool CheckConfig();

  void RotateJointRightViaMailbox(double speedJointRadPerSec);

  void RotateJointLeftViaMailbox(double speedJointRadPerSec);

  void StopViaMailbox();

  JointStatus GetJointStatusViaMailbox();

  void ResetTimeoutViaMailbox();

  void ResetI2TExceededViaMailbox();

  void StartInitialization();

  bool IsInitialized();

  const ProcessReturn& GetProcessReturnData();

  void ReqVelocityMotorRPM(int32_t value);

  void ReqMotorStopViaProcess();

  void ReqSetPositionToReferenceViaProcess();

  void ReqInitializationViaProcess();

  double GetCurrentAViaMailbox();

  void RotateMotorRightViaMailbox(int32_t speedMotorRPM);

  void RotateMotorLeftViaMailbox(int32_t speedMotorRPM);

  double GetJointVelocityRadPerSec();

  void SetTargetCurrentA(double current);
};

#endif
