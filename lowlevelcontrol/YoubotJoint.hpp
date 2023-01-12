#ifndef YOUBOT_JOINT_HPP
#define YOUBOT_JOINT_HPP

#include "EtherCATMaster.hpp"
#include <map>

namespace youbot {

  class YoubotJoint {
    int slaveIndex;
    EtherCATMaster* center;
    std::map<std::string, double> config;

    // Read during configuration
    uint32_t ticksperround = -1;
    int firmwareversion = -1, controllerNum = -1;
    double cooldowntime_sec = -1;
    double gearRatio = -1;
    bool calibrationDirection = false;
    double torqueconstant = false;
    double qMinDeg, qMaxDeg;

    struct Conversion {
      int ticksperround;
      double qCalibrationRad, gearRatio;
      bool intialized;
      double Ticks2qRad(int32_t ticks) const;
      int32_t qRad2Ticks(double qDeg) const;
      double RPM2qRadPerSec(int32_t RPM) const;
      int32_t qRadPerSec2RPM(double degpersec) const;
      Conversion(bool qDirectionSameAsEnc, int32_t ticksPerRound,
        double gearRatio, double qCalibrationDeg);
      Conversion() :intialized(0) {};
    } conversion;

    void _getFirmwareVersionViaMailbox();

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
      bool PositionReached() const;
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
      double qRad;
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
    YoubotJoint() = delete;

    YoubotJoint(YoubotJoint&) = delete;

    YoubotJoint(const YoubotJoint&) = delete;

    YoubotJoint(int slaveIndex, const std::map<std::string, double>& config, EtherCATMaster* center);

    void ConfigParameters(bool forceConfiguration = false);

    bool CheckConfig();

    void Initialize();

    // Mailbox-based Set/Get mothods
    bool IsConfiguratedViaMailbox();

    void SetConfiguratedViaMailbox();

    void RotateJointRightViaMailbox(double speedJointRadPerSec);

    void RotateJointLeftViaMailbox(double speedJointRadPerSec);

    void StopViaMailbox();

    JointStatus GetJointStatusViaMailbox();

    void ResetTimeoutViaMailbox();

    void ResetI2TExceededViaMailbox();

    void StartInitializationViaMailbox();

    bool IsInitializedViaMailbox();

    double GetThermalWindingTimeSecViaMailbox();

    double GetCurrentAViaMailbox();

    void RotateMotorRightViaMailbox(int32_t speedMotorRPM);

    void RotateMotorLeftViaMailbox(int32_t speedMotorRPM);
    
    void SetTargetCurrentAViaMailbox(double current);

    bool IsCalibratedViaMailbox();

    void SetCalibratedViaMailbox();

    long GetI2tLimitValueViaMailbox();

    long GetCurrentI2tValueViaMailbox();

    double GetJointVelocityRadPerSecViaMailbox();

    void SetJointVelocityRadPerSecViaMailbox(double value);

    // Process message-based req/get methods will be sent with the next ExcangeMessage/show the results of the latest one
    const ProcessReturn& GetProcessReturnData();

    void ReqJointPositionRad(double value);

    double GetJointPositionRad();

    void ReqVelocityJointRadPerSec(double value);

    void ReqVelocityMotorRPM(int32_t value);

    void ReqEncoderReference(int32_t value);

    void ReqMotorStopViaProcess();

    void ReqVoltagePWM(int32_t value);

    void ReqInitializationViaProcess();

    typedef std::shared_ptr<YoubotJoint> Ptr;

    // Only for test purposes
    void I2tResetTest();
  };
}
#endif
