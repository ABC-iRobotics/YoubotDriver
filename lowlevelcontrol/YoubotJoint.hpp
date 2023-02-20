#ifndef YOUBOT_JOINT_ABSTRACT_HPP
#define YOUBOT_JOINT_ABSTRACT_HPP

#include <map>
#include <string>
#include "EtherCATMaster.hpp"
#include "YoubotJointState.hpp"

namespace youbot {

  class YoubotJoint {
  public:
    struct Parameters {
      std::string commutationmode;
      uint32_t ticksperround;
      int firmwareversion, controllerNum;
      double cooldowntime_sec;
      bool calibrationDirection;
      double qMinDeg, qMaxDeg;
      double qCalibrationRad=0, gearRatio=1;
      double torqueconstantNmPerA=1;
      bool intialized = false;
    };

    // Not available constructors
    YoubotJoint() = delete;
    YoubotJoint(YoubotJoint&) = delete;
    YoubotJoint(const YoubotJoint&) = delete;

    // Constructor
    YoubotJoint(int slaveIndex, const std::map<std::string, double>& config,
      EtherCATMaster::Ptr center);

    // Main initialization routine
    void InitializeJoint(bool forceConfiguration = false);

    // Submethods of initialization
    void CollectBasicParameters(); // get essential paramters from the config and the motor driver
    virtual void ConfigControlParameters(bool forceConfiguration = false) = 0; // set the control parameters to the driver
    virtual bool CheckControlParameters() = 0; // check the control parameters to the driver
    virtual void InitCommutation(); // set commutation/encoder of the driver

    // Variable getters
    const Parameters& GetParameters() const;
    const std::map<std::string, double>& GetConfig() const;
    int GetSlaveIndex() const;

    // Conversions
    double Ticks2qRad(int32_t ticks) const;
    int32_t qRad2Ticks(double qDeg) const;
    double RPM2qRadPerSec(int32_t RPM) const;
    int32_t qRadPerSec2RPM(double degpersec) const;
    double mA2Nm(int32_t mA) const;
    int32_t Nm2mA(double Nm) const;

    // Mailbox-based Set/Get mothods
    virtual bool IsConfiguratedViaMailbox() = 0;
    virtual void SetConfiguratedViaMailbox() = 0;
    virtual void RotateJointRightViaMailbox(double speedJointRadPerSec) = 0;
    virtual void RotateJointLeftViaMailbox(double speedJointRadPerSec) = 0;
    virtual void StopViaMailbox() = 0;
    virtual JointStatus GetJointStatusViaMailbox() = 0;
    virtual void ResetTimeoutViaMailbox() = 0;
    virtual void ResetI2TExceededViaMailbox() = 0;
    virtual void StartInitializationViaMailbox() = 0;
    virtual double GetThermalWindingTimeSecViaMailbox() = 0;
    virtual double GetCurrentAViaMailbox() = 0;
    virtual void RotateMotorRightViaMailbox(int32_t speedMotorRPM) = 0;
    virtual void RotateMotorLeftViaMailbox(int32_t speedMotorRPM) = 0;
    virtual void SetTargetCurrentAViaMailbox(double current) = 0;
    virtual bool IsCalibratedViaMailbox() = 0;
    virtual void SetCalibratedViaMailbox() = 0;
    virtual long GetI2tLimitValueViaMailbox() = 0;
    virtual long GetCurrentI2tValueViaMailbox() = 0;
    virtual double GetJointVelocityRadPerSecViaMailbox() = 0;
    virtual void SetJointVelocityRadPerSecViaMailbox(double value) = 0;
    virtual void GetFirmwareVersionViaMailbox(int& controllernum,
      int& firmwareversion) = 0;
    virtual unsigned int GetEncoderResolutionViaMailbox() = 0;
    virtual std::string GetCommutationModeViaMailbox() = 0;

    // Process message-based req/get methods will be sent with the next ExcangeMessage/show the results of the latest one
    virtual void ReqStop() = 0;

    // Req motor quantity
    virtual void ReqMotorSpeedRPM(int32_t value) = 0;
    virtual void ReqEncoderReference(int32_t value) = 0;
    virtual void ReqMotorPositionTick(int32_t value) = 0;
    virtual void ReqVoltagePWM(int32_t value) = 0;
    virtual void ReqMotorCurrentmA(int32_t value) = 0;

    // Req joint quantity
    void ReqJointPositionRad(double value);
    void ReqJointSpeedRadPerSec(double value);
    void ReqJointTorqueNm(double value);
    virtual void ReqNoAction() = 0;
    virtual void ReqInitializationViaProcess() = 0;

    // Thread safe getters
    // Get joint quantity
    Data<double> GetQLatestRad() const;
    Data<double> GetDQLatestRad() const;
    Data<double> GetTauLatestNm() const;
    JointState GetLatestState() const;

    // Get motor quantity
    Data<int32_t> GetTicksLatest() const;
    Data<int32_t> GetRPMLatest() const;
    Data<int32_t> GetMALatest() const;
    Data<JointStatus> GetStatusLatest() const;

    virtual void CheckI2tAndTimeoutError(JointStatus status) = 0;

    void LogLatestState() const;

    typedef std::shared_ptr<YoubotJoint> Ptr;

  private:
    const int slaveIndex;
    const std::map<std::string, double> config;
    Parameters parameters;

  protected:
    std::atomic<Data<int32_t>> ticksLatest, RPMLatest, mALatest, UpwmLatest;
    std::atomic<Data<JointStatus>> statusLatest;
    EtherCATMaster::Ptr center;
  };
}
#endif
