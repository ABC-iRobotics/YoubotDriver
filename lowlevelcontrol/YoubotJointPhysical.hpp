#ifndef YOUBOT_JOINT_REAL_HPP
#define YOUBOT_JOINT_REAL_HPP

#include "EtherCATMaster.hpp"
#include "YoubotJoint.hpp"
#include <map>

namespace youbot {

  class YoubotJointPhysical : public YoubotJoint {
  public:
    // Deleted constructor
    YoubotJointPhysical() = delete;
    YoubotJointPhysical(YoubotJointPhysical&) = delete;
    YoubotJointPhysical(const YoubotJointPhysical&) = delete;

    YoubotJointPhysical(int slaveIndex, const std::map<std::string, double>& config,
      EtherCATMaster::Ptr center);

    void Init();

    void ConfigControlParameters(bool forceConfiguration = false) override;
    bool CheckControlParameters() override;

    // Mailbox-based Set/Get mothods
    bool IsConfiguratedViaMailbox() override;
    void SetConfiguratedViaMailbox() override;
    void RotateJointRightViaMailbox(double speedJointRadPerSec) override;
    void RotateJointLeftViaMailbox(double speedJointRadPerSec) override;
    void StopViaMailbox() override;
    JointStatus GetJointStatusViaMailbox() override;
    void ResetTimeoutViaMailbox() override;
    void ResetI2TExceededViaMailbox() override;
    void StartInitializationViaMailbox() override;
    double GetThermalWindingTimeSecViaMailbox() override;
    double GetCurrentAViaMailbox() override;
    void RotateMotorRightViaMailbox(int32_t speedMotorRPM) override;
    void RotateMotorLeftViaMailbox(int32_t speedMotorRPM) override;
    void SetTargetCurrentAViaMailbox(double current) override;
    bool IsCalibratedViaMailbox() override;
    void SetCalibratedViaMailbox() override;
    long GetI2tLimitValueViaMailbox() override;
    long GetCurrentI2tValueViaMailbox() override;
    double GetJointVelocityRadPerSecViaMailbox() override;
    void SetJointVelocityRadPerSecViaMailbox(double value) override;
    void GetFirmwareVersionViaMailbox(int& controllernum,
      int& firmwareversion) override;
    unsigned int GetEncoderResolutionViaMailbox() override;
    std::string GetCommutationModeViaMailbox() override;

    // Process message-based req/get methods will be sent with the next ExcangeMessage/show the results of the latest one
    void ReqStop() override;
    void ReqNoAction() override;

    // Req motor quantity
    void ReqMotorSpeedRPM(int32_t value) override;
    void ReqEncoderReference(int32_t value) override;
    void ReqMotorPositionTick(int32_t value) override;
    void ReqVoltagePWM(int32_t value) override;
    void ReqMotorCurrentmA(int32_t value) override;

    // Get motor quantity
    void ReqInitializationViaProcess() override;
    void CheckI2tAndTimeoutError(JointStatus status) override;

    // Only for test purposes
    void I2tResetTest();

  private:
    void _loadExchangeDataFromBuffer(); // registered to be called at the end of EtherCATMaster::ExchangeProcessMsgs
  };
}
#endif
