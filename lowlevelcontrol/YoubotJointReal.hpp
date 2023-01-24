#ifndef YOUBOT_JOINT_REAL_HPP
#define YOUBOT_JOINT_REAL_HPP

#include "EtherCATMaster.hpp"
#include "YoubotJointAbstract.hpp"
#include <map>

namespace youbot {

  class YoubotJointReal : public YoubotJointAbstract {
  public:
    // Deleted constructor
    YoubotJointReal() = delete;
    YoubotJointReal(YoubotJointReal&) = delete;
    YoubotJointReal(const YoubotJointReal&) = delete;

    YoubotJointReal(int slaveIndex, const std::map<std::string, double>& config, EtherCATMaster* center);

    void ConfigParameters(bool forceConfiguration = false) override;
    bool CheckConfig() override;
    void InitCommutation() override;

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
    bool IsInitializedViaMailbox() override;
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

    // Process message-based req/get methods will be sent with the next ExcangeMessage/show the results of the latest one
    const ProcessReturn& GetProcessReturnData() override;
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
    EtherCATMaster* center;
  };
}
#endif
