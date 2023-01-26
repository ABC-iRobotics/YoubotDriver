#ifndef YOUBOT_JOINT_VIRTUAL_HPP
#define YOUBOT_JOINT_VIRTUAL_HPP

#include "EtherCATMaster.hpp"
#include "YoubotJoint.hpp"
#include <map>
#include <chrono>

namespace youbot {

  class YoubotJointVirtual : public YoubotJoint {
    struct ProcessCommand {
      enum {
        POSITION,
        VELOCITY,
        CURRENT,
        PWM,
        ENCODER_ZERO,
        INITIALIZE,
        NO_MORE_ACTION
      } type;
      int32_t value;
    } processCommand;

    ProcessReturn processReturnAfterExchange; //TODO fill after exchange

    struct CommutationState {
      bool initialized = false;
      bool started = false;
      std::chrono::steady_clock::time_point started_at;
      void Update() {
        if (!initialized && started) {
          if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - started_at).count() > 500)
            initialized = true;
        }
      }
    } commutationState;
    
    bool configurated = false; // Set by configuration
    bool configurated_flag = false; // Set via mailbox message
    bool calibrated_flag = false; // Set via mailbox message
    bool I2terror = false;
    bool timeout = true;
    enum ControlMode {
      POSITION,
      VELOCITY,
      CURRENT,
      PWM
    } controlMode = VELOCITY;

    int32_t target = 0;

    JointStatus _getStatus();

    int64_t ticks = 0;
    int32_t RPM = 0;
    int32_t current_mA = 0;
    double qRad; // todo: random.
    std::chrono::steady_clock::time_point updated_at = std::chrono::steady_clock::now();

    void _updateFor(double elapsedTime) {
      if (timeout || I2terror) { // forcestop
        RPM = 0;
        current_mA = 0;
      }
      else {
        switch (controlMode) {
        case CURRENT:
          // torque
          // solve math
          break;
        case VELOCITY:
          // torque
          // solve math
          break;
        case POSITION:
          // torque
          // solve math
          break;
        }
      }
    }

    void _update() {
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

    void _calledAtExchange() {
      _update();
      // Update return values

      // Update command
    }

  public:
    // Deleted constructor
    YoubotJointVirtual() = delete;
    YoubotJointVirtual(YoubotJointVirtual&) = delete;
    YoubotJointVirtual(const YoubotJointVirtual&) = delete;

    YoubotJointVirtual(int slaveIndex,
      const std::map<std::string, double>& config, EtherCATMaster::Ptr center);

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
  };
}
#endif
