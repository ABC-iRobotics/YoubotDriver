#ifndef JOINT_HPP
#define JOINT_HPP

#include <map>
#include <string>
#include "EtherCATMaster.hpp"
#include "JointState.hpp"

namespace youbot {

  /// <summary>
  /// Virtual class to describe the methods, parameters realted to the joints
  /// 
  /// Virtual because there is an implementation used for simulations
  /// 
  /// (later with a virtual ethercat bus, that correctly simulates the motor drivers and dynamics the physical implementation is enough)
  /// </summary>
  class Joint {
  public:
    /// <summary>
    /// Parameters of the given joint read from the motor driver or the config file
    /// </summary>
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
    Joint() = delete;
    Joint(Joint&) = delete;
    Joint(const Joint&) = delete;

    /// <summary>
    /// Constructor of the virtual class
    /// </summary>
    /// <param name="slaveIndex"> slave ID (0..(N-1))</param>
    /// <param name="config"> the NameValueMap read from the config, contains motor parameters, kinematic, dynamic parameters </param>
    /// <param name="center"> pointer to the EtherCAT Master to be used for communication </param>
    Joint(int slaveIndex, const std::map<std::string, double>& config,
      EtherCATMaster::Ptr center);

    /// <summary>
    ///   Main initialization routine
    /// 
    /// Currently:
    ///  
    /// - read the most important values from the NameValueMap and the motor driver
    /// 
    /// - config the control parameters of the motor driver
    /// 
    /// - init the commutation of the motor 
    /// </summary>
    /// <param name="forceConfiguration"> true if must be configurated although it is already configurated (according to the given user-flag of the motor driver) </param>
    void InitializeJoint(bool forceConfiguration = false);

    // Submethods of initialization
    void CollectBasicParameters(); ///< get essential paramters from the config and the motor driver
    virtual void ConfigControlParameters(bool forceConfiguration = false) = 0; ///< set the control parameters to the driver
    virtual bool CheckControlParameters() = 0; ///< check the control parameters to the driver
    virtual void InitCommutation(); ///< set commutation/encoder of the driver

    // Variable getters
    const Parameters& GetParameters() const; ///< Return the const ref. of the struct of the parameters
    const std::map<std::string, double>& GetConfig() const; ///< Returns the config NameValueMap
    int GetSlaveIndex() const; ///< return the slave ID (0..(N-1))

    // Conversions
    double Ticks2qRad(int32_t motorticks) const; ///< Conversion for motor_ticks->joint_rad
    int32_t qRad2Ticks(double qDeg) const; ///< Conversion for joint_rad->motor_ticks
    double RPM2qRadPerSec(int32_t RPM) const; ///< Conversion for motor_RPM->joint_rad/s
    int32_t qRadPerSec2RPM(double degpersec) const; ///< Conversion for joint_rad/s->motor_RPM
    double mA2Nm(int32_t mA) const; ///< Conversion for motor_current_mA->joint_torque_NM
    int32_t Nm2mA(double Nm) const; ///< Conversion for joint_torque_NM->motor_current_mA

    // Mailbox-based Set/Get mothods
    virtual bool IsConfiguratedViaMailbox() = 0; ///< Mailbox: Check the is_configurated flag in the motor driver
    virtual void SetConfiguratedViaMailbox() = 0; ///< Mailbox: Set the is_configurated flag in the motor driver
    virtual void RotateJointRightViaMailbox(double speedJointRadPerSec) = 0; ///< Mailbox: Move with constant + speed in joint [rad/s]
    virtual void RotateJointLeftViaMailbox(double speedJointRadPerSec) = 0; ///< Mailbox: Move with constant - speed in joint [rad/s]
    virtual void StopViaMailbox() = 0; ///< Mailbox: Stop movement
    virtual JointStatus GetJointStatusViaMailbox() = 0; ///< Mailbox: return joint status flags
    virtual void ResetTimeoutViaMailbox() = 0; ///< Mailbox: reset the timeout flag
    virtual void ResetI2TExceededViaMailbox() = 0; ///< Mailbox: reset i2t flag (does need a rest of thermal winding time)
    virtual void StartInitializationViaMailbox() = 0; ///< Mailbox: start commutation initialization
    virtual double GetThermalWindingTimeSecViaMailbox() = 0; ///< Mailbox: get the length of thermal winding time [s]
    virtual double GetCurrentAViaMailbox() = 0; ///< Mailbox: get the current in [A]
    virtual void RotateMotorRightViaMailbox(int32_t speedMotorRPM) = 0; ///< Mailbox: Move with constant + speed in motor [RPM]
    virtual void RotateMotorLeftViaMailbox(int32_t speedMotorRPM) = 0; ///< Mailbox: Move with constant - speed in motor [RPM]
    virtual void SetTargetCurrentAViaMailbox(double current) = 0; ///< Mailbox: set motor target current
    virtual bool IsCalibratedViaMailbox() = 0; ///< Mailbox: Check the is_calibrated flag in the motor driver
    virtual void SetCalibratedViaMailbox() = 0; ///< Mailbox: set the is_calibrated flag in the motor driver
    virtual long GetI2tLimitValueViaMailbox() = 0; ///< Mailbox: get I2t limit value of the motor driver
    virtual long GetCurrentI2tValueViaMailbox() = 0; ///< Mailbox: get current i2t value
    virtual double GetJointVelocityRadPerSecViaMailbox() = 0; ///< Mailbox: get joint speed in rad/s
    virtual void SetJointVelocityRadPerSecViaMailbox(double value) = 0; ///< Mailbox: set target joint speed rad/s
    virtual void GetFirmwareVersionViaMailbox(int& controllernum,
      int& firmwareversion) = 0; ///< Mailbox: get controller and firmware version
    virtual unsigned int GetEncoderResolutionViaMailbox() = 0; ///< Mailbox: get encoder resolution (set in the driver)
    virtual std::string GetCommutationModeViaMailbox() = 0; ///< Mailbox: get commutation mode

    // Process message-based req/get methods will be sent with the next ExcangeMessage/show the results of the latest one
    virtual void ReqStop() = 0; ///< Process buffer: Set Stop command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)

    // Req motor quantity
    virtual void ReqMotorSpeedRPM(int32_t value) = 0; ///< Process buffer: Set motor speed command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
    virtual void ReqEncoderReference(int32_t value) = 0; ///< Process buffer: Set encoder zero command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
    virtual void ReqMotorPositionTick(int32_t value) = 0; ///< Process buffer: Set motor position command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
    virtual void ReqVoltagePWM(int32_t value) = 0; ///< Process buffer: Set motor voltage command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
    virtual void ReqMotorCurrentmA(int32_t value) = 0; ///< Process buffer: Set motor current command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
    void ReqMotorTorqueNm(double value); ///< Process buffer: Set motor current command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)

    // Req joint quantity
    void ReqJointPositionRad(double value); ///< Process buffer: Set joint position command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
    void ReqJointSpeedRadPerSec(double value); ///< Process buffer: Set joint speed command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
    void ReqJointTorqueNm(double value); ///< Process buffer: Set joint torque command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
    virtual void ReqNoAction() = 0; ///< Process buffer: Set no-action command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out)
    virtual void ReqInitializationViaProcess() = 0; ///< Process buffer: Set commutation initialization command into (EtherCATMaster::ExchangeProcessMsg is needed to sent out) (NOT TESTED)

    // Thread safe getters
    // Get joint quantity
    Data<double> GetQLatestRad() const; ///< Thread-safe get latest joint position, return the value from the latest mailbox or process message
    Data<double> GetDQLatestRad() const; ///< Thread-safe get latest joint speed, return the value from the latest mailbox or process message
    Data<double> GetTauLatestNm() const; ///< Thread-safe get latest joint torque, return the value from the latest mailbox or process message
    JointState GetLatestState() const; ///< Thread-safe get latest joint full-state, return the value from the latest mailbox or process message

    // Get motor quantity
    Data<int32_t> GetTicksLatest() const; ///< Thread-safe get latest motor position, return the value from the latest mailbox or process message
    Data<int32_t> GetRPMLatest() const; ///< Thread-safe get latest motor speed, return the value from the latest mailbox or process message
    Data<int32_t> GetMALatest() const; ///< Thread-safe get latest motor current, return the value from the latest mailbox or process message
    Data<JointStatus> GetStatusLatest() const; ///< Thread-safe get joint status, return the value from the latest mailbox or process message

    virtual void CheckI2tAndTimeoutError(JointStatus status) = 0; ///< Call error if the status contains timeout or I2t

    void LogLatestState() const; ///< Save the latest state into log

    typedef std::shared_ptr<Joint> Ptr;

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
