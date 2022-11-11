#ifndef TMCL_MAILBOX_MESSAGE_HPP
#define TMCL_MAILBOX_MESSAGE_HPP

#include "MailboxMessage.hpp"
#include "TMCLDefinitions.hpp"


class GetFirmware : public MailboxMessage<8, 8> {
public:
  GetFirmware(int slaveIndex);

  void GetOutput(int& controllernum, int& firmwarenum) const;

  static std::shared_ptr<GetFirmware> InitSharedPtr(int slaveIndex);
};

template <typename ValueType, TMCL::Module module_, TMCL::Cmd cmd,
  TMCL::AxisParam param= TMCL::AxisParam(0), uint32_t defaultValue=0,
  TMCL::MotorBank bankNumber= TMCL::MotorBank(0)>
class TMCLTemplate : public MailboxMessage<8, 8> {
  uint8_t& _toModuleAddress = toSlaveBuff[0];
  uint8_t& _toCommandNumber = toSlaveBuff[1];
  uint8_t& _toTypeNumber = toSlaveBuff[2];
  uint8_t& _toMotorNumber = toSlaveBuff[3];

  uint8_t& _fromReplyAddress = fromSlaveBuff[0];
  uint8_t& _fromModuleAddress = fromSlaveBuff[1];
  uint8_t& _fromStatus = fromSlaveBuff[2];
  uint8_t& _fromCommandNumber = fromSlaveBuff[3];

  void SetValue(const ValueType& value) {
    toSlaveBuff[4] = (uint32_t)value >> 24;
    toSlaveBuff[5] = (uint32_t)value >> 16;
    toSlaveBuff[6] = (uint32_t)value >> 8;
    toSlaveBuff[7] = (uint32_t)value & 0xff;
  };

public:
  TMCLTemplate(unsigned int slaveIndex) : MailboxMessage(slaveIndex) {
    _fromStatus = 0;
    _toModuleAddress = (uint8_t)module_;
    _toCommandNumber = (uint8_t)cmd;
    _toTypeNumber = (uint8_t)param;
    _toMotorNumber = (uint8_t)bankNumber;
    toSlaveBuff[4] = defaultValue >> 24;
    toSlaveBuff[5] = defaultValue >> 16;
    toSlaveBuff[6] = defaultValue >> 8;
    toSlaveBuff[7] = defaultValue & 0xff;
  }

  TMCLTemplate(unsigned int slaveIndex, ValueType value) : MailboxMessage(slaveIndex) {
    _fromStatus = 0;
    _toModuleAddress = (uint8_t)module_;
    _toCommandNumber = (uint8_t)cmd;
    _toTypeNumber = (uint8_t)param;
    _toMotorNumber = (uint8_t)bankNumber;
    SetValue(value);
  }

  ValueType GetReplyValue() const {
    return ValueType (fromSlaveBuff[4] << 24 | fromSlaveBuff[5] << 16 | fromSlaveBuff[6] << 8 | fromSlaveBuff[7]);
  }

  TMCL::ReplyStatus GetRecStatusFlag() const {
    return (TMCL::ReplyStatus)_fromStatus;
  }

  std::string  RecvStatusAsString() const {
    printf("Ret with: address %d, moduleAdress %d, status %d, commandNumber %d, value %d\n",
      _fromReplyAddress, _fromModuleAddress, _fromStatus, _fromCommandNumber, GetReplyValue());
    return TMCL::RecvStatusToString(_fromStatus);
  }

  static std::shared_ptr<TMCLTemplate> InitSharedPtr(int slaveIndex) {
    return std::make_shared<TMCLTemplate>(slaveIndex);
  }

  static std::shared_ptr<TMCLTemplate> InitSharedPtr(int slaveIndex, ValueType value) {
    return std::make_shared<TMCLTemplate>(slaveIndex, value);
  }
};

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::ACTUAL_POSITION> GetPosition;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::ACTUAL_POSITION, 10000> SetEncoder;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE,
  TMCL::Cmd::GAP, TMCL::AxisParam::ERROR_STATUS_FLAG> GetErrorStatusFlag;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::ENCODER_STEPS_PER_ROTATION> GetEncoderStepsPerRotation;

typedef TMCLTemplate<bool, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::ENCODER_DIRECTION> GetEncoderDirection; // 0/1

typedef TMCLTemplate<bool, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::ENCODER_DIRECTION, 1> SetEncoderDirection; // EEPROM_LOCKED!!

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::POSITION_PID_P2> GetP2ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::POSITION_PID_P2, 2000> SetP2ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::POSITION_PID_P1> GetP1ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::POSITION_PID_P1, 500> SetP1ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::POSITION_PID_I2> GetI2ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::POSITION_PID_I2, 0> SetI2ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::POSITION_PID_I1> GetI1ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::POSITION_PID_I1, 0> SetI1ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::POSITION_PID_D2> GetD2ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::POSITION_PID_D2, 0> SetD2ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::POSITION_PID_D1> GetD1ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::POSITION_PID_D1, 0> SetD1ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::VELOCITY_PID_P2> GetP2ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::VELOCITY_PID_P2, 1000> SetP2ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::VELOCITY_PID_P1> GetP1ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::VELOCITY_PID_P1, 200> SetP1ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::VELOCITY_PID_I2> GetI2ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::VELOCITY_PID_I2, 1000> SetI2ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::VELOCITY_PID_I1> GetI1ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::VELOCITY_PID_I1, 200> SetI1ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::VELOCITY_PID_D2> GetD2ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::VELOCITY_PID_D2, 0> SetD2ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::VELOCITY_PID_D1> GetD1ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::VELOCITY_PID_D1, 0> SetD1ParameterVelocityControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::CURRENT_PID_P2> GetP2ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::CURRENT_PID_P2, 25> SetP2ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::CURRENT_PID_P1> GetP1ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::CURRENT_PID_P1, 50> SetP1ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::CURRENT_PID_I2> GetI2ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::CURRENT_PID_I2, 60> SetI2ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::CURRENT_PID_I1> GetI1ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::CURRENT_PID_I1, 50> SetI1ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::CURRENT_PID_D2> GetD2ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::CURRENT_PID_D2, 0> SetD2ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::CURRENT_PID_D1> GetD1ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::CURRENT_PID_D1, 0> SetD1ParameterCurrentControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::ACTUAL_MOTOR_CURRENT> GetCurrent;//[mA]

double ConvertTemperature(uint32_t adc);

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::ACTUAL_TEMPERATURE> GetTemperature; // analog-digital-converter value

typedef TMCLTemplate<bool, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::CLEAR_MOTOR_CONTROLLER_TIMEOUT_FLAG, 1> ClearMotorControllerTimeoutFlag;

typedef TMCLTemplate<int32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::TARGET_SPEED> GetTargetSpeed;

typedef TMCLTemplate<int32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::MOTOR_HALTED_VELOCITY> GetMotorHaltedVelocity;

typedef TMCLTemplate<int32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::ACTUAL_SPEED> GetActualSpeed;

typedef TMCLTemplate<int32_t, TMCL::Module::DRIVE, TMCL::Cmd::ROL,
  TMCL::AxisParam(0), 10> RotateLeft; // does change to velocity mode and set target speed

typedef TMCLTemplate<int32_t, TMCL::Module::DRIVE, TMCL::Cmd::ROR,
  TMCL::AxisParam(0), 10> RotateRight; // does change to velocity mode and set target speed

typedef TMCLTemplate<bool, TMCL::Module::DRIVE, TMCL::Cmd::MST> MotorStop;

typedef TMCLTemplate<bool, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::INITIALIZE,1> SetInitialize; // only command - no values

// Setting a user variables, that will allow to check if the motor is already calibrated
typedef TMCLTemplate<bool, TMCL::Module::DRIVE, TMCL::Cmd::SGP,
  TMCL::AxisParam(TMCL::UserVariables::NEED_CALIBRATION),
  0, TMCL::MotorBank::USER_VARIABLES> SetIsCalibrated; // by default it is one, after calibration can be set to zero

typedef TMCLTemplate<bool, TMCL::Module::DRIVE, TMCL::Cmd::GGP,
  TMCL::AxisParam(TMCL::UserVariables::NEED_CALIBRATION),
  0, TMCL::MotorBank::USER_VARIABLES> GetNeedCalibration;

typedef TMCLTemplate<bool, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::CLEAR_I2T_FLAG> ClearI2TFlag; // only command - no values

typedef TMCLTemplate<bool, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::INITIALIZE> GetInitialized; // the set does need a few second - and movement

#endif
