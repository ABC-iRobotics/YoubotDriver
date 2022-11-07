#ifndef TMCL_MAILBOX_MESSAGE_HPP
#define TMCL_MAILBOX_MESSAGE_HPP

#include "MailboxMessage.hpp"
#include "TMCLDefinitions.hpp"


class GetFirmware : public MailboxMessage<8, 8> {
public:
  GetFirmware(int slaveIndex);

  void GetOutput(long& controllernum, long& firmwarenum) const;

  static std::shared_ptr<GetFirmware> InitSharedPtr(int slaveIndex);
};

template <typename ValueType, TMCL::Module module_, TMCL::Cmd cmd,
  TMCL::AxisParam param, uint32_t defaultValue>
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
    toSlaveBuff[4] = value >> 24;
    toSlaveBuff[5] = value >> 16;
    toSlaveBuff[6] = value >> 8;
    toSlaveBuff[7] = value & 0xff;
  };

public:
  TMCLTemplate(unsigned int slaveIndex) : MailboxMessage(slaveIndex) {
    _fromStatus = 0;
    _toModuleAddress = (uint8_t)module_;
    _toCommandNumber = (uint8_t)cmd;
    _toTypeNumber = (uint8_t)param;
    _toMotorNumber = 0;
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
    _toMotorNumber = 0;
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
  TMCL::AxisParam::ACTUAL_POSITION,0> GetPosition;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::ACTUAL_POSITION, 10000> SetEncoder;

typedef TMCLTemplate<TMCL::StatusErrorFlags, TMCL::Module::DRIVE,
  TMCL::Cmd::GAP, TMCL::AxisParam::ERROR_STATUS_FLAG, 0> GetErrorStatusFlag;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::ENCODER_STEPS_PER_ROTATION, 0> GetEncoderStepsPerRotation;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::ENCODER_DIRECTION, 0> GetEncoderDirection; // 0/1

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::ENCODER_DIRECTION, 1> SetEncoderDirection; // EEPROM_LOCKED!!

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::POSITION_PID_P2, 0> GetP2ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::POSITION_PID_P2, 2000> SetP2ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::POSITION_PID_P1, 0> GetP1ParameterPositionControl;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP,
  TMCL::AxisParam::POSITION_PID_P1, 500> SetP1ParameterPositionControl;


typedef TMCLTemplate<int32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::ACTUAL_MOTOR_CURRENT, 0> GetCurrent;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP,
  TMCL::AxisParam::MAX_CURRENT, 0> GetMaxCurrent;

#endif
