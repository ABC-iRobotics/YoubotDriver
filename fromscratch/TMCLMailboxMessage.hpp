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

template <typename ValueType, TMCL::Module module_, TMCL::Cmd cmd, TMCL::AxisParam param>
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
    return status_ == (TMCL::ReplyStatus)_fromStatus;
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

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::GAP, TMCL::AxisParam::ACTUAL_POSITION> GetPosition;

typedef TMCLTemplate<uint32_t, TMCL::Module::DRIVE, TMCL::Cmd::SAP, TMCL::AxisParam::ACTUAL_POSITION> SetEncoder;

typedef TMCLTemplate<TMCL::StatusErrorFlags, TMCL::Module::DRIVE, TMCL::Cmd::GAP, TMCL::AxisParam::ERROR_STATUS_FLAG> GetErrorStatusFlag;




#endif
