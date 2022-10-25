#ifndef TMCL_MESSAGES
#define TMCL_MESSAGES

#include <atomic>
#include <memory>

extern "C" {
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatmain.h"
}

class EthercatRequest {
public:
  bool SendToSlave(unsigned int mailboxTimeout);

  bool ReceiveFromSlave(unsigned int mailboxTimeout);

  EthercatRequest(unsigned int slaveIndex);

  bool IsSendSuccessful() const;

  bool IsReceiveSuccessful() const;

  typedef std::shared_ptr<EthercatRequest> EthercatRequestPtr;

private:
  enum class Status {
    INITIALIZED = 0,
    FAILED_SEND = 1,
    SENT = 2,
    FALED_RECEIVE = 3,
    RECEIVED = 4
  };

  volatile std::atomic<Status> status;

  const unsigned int slaveIndex;

protected:
  ec_mbxbuft mailboxToSlave; // initialize from subclasses before send
  ec_mbxbuft mailboxFromSlave; // can be read from subclasses after successful receive
};

class TMCLRequest : public EthercatRequest {
public:
  enum TMCL_Module : uint8 {
    DRIVE = 0,
    GRIPPER = 1
  };

  enum TMCL_CommandNumber : uint8 {
    ROR = 1,  //Rotate right
    ROL = 2,  //Rotate left
    MST = 3,  //Motor stop
    MVP = 4,  //Move to position
    SAP = 5,  //Set axis parameter
    GAP = 6,  //Get axis parameter
    STAP = 7, //Store axis parameter into EEPROM
    RSAP = 8, //Restore axis parameter from EEPROM
    SGP = 9,  //Set global parameter
    GGP = 10, //Get global parameter
    STGP = 11, //Store global parameter into EEPROM
    RSGP = 12, //Restore global parameter from EEPROM
    RFS = 13,
    SIO = 14,
    GIO = 15,
    SCO = 30,
    GCO = 31,
    CCO = 32,
    FIRMWARE_VERSION = 136
  };

  enum TMCL_Status : uint8 {
    NO_ERROR_ = 100,
    INVALID_COMMAND = 2,
    WRONG_TYPE = 3,
    INVALID_VALUE = 4,
    CONFIGURATION_EEPROM_LOCKED = 5,
    COMMAND_NOT_AVAILABLE = 6,
    REPLY_WRITE_PROTECTED = 8
  };
protected:
  TMCLRequest(unsigned int slaveIndex) : EthercatRequest(slaveIndex) {}

  uint8& _toModuleAddress = mailboxToSlave[0];
  uint8& _toCommandNumber = mailboxToSlave[1];
  uint8& _toTypeNumber = mailboxToSlave[2];
  uint8& _toMotorNumber = mailboxToSlave[3];

  void SetValue(const uint32& value);

  uint8& _fromReplyAddress = mailboxFromSlave[0];
  uint8& _fromModuleAddress = mailboxFromSlave[1];
  uint8& _fromStatus = mailboxFromSlave[2];
  uint8& _fromCommandNumber = mailboxFromSlave[3];

  uint32 GetReplyValue() const;
};

class FirmWareRequest : public TMCLRequest {
public:
  FirmWareRequest(unsigned int slaveNumber);

  void GetOutput(long& controllernum, long& firmwareversion) const;
};

template<uint8 module, uint8 command >
class TMCL_0_0 : public TMCLRequest {
public:
  TMCL_0_0(unsigned int slaveNumber);

  bool IsOK(uint8& status_) const;
};

typedef TMCL_0_0<TMCLRequest::DRIVE, TMCLRequest::MST> MotorStop;

template<uint8 module, uint8 paramtype>
class TMCL_0_1 : public TMCLRequest {
public:
  TMCL_0_1(unsigned int slaveNumber);

  bool IsOK(uint8& status_) const;

  int32 GetValue() const {
    return GetReplyValue();
  }
};

typedef TMCL_0_1<TMCLRequest::DRIVE, 130> PParameterFirstParametersPositionControl;

typedef TMCL_0_1<TMCLRequest::DRIVE, 1> GetPosition;


//typedef TMCL_0_1<130, TMCLRequest::DRIVE> PParameterFirstParametersPositionControl;

template<uint8 command, uint8 module>
inline TMCL_0_0<command, module>::TMCL_0_0(unsigned int slaveNumber) : TMCLRequest(slaveNumber) {
  _toModuleAddress = module;
  _toCommandNumber = command;
  _toTypeNumber = 0;
  _toMotorNumber = 0;
  SetValue(0);
}

template<uint8 command, uint8 module>
inline bool TMCL_0_0<command, module>::IsOK(uint8& status_) const {
  if (!IsReceiveSuccessful())
    throw std::runtime_error("");
  status_ = _fromStatus;
  return status_ == NO_ERROR_;
}


template<uint8 module, uint8 paramtype>
inline TMCL_0_1<module, paramtype>::TMCL_0_1(unsigned int slaveNumber)
  : TMCLRequest(slaveNumber) {
  _toModuleAddress = module;
  _toCommandNumber = GAP;
  _toTypeNumber = paramtype;
  _toMotorNumber = 0;
  SetValue(0);
}

template<uint8 module, uint8 paramtype>
inline bool TMCL_0_1<module, paramtype>::IsOK(uint8& status_) const {
  if (!IsReceiveSuccessful())
    throw std::runtime_error("");
  status_ = _fromStatus;
  return status_ == NO_ERROR_;
}



#endif
