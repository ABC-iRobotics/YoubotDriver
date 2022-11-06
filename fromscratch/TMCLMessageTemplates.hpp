#ifndef TMCL_MESSAGE_TEMPLATES
#define TMCL_MESSAGE_TEMPLATES

#include "EthercatMailboxRequest.hpp"
#include <string>
#include <sstream>

class TMCLRequest : public EthercatMailboxRequest {
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
    CLE = 36, //ClearErrorFlags
    FIRMWARE_VERSION = 136
  };

  enum AxisParameter : uint8 {
    TARGET_POSITION = 0,
    ACTUAL_POSITION = 1, // ok
    TARGET_SPEED = 2,
    ACTUAL_SPEED = 3,
    MAX_CURRENT = 4,
    INITIALIZE = 15,
    POSITION_PID_P1 = 130,
    POSITION_PID_I1 = 131,
    POSITION_PID_D1 = 132,
    POSITION_PID_I_CLIPPING1 = 135,
    VELOCITY_PID_P1 = 140,
    VELOCITY_PID_I1 = 141,
    VELOCITY_PID_D1 = 142,
    VELOCITY_PID_I_CLIPPING1 = 143,
    ACTUAL_MOTOR_CURRENT = 150,
    ACTUAL_VOLTAGE = 151, // in 0.01V
    ACTUAL_TEMPERATURE = 152,
    ERROR_STATUS_FLAG = 156,
    CLEAR_MOTOR_CONTROLLER_TIMEOUT_FLAG = 158, // only in the drive..
    COMMUTATION_MODE = 159,
    CURRENT_PID_P1 = 168,
    CURRENT_PID_I1 = 169,
    CURRENT_PID_D1 = 170,
    CURRENT_PID_I_CLIPPING1 = 171,
    CURRENT_PID_P2 = 172,
    CURRENT_PID_I2 = 173,
    CURRENT_PID_D2 = 174,
    CURRENT_PID_I_CLIPPING2 = 175,
    POSITION_PID_P2 = 230,
    POSITION_PID_I2 = 231,
    POSITION_PID_D2 = 232,
    POSITION_PID_I_CLIPPING2 = 233,
    VELOCITY_PID_P2 = 234,
    VELOCITY_PID_I2 = 235,
    VELOCITY_PID_D2 = 236,
    VELOCITY_PID_I_CLIPPING2 = 237,
    ENCODER_STEPS_PER_ROTATION = 250,
    ENCODER_DIRECTION = 251
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

  static std::string RecvStatusToString(uint8 in);

  enum StatusErrorFlags : uint32 {
    OVER_CURRENT = 0x1,
    UNDER_VOLTAGE = 0x2,
    OVER_VOLTAGE = 0x4,
    OVER_TEMPERATURE = 0x8,
    MOTOR_HALTED = 0x10,
    HALL_SENSOR_ERROR = 0x20,
        ENCODER_ERROR = 0x40,
        INITIALIZATION_ERROR = 0x80,
    PWM_MODE_ACTIVE = 0x100,
    VELOCITY_MODE_ACTIVE = 0x200,
    POSITION_MODE_ACTIVE = 0x400,
    TORQUE_MODE_ACTIVE = 0x800,
        EMERGENCY_STOP = 0x1000,
        FREERUNNING = 0x2000,
    POSITION_REACHED = 0x4000,
    INITIALIZED = 0x8000,
    TIMEOUT = 0x10000,
    I2T_EXCEEDED = 0x20000
  };

  static std::string StatusErrorFlagsToString(uint32 in);
public:
  std::string  RecvStatusAsString() const {
    printf("Ret with: address %d, moduleAdress %d, status %d, commandNumber %d, value %d\n",
      _fromReplyAddress, _fromModuleAddress, _fromStatus, _fromCommandNumber, GetReplyValue());
    return RecvStatusToString(_fromStatus);
  }
protected:
  TMCLRequest(unsigned int slaveIndex) : EthercatMailboxRequest(slaveIndex) {
    _fromStatus = 0;
  }

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

  bool GetRecStatusFlag(uint8& status_) const;
};

template<uint8 parameter>
class TMCL_SetAxisParam : public TMCLRequest {
public:
  TMCL_SetAxisParam(int slaveNumber, uint32 value) : TMCLRequest(slaveNumber) {
    _toModuleAddress = DRIVE;
    _toCommandNumber = SAP;
    _toTypeNumber = parameter;
    _toMotorNumber = 0;
    SetValue(value);
  }

  using TMCLRequest::GetRecStatusFlag;
};

template<uint8 command>
class TMCL_CmdWithValue : public TMCLRequest {
public:
  TMCL_CmdWithValue(unsigned int slaveNumber, int32 value) : TMCLRequest(slaveNumber) {
    _toModuleAddress = DRIVE;
    _toCommandNumber = command;
    _toTypeNumber = 0;
    _toMotorNumber = 0;
    SetValue(value);
  }

  using TMCLRequest::GetRecStatusFlag;
};
template<uint8 command >
class TMCL_OnlyCommand : public TMCLRequest {
public:
  TMCL_OnlyCommand(unsigned int slaveNumber) : TMCLRequest(slaveNumber) {
    _toModuleAddress = DRIVE;
    _toCommandNumber = command;
    _toTypeNumber = 0;
    _toMotorNumber = 0;
    SetValue(0);
  }

  using TMCLRequest::GetRecStatusFlag;
};

typedef TMCL_OnlyCommand<TMCLRequest::MST> MotorStop;

template<uint8 paramtype>
class TMCL_GetAxisParam : public TMCLRequest {
public:
  TMCL_GetAxisParam(unsigned int slaveNumber) : TMCLRequest(slaveNumber) {
    _toModuleAddress = DRIVE;
    _toCommandNumber = GAP;
    _toTypeNumber = paramtype;
    _toMotorNumber = 0;
    SetValue(0);
  }

  uint32 GetValue() const {
    return (uint32)GetReplyValue();
  }

  using TMCLRequest::GetRecStatusFlag;
};

#endif
