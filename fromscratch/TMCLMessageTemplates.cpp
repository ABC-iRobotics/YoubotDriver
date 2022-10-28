#include "TMCLMessageTemplates.hpp"
#include <Exceptions.hpp>

#include <stdio.h>

#define USER_VARIABLE_BANK 2

//Opcodes of TMCL control functions (to be used to run or abort a TMCL program in the module)
#define TMCL_APPL_STOP 128
#define TMCL_APPL_RUN 129
#define TMCL_APPL_RESET 131

//Options for MVP commandds
#define MVP_ABS 0
#define MVP_REL 1
#define MVP_COORD 2

//Options for RFS command
#define RFS_START 0
#define RFS_STOP 1
#define RFS_STATUS 2

//Result codes for GetResult
#define TMCL_RESULT_OK 0
#define TMCL_RESULT_NOT_READY 1
#define TMCL_RESULT_CHECKSUM_ERROR 2

enum YouBotJointControllerMode {
  MOTOR_STOP = 0,
  POSITION_CONTROL = 1,
  VELOCITY_CONTROL = 2,
  NO_MORE_ACTION = 3,
  SET_POSITION_TO_REFERENCE = 4,
  CURRENT_MODE = 6,
  INITIALIZE = 7
};

enum ParameterType {
  MOTOR_CONTOLLER_PARAMETER,
  API_PARAMETER
};

enum GripperErrorFlags {
  STALL_GUARD_STATUS = 0x1,
  GRIPPER_OVER_TEMPERATURE = 0x2,
  PRE_WARNING_OVER_TEMPERATURE = 0x4,
  SHORT_TO_GROUND_A = 0x8,
  SHORT_TO_GROUND_B = 0x10,
  OPEN_LOAD_A = 0x20,
  OPEN_LOAD_B = 0x40,
  STAND_STILL = 0x80
};

void TMCLRequest::SetValue(const uint32& value) {
  mailboxToSlave[4] = value >> 24;
  mailboxToSlave[5] = value >> 16;
  mailboxToSlave[6] = value >> 8;
  mailboxToSlave[7] = value & 0xff;
};

bool TMCLRequest::GetRecStatusFlag(uint8& status_) const {
  if (!IsReceiveSuccessful())
    throw std::runtime_error("TMCLRequest::GetRecStatusFlag : haven't received anything");
  status_ = _fromStatus;
  return status_ == NO_ERROR_;
};

std::string TMCLRequest::RecvStatusToString(uint8 in) {
  std::stringstream ss;
  if (in == NO_ERROR_)
    ss << " NO_ERROR";
  if (in == INVALID_COMMAND)
    ss << " INVALID_COMMAND";
  if (in == WRONG_TYPE)
    ss << " WRONG_TYPE";
  if (in == INVALID_VALUE)
    ss << " INVALID_VALUE";
  if (in == CONFIGURATION_EEPROM_LOCKED)
    ss << " CONFIGURATION_EEPROM_LOCKED";
  if (in == COMMAND_NOT_AVAILABLE)
    ss << " COMMAND_NOT_AVAILABLE";
  if (in == REPLY_WRITE_PROTECTED)
    ss << " REPLY_WRITE_PROTECTED";
  return ss.str();
}

std::string TMCLRequest::StatusErrorFlagsToString(uint8 in) {
  std::stringstream ss;
  if (in & OVER_CURRENT)
    ss << " OVER_CURRENT";
  if (in & UNDER_VOLTAGE)
    ss << " UNDER_VOLTAGE";
  if (in & OVER_VOLTAGE)
    ss << " OVER_VOLTAGE";
  if (in & OVER_TEMPERATURE)
    ss << " OVER_TEMPERATURE";
  if (in & MOTOR_HALTED)
    ss << " MOTOR_HALTED";
  if (in & HALL_SENSOR_ERROR)
    ss << " HALL_SENSOR_ERROR";
  if (in & PWM_MODE_ACTIVE)
    ss << " PWM_MODE_ACTIVE";
  if (in & VELOCITY_MODE_ACTIVE)
    ss << " VELOCITY_MODE_ACTIVE";
  if (in & POSITION_MODE_ACTIVE)
    ss << " POSITION_MODE_ACTIVE";
  if (in & TORQUE_MODE_ACTIVE)
    ss << " TORQUE_MODE_ACTIVE";
  if (in & POSITION_REACHED)
    ss << " POSITION_REACHED";
  if (in & INITIALIZED)
    ss << " INITIALIZED";
  if (in & TIMEOUT)
    ss << " TIMEOUT";
  if (in & I2T_EXCEEDED)
    ss << " I2T_EXCEEDED";
  return ss.str();
}

uint32 TMCLRequest::GetReplyValue() const {
   return (mailboxFromSlave[4] << 24 | mailboxFromSlave[5] << 16 | mailboxFromSlave[6] << 8 | mailboxFromSlave[7]);
}