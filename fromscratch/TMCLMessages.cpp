#include "TMCLMessages.hpp"
#include <Exceptions.hpp>

extern "C" {
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include "ethercatprint.h"
}

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

/*
enum ProcessDataErrorFlags {
    OVER_CURRENT = 1,
    UNDER_VOLTAGE = 2,
    OVER_VOLTAGE = 4,
    OVER_TEMPERATURE = 8,
    HALTED = 16,
    HALL_SENSOR = 32,
    ENCODER = 64,
    MOTOR_WINDING = 128,
    CYCLE_TIME_VIOLATION = 256,
    INIT_SIN_COMM = 512,
};
*/

enum MailboxErrorFlags {
  OVER_CURRENT = 0x1,
  UNDER_VOLTAGE = 0x2,
  OVER_VOLTAGE = 0x4,
  OVER_TEMPERATURE = 0x8,
  MOTOR_HALTED = 0x10,
  HALL_SENSOR_ERROR = 0x20,
  //    ENCODER_ERROR = 0x40,
  //    INITIALIZATION_ERROR = 0x80,
  //    PWM_MODE_ACTIVE = 0x100,
  VELOCITY_MODE = 0x200,
  POSITION_MODE = 0x400,
  TORQUE_MODE = 0x800,
  //    EMERGENCY_STOP = 0x1000,
  //    FREERUNNING = 0x2000,
  POSITION_REACHED = 0x4000,
  INITIALIZED = 0x8000,
  TIMEOUT = 0x10000,
  I2T_EXCEEDED = 0x20000

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

bool EthercatRequest::SendToSlave(unsigned int mailboxTimeout) {
  if (status >= Status::SENT)
    throw std::runtime_error(""); //TODO
  if (ec_mbxsend(slaveIndex, &mailboxToSlave, mailboxTimeout) > 0) {
    status = Status::SENT;
    return true;
  }
  status = Status::FAILED_SEND;
  return false;
}

bool EthercatRequest::ReceiveFromSlave(unsigned int mailboxTimeout) {
  if (status < Status::SENT || status >= Status::RECEIVED)
    throw std::runtime_error(""); //TODO
  if (ec_mbxreceive(slaveIndex, &mailboxFromSlave, mailboxTimeout) > 0) {
    status = Status::RECEIVED;
    return true;
  }
  status = Status::FALED_RECEIVE;
  return false;
}

EthercatRequest::EthercatRequest(unsigned int slaveIndex) : status(Status::INITIALIZED), slaveIndex(slaveIndex) {}

bool EthercatRequest::IsSendSuccessful() const {
  return status >= Status::SENT;
}

bool EthercatRequest::IsReceiveSuccessful() const {
  return status >= Status::RECEIVED;
}

void TMCLRequest::SetValue(const uint32& value) {
  mailboxToSlave[4] = value >> 24;
  mailboxToSlave[5] = value >> 16;
  mailboxToSlave[6] = value >> 8;
  mailboxToSlave[7] = value & 0xff;
}

uint32 TMCLRequest::GetReplyValue() const {
  return (mailboxFromSlave[4] << 24 | mailboxFromSlave[5] << 16 | mailboxFromSlave[6] << 8 | mailboxFromSlave[7]);
}

FirmWareRequest::FirmWareRequest(unsigned int slaveNumber) : TMCLRequest(slaveNumber) {
  _toModuleAddress = DRIVE;
  _toCommandNumber = FIRMWARE_VERSION;
  _toTypeNumber = 0;
  _toMotorNumber = 0;
  SetValue(0);
}

void FirmWareRequest::GetOutput(long& controllernum, long& firmwarenum) const {
  if (!IsReceiveSuccessful())
    throw std::runtime_error("");
  char* ptr, * ptr2;
  controllernum = strtol((char*)mailboxFromSlave, &ptr, 10);
  firmwarenum = strtol(ptr + 1, &ptr2, 10);
}