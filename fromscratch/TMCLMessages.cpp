#include "TMCLMessages.hpp"

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

GetMotorControllerStatus::GetMotorControllerStatus(unsigned int slaveNumber) : TMCLRequest(slaveNumber) {
  _toModuleAddress = DRIVE;
  _toCommandNumber = GAP;
  _toTypeNumber = TMCLRequest::ERROR_STATUS_FLAG;
  _toMotorNumber = 0;
  SetValue(0);
}

std::string GetMotorControllerStatus::StatusErrorFlagsAsString() {
  return StatusErrorFlagsToString(GetReplyValue());
}