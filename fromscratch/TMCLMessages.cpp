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


#include "TMCLProtocolDefinitions.hpp"
#include <stdio.h>

using namespace youbot;

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

FirmWareRequest::FirmWareRequest(unsigned int slaveNumber) : EthercatRequest(slaveNumber) {
  mailboxToSlave[0] = DRIVE; //module
  mailboxToSlave[1] = FIRMWARE_VERSION; //command
  mailboxToSlave[2] = 0; //type number 0/234
  mailboxToSlave[3] = 0; //motor or bank number
  mailboxToSlave[4] = 0;
  mailboxToSlave[5] = 0;
  mailboxToSlave[6] = 0;
  mailboxToSlave[7] = 0;
}

void FirmWareRequest::GetOutput(long& controllernum, long& firmwarenum) const {
  if (!IsReceiveSuccessful())
    throw std::runtime_error("");
  char* ptr, * ptr2;
  controllernum = strtol((char*)mailboxFromSlave, &ptr, 10);
  firmwarenum = strtol(ptr + 1, &ptr2, 10);
}