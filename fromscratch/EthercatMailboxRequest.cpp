#include "EthercatMailboxRequest.hpp"
#include <Exceptions.hpp>
#include <sstream>

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

bool TryToSend(unsigned int mailboxTimeoutUS,
  unsigned char numOfRetrieves, unsigned int sleepBeforeRecUS) {
  


}


bool EthercatMailboxRequest::SendToSlave(unsigned int mailboxTimeout) {
  if (status >= Status::SENT)
    throw std::runtime_error("EthercatMailboxRequest::SendToSlave : Multiple send commands detected!");
  if (ec_mbxsend(slaveIndex, &mailboxToSlave, mailboxTimeout) > 0) {
    status = Status::SENT;
    return true;
  }
  status = Status::FAILED_SEND;
  return false;
}

bool EthercatMailboxRequest::ReceiveFromSlave(unsigned int mailboxTimeout) {
  if (status < Status::SENT || status >= Status::RECEIVED)
    throw std::runtime_error("EthercatMailboxRequest::ReceiveFromSlave : Receive before successful send detected!"); 
  if (ec_mbxreceive(slaveIndex, &mailboxFromSlave, mailboxTimeout) > 0) {
    status = Status::RECEIVED;
    return true;
  }
  status = Status::FALED_RECEIVE;
  return false;
}

EthercatMailboxRequest::EthercatMailboxRequest(unsigned int slaveIndex) : status(Status::INITIALIZED), slaveIndex(slaveIndex) {}

bool EthercatMailboxRequest::IsSendSuccessful() const {
  return status >= Status::SENT;
}

bool EthercatMailboxRequest::IsReceiveSuccessful() const {
  return status >= Status::RECEIVED;
}

std::string EthercatMailboxRequest::StatusToString() const {
  std::stringstream ss;
  if (status == Status::INITIALIZED)
    ss << " INITIALIZED";
  if (status == Status::FAILED_SEND)
    ss << " FAILED_SEND";
  if (status == Status::SENT)
    ss << " SENT";
  if (status == Status::FALED_RECEIVE)
    ss << " FALED_RECEIVE";
  if (status == Status::RECEIVED)
    ss << " RECEIVED";
  return ss.str();
}
