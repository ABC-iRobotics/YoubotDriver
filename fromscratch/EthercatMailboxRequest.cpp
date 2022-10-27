#include "EthercatMailboxRequest.hpp"
#include <Exceptions.hpp>
#include "Time.hpp"

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

bool EthercatMailboxRequest::TryToSend(unsigned int mailboxTimeoutUS,
  unsigned char numOfRetrieves, unsigned int sleepBeforeRecUS) {
  std::lock_guard<mutex> lock(under_process);
  bool now = false;
  if (status < Status::SENT_SUCCESSFUL) {
    if (ec_mbxsend(slaveIndex, &mailboxToSlave, mailboxTimeoutUS) > 0) {
      status = Status::SENT_SUCCESSFUL;
      now = true;
    }
    else
      return false;
  }
  if (status == Status::SENT_SUCCESSFUL) {
    if (sleepBeforeRecUS>0 && now)
      SLEEP_MICROSEC(sleepBeforeRecUS);
    for (unsigned int i = 0; i < numOfRetrieves; i++) {
      if (ec_mbxreceive(slaveIndex, &mailboxFromSlave, mailboxTimeoutUS) > 0) {
        status = Status::RECEIVED_SUCCESSFUL;
        return true;
      }
    }
    return false;
  }
  return false;
}

EthercatMailboxRequest::EthercatMailboxRequest(unsigned int slaveIndex) : status(Status::INITIALIZED), slaveIndex(slaveIndex) {}

bool EthercatMailboxRequest::IsSendSuccessful() const {
  return status >= Status::SENT_SUCCESSFUL;
}

bool EthercatMailboxRequest::IsReceiveSuccessful() const {
  return status >= Status::RECEIVED_SUCCESSFUL;
}

std::string EthercatMailboxRequest::StatusToString() const {
  if (status == Status::INITIALIZED)
    return "INITIALIZED";
  if (status == Status::SENT_SUCCESSFUL)
    return "SENT_SUCCESSFUL";
  if (status == Status::RECEIVED_SUCCESSFUL)
    return "RECEIVED_SUCCESSFUL";
  return "NO_EXPECTED";
}
