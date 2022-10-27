#ifndef ETHERCAT_MAILBOX_REQUEST
#define ETHERCAT_MAILBOX_REQUEST

#include <atomic>
#include <memory>
#include <string>

extern "C" {
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatmain.h"
}

class EthercatMailboxRequest {
public:
  EthercatMailboxRequest() = delete;

  EthercatMailboxRequest(EthercatMailboxRequest&) = delete;

  EthercatMailboxRequest(const EthercatMailboxRequest&) = delete;

  bool TryToSend(unsigned int mailboxTimeoutUS,
    unsigned char numOfRetrieves, unsigned int sleepBeforeRecUS);

  bool SendToSlave(unsigned int mailboxTimeout);

  bool ReceiveFromSlave(unsigned int mailboxTimeout);

  EthercatMailboxRequest(unsigned int slaveIndex);

  bool IsSendSuccessful() const;

  bool IsReceiveSuccessful() const;

  typedef std::shared_ptr<EthercatMailboxRequest> EthercatRequestPtr;

  std::string StatusToString() const;

private:
  enum class Status {
    INITIALIZED = 0,
    SENT_SUCCESSFUL = 2,
    RECEIVED_SUCCESSFUL = 4
  };

  volatile std::atomic<Status> status;

  const unsigned int slaveIndex;

  //TODO: mutex - under process

protected:
  ec_mbxbuft mailboxToSlave; // initialize from subclasses before send
  ec_mbxbuft mailboxFromSlave; // can be read from subclasses after successful receive
};

#endif
