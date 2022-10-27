#ifndef ETHERCAT_MAILBOX_REQUEST
#define ETHERCAT_MAILBOX_REQUEST

#include <atomic>
#include <memory>
extern "C" {
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatmain.h"
}

class EthercatMailboxRequest {
public:
  bool SendToSlave(unsigned int mailboxTimeout);

  bool ReceiveFromSlave(unsigned int mailboxTimeout);

  EthercatMailboxRequest(unsigned int slaveIndex);

  bool IsSendSuccessful() const;

  bool IsReceiveSuccessful() const;

  typedef std::shared_ptr<EthercatMailboxRequest> EthercatRequestPtr;

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

#endif
