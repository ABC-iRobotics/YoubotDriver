#ifndef TMCL_MESSAGES
#define TMCL_MESSAGES

#include <atomic>
#include <memory>

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

class FirmWareRequest : public EthercatRequest  {
public:
  FirmWareRequest(unsigned int slaveNumber);

  void GetOutput(long& controllernum, long& firmwareversion) const;
};


#endif