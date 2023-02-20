#ifndef SOEM_MESSAGE_CENTER_HPP
#define SOEM_MESSAGE_CENTER_HPP

#include <string>
#include <mutex>
#include <vector>

extern "C" {
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatmain.h"
}
#include "MailboxMessage.hpp"
#include "EtherCATMaster.hpp"

#include <iostream>

#define MIN(a,b) a<b?a:b

namespace youbot {
  namespace intrinsic {

    class SimpleOpenEtherCATMaster : public EtherCATMaster {
      // For mailbox messages
      std::mutex slaveBufferMutexes[50];
      struct MailboxBuffers {
        ec_mbxbuft fromSlave, toSlave;
      } * mailboxBuffers = NULL;

      // For process messages
      struct ProcessBuffers {
        ProcessBuffer toSlave, fromSlave;
        uint8_t fromMsgSize = 0;
      } * processBuffers = NULL;

      // Mutex for ethercat communications in general
      std::mutex ethercatComm;

      static bool exist; // to avoid multiple openings/initializations

      char IOmap_[4096] = { 0 }; // used by soem

      const std::string adapterName;

    public:
      SimpleOpenEtherCATMaster(const std::string& adapterName) :adapterName(adapterName) {};

      void Init();

      ~SimpleOpenEtherCATMaster();

      Type GetType() const override;

      // index: 0..(num-1)
      int getSlaveNum() const override;

      std::string getSlaveName(int cnt) const override;

      virtual MailboxStatus SendMailboxMessage(MailboxMessage::MailboxMessagePtr ptr) override;

      virtual void GetProcessMsg(ProcessBuffer& buff, uint8_t slaveNumber) const override;

      virtual void SetProcessFromSlaveSize(uint8_t size, uint8_t slaveNumber) override;

      virtual int SetProcessMsg(const ProcessBuffer& buffer, uint8_t slaveNumber) override;

      virtual void ExchangeProcessMsg() override;
    };
  }
}

#endif
