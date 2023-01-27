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
      };
      MailboxBuffers* mailboxBuffers = NULL;

      // For process messages
      struct ProcessBuffers {
        ProcessBuffer toSlave, fromSlave;
        uint8_t fromMsgSize = 0;
      };
      ProcessBuffers* processBuffers = NULL;

      // Mutex for ethercat communications in general
      std::mutex ethercatComm;

      static bool opened; // to avoid multiple openings/initializations

      char IOmap_[4096] = { 0 }; // used by soem

    public:
      SimpleOpenEtherCATMaster() {};

      ~SimpleOpenEtherCATMaster();

      Type GetType() const override;

      // index: 0..(num-1)
      int getSlaveNum() const override;

      std::string getSlaveName(int cnt) const override;

      virtual bool OpenConnection(const std::string& adapterName) override;

      virtual void CloseConnection() override;

      virtual MailboxStatus SendMessage_(MailboxMessage::MailboxMessagePtr ptr) override;

      virtual void GetProcessMsg(ProcessBuffer& buff, uint8_t slaveNumber) const override;

      virtual void SetProcessFromSlaveSize(uint8_t size, uint8_t slaveNumber) override;

      virtual int SetProcessMsg(const ProcessBuffer& buffer, uint8_t slaveNumber) override;

      virtual void ExchangeProcessMsg() override;

      virtual bool isOpened() const override;
    };
  }
}

#endif
