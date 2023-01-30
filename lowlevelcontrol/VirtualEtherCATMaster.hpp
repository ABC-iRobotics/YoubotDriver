#ifndef VIRTUAL_MESSAGE_CENTER_HPP
#define VIRTUAL_MESSAGE_CENTER_HPP

#include <string>
#include <mutex>
#include <vector>
#include "EtherCATMaster.hpp"

#include <iostream>

#define MIN(a,b) a<b?a:b

namespace youbot {
  namespace intrinsic {

    class VirtualEtherCATMaster : public EtherCATMaster {

    public:
      VirtualEtherCATMaster() {};

      ~VirtualEtherCATMaster();

      Type GetType() const override;

      // index: 0..(num-1)
      int getSlaveNum() const override;

      std::string getSlaveName(int cnt) const override;

      virtual MailboxStatus SendMessage_(MailboxMessage::MailboxMessagePtr ptr) override;

      virtual void GetProcessMsg(ProcessBuffer& buff, uint8_t slaveNumber) const override;

      virtual void SetProcessFromSlaveSize(uint8_t size, uint8_t slaveNumber) override;

      virtual int SetProcessMsg(const ProcessBuffer& buffer, uint8_t slaveNumber) override;

      virtual void ExchangeProcessMsg() override; // call the functions
    };
  }
}

#endif
