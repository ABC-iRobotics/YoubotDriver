#ifndef VIRTUAL_MESSAGE_CENTER_HPP
#define VIRTUAL_MESSAGE_CENTER_HPP

#include <string>
#include <mutex>
#include <vector>
#include <functional>
#include "EtherCATMaster.hpp"

#include <iostream>

#define MIN(a,b) a<b?a:b

namespace youbot {
  namespace intrinsic {

    // only function ExchangeProcessMsg is implemented
    // and RegisterSlave must be called once by all Slaves processmessages influence 
    class VirtualEtherCATMaster : public EtherCATMaster {
      //std::vector<> vector of functions

    public:
      VirtualEtherCATMaster() {};

      ~VirtualEtherCATMaster();

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

      virtual void ExchangeProcessMsg() override; // call the functions

      virtual bool isOpened() const override;

      void RegisterSlave(std::function<void(void)> in) {
        saved.push_back(in);
      }

    private:
      std::vector<std::function<void(void)>> saved;
    };
  }
}

#endif
