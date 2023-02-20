#ifndef ETHERCATMASTER_HPP
#define ETHERCATMASTER_HPP

#include <string>
#include "MailboxMessage.hpp"
#include "ProcessBuffer.hpp"
#include <memory>
#include <functional>
#include <vector>

namespace youbot {

  class EtherCATMaster {
  public:
    enum Type {
      PHYSICAL,
      VIRTUAL
    };

    virtual Type GetType() const = 0;

    ~EtherCATMaster();

    enum class MailboxStatus : uint8_t {
      INITIALIZED = 0,
      SENT_SUCCESSFUL = 2,
      RECEIVED_SUCCESSFUL = 4
    };

    // Copy recieved process message of slavenumber (0..(N-1)) into the buffer
    virtual void GetProcessMsg(ProcessBuffer& buff, uint8_t slaveNumber) const = 0;

    // Set the useful size of recieved process messages to be copyied (by default zero)
    // of slavenumber (0..(N-1))
    virtual void SetProcessFromSlaveSize(uint8_t size, uint8_t slaveNumber) = 0;

    // Prepare the buffer to send out to slave of slavenumber (0..(N-1))
    virtual int SetProcessMsg(const ProcessBuffer& buffer, uint8_t slaveNumber) = 0;

    // Exchange prepared slave buffers
    virtual void ExchangeProcessMsg() = 0;


    virtual MailboxStatus SendMailboxMessage(MailboxMessage::MailboxMessagePtr ptr) = 0;
    virtual int getSlaveNum() const = 0; // cnt: 0..N-1

    virtual std::string getSlaveName(int cnt) const = 0; // cnt: 0..N-1

    typedef std::shared_ptr<EtherCATMaster> Ptr;

    static Ptr CreatePhysical(const std::string& adapterName);

    static Ptr CreateVirtual();

    void RegisterAfterExchangeCallback(std::function<void(void)> in);

  protected:
    void _callAfterExchangeCallbacks(); // must be called at the end of the exchange

  private:
    std::vector<std::function<void(void)>> afterExchangeCallbacks;
  };
}
#endif
