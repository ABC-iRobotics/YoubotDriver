#ifndef V_MESSAGE_CENTER_HPP
#define V_MESSAGE_CENTER_HPP

#include <string>
#include "MailboxMessage.hpp"
#include "ProcessBuffer.hpp"

class VMessageCenter {
public:
  enum class MailboxStatus : uint8_t {
    INITIALIZED = 0,
    SENT_SUCCESSFUL = 2,
    RECEIVED_SUCCESSFUL = 4
  };

  // Copy recieved rocess message of slavenumber (0..(N-1)) into the buffer
  virtual void GetProcessMsg(ProcessBuffer& buff, uint8_t slaveNumber) const = 0;

  // Set the useful size of recieved process messages to be copyied (by default zero)
  // of slavenumber (0..(N-1))
  virtual void SetProcessFromSlaveSize(uint8_t size, uint8_t slaveNumber) = 0;

  // Prepare the buffer to send out to slave of slavenumber (0..(N-1))
  virtual int SetProcessMsg(const ProcessBuffer& buffer, uint8_t slaveNumber) = 0;

  // Exchange prepared slave buffers
  virtual void ExchangeProcessMsg() = 0;

  virtual MailboxStatus SendMessage_(MailboxMessage::MailboxMessagePtr ptr) = 0;

  virtual bool OpenConnection(const std::string& adapterName) = 0;

  virtual void CloseConnection() = 0;

  static VMessageCenter* GetSingleton();

  virtual int getSlaveNum() const = 0; // cnt: 0..N-1

  virtual std::string getSlaveName(int cnt) const = 0; // cnt: 0..N-1
};

#endif
