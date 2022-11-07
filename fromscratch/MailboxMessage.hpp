#ifndef MAILBOX_MESSAGE_HPP
#define MAILBOX_MESSAGE_HPP

#include <memory>

class VMailboxMessage {
public:
  virtual uint8_t* getToSlaveBuff() = 0;

  virtual const uint8_t* getFromSlaveBuff() const = 0;

  virtual int getSlaveIndex() const = 0; //0..N-1

  virtual int getToSlaveBuffSize() const = 0;

  virtual int getFromSlaveBuffSize() const = 0;

  typedef std::shared_ptr<VMailboxMessage> MailboxMessagePtr;
};

template <int toSlaveBuffSize, int fromSlaveBuffSize>
class MailboxMessage : public VMailboxMessage {
  int slaveIndex; //0..N-1
protected:
  uint8_t toSlaveBuff[toSlaveBuffSize];
  uint8_t fromSlaveBuff[fromSlaveBuffSize];
public:
  MailboxMessage() = delete;

  MailboxMessage(MailboxMessage&) = delete;

  MailboxMessage(const MailboxMessage&) = delete;

  MailboxMessage(int slaveIndex) : slaveIndex(slaveIndex) {}

  uint8_t* getToSlaveBuff() { return toSlaveBuff; }

  const uint8_t* getFromSlaveBuff() const { return fromSlaveBuff; }

  int getSlaveIndex() const { return slaveIndex; }

  int getToSlaveBuffSize() const { return toSlaveBuffSize; }

  int getFromSlaveBuffSize() const { return fromSlaveBuffSize; }
};

#endif
