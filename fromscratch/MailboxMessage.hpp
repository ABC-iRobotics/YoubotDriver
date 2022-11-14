#ifndef MAILBOX_MESSAGE_HPP
#define MAILBOX_MESSAGE_HPP

#include <memory>

class MailboxMessage {
  int slaveIndex; //0..N-1
protected:
  uint8_t* toSlaveBuff;// of toSlaveBuffSize
  uint8_t* fromSlaveBuff;// of fromSlaveBuffSize
  int toSlaveBuffSize, fromSlaveBuffSize;
public:
  MailboxMessage() = delete;

  MailboxMessage(MailboxMessage&) = delete;

  MailboxMessage(const MailboxMessage&) = delete;

  MailboxMessage(int slaveIndex, int toSlaveBuffSize, int fromSlaveBuffSize)
	: slaveIndex(slaveIndex), toSlaveBuffSize(toSlaveBuffSize), fromSlaveBuffSize(fromSlaveBuffSize),
	toSlaveBuff(new uint8_t[toSlaveBuffSize]), fromSlaveBuff(new uint8_t[fromSlaveBuffSize]) {}

  ~MailboxMessage() {
	delete toSlaveBuff;
	delete fromSlaveBuff;
  }

  uint8_t* getToSlaveBuff() { return toSlaveBuff; }

  const uint8_t* getFromSlaveBuff() const { return fromSlaveBuff; }

  int getSlaveIndex() const { return slaveIndex; }

  int getToSlaveBuffSize() const { return toSlaveBuffSize; }

  int getFromSlaveBuffSize() const { return fromSlaveBuffSize; }

  typedef std::shared_ptr<MailboxMessage> MailboxMessagePtr;
};

#endif
