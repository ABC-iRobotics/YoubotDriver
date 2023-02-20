#ifndef MAILBOX_MESSAGE_HPP
#define MAILBOX_MESSAGE_HPP

#include <memory>

namespace youbot {

  /// <summary>
  /// Base class used for Mailbox communication
  /// 
  /// Initialized with slave ID and message content
  /// the EtherCAT Master driver will copy the response into it as well.
  /// 
  /// Subclasses can provide simple wrappers for the different message-answer types 
  /// </summary>
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

	/// <summary>
	/// Constructor setting the buffer sizes, the content must be set after it.
	/// </summary>
	/// <param name="slaveIndex"></param>
	/// <param name="toSlaveBuffSize"></param>
	/// <param name="fromSlaveBuffSize"></param>
	MailboxMessage(int slaveIndex, int toSlaveBuffSize, int fromSlaveBuffSize)
	  : slaveIndex(slaveIndex), toSlaveBuffSize(toSlaveBuffSize), fromSlaveBuffSize(fromSlaveBuffSize),
	  toSlaveBuff(new uint8_t[toSlaveBuffSize]), fromSlaveBuff(new uint8_t[fromSlaveBuffSize]) {}

	/// Destructor: delete the buffers
	~MailboxMessage() {
	  delete toSlaveBuff;
	  delete fromSlaveBuff;
	}

	/// <summary>
	/// Returns the ptr of the buffer to be sent
	/// </summary>
	/// <returns> ptr </returns>
	uint8_t* getToSlaveBuff() { return toSlaveBuff; }

	/// <summary>
	/// Returns the ptr of the buffer received
	/// </summary>
	/// <returns> ptr </returns>
	const uint8_t* getFromSlaveBuff() const { return fromSlaveBuff; }

	/// <summary>
	/// Simple getter
	/// </summary>
	/// <returns> Targeted etherCAT slave id (0..(N-1)) </returns>
	int getSlaveIndex() const { return slaveIndex; }

	/// <summary>
	/// Simple getter
	/// </summary>
	/// <returns> Size of the buffer to be sent out </returns>
	int getToSlaveBuffSize() const { return toSlaveBuffSize; }

	/// <summary>
	/// Simple getter
	/// </summary>
	/// <returns> Size of the buffer for the received message </returns>
	int getFromSlaveBuffSize() const { return fromSlaveBuffSize; }

	typedef std::shared_ptr<MailboxMessage> Ptr;
  };
}
#endif
