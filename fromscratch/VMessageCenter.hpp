#ifndef V_MESSAGE_CENTER_HPP
#define V_MESSAGE_CENTER_HPP

#include <string>
#include "MailboxMessage.hpp"

class VMessageCenter {
public:
  enum class MailboxStatus : uint8_t {
    INITIALIZED = 0,
    SENT_SUCCESSFUL = 2,
    RECEIVED_SUCCESSFUL = 4
  };

  virtual MailboxStatus SendMessage_(MailboxMessage::MailboxMessagePtr ptr) = 0;

  virtual bool OpenConnection(const std::string& adapterName) = 0;

  virtual void CloseConnection() = 0;

  static VMessageCenter* GetSingleton();

  virtual int getSlaveNum() const = 0; // cnt: 0..N-1

  virtual std::string getSlaveName(int cnt) const = 0; // cnt: 0..N-1
};

#endif
