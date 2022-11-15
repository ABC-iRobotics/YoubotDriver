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
#include "VMessageCenter.hpp"

class SOEMMessageCenter : public VMessageCenter {
  // For mailbox messages
  std::mutex slaveBufferMutexes[50];
  struct MailboxBuffers {
    ec_mbxbuft fromSlave, toSlave;
  };
  MailboxBuffers* mailboxBuffers;

  // Mutex for ethercat communications in general
  std::mutex ethercatComm;

  static bool opened;

  char IOmap_[4096]; // used by soem

public:
  SOEMMessageCenter() {};

  ~SOEMMessageCenter();

  // index: 0..(num-1)
  int getSlaveNum() const override;

  std::string getSlaveName(int cnt) const override;

  virtual bool OpenConnection(const std::string& adapterName) override;

  virtual void CloseConnection() override;

  virtual MailboxStatus SendMessage_(MailboxMessage::MailboxMessagePtr ptr) override;

};

#endif
