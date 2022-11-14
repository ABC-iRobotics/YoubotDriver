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
  std::mutex slaveBufferMutexes[50];
  struct MailboxBuffers {
    ec_mbxbuft fromSlave, toSlave;
  };
  std::vector<MailboxBuffers> mailboxBuffers;

  std::mutex ethercatComm;

  static bool opened;

  char IOmap_[4096];

public:
  SOEMMessageCenter() {};

  virtual MailboxStatus SendMessage_(MailboxMessage::MailboxMessagePtr ptr) override;

  virtual bool OpenConnection(const std::string& adapterName) override;

  virtual void CloseConnection() override;

  ~SOEMMessageCenter();

  // index: 0..(num-1)
  int getSlaveNum() const override;
  std::string getSlaveName(int cnt) const override;
};

#endif
