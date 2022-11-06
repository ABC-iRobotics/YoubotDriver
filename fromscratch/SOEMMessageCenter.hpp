#ifndef SOEM_MESSAGE_CENTER_HPP
#define SOEM_MESSAGE_CENTER_HPP

#include <atomic>
#include <memory>
#include <string>
#include <mutex>
#include <vector>

extern "C" {
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatmain.h"
}
#include "Logger.hpp"

using namespace youbot;

class VMailboxMessage {
public:
  virtual uint8* getToSlaveBuff() = 0;

  virtual const uint8* getFromSlaveBuff() const = 0;

  virtual int getSlaveIndex() const = 0;

  virtual int getToSlaveBuffSize() const = 0;

  virtual int getFromSlaveBuffSize() const = 0;

  typedef std::shared_ptr<VMailboxMessage> MailboxMessagePtr;
};

template <int toSlaveBuffSize, int fromSlaveBuffSize>
class MailboxMessage : public VMailboxMessage {
  int slaveIndex; //0..N-1
protected:
  uint8 toSlaveBuff[toSlaveBuffSize];
  uint8 fromSlaveBuff[fromSlaveBuffSize];
public:
  MailboxMessage() = delete;

  MailboxMessage(MailboxMessage&) = delete;

  MailboxMessage(const MailboxMessage&) = delete;

  MailboxMessage(int slaveIndex) : slaveIndex(slaveIndex) {}

  uint8* getToSlaveBuff() {
    return toSlaveBuff;
  }

  const uint8* getFromSlaveBuff() const {
    return fromSlaveBuff;
  }

  int getSlaveIndex() const {
    return slaveIndex;
  }

  int getToSlaveBuffSize() const {
    return toSlaveBuffSize;
  }

  int getFromSlaveBuffSize() const {
    return fromSlaveBuffSize;
  }
};

class GetFirmware : public MailboxMessage<8, 8> {
public:
  GetFirmware(int slaveIndex) : MailboxMessage(slaveIndex) {
    uint8* buff = getToSlaveBuff();
    *buff = 0;
    *(buff + 1) = 136;
    *(buff + 2) = 0;
    *(buff + 3) = 0;
  }

  void GetOutput(long& controllernum, long& firmwarenum) const {
    char* ptr, * ptr2;
    controllernum = strtol((const char*)getFromSlaveBuff(), &ptr, 10);
    firmwarenum = strtol(ptr + 1, &ptr2, 10);
  }
};


class SOEMMessageCenter {
  std::mutex slaveBufferMutexes[50];
  struct MailboxBuffers {
    ec_mbxbuft fromSlave, toSlave;
  };
  std::vector<MailboxBuffers> mailboxBuffers;

  std::mutex ethercatComm;

  static bool opened;

  char IOmap_[4096];

public:

  enum class MailboxStatus : uint8 {
    INITIALIZED = 0,
    SENT_SUCCESSFUL = 2,
    RECEIVED_SUCCESSFUL = 4
  };

  SOEMMessageCenter() {};

  MailboxStatus SendMessage(VMailboxMessage::MailboxMessagePtr ptr) {
    //copy from buffer
    int i = ptr->getSlaveIndex();
    //{
      //std::lock_guard<std::mutex> lock(slaveBufferMutexes[i]);
      memcpy((void*)mailboxBuffers[i].toSlave,
        ptr->getToSlaveBuff(), 
        ptr->getToSlaveBuffSize());
    //}
    //send and receive
    int mailboxTimeoutUS = 4000;
    MailboxStatus status = MailboxStatus::INITIALIZED;
    {
      std::lock_guard<std::mutex> lock(ethercatComm);
      if (ec_mbxsend(i + 1, &mailboxBuffers[i].toSlave, mailboxTimeoutUS) >= 0) {
        status = MailboxStatus::SENT_SUCCESSFUL;
        if (ec_mbxreceive(i + 1, &mailboxBuffers[i].fromSlave, mailboxTimeoutUS) >= 0)
          status = MailboxStatus::RECEIVED_SUCCESSFUL;
      }
    }
    // copy to buffer
    {
      std::lock_guard<std::mutex> lock(slaveBufferMutexes[i]);
      memcpy((void*)ptr->getFromSlaveBuff(),
        mailboxBuffers[i].fromSlave,
        ptr->getFromSlaveBuffSize());
    }
    return status;
  }

  bool OpenConnection(const std::string& adapterName);

  void CloseConnection();

  // index: 0..(num-1)
  uint8* getToSlaveMailboxBuffer(int cnt) const;
  uint8* getFromSlaveMailboxBuffer(int cnt) const;
  int getSlaveNum() const;
  std::string getSlaveName(int cnt) const;
};

#endif
