#include "SOEMMessageCenter.hpp"
#include <exception>
#include <stdexcept>
#include "Time.hpp"
#include <string>

extern "C" {
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatconfig.h"
#include "ethercatcoe.h"
#include "ethercatdc.h"
#include "ethercatprint.h"
}

#include "Logger.hpp"

using namespace youbot;

bool SOEMMessageCenter::opened = false;

bool SOEMMessageCenter::OpenConnection(const std::string& adapterName) {
  std::lock_guard<std::mutex> lock(ethercatComm);

  if (opened) {
    log(_A, Log::fatal, "SOEMMessageCenter is already opened.");
    throw std::runtime_error("SOEMMessageCenter is already opened.");
  }

  //initialize to zero
  for (unsigned int i = 0; i < 4096; i++)
    IOmap_[i] = 0;

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(adapterName.c_str())) {
    log(Log::info, "Initializing EtherCAT on " + adapterName + " with communication thread");
    /* find and auto-config slaves */
    if (ec_config(TRUE, &IOmap_) > 0) {
      log(Log::info, std::to_string(ec_slavecount) + " EtherCAT slaves found and configured.");

      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
      if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        log(Log::info, "Not all EtherCAT slaves reached safe operational state.");
        ec_readstate();
        //If not all slaves operational find out which one
        for (int i = 1; i <= ec_slavecount; i++)
          if (ec_slave[i].state != EC_STATE_SAFE_OP) {
            log(Log::info, "Slave " + std::to_string(i) +
              " State=" + std::to_string(ec_slave[i].state) +
              " StatusCode=" + std::to_string(ec_slave[i].ALstatuscode) +
              " : " + ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
      }
      //Read the state of all slaves
      ec_readstate();

      log(Log::info, "Request operational state for all EtherCAT slaves");

      ec_slave[0].state = EC_STATE_OPERATIONAL;
      // request OP state for all slaves
      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* request OP state for all slaves */
      ec_writestate(0);
      // wait for all slaves to reach OP state

      ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
      if (ec_slave[0].state == EC_STATE_OPERATIONAL)
        log(Log::info, "Operational state reached for all EtherCAT slaves.");
      else {
        log(Log::fatal, "Not all EtherCAT slaves reached operational state.");
        throw std::runtime_error("Not all EtherCAT slaves reached operational state.");
      }
    }
    else {
      log(Log::fatal, "No EtherCAT slaves found!");
      throw std::runtime_error("No EtherCAT slaves found!");
    }
  }
  else {
    log(Log::fatal, "No socket connection on  socket Excecute as root");
    throw std::runtime_error("No socket connection on  socket Excecute as root");
  }

  //reserve memory for all slave with a input/output buffer
  for (int cnt = 1; cnt <= ec_slavecount; cnt++)
    log(Log::info, "Slave: " + std::to_string(cnt) + " Name: " + ec_slave[cnt].name +
      " Output size: " + std::to_string(ec_slave[cnt].Obits)
      + "bits Input size: " + std::to_string(ec_slave[cnt].Ibits)
      + "bits State: " + std::to_string(ec_slave[cnt].state)
      + " delay: " + std::to_string(ec_slave[cnt].pdelay));
  if (ec_slavecount > 0)
    log(Log::info, std::to_string(ec_slavecount) + " EtherCAT slaves found");
  else {
    log(Log::fatal, "No EtherCAT slave could be found");
    throw std::runtime_error("No EtherCAT slave could be found");
  }

  mailboxBuffers = new MailboxBuffers[ec_slavecount];
  processBuffers = new ProcessBuffers[ec_slavecount];

  opened = true;
  return opened;
}

void SOEMMessageCenter::CloseConnection() {
  // Request safe operational state for all slaves
  ec_slave[0].state = EC_STATE_SAFE_OP;

  /* request SAFE_OP state for all slaves */
  ec_writestate(0);

  //stop SOEM, close socket
  ec_close();


  delete[] mailboxBuffers;
  delete[] processBuffers;

  opened = false;
}

SOEMMessageCenter::~SOEMMessageCenter() {
  if (opened)
    CloseConnection();
}

int SOEMMessageCenter::getSlaveNum() const {
  return ec_slavecount;
}

std::string SOEMMessageCenter::getSlaveName(int cnt) const {
  return ec_slave[cnt + 1].name;
}

SOEMMessageCenter::MailboxStatus SOEMMessageCenter::SendMessage_(MailboxMessage::MailboxMessagePtr ptr) {
  //copy from buffer
  int i = ptr->getSlaveIndex();
  {
    std::lock_guard<std::mutex> lock(slaveBufferMutexes[i]);
    memcpy((void*)mailboxBuffers[i].toSlave,
      ptr->getToSlaveBuff(),
      ptr->getToSlaveBuffSize());
  }
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

void SOEMMessageCenter::GetProcessMsg(ProcessBuffer& buff, uint8_t slaveNumber) const {
  buff = processBuffers[slaveNumber].fromSlave.Get();
}

void SOEMMessageCenter::SetProcessFromSlaveSize(uint8_t size, uint8_t slaveNumber) {
  processBuffers[slaveNumber].fromMsgSize = size;
}

int SOEMMessageCenter::SetProcessMsg(const ProcessBuffer& buffer, uint8_t slaveNumber) {
  processBuffers[slaveNumber].toSlave.Set(buffer);
  return buffer.Size();
}

void SOEMMessageCenter::ExchangeProcessMsg() {
  for (int i = 0; i < getSlaveNum(); i++)
    processBuffers[i].toSlave.Get().CopyTo(ec_slave[i + 1].outputs, ec_slave[i + 1].Obytes);
  {
    static long communicationErrors = 0;
    static long maxCommunicationErrors = 100;
    std::lock_guard<std::mutex> lock(ethercatComm);
    ec_send_processdata();
    if (ec_receive_processdata(EC_TIMEOUTRET)<=0) { //TODO error handling...
      if (communicationErrors == 0)
        log(_A, _B, _C, Log::warning, "Receiving data failed");
      communicationErrors++;
    }
    else
      communicationErrors = 0;
    if (communicationErrors > maxCommunicationErrors) {
      log(_A, _B, _C, Log::fatal, "Lost EtherCAT connection");
      exit(-1);
    }
    /*
    if (ec_iserror()) {
      std::cout << ec_iserror() << std::endl;
      LOG(warning) << "there is an error in the soem driver" << std::endl;
    }
    */
  }
  for (int i = 0; i < getSlaveNum(); i++) {
    uint8_t buff_size = MIN((uint8_t)ec_slave[i + 1].Ibytes, processBuffers[i].fromMsgSize);
    ProcessBuffer saved(buff_size, ec_slave[i + 1].inputs);
    processBuffers[i].fromSlave.Set(saved);
  }
}