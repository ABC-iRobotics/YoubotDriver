#include "SOEMMessageCenter.hpp"
#include <Exceptions.hpp>
#include "Time.hpp"

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

  if (opened)
    throw std::runtime_error("SOEMMessageCenter is already opened.");

  //initialize to zero
  for (unsigned int i = 0; i < 4096; i++)
    IOmap_[i] = 0;

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(adapterName.c_str())) {
    LOG(info) << "Initializing EtherCAT on " << adapterName.c_str() << " with communication thread";
    /* find and auto-config slaves */
    if (ec_config(TRUE, &IOmap_) > 0) {
      LOG(trace) << ec_slavecount << " EtherCAT slaves found and configured.";

      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
      if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        LOG(warning) << "Not all EtherCAT slaves reached safe operational state.";
        ec_readstate();
        //If not all slaves operational find out which one
        for (int i = 1; i <= ec_slavecount; i++)
          if (ec_slave[i].state != EC_STATE_SAFE_OP) {
            LOG(info) << "Slave " << i <<
              " State=" << ec_slave[i].state <<
              " StatusCode=" << ec_slave[i].ALstatuscode <<
              " : " << ec_ALstatuscode2string(ec_slave[i].ALstatuscode);
          }
      }
      //Read the state of all slaves
      ec_readstate();

      LOG(info) << "Request operational state for all EtherCAT slaves";

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
        LOG(info) << "Operational state reached for all EtherCAT slaves.";
      else
        throw std::runtime_error("Not all EtherCAT slaves reached operational state.");
    }
    else
      throw std::runtime_error("No EtherCAT slaves found!");
  }
  else
    throw std::runtime_error("No socket connection on  socket \nExcecute as root");

  //reserve memory for all slave with a input/output buffer
  for (int cnt = 1; cnt <= ec_slavecount; cnt++)
    LOG(trace) << "Slave: " << cnt << " Name: " << ec_slave[cnt].name << " Output size: " << ec_slave[cnt].Obits
      << "bits Input size: " << ec_slave[cnt].Ibits << "bits State: " << ec_slave[cnt].state
      << " delay: " << ec_slave[cnt].pdelay; //<< " has dclock: " << (bool)ec_slave[cnt].hasdc;
  if (ec_slavecount > 0)
    LOG(info) << ec_slavecount << " EtherCAT slaves found";
  else
    throw std::runtime_error("No EtherCAT slave could be found");

  mailboxBuffers = new MailboxBuffers[ec_slavecount];

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
