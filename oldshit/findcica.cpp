/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : slaveinfo [ifname] [-sdo] [-map]
 * Ifname is NIC interface, f.e. eth0.
 * Optional -sdo to display CoE object dictionary.
 * Optional -map to display slave PDO mapping
 *
 * This shows the configured slave data.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>

#include "adapters.hpp"
#include "EthercatMaster.hpp"
#include "DataObjectLockFree.hpp"
#include "Logger.hpp"
#include "TMCLProtocolDefinitions.hpp"
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

using namespace youbot;

char ifbuf[1024];

int main(int argc, char *argv[])
{
  char name[5000];
  printf("sg\n");
  if (findYouBotEtherCatAdapter(name)) {
	printf("\n\n\nAdapter found: %s\n", name);
  }


  char IOmap_[4096];


  const std::string configFileName;

  const std::string configFilepath;

  std::vector<ec_slavet> ethercatSlaveInfo;
  bool ethercatConnectionEstablished;
  ec_mbxbuft mailboxBufferSend;
  ec_mbxbuft mailboxBufferReceive;
  std::vector<SlaveMessageOutput*> ethercatOutputBufferVector;
  std::vector<SlaveMessageInput*> ethercatInputBufferVector;
  unsigned int nrOfSlaves;
  unsigned int ethercatTimeout;
  unsigned int mailboxTimeout = 4000;
  ConfigFile* configfile;
  std::vector<int> identified_slaves;

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(name)) {
    LOG(info) << "Initializing EtherCAT on " << name << " with communication thread";
    /* find and auto-config slaves */
    if (ec_config(TRUE, &IOmap_) > 0) {
      LOG(trace) << ec_slavecount << " EtherCAT slaves found and configured.";

      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
      if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        LOG(warning) << "Not all EtherCAT slaves reached safe operational state.";
        ec_readstate();
        //If not all slaves operational find out which one
        for (int i = 1; i <= ec_slavecount; i++) {
          if (ec_slave[i].state != EC_STATE_SAFE_OP) {
            LOG(info) << "Slave " << i <<
              " State=" << ec_slave[i].state <<
              " StatusCode=" << ec_slave[i].ALstatuscode <<
              " : " << ec_ALstatuscode2string(ec_slave[i].ALstatuscode);
          }
        }
      }
      //Read the state of all slaves
      //ec_readstate();

      LOG(trace) << "Request operational state for all EtherCAT slaves";

      ec_slave[0].state = EC_STATE_OPERATIONAL;
      // request OP state for all slaves
      /* send one valid process data to make outputs in slaves happy*/
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* request OP state for all slaves */
      ec_writestate(0);
      // wait for all slaves to reach OP state

      ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
      if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
        LOG(trace) << "Operational state reached for all EtherCAT slaves.";
      }
      else {
        throw std::runtime_error("Not all EtherCAT slaves reached operational state.");
      }
    }
    else {
      throw std::runtime_error("No EtherCAT slaves found!");
    }
  }
  else {
    throw std::runtime_error("No socket connection on  socket \nExcecute as root");
  }

  std::string baseJointControllerName = "TMCM-174";
  std::string baseJointControllerNameAlternative = "TMCM-1632";
  std::string manipulatorJointControllerName = "TMCM-174";
  std::string ManipulatorJointControllerNameAlternative = "TMCM-1610";

  nrOfSlaves = 0;
  /*
  ConfigFile configfile;
  configfile->readInto(baseJointControllerName, "BaseJointControllerName");
  configfile->readInto(baseJointControllerNameAlternative, "BaseJointControllerNameAlternative");
  configfile->readInto(manipulatorJointControllerName, "ManipulatorJointControllerName");
  configfile->readInto(ManipulatorJointControllerNameAlternative, "ManipulatorJointControllerNameAlternative");*/
  std::string actualSlaveName;

  //reserve memory for all slave with a input/output buffer
  for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
    LOG(trace) << "Slave: " << cnt << " Name: " << ec_slave[cnt].name << " Output size: " << ec_slave[cnt].Obits
      << "bits Input size: " << ec_slave[cnt].Ibits << "bits State: " << ec_slave[cnt].state
      << " delay: " << ec_slave[cnt].pdelay; //<< " has dclock: " << (bool)ec_slave[cnt].hasdc;

    ethercatSlaveInfo.push_back(ec_slave[cnt]);

    actualSlaveName = ec_slave[cnt].name;
    if ((actualSlaveName == baseJointControllerName || actualSlaveName == baseJointControllerNameAlternative ||
      actualSlaveName == manipulatorJointControllerName || actualSlaveName == ManipulatorJointControllerNameAlternative
      ) && ec_slave[cnt].Obits > 0 && ec_slave[cnt].Ibits > 0) {
      identified_slaves.push_back(cnt);
      nrOfSlaves++;
      ethercatOutputBufferVector.push_back((SlaveMessageOutput*)(ec_slave[cnt].outputs));
      ethercatInputBufferVector.push_back((SlaveMessageInput*)(ec_slave[cnt].inputs));
    }
  }
  if (nrOfSlaves > 0) {
    LOG(info) << nrOfSlaves << " EtherCAT slaves found";
  }
  else {
    throw std::runtime_error("No EtherCAT slave could be found");
    return -1;
  }

  int iSlave = 3;
  
  mailboxBufferSend[0] = DRIVE; //module
  mailboxBufferSend[1] = FIRMWARE_VERSION; //command
  mailboxBufferSend[2] = 0; //type number 0/234
  mailboxBufferSend[3] = 0; //motor or bank number
  mailboxBufferSend[4] = 0;
  mailboxBufferSend[5] = 0;
  mailboxBufferSend[6] = 0;
  mailboxBufferSend[7] = 0;
  if (ec_mbxsend(iSlave, &mailboxBufferSend, mailboxTimeout)) {
    //printf("Send successful\n");
  }
  else {
    //printf("Send unsuccessful\n");
  }
  


  // Bouml preserved body begin 00052FF1
  int n = ec_mbxreceive(iSlave, &mailboxBufferReceive, mailboxTimeout);
  int k = 0;
  while (n>0) {
    //    LOG(trace) << "received mailbox message (buffer two) slave " << mailboxMsg.getSlaveNo();
    if (n > 0) {
      int replyAddress = (int)mailboxBufferReceive[0];
      int moduleAddress = (int)mailboxBufferReceive[1];
      int status = (int)mailboxBufferReceive[2];
      int commandNumber = (int)mailboxBufferReceive[3];
      long value = (mailboxBufferReceive[4] << 24 | mailboxBufferReceive[5] << 16 | mailboxBufferReceive[6] << 8 | mailboxBufferReceive[7]);
      //printf("Recieve successful %d %d %d %d %d %d\n", n, replyAddress, moduleAddress, status, commandNumber, value);
      k++;
    }
     //ec_mbxempty(iSlave, mailboxTimeout);
     n = ec_mbxreceive(iSlave, &mailboxBufferReceive, mailboxTimeout*100);
  }
  printf("Recieve unsuccessful (%d)\n",n);
  printf("%d\n", k);
    




  // Request safe operational state for all slaves
  ec_slave[0].state = EC_STATE_SAFE_OP;

  /* request SAFE_OP state for all slaves */
  ec_writestate(0);

  //stop SOEM, close socket
  ec_close();

  return (0);
}
/*
#include "YouBotManipulator.hpp"

int main2(int argc, char* argv[])
{ 
  try {
	youbot::YouBotManipulator arm("config",
	  //"D:/Tresors/WORK/PROJECT - Ruzics Barna KUKA/myYouBotDriver/config");
	  "C:/Users/kutij/Desktop/myYouBotDriver/src/config");

  }
  catch (std::exception& e) {
	std::cout << e.what() << std::endl;
  }
  catch (...) {
	std::cout << "unhandled exception" << std::endl;
  }

  return (0);
}
*/