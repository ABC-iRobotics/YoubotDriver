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
#include "DataObjectLockFree.hpp"
#include "Logger.hpp"

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

#include "TMCLMessages.hpp"
#include "Time.hpp"

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

  bool ethercatConnectionEstablished;
  ec_mbxbuft mailboxBufferSend;
  ec_mbxbuft mailboxBufferReceive;
  unsigned int nrOfSlaves;
  unsigned int ethercatTimeout;
  unsigned int mailboxTimeout = 4000;

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

    //ethercatSlaveInfo.push_back(ec_slave[cnt]);

    actualSlaveName = ec_slave[cnt].name;
    if ((actualSlaveName == baseJointControllerName || actualSlaveName == baseJointControllerNameAlternative ||
      actualSlaveName == manipulatorJointControllerName || actualSlaveName == ManipulatorJointControllerNameAlternative
      ) && ec_slave[cnt].Obits > 0 && ec_slave[cnt].Ibits > 0) {
      //identified_slaves.push_back(cnt);
      nrOfSlaves++;
      //ethercatOutputBufferVector.push_back((SlaveMessageOutput*)(ec_slave[cnt].outputs));
      //ethercatInputBufferVector.push_back((SlaveMessageInput*)(ec_slave[cnt].inputs));
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
  {
    FirmWareRequest req3(3);
    req3.SendToSlave(mailboxTimeout);
    FirmWareRequest req4(4);
    req4.SendToSlave(mailboxTimeout);

    req4.ReceiveFromSlave(mailboxTimeout);
    req3.ReceiveFromSlave(mailboxTimeout);

    long controllertype, firmwareversion;
    req3.GetOutput(controllertype, firmwareversion);
    printf("c: %d, f: %d\n", controllertype, firmwareversion);
    req4.GetOutput(controllertype, firmwareversion);
    printf("c: %d, f: %d\n", controllertype, firmwareversion);
  }
  /*
  SetEncoder param0(2,1000);
  if (!param0.SendToSlave(mailboxTimeout))
    printf("Send error...\n");
  if (!param0.ReceiveFromSlave(mailboxTimeout))
    printf("Rec error...\n");
*/

  for (unsigned long i = 0; i < 1e4; i++)
  {
    GetMaxCurrent param2(2);

    if (!param2.SendToSlave(mailboxTimeout))
      printf("send error\n");

    if (!param2.ReceiveFromSlave(mailboxTimeout))
      printf("receive error\n");
    
    if (param2.IsReceiveSuccessful()) {

      uint8 status;
      param2.GetRecStatusFlag(status);
      printf("status: %d\n", status);

      std::string a = TMCLRequest::StatusErrorFlagsToString(param2.GetValue());
      printf("Pos value: %s\n", a.c_str());
    }
    else
      printf("unsuccessful rec\n");
   
    //else
      //printf("unsuccessful send\n");
      
  }
  






  // Request safe operational state for all slaves
  ec_slave[0].state = EC_STATE_SAFE_OP;

  /* request SAFE_OP state for all slaves */
  ec_writestate(0);

  //stop SOEM, close socket
  ec_close();

  return (0);
}