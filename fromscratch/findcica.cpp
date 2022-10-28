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
char IOmap_[4096];

void initEtherCat(const char* name) {
  //initialize to zero
  for (unsigned int i = 0; i < 4096; i++)
    IOmap_[i] = 0;

  //bool ethercatConnectionEstablished;
  //ec_mbxbuft mailboxBufferSend;
  //ec_mbxbuft mailboxBufferReceive;
  unsigned int nrOfSlaves;
  //unsigned int ethercatTimeout;

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
  }
}

//TODO: mutex to make impossible reading until send&rec...
template <int toSlaveSize, int fromSlaveSize>
class EthercatProcessRequest {
  int slaveNum;
public:
  EthercatProcessRequest(int slaveNum) : slaveNum(slaveNum) {}

  void PrepareToSlaveMsg(uint8* buffer) {
    memcpy(ec_slave[iSlave].outputs, buffer, toSlaveSize);
  }

  uint8* GetOutput(int& length) {
    length = fromSlaveSize;
    return ec_slave[iSlave].inputs;
  }
};

class TMCLModuleProcessRequest : private EthercatProcessRequest<5,20> {
public:

};

class EthercatProcessComm {
public:
  void Do() {
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
  }
};

int main(int argc, char *argv[])
{
  char name[1000];
  printf("sg\n");
  if (findYouBotEtherCatAdapter(name)) {
    printf("\n\n\nAdapter found: %s\n", name);
  }
  else {
    printf("\n\n\nAdapter with turned on youBot arm NOT found!\n");
    return -1;
  }

  initEtherCat(name);
 
  




  unsigned int mailboxTimeout = 4000;



  int iSlave = 3;

  {
    SetEncoder param0(iSlave, (uint32)100000);
    param0.TryToSend(mailboxTimeout, 10, 0);
    printf(" %s\n", param0.StatusToString().c_str());
  }

  SLEEP_MILLISEC(10);

  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  uint8* out = ec_slave[iSlave].outputs;
  uint8* in = ec_slave[iSlave].inputs;

  uint8* cmd = out + 4;
  *cmd = 7;

  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  printf(" %d %d %d %d %d \n", *out, *(out + 1), *(out + 2), *(out + 3), *(out + 4));
  printf(" %d %d %d %d %d \n", *in, *(in + 1), *(in + 2), *(in + 3), *(in + 4));

  SLEEP_MILLISEC(30);

  *cmd = 2;

  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);

  printf(" %d %d %d %d %d \n", *out, *(out + 1), *(out + 2), *(out + 3), *(out + 4));
  printf(" %d %d %d %d %d \n", *in, *(in + 1), *(in + 2), *(in + 3), *(in + 4));

  {
    FirmWareRequest req3(4);
    req3.TryToSend(mailboxTimeout, 10, 0);
    FirmWareRequest req4(3);
    req4.TryToSend(mailboxTimeout, 10, 0);

    long controllertype, firmwareversion;
    req3.GetOutput(controllertype, firmwareversion);
    printf("c: %d, f: %d\n", controllertype, firmwareversion);
    req4.GetOutput(controllertype, firmwareversion);
    printf("c: %d, f: %d\n", controllertype, firmwareversion);
  }
  
  {
    //SetEncoderDirection param0(2, (uint32)1);
    GetP2ParameterPositionControl param0(iSlave);
    param0.TryToSend(mailboxTimeout, 10, 0);
    printf(" %s\n", param0.StatusToString().c_str());
  }
  
  for (unsigned long i = 0; i < 1e4; i++)
  {

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    /*
    GetPosition param2(iSlave);
    param2.TryToSend(mailboxTimeout, 10, 0);
    printf(" %s\n", param2.StatusToString().c_str());
    
    if (param2.IsReceiveSuccessful()) {

      uint8 status;
      param2.GetRecStatusFlag(status);
      printf("status: %d\n", status);


      printf("Pos value: %u\n", param2.GetValue());
      //std::string a = TMCLRequest::StatusErrorFlagsToString(param2.GetValue());
      //printf("Pos value: %s\n", a.c_str());
    }
    else
      printf("unsuccessful rec\n");
      */
  }
  
  
  /*
  int ethercatTimeout = 4000;
  int communicationErrors = 0;

  //send and receive data from ethercat
  if (ec_send_processdata() == 0) {
    LOG(warning) << "Sending process data failed";
  }

  if (ec_receive_processdata(ethercatTimeout) == 0) {
    if (communicationErrors == 0) {
      LOG(warning) << "Receiving data failed";
    }
    communicationErrors++;
  }
  else {
    communicationErrors = 0;
  }

  */















  // Request safe operational state for all slaves
  ec_slave[0].state = EC_STATE_SAFE_OP;

  /* request SAFE_OP state for all slaves */
  ec_writestate(0);

  //stop SOEM, close socket
  ec_close();

  return (0);
}