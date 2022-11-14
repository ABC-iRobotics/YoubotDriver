#include <stdio.h>
#include <iostream>
#include <string.h>

#include "adapters.hpp"
#include "Time.hpp"

/*
#include "DataObjectLockFree.hpp"

class EthercatComm {
  std::mutex ethercatComm;
  std::mutex ethercatPrcBuffer;

public:
  void SendRcvProcessData() {
    // lock guard ethercatComm ethercatPrcBuffer

    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
  }

  std::mutex& GetCommMutex() {
    return ethercatComm;
  }

  std::mutex& GetPrcBufferMutex() {
    return ethercatPrcBuffer;
  }

  typedef std::shared_ptr<EthercatComm> EthercatCommPtr;

  static EthercatCommPtr GetSingleton() {
    static EthercatCommPtr singleton = std::make_shared<EthercatComm>();
    return singleton;
  }
};

template <int toSlaveSize, int fromSlaveSize>
class EthercatProcessRequest {
  int slaveNum;
public:
  EthercatProcessRequest(int slaveNum) : slaveNum(slaveNum) {
  
  }

  void PrepareToSlaveMsg(uint8* buffer) {
    memcpy(ec_slave[iSlave].outputs, buffer, toSlaveSize);
  }

  uint8* GetOutput(int& length) {
    length = fromSlaveSize;
    return ec_slave[iSlave].inputs;
  }
};

class TMCLModuleProcessRequest : protected EthercatProcessRequest<5,20> {
  uint8 toSlave[5];
  
  using EthercatProcessRequest::PrepareToSlaveMsg;
  using EthercatProcessRequest::GetOutput;

public:
  TMCLModuleProcessRequest(int slaveNum) : EthercatProcessRequest(slaveNum) {
    for (int i = 0; i < 5; i++)
      toSlave[i] = 0;
  }

};
*/
#include "VMessageCenter.hpp"
#include "TMCLMailboxMessage.hpp"
#include "YoubotConfig.hpp"
#include "YoubotManipulator.hpp"

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

  auto center = VMessageCenter::GetSingleton();
  if (!center->OpenConnection(name)) {
    return -1;
  }

  YoubotConfig config("C:/Users/kutij/Desktop/myYouBotDriver/src/fromscratch/youBotArmConfig_fromKeisler.json");
  //youBotArmConfig_fromfactory.json");
  //youBotArmConfig_fromMoveIt.json");
  //youBotArmConfig_fromKeisler.json");
    
  YoubotManipulator man(config, center);
  
  man.ConfigJoints();
  /*
  if (man.CheckJointConfigs())
    std::cout << "OK!!" << std::endl;
  */

  man.InitializeAllJoints();

  return 0;
  
  center->CloseConnection();

  return 0;


  /*
  SLEEP_MILLISEC(10);


  ec_send_processdata();
  ec_receive_processdata(EC_TIMEOUTRET);


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
    req3.TryToSend(mailboxTimeoutUS, 10, 0);
    FirmWareRequest req4(3);
    req4.TryToSend(mailboxTimeoutUS, 10, 0);

    long controllertype, firmwareversion;
    req3.GetOutput(controllertype, firmwareversion);
    printf("c: %d, f: %d\n", controllertype, firmwareversion);
    req4.GetOutput(controllertype, firmwareversion);
    printf("c: %d, f: %d\n", controllertype, firmwareversion);
  }
  
  {
    //SetEncoderDirection param0(2, (uint32)1);
    GetP2ParameterPositionControl param0(iSlave);
    param0.TryToSend(mailboxTimeoutUS, 10, 0);
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
  //}
  
  
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
}