#include <stdio.h>
#include <iostream>
#include <string.h>

#include "adapters.hpp"

//#include "TMCLMessages.hpp"
#include "Time.hpp"

/*
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
#include "SOEMMessageCenter.hpp"
#include "TMCLMailboxMessage.hpp"

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

  {
    auto ptr = SetMaxCurrent::InitSharedPtr(3,4000);
    center->SendMessage_(ptr);
    printf(" ? %lu \n", ptr->GetReplyValue());
    std::cout << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }
  for (int i = 0; i < 50;i++) {
    auto ptr = GetCurrent::InitSharedPtr(3);
    center->SendMessage_(ptr);
    printf(" ? %lu \n", ptr->GetReplyValue());
    std::cout << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }
  {
    auto ptr = GetMaxCurrent::InitSharedPtr(3);
    center->SendMessage_(ptr);
    printf(" ? %lu \n", ptr->GetReplyValue());
    std::cout << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }
  center->CloseConnection();

  return 0;

  {
    auto ptr = GetFirmware::InitSharedPtr(3);
    center->SendMessage_(ptr);
    long a, b;
    ptr->GetOutput(a, b);
    printf("%d %d\n", a, b);
  }
  {
    auto ptr = GetPosition::InitSharedPtr(3);
    center->SendMessage_(ptr);
    printf(" ? %lu \n", ptr->GetReplyValue());
    std::cout << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }
  {
    auto ptr = SetEncoder::InitSharedPtr(3,10000);
    auto status = center->SendMessage_(ptr);
    std::cout << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }
  {
    auto ptr = GetErrorStatusFlag::InitSharedPtr(3);
    center->SendMessage_(ptr);
    printf(" ? %s \n", TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str());
    std::cout << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }
  center->CloseConnection();



  return 0;

  /*

  unsigned int mailboxTimeoutUS = 40;



  int iSlave = 3;
  {
    SetEncoder param0(iSlave, (uint32)100000);
    param0.TryToSend(mailboxTimeoutUS, 1000, 100);
    printf("SetEncoder: %s\n", param0.StatusToString().c_str());
  }

  {
    GetMotorControllerStatus param0(iSlave);
    param0.TryToSend(mailboxTimeoutUS * 10, 1000, 100);
    printf("GetMotorControllerStatus: %s\n", param0.StatusToString().c_str());
    uint8 status;
    if (param0.GetRecStatusFlag(status))
      printf(" Res: %s\n", param0.StatusErrorFlagsAsString().c_str());
  }
  {
    GetIsUnderInitialization param0(iSlave);
    param0.TryToSend(mailboxTimeoutUS * 10, 1, 100);
    printf("GetIsUnderInitialization: %s\n", param0.StatusToString().c_str());
    printf(" Res: %s\n", param0.RecvStatusAsString().c_str());
  }
  if (1) {
    ClearErrorFlags param0(iSlave);
    param0.TryToSend(mailboxTimeoutUS * 10, 10, 100);
    printf("ClearErrorFlags: %s\n", param0.StatusToString().c_str());
    printf(" Res: %s\n", param0.RecvStatusAsString().c_str());
  }
  {
    GetMotorControllerStatus param0(iSlave);
    param0.TryToSend(mailboxTimeoutUS * 10, 1000, 100);
    printf("GetMotorControllerStatus: %s\n", param0.StatusToString().c_str());
    uint8 status;
    //if (param0.GetRecStatusFlag(status))
      printf(" Res: %s\n", param0.StatusErrorFlagsAsString().c_str());
  }
  {
    //SetEncoderDirection param0(2, (uint32)1);
    GetP1ParameterPositionControl param0(iSlave);
    param0.TryToSend(mailboxTimeoutUS*10, 1, 100);
    printf("GetP1ParameterPositionControl: %s\n", param0.StatusToString().c_str());
    printf(" Res: %s\n", param0.RecvStatusAsString().c_str());
  }
  return 0;
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










  // Request safe operational state for all slaves
  //ec_slave[0].state = EC_STATE_SAFE_OP;

  /* request SAFE_OP state for all slaves */
  //ec_writestate(0);

  //stop SOEM, close socket
  //ec_close();

  //return (0);*/
}