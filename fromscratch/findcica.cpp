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

  YoubotConfig config("C:/Users/kutij/Desktop/myYouBotDriver/src/fromscratch/youBotArmConfig_fromdriver.json");

  YoubotManipulator man(config, center);

  return 0;



  std::cout << center->getSlaveNum() << std::endl;
  for (int i = 0; i < center->getSlaveNum(); i++)
    std::cout << center->getSlaveName(i) << std::endl;

  int iSlave = 1;

  // 1. Stop the motor
  {
    auto ptr = MotorStop::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << " MotorStop: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  // Get Status
  uint32_t status;
  {
    auto ptr = GetErrorStatusFlag::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    status = ptr->GetReplyValue();
    std::cout << "GetErrorStatusFlag: " <<
      TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
      << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }

  // If timeout clear it..
  if (status & (uint32_t)TMCL::StatusErrorFlags::TIMEOUT) {
    auto ptr = ClearMotorControllerTimeoutFlag::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << "  ClearMotorControllerTimeoutFlag: " << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }
  {
    auto ptr = GetErrorStatusFlag::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    status = ptr->GetReplyValue();
    std::cout << "GetErrorStatusFlag: " <<
      TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
      << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  // If I2T exceeded
  if (status & (uint32_t)TMCL::StatusErrorFlags::I2T_EXCEEDED) {
    auto ptr = ClearI2TFlag::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << "  ClearI2TFlag: " << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }
  {
    auto ptr = GetErrorStatusFlag::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    status = ptr->GetReplyValue();
    std::cout << "GetErrorStatusFlag: " <<
      TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
      << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  uint32_t pos;
  {
    auto ptr = GetPosition::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    pos = ptr->GetReplyValue();
    std::cout << " GetPosition: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  // If not initialized
  if (!(status & (uint32_t)TMCL::StatusErrorFlags::INITIALIZED)) {
    {
      auto ptr = SetInitialize::InitSharedPtr(iSlave, 1);
      center->SendMessage_(ptr);
      std::cout << "  SetInitialize: " << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
    }
    do {
      SLEEP_MILLISEC(100);
      auto ptr = GetErrorStatusFlag::InitSharedPtr(iSlave);
      center->SendMessage_(ptr);
      status = ptr->GetReplyValue();
      std::cout << "GetErrorStatusFlag: " <<
        TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
        << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    } while (!(status & (uint32_t)TMCL::StatusErrorFlags::INITIALIZED));
  }
  {
    auto ptr = GetInitialized::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << "GetInitialized: " <<
      ptr->GetReplyValue()
      << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }

  // If timeout clear it..
  if (status & (uint32_t)TMCL::StatusErrorFlags::TIMEOUT) {
    auto ptr = ClearMotorControllerTimeoutFlag::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << "  ClearMotorControllerTimeoutFlag: " << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }
  {
    auto ptr = GetErrorStatusFlag::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << "GetErrorStatusFlag: " <<
      TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
      << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  {
    auto ptr = GetPosition::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    pos = ptr->GetReplyValue();
    std::cout << " GetPosition: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }











  {
    auto ptr = MotorStop::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << " MotorStop: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  {
    auto ptr = GetErrorStatusFlag::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    status = ptr->GetReplyValue();
    std::cout << "GetErrorStatusFlag: " <<
      TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
      << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  {
    auto ptr = RotateLeft::InitSharedPtr(iSlave, -600);
    center->SendMessage_(ptr);
    std::cout << " RotateRight: " << ptr->GetReplyValue() <<
      " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }

  for (int i = 0; i < 100; i++) {
    {
      auto ptr = GetErrorStatusFlag::InitSharedPtr(iSlave);
      center->SendMessage_(ptr);
      std::cout << "GetErrorStatusFlag: " <<
        TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
        << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    {
      auto ptr = GetCurrent::InitSharedPtr(iSlave);
      center->SendMessage_(ptr);
      std::cout << " GetCurrent: " << (int32_t)ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
      if ((int32_t)ptr->GetReplyValue() > 2000)
        break;
    }
    {
      auto ptr = GetTargetSpeed::InitSharedPtr(iSlave);
      center->SendMessage_(ptr);
      std::cout << " GetTargetSpeed: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    {
      auto ptr = GetActualSpeed::InitSharedPtr(iSlave);
      center->SendMessage_(ptr);
      std::cout << " GetActualSpeed: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    {
      auto ptr = GetPosition::InitSharedPtr(iSlave);
      center->SendMessage_(ptr);
      std::cout << " GetPosition: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
  }

  {
    auto ptr = MotorStop::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << " MotorStop: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }


  return 0;
  
  

  return 0;

  if (0) {
    auto ptr = RotateRight::InitSharedPtr(iSlave, 20);
    center->SendMessage_(ptr);
    std::cout << " RotateRight: " << ptr->GetReplyValue() <<
      " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  } else {
    auto ptr = RotateLeft::InitSharedPtr(iSlave, 20);
    center->SendMessage_(ptr);
    std::cout << " RotateRight: " << ptr->GetReplyValue() <<
      " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  for (int i = 0; i < 300;i++) {

    {
      auto ptr = GetErrorStatusFlag::InitSharedPtr(iSlave);
      center->SendMessage_(ptr);
      std::cout << "GetErrorStatusFlag: " <<
        TMCL::StatusErrorFlagsToString(ptr->GetReplyValue()).c_str()
        << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    {
      auto ptr = GetCurrent::InitSharedPtr(iSlave);
      center->SendMessage_(ptr);
      std::cout << " GetCurrent: " << (int32_t)ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
      if ((int32_t)ptr->GetReplyValue() > 2000)
        break;
    }
    {
      auto ptr = GetActualSpeed::InitSharedPtr(iSlave);
      center->SendMessage_(ptr);
      std::cout << " GetActualSpeed: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
    {
      auto ptr = GetPosition::InitSharedPtr(iSlave);
      center->SendMessage_(ptr);
      std::cout << " GetPosition: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
    }
  }
  
  {
    auto ptr = MotorStop::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << " MotorStop: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }



  {
    auto ptr = GetNeedCalibration::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << " GetNeedCalibration: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  {
    auto ptr = SetIsCalibrated::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << " SetIsCalibrated: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }

  {
    auto ptr = GetNeedCalibration::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << " GetNeedCalibration: " << ptr->GetReplyValue() << " (" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }


  center->CloseConnection();

  return 0;

  /*
  {
    auto ptr = SetMaxCurrent::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << " SetMaxCurrent: " << ptr->GetReplyValue() << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  {
    auto ptr = SetMaxCurrent2::InitSharedPtr(iSlave, 1000);
    center->SendMessage_(ptr);
    std::cout << " SetMaxCurrent2: " << ptr->GetReplyValue() << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  {
    auto ptr = GetMaxCurrent::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << "GetMaxCurrent: " << ptr->GetReplyValue()
      << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  {
    auto ptr = GetMaxCurrent2::InitSharedPtr(iSlave);
    center->SendMessage_(ptr);
    std::cout << "GetMaxCurrent2: " << ptr->GetReplyValue()
      << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }*/
  /*
      {
        auto ptr = GetMotorHaltedVelocity::InitSharedPtr(3);
        center->SendMessage_(ptr);
        std::cout << " GetMotorHaltedVelocity: " << ptr->GetReplyValue() << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
      }
  {
    auto ptr = GetD2ParameterPositionControl::InitSharedPtr(3);
    center->SendMessage_(ptr);
    printf(" ? %lu \n", ptr->GetReplyValue());
    std::cout << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }
  {
    auto ptr = GetD1ParameterPositionControl::InitSharedPtr(3);
    center->SendMessage_(ptr);
    printf(" ? %lu \n", ptr->GetReplyValue());
    std::cout << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << std::endl;
  }*/

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
  {
    auto ptr = GetSupplyVoltage::InitSharedPtr(3);
    center->SendMessage_(ptr);
    std::cout << " GetSupplyVoltage: " << ptr->GetReplyValue() << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }
  {
    auto ptr = GetTemperature::InitSharedPtr(3);
    center->SendMessage_(ptr);
    std::cout << " GetTemperature: " << ConvertTemperature(ptr->GetReplyValue()) << "(" << TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) << ")" << std::endl;
  }*/
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
}