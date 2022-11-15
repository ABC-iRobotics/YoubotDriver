#include <stdio.h>
#include <iostream>
#include <string.h>

#include "adapters.hpp"
#include "Time.hpp"
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
  for (int i = 0; i < 2; i++) {
    man.GetJoint(i).ResetTimeoutViaMailbox();
    man.GetJoint(i).ResetI2TExceededViaMailbox();
  }

  man.GetJoint(0).ReqVelocityMotorRPM(250);

  for (int i = 0; i < 100; i++) {
    center->ExchangeProcessMsg();
    man.GetJoint(0).GetProcessReturnData().Print();
    SLEEP_MILLISEC(10);
  }

  man.GetJoint(0).ReqMotorStopViaProcess();
  std::cout << "STOP sent" << std::endl;
  for (int i = 0; i < 100; i++) {
    center->ExchangeProcessMsg();
    man.GetJoint(0).GetProcessReturnData().Print();
    SLEEP_MILLISEC(10);
  }

  std::cout << "Current via mailbox: " << man.GetJoint(0).GetCurrentAViaMailbox() << std::endl;
  return 0;

  for (int i = 0; i < 100; i++) {
    SLEEP_MILLISEC(10);
    center->ExchangeProcessMsg(); // TODO save timestamp
  }

  ProcessBuffer sg2(20);
  center->GetProcessMsg(sg2, 2);
  sg2.Print();

  return 0;
  
  center->CloseConnection();

  return 0;

}