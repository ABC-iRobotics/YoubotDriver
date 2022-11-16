#include <stdio.h>
#include <iostream>
#include <string.h>

#include "adapters.hpp"
#include "Time.hpp"
#include "YoubotManipulator.hpp"

#include "Logger.hpp"

int main(int argc, char *argv[])
{
  // Get Configfile
  YoubotConfig config("C:/Users/kutij/Desktop/myYouBotDriver/src/lowlevelcontrol/youBotArmConfig_fromKeisler.json");
    //youBotArmConfig_fromfactory.json");
    //youBotArmConfig_fromMoveIt.json");
    //youBotArmConfig_fromKeisler.json");
  
  // Initialize logger
  Log::Setup(config.logConfig);

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

  // Find appropriate ethernet adapter and open connection
  {
    char name[1000];
    printf("sg\n");
    if (findYouBotEtherCatAdapter(name))
      printf("\n\n\nAdapter found: %s\n", name);
    else {
      printf("\n\n\nAdapter with turned on youBot arm NOT found!\n");
      return -1;
    }
    if (!VMessageCenter::GetSingleton()->OpenConnection(name))
      return -1;
  }

  // Get Message center
  auto center = VMessageCenter::GetSingleton();
 
  YoubotManipulator man(config, center);
  
  man.ConfigJoints();

  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

  std::cout << "Time difference = " <<
    std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

  return 0;
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