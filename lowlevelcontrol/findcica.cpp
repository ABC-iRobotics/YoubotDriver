#include <stdio.h>
#include <iostream>
#include <string.h>
#include <string>

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
    if (findYouBotEtherCatAdapter(name))
      log(Log::info, "Adapter found:" + std::string(name));
    else {
      log(Log::fatal, "Adapter with turned on youBot arm NOT found!");
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

  //std::cout << "Time difference = " <<
  //  std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

  /*
  if (man.CheckJointConfigs())
    std::cout << "OK!!" << std::endl;
  */

  man.InitializeAllJoints();
  for (int i = 0; i < 2; i++) {
    man.GetJoint(i).ResetTimeoutViaMailbox();
    man.GetJoint(i).ResetI2TExceededViaMailbox();
  }

  man.Calibrate();
  
  SLEEP_SEC(1);

  for (int i = 0; i < 5; i++)
    man.GetJoint(i).ResetI2TExceededViaMailbox();
  for (int i = 0; i < 5; i++)
    man.GetJoint(i).ResetTimeoutViaMailbox();

  man.ReqJointPosition(0, 0, 0, 0, 0);
  while (1) {
    center->ExchangeProcessMsg();
    man.GetJoint(2).GetProcessReturnData().Print();
    SLEEP_MILLISEC(10);
  }

  center->CloseConnection();

  return 0;

}