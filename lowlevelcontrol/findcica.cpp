#include "adapters.hpp"
#include "YoubotManipulator.hpp"
#include "Time.hpp"
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

  YoubotManipulator man(config, VMessageCenter::GetSingleton());
  
  man.ConfigJoints();
  /*
  if (man.CheckJointConfigs())
    std::cout << "OK!!" << std::endl;
  */
  man.InitializeAllJoints();
  man.Calibrate();
  
  SLEEP_SEC(1);

  for (int i = 0; i < 5; i++)
    man.GetJoint(i)->ResetI2TExceededViaMailbox();
  for (int i = 0; i < 5; i++)
    man.GetJoint(i)->ResetTimeoutViaMailbox();

  auto center = VMessageCenter::GetSingleton();

  man.ReqJointPosition(0, 0, 0, 0, 0);
  while (1) {
    center->ExchangeProcessMsg();
    man.GetJoint(2)->GetProcessReturnData().Print();
    SLEEP_MILLISEC(10);
  }

  center->CloseConnection();
  return 0;
}