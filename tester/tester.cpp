#include "adapters.hpp"
#include "YoubotManipulator.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include "Eigen/dense" // just to test it...

using namespace youbot;

int main(int argc, char *argv[])
{
  // Get Configfile
  YoubotConfig config(std::string(CONFIG_FOLDER) + "youBotArmConfig_fromKeisler.json");
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
    if (!EtherCATMaster::GetSingleton()->OpenConnection(name))
      return -1;
  }

  YoubotManipulator man(config, EtherCATMaster::GetSingleton());
  man.ConfigJoints();
  // if (man.CheckJointConfigs())  std::cout << "OK!!" << std::endl;
  man.InitializeAllJoints();
  man.Calibrate();
  
  SLEEP_SEC(1);
  man.ResetErrorFlags();
  EtherCATMaster::GetSingleton()->StartProcessThread(30);

  man.ReqJointPositionRad(0, 0, 0, 0, 0);
  SLEEP_SEC(3);
  for (int j = 0; j < 1000;j++) {

    man.ReqJointPositionRad(10./180.*M_PI * sin(2. * M_PI / 100. * double(j)),
      12. / 180. * M_PI * sin(2. * M_PI / 150. * double(j)),
      14. / 180. * M_PI * sin(2. * M_PI / 200. * double(j)),
      16. / 180. * M_PI * sin(2. * M_PI / 250. * double(j)),
      18. / 180. * M_PI * sin(2. * M_PI / 300. * double(j)));

    log(Log::debug,"new data\n");
    for (int i = 0; i < 5; i++)
      man.GetJoint(i)->GetProcessReturnData().Print();
    SLEEP_MILLISEC(10);
  }

  EtherCATMaster::GetSingleton()->StopProcessThread();

  return 0;
}