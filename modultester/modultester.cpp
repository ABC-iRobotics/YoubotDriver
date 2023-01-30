#include "adapters.hpp"
#include "YoubotManipulatorModul.hpp"
#include "Time.hpp"
#include "Logger.hpp"

using namespace youbot;

EtherCATMaster::Ptr center;

int main(int argc, char *argv[])
{
  std::string configpath = std::string(CONFIG_FOLDER) + "youBotArmConfig_fromKeisler.json";
  //youBotArmConfig_fromfactory.json");
  //youBotArmConfig_fromMoveIt.json");
  //youBotArmConfig_fromKeisler.json");

  YoubotManipulatorModul modul(configpath, true);

  modul.StartThreadAndInitialize();

  do {
    SLEEP_SEC(1);

  } while (modul.GetStatus().motion == ManipulatorTask::INITIALIZATION);

  for (int i = 0; i < 100; i++) {
    SLEEP_MILLISEC(10);
    modul.GetStatus().LogStatus();
  }

  modul.StopThread();

  return 0;
}