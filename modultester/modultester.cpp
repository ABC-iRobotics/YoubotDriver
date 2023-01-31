#include "adapters.hpp"
#include "YoubotManipulatorModul.hpp"
#include "RawConstantJointSpeedTask.hpp"
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

  YoubotManipulatorModul modul(configpath, false);

  modul.StartThreadAndInitialize();

  // Wait until initialization ends
  do {
    SLEEP_SEC(1);
  } while (modul.GetStatus().motion == ManipulatorTask::INITIALIZATION);

  // Create and start a task
  Eigen::VectorXd dq(5);
  dq << 0.1, 0.1, -0.1, -0.1, 0.1;
  ManipulatorTask::Ptr task = std::make_shared<RawConstantJointSpeedTask>(dq, 10);
  //modul.NewManipulatorTask(task, 5);

  ManipulatorTask::Ptr task2 = std::make_shared<ZeroCurrentManipulatorTask>();
  modul.NewManipulatorTask(task2, 50);
  

  // Lets see what's happening
  for (int i = 0; i < 700; i++) {
    SLEEP_MILLISEC(10);
    modul.GetStatus().LogStatus();
  }

  // Stop and go home
  modul.StopThread();

  return 0;
}