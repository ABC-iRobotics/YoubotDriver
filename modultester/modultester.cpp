#include "adapters.hpp"
#include "Manager.hpp"
#include "MTaskRawConstantJointSpeed.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include "MTaskCommutation.hpp"
#include "MTaskCalibration.hpp"

using namespace youbot;

EtherCATMaster::Ptr center;

int main(int argc, char *argv[])
{
  std::string configpath = std::string(CONFIG_FOLDER) + "youBotArmConfig_fromKeisler.json";
  //youBotArmConfig_fromfactory.json");
  //youBotArmConfig_fromMoveIt.json");
  //youBotArmConfig_fromKeisler.json");

  Manager modul(configpath, true);

  modul.StartThreadAndInitialize();

  std::string sg = modul.GetStatus().manipulatorStatus.ToString();
  // Wait until configuration ends
  do {
    SLEEP_MILLISEC(10);
  } while (!modul.GetStatus().manipulatorStatus.IsConfigurated());
  std::string sg2 = modul.GetStatus().manipulatorStatus.ToString();

  // Commutation initialization
  {
    MTask::Ptr task0 = std::make_shared<MTaskCommutation>();
    modul.NewManipulatorTask(task0, 5);
    // Wait until initialization ends
    do {
      SLEEP_MILLISEC(10);
      modul.GetStatus().LogStatus();
    } while (modul.GetStatus().motion == MTask::INITIALIZATION);
  }
  
  // Free drive
  {
    MTask::Ptr task2 = std::make_shared<ZeroCurrentManipulatorTask>();
    modul.NewManipulatorTask(task2, 50);
    for (int i = 0; i < 7000; i++) {
      SLEEP_MILLISEC(10);
      modul.GetStatus().LogStatus();
    }
  }

  // Create and start a task
  Eigen::VectorXd dq(5);
  dq << 0.1, 0.1, -0.1, 0.1, -0.1;
  MTask::Ptr task = std::make_shared<MTaskRawConstantJointSpeed>(dq, 10);
  modul.NewManipulatorTask(task, 5);

  
  //modul.NewManipulatorTask(task2, 50);
  
  // Lets see what's happening
  for (int i = 0; i < 700; i++) {
    SLEEP_MILLISEC(10);
    modul.GetStatus().LogStatus();
  }

  // Stop and go home
  modul.StopThread();

  return 0;
}