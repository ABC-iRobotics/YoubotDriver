#include "adapters.hpp"
#include "Manager.hpp"
#include "MTaskRawConstantJointSpeed.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include "MTaskCommutation.hpp"
#include "MTaskCalibration.hpp"
#include "MTaskZeroCurrent.hpp"
#include "MTaskStop.hpp"

using namespace youbot;

EtherCATMaster::Ptr center;

int main(int argc, char *argv[])
{
  std::string configpath = std::string(CONFIG_FOLDER) + "youBotArmConfig_fromKeisler.json";
  //youBotArmConfig_fromfactory.json");
  //youBotArmConfig_fromMoveIt.json");
  //youBotArmConfig_fromKeisler.json");

  Manager modul(configpath, false);

  modul.StartThreadAndInitialize();

  std::string sg = modul.GetStatus().manipulatorStatus.ToString();
  // Wait until configuration ends
  do {
    SLEEP_MILLISEC(10);
  } while (!modul.GetStatus().manipulatorStatus.IsConfigurated());

  // Wait until commutation and calibration ends
  do {
    SLEEP_MILLISEC(10);
  } while (!modul.GetStatus().manipulatorStatus.IsCalibrated());

  // Create and start a task
  Eigen::VectorXd dq(5);
  dq << 0.1, 0.1, -0.1, 0.1, -0.1;
  MTask::Ptr task = std::make_shared<MTaskRawConstantJointSpeed>(dq, 10);
  modul.NewManipulatorTask(task, 5);

  auto start = std::chrono::steady_clock::now();
  do {
    //modul.GetStatus().LogStatus();
    SLEEP_MILLISEC(10);
  } while (std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::steady_clock::now() - start).count() < 5);



  // Stop and go home
  modul.StopThread();

  return 0;

 

  
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