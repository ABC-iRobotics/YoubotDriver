#include "adapters.hpp"
#include "Manager.hpp"
#include "MTaskRawConstantJointSpeed.hpp"
#include "MTaskRawConstantJointPosition.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include "MTaskCommutation.hpp"
#include "MTaskCalibration.hpp"
#include "MTaskZeroCurrent.hpp"
#include "MTaskStop.hpp"

using namespace youbot;

EtherCATMaster::Ptr center;

#include <iostream>

int main(int argc, char *argv[]) {
  std::string configpath = std::string(CONFIG_FOLDER) + "youBotArmConfig_fromKeisler.json";
  //youBotArmConfig_fromfactory.json");
  //youBotArmConfig_fromMoveIt.json");
  //youBotArmConfig_fromKeisler.json");

  Config config(configpath); // TODO: disgusting
  config.Init();

  Manager modul(configpath, true);
  modul.StartThreadAndInitialize();
  while (modul.GetStatus().motion != MTask::STOPPED) // while till config and auto tasks end
    SLEEP_MILLISEC(10);

  SLEEP_SEC(3);



  log(Log::error, "STARTED");
  // Create and start a task
  {
    MTask::Ptr task = std::make_shared<MTaskRawConstantJointPosition>(Eigen::VectorXd::Zero(5), &config);
    modul.NewManipulatorTask(task, 10);
    auto start = std::chrono::steady_clock::now();
    do {
      modul.GetStatus().LogStatus();
      SLEEP_MILLISEC(10);
    } while (std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::steady_clock::now() - start).count() < 10);
  }
  // Stop and go home
  modul.StopThread();
  return 0;
  // Create and start a task
  {
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
  }


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