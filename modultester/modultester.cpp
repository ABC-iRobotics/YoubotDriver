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

  bool virtual_robot = true;
  Manager modul(configpath, virtual_robot);
  modul.StartThreadAndInitialize();
  // wait till config and auto tasks finish
  {
    auto start = std::chrono::steady_clock::now();
    bool finished;
    do {
      SLEEP_MILLISEC(10);
      if (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start).count() > 100)
        throw std::runtime_error("Overtime during initialization.");
      auto status = modul.GetStatus();
      finished = (status.motion == MTask::STOPPED) // there is no task in progress
        && (status.manipulatorStatus.IsCalibrated()) // it is calibrated
        && (status.manipulatorStatus.IsCommutationInitialized()); // it is commutated
    } while (!finished);
  }
  
  log(Log::info, "STARTED");
  // Create and start a task
  if (1) {
    MTask::Ptr task = std::make_shared<MTaskRawConstantJointPosition>(Eigen::VectorXd::Zero(5));
    modul.NewManipulatorTask(task, 1e8);
    auto start = std::chrono::steady_clock::now();
    do {
      auto status = modul.GetStatus();
      log(Log::info, "q1: " + std::to_string(status.joint[0].q.value));
      status.LogStatus();
      //modul.GetStatus().LogStatus();
      SLEEP_MILLISEC(10);
    } while (!task->Finished());
    // Stop and go home
    modul.StopThread();
    return 0;
  }
 
  
  // Create and start a task
  if (0) {
    MTask::Ptr task = std::make_shared<MTaskZeroCurrent>();
    modul.NewManipulatorTask(task, 1e8);
    auto start = std::chrono::steady_clock::now();
    do {
      auto status = modul.GetStatus();
      log(Log::info, "tau1: " + std::to_string(status.joint[0].tau.value));
      //modul.GetStatus().LogStatus();
      SLEEP_MILLISEC(10);
    } while (!task->Finished());
    // Stop and go home
    modul.StopThread();
    return 0;
  }

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