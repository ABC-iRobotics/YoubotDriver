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

int main(int argc, char *argv[])
{
  std::string configpath("C:/Users/kutij/Desktop/myYouBotDriver/src/lowlevelcontrol/youBotArmConfig_fromKeisler.json");

  Manager modul(configpath, false);
  modul.StartThreadAndInitialize();
  while (modul.GetStatus().motion != MTask::STOPPED) // while till config and auto tasks end
    SLEEP_MILLISEC(10);

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