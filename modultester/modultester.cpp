#include "adapters.hpp"
#include "YoubotManipulatorModul.hpp"
#include "Time.hpp"
#include "Logger.hpp"

using namespace youbot;

EtherCATMaster::Ptr center;

void GoToZero(YoubotManipulator& man, int time_ms) {
  log(Log::info, "Goint to zero started!");
  auto start = std::chrono::steady_clock::now();
  man.ReqJointPositionRad(0, 0, 0, 0, 0);
  do {
    center->ExchangeProcessMsg();
    man.LogStatusProcess();
    SLEEP_MILLISEC(5)
  } while (std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count() < time_ms);
}

void StopThere(YoubotManipulator& man, int time_ms) {
  log(Log::info, "Stopped!");
  auto start = std::chrono::steady_clock::now();
  man.ReqManipulatorStop();
  do {
    center->ExchangeProcessMsg();
    man.LogStatusProcess();
    SLEEP_MILLISEC(5)
  } while (std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count() < time_ms);
}

void GoTowardsZero(YoubotManipulator& man, int time_ms, double velocityRadPerSec, YoubotConfig& config) {
  log(Log::info, "GoTowardsZero!");
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 5; i++) {
    bool backward = bool(config.jointConfigs[i].at("CalibrationDirection"))
      ^ bool(config.jointConfigs[i].at("qDirectionSameAsEnc"));
    man.GetJoint(i)->ReqJointSpeedRadPerSec(backward ? velocityRadPerSec : -velocityRadPerSec);
  }
  do {
    center->ExchangeProcessMsg();
    man.LogStatusProcess();
    SLEEP_MILLISEC(5)
  } while (std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count() < time_ms);
}

void FreeDriveMode(YoubotManipulator& man, int time_ms) {
  log(Log::info, "FreeDriveMode!");
  auto start = std::chrono::steady_clock::now();
  man.ReqJointTorqueNm(0, 0, 0, 0, 0);
  do {
    center->ExchangeProcessMsg();
    man.LogStatusProcess();
    SLEEP_MILLISEC(5)
  } while (std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count() < time_ms);
}

void WaveDemo(YoubotManipulator& man, int time_ms) {
  log(Log::info, "WaveDemo!");
  auto start = std::chrono::steady_clock::now();
  do {
    int dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start).count();
    double t = double(dt_ms) / 1000.;
    man.ReqJointPositionRad(10. / 180. * M_PI * sin(2. * M_PI / 1. * t),
      12. / 180. * M_PI * sin(2. * M_PI / 3.5 * t),
      14. / 180. * M_PI * sin(2. * M_PI / 4. * t),
      16. / 180. * M_PI * sin(2. * M_PI / 4.5 * t),
      18. / 180. * M_PI * sin(2. * M_PI / 5. * t));
    center->ExchangeProcessMsg();
    man.LogStatusProcess();
    SLEEP_MILLISEC(5)
  } while (std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count() < time_ms);
}

void ZeroVelocityDemo(YoubotManipulator& man, int time_ms) {
  log(Log::info, "ZeroVelocityDemo!");
  auto start = std::chrono::steady_clock::now();
  man.ReqJointSpeedRadPerSec(0, 0, 0, 0, 0);
  do {
    center->ExchangeProcessMsg();
    man.LogStatusProcess();
    SLEEP_MILLISEC(5)
  } while (std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start).count() < time_ms);
}

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

  } while (modul.GetStatus().motion == YoubotManipulatorMotionLayer::INITIALIZATION);
  SLEEP_SEC(3);//wait in the thread
  modul.StopThread();


  return 0;
}