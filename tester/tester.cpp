#include "adapters.hpp"
#include "YoubotManipulator.hpp"
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
  // Get Configfile
  YoubotConfig config(std::string(CONFIG_FOLDER) + "youBotArmConfig_fromKeisler.json");
  //youBotArmConfig_fromfactory.json");
  //youBotArmConfig_fromMoveIt.json");
  //youBotArmConfig_fromKeisler.json");
  config.Init();

  // Initialize logger
  Log::Setup(config.logConfig);

  // Init physical ethercat class
  bool physical = true;
  if (physical) {
    // Find appropriate ethernet adapter and open connection
    char name[1000];
    if (findYouBotEtherCatAdapter(name))
      log(Log::info, "Adapter found:" + std::string(name));
    else {
      log(Log::fatal, "Adapter with turned on youBot arm NOT found!");
      return -1;
    }
    center = EtherCATMaster::CreatePhysical(name);
  }
  else
    center = EtherCATMaster::CreateVirtual();

  YoubotManipulator man(config, center);
  man.InitializeManipulator();
  man.Calibrate();
  
  SLEEP_SEC(1);
  man.CheckAndResetErrorFlags();
  
  GoTowardsZero(man, 5000, 0.05, config);

  //ZeroVelocityDemo(man, 5000);

  // Go to zero and stay there
  //GoToZero(man, 3000);
  
  // Stop at the given position
  //StopThere(man, 10000);
  /*
  // Zero current mode: ~free drive
  FreeDriveMode(man, 3000);*/

  // Wave movement
  //WaveDemo(man, 10000);

  return 0;
}