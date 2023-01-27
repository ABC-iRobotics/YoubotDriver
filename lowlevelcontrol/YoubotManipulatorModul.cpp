#include "YoubotManipulatorModul.hpp"
#include "adapters.hpp"
#include "YoubotJointVirtual.hpp"
#include <stdexcept>
#include "Logger.hpp"
#include "Time.hpp"

using namespace youbot;

void youbot::YoubotManipulatorModul::AddCommand(const Command& c) {
  std::lock_guard<std::mutex> lock(command_mutex);
  saved_commands.push_back(c);
}

youbot::YoubotManipulatorModul::Status youbot::YoubotManipulatorModul::GetStatus() {
  std::lock_guard<std::mutex> lock(status_mutex);
  return status;
}

void youbot::YoubotManipulatorModul::GetTrueStatus(double& q0, double& q1, double& q2, double& q3, double& q4) {
  std::lock_guard<std::mutex> lock(status_mutex);
  if (center->GetType() == center->VIRTUAL) {
    q0 = std::dynamic_pointer_cast<YoubotJointVirtual>(man->GetJoint(0))->GetJointPositionTRUE();
    q1 = std::dynamic_pointer_cast<YoubotJointVirtual>(man->GetJoint(1))->GetJointPositionTRUE();
    q2 = std::dynamic_pointer_cast<YoubotJointVirtual>(man->GetJoint(2))->GetJointPositionTRUE();
    q3 = std::dynamic_pointer_cast<YoubotJointVirtual>(man->GetJoint(3))->GetJointPositionTRUE();
    q4 = std::dynamic_pointer_cast<YoubotJointVirtual>(man->GetJoint(4))->GetJointPositionTRUE();
  }
  else {
    q0 = man->GetJoint(0)->GetQLatestRad().value;
    q1 = man->GetJoint(1)->GetQLatestRad().value;
    q2 = man->GetJoint(2)->GetQLatestRad().value;
    q3 = man->GetJoint(3)->GetQLatestRad().value;
    q4 = man->GetJoint(4)->GetQLatestRad().value;
  }
}

youbot::YoubotManipulatorModul::YoubotManipulatorModul(const std::string& configfilepath, bool virtual_) {
  // Get Configfile
  YoubotConfig config(configfilepath);
  // Initialize logger
  Log::Setup(config.logConfig);
  // Initialize etherCAT bus
  if (virtual_)
    center = EtherCATMaster::CreateVirtual();
  else {
    center = EtherCATMaster::CreatePhysical();
    // Find appropriate ethernet adapter and open connection
    char name[1000];
    if (findYouBotEtherCatAdapter(name))
      log(Log::info, "Adapter found:" + std::string(name));
    else {
      log(Log::fatal, "Adapter with turned on youBot arm NOT found!");
      throw std::runtime_error("Adapter with turned on youBot arm NOT found!");
    }
    if (!center->OpenConnection(name))
      throw std::runtime_error("Connection couldn't be opened!");
  }
  // Initialize manipulator
  man = std::make_unique<YoubotManipulator>(config, center);
  man->InitializeManipulator();
  man->Calibrate();
}
