#include "Manager.hpp"
#include <stdexcept>
#include "Logger.hpp"
#include "Time.hpp"

using namespace youbot;

youbot::MotionLayer::Status youbot::Manager::GetStatus() const {
  if (man != nullptr)
	return man->GetStatus();
  else
	return {};
}

Manager::~Manager() {
  if (threadrunning)
    StopThread(false);
}

void Manager::StartThreadAndInitialize() {
  threadrunning = true;
  t = std::thread([this] { _thread(configfilepath, virtual_); });
  t.detach();
}

void Manager::StopThread(bool waitin) {
  if (threadrunning) {
    man->StopManipulatorTask();
    threadtostop = true;
    if (waitin) {
      do {
        SLEEP_MILLISEC(3);
      } while (threadrunning);
    }
  }
}

Eigen::VectorXd youbot::Manager::GetTrueStatus() const {
  if (man != nullptr)
    return man->GetTrueStatus();
  else
    return Eigen::VectorXd(5);
}

youbot::Manager::Manager(
  const std::string& configfilepath, bool virtual_)
  : configfilepath(configfilepath), virtual_(virtual_) {}

void Manager::NewManipulatorTask(ManipulatorTask::Ptr task, double time_limit) {
  std::lock_guard<std::mutex> guard(new_task_mutex);
  new_man_task = { task, time_limit };
  if (man != nullptr)
    man->StopManipulatorTask();
}

#include <iostream>
void Manager::_thread(const std::string& configfilepath, bool virtual_) {
  try {
    ManipulatorTask::Ptr idle_ptr = std::make_shared<IdleManipulatorTask>();
    threadtostop = false;
    if (man == nullptr)
      man = std::make_unique<MotionLayer>(configfilepath, virtual_);
    // Check if auto commutation and autocalibration is necessary
    bool autocommutation = false;
    bool autocalibration = false;
    {
      Config config(configfilepath);
      config.Init();
      if (config.manipulatorConfig.find("AutoCommutation") != config.manipulatorConfig.end())
        if (config.manipulatorConfig.at("AutoCommutation").compare("true") == 0) {
          autocommutation = true;
          if (config.manipulatorConfig.find("AutoCalibration") != config.manipulatorConfig.end())
            if (config.manipulatorConfig.at("AutoCalibration").compare("true") == 0)
              autocalibration = true;
        }
    }
    // Initialize the manipulator (parameters, config, ...)
    man->Initialize();
    // Command based operation, checking stop...
    while (!threadtostop) { // How can I stop commutation OR calibration? it will stop it again...
      NewTask new_man_task_;
      // if auto commutation then start it as soon as possible
      if (autocommutation) {
        new_man_task_ = { std::make_shared<InitializeCommutationManipulatorTask>() , 5 };
        autocommutation = false;
      }
      // if autocalibration and initialized then do it or delete the need of autocalibration
      if (new_man_task_.ptr == nullptr && autocalibration) {
        if (man->GetStatus().manipulatorStatus.IsCommutationInitialized())
          new_man_task_ = { std::make_shared<CalibrateManipulatorTask>() , 15 };
        autocalibration = false;
      }
      if (new_man_task_.ptr == nullptr) {
        std::lock_guard<std::mutex> guard(new_task_mutex);
        new_man_task_ = new_man_task;
        new_man_task = {};
      }
      if (new_man_task_.ptr == nullptr)
        new_man_task_ = { idle_ptr, 0.1 };
      man->DoManipulatorTask(new_man_task_.ptr, new_man_task_.time_limit);
    }
  }
  catch (const std::runtime_error& error) {
    log(Log::fatal, "Error in the spearated thread: " + std::string(error.what()) + "Thread stopped");
    std::cout << "Error in the spearated thread: " + std::string(error.what()) + " (Thread stopped)" << std::endl;
    SLEEP_MILLISEC(100);
  }
  threadrunning = false;
}
