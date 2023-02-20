#include "Module.hpp"
#include <stdexcept>
#include "Logger.hpp"
#include "Time.hpp"

using namespace youbot;

youbot::MotionLayer::Status youbot::Module::GetStatus() const {
  if (man != nullptr)
	return man->GetStatus();
  else
	return {};
}

Module::~Module() {
  if (threadrunning)
    StopThread(false);
}

void Module::StartThreadAndInitialize() {
  threadrunning = true;
  t = std::thread([this] { _thread(configfilepath, virtual_); });
  t.detach();
}

void Module::StopThread(bool waitin) {
  if (threadrunning) {
    man->StopTask();
    threadtostop = true;
    if (waitin) {
      do {
        SLEEP_MILLISEC(3);
      } while (threadrunning);
    }
  }
}

Eigen::VectorXd youbot::Module::GetTrueStatus() const {
  if (man != nullptr)
    return man->GetTrueStatus();
  else
    return Eigen::VectorXd(5);
}

youbot::Module::Module(
  const std::string& configfilepath, bool virtual_)
  : configfilepath(configfilepath), virtual_(virtual_) {}

void Module::NewManipulatorTask(ManipulatorTask::Ptr task, double time_limit) {
  std::lock_guard<std::mutex> guard(new_task_mutex);
  new_man_task = { task, time_limit };
  if (man != nullptr)
    man->StopTask();
}

#include <iostream>
void Module::_thread(const std::string& configfilepath, bool virtual_) {
  try {
    ManipulatorTask::Ptr idle_ptr = std::make_shared<IdleManipulatorTask>();
    threadtostop = false;
    if (man == nullptr)
      man = std::make_unique<MotionLayer>(configfilepath, virtual_);
    man->Initialize();

    // Command based operation, checking stop...
    while (!threadtostop) {
      NewTask new_man_task_;
      {
        std::lock_guard<std::mutex> guard(new_task_mutex);
        new_man_task_ = new_man_task;
        new_man_task = {};
      }
      if (new_man_task_.ptr == nullptr)
        new_man_task_ = { idle_ptr, 0.1 };
      man->DoTask(new_man_task_.ptr, new_man_task_.time_limit);
    }
  }
  catch (const std::runtime_error& error) {
    log(Log::fatal, "Error in the spearated thread: " + std::string(error.what()) + "Thread stopped");
    std::cout << "Error in the spearated thread: " + std::string(error.what()) + " (Thread stopped)" << std::endl;
    SLEEP_MILLISEC(100);
  }
  threadrunning = false;
}
