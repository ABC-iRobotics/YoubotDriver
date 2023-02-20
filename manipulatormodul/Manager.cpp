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
