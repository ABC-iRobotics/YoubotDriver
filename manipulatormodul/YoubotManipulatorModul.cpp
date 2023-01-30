#include "YoubotManipulatorModul.hpp"
#include <stdexcept>
#include "Logger.hpp"
#include "Time.hpp"

using namespace youbot;

youbot::YoubotManipulatorMotionLayer::Status youbot::YoubotManipulatorModul::GetStatus() const {
  if (man)
	return man->GetStatus();
  else
	return {};
}

YoubotManipulatorModul::~YoubotManipulatorModul() {
  if (threadrunning)
    StopThread(false);
}

void YoubotManipulatorModul::StartThreadAndInitialize() {
  threadrunning = true;
  t = std::thread([this] { _thread(configfilepath, virtual_); });
  t.detach();
}

void YoubotManipulatorModul::StopThread(bool waitin) {
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

Eigen::VectorXd youbot::YoubotManipulatorModul::GetTrueStatus() const {
  return man->GetTrueStatus();
}

youbot::YoubotManipulatorModul::YoubotManipulatorModul(
  const std::string& configfilepath, bool virtual_)
  : configfilepath(configfilepath), virtual_(virtual_) {}

void YoubotManipulatorModul::NewManipulatorTask(ManipulatorTask::Ptr task, double time_limit) {
  std::lock_guard<std::mutex> guard(new_task_mutex);
  new_man_task = { task, time_limit };
  man->StopTask();
}

#include <iostream>
void YoubotManipulatorModul::_thread(const std::string& configfilepath, bool virtual_) {
  try {
    ManipulatorTask::Ptr idle_ptr = std::make_shared<IdleManipulatorTask>();
    threadtostop = false;
    if (!man)
      man = std::make_unique<YoubotManipulatorMotionLayer>(configfilepath, virtual_);
    man->Initialize();

    // Command based operation, checking stop...
    while (!threadtostop) {
      NewTask new_man_task_;
      {
        std::lock_guard<std::mutex> guard(new_task_mutex);
        new_man_task_ = new_man_task;
        new_man_task = {};
      }
      if (new_man_task_.ptr != nullptr)
        man->DoTask(new_man_task_.ptr, new_man_task_.time_limit);
      else
        man->DoTask(idle_ptr, 0.1);
    }
  }
  catch (const std::runtime_error& error) {
    log(Log::fatal, "Error in the spearated thread: " + std::string(error.what()) + "Thread stopped");
    std::cout << "Error in the spearated thread: " + std::string(error.what()) + " (Thread stopped)" << std::endl;
    SLEEP_MILLISEC(100);
  }
  threadrunning = false;
}
