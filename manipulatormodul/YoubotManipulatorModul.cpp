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
