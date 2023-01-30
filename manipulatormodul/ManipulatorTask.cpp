#include "ManipulatorTask.hpp"
#include "Time.hpp"

using namespace youbot;


std::string youbot::ManipulatorTask::Type2String(TaskType type) {
  switch (type)
  {
  case youbot::ManipulatorTask::INITIALIZATION:
    return "INITIALIZATION";
  case youbot::ManipulatorTask::STOPPED:
    return "STOPPED";
  case youbot::ManipulatorTask::RAW_CONSTANT_JOINTSPEED:
    return "CONSTANT_JOINTSPEED";
  default:
    return "Conversion not implemented";
  }
}

youbot::ManipulatorCommand::ManipulatorCommand(Type type, const Eigen::VectorXd& value)
    : type(type), value(value) {}

// Called by the motion layer

void youbot::ManipulatorTask::Initialize(const JointsState& start_state_) { // Called by the motion layer
  start_state = start_state;
  started_at = std::chrono::steady_clock::now();
  started = true;
}

bool youbot::ManipulatorTask::Finished() const {
  if (!started)
    return false;
  return _taskFinished();
}
