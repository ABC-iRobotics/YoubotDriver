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

ManipulatorCommand IdleManipulatorTask::GetCommand(const JointsState& new_state) {
  Eigen::VectorXd dq(5);
  dq << 0, 0, 0, 0, 0;
  return ManipulatorCommand(ManipulatorCommand::JOINT_VELOCITY, dq);
}

ManipulatorTask::TaskType IdleManipulatorTask::GetType() const {
  return ManipulatorTask::STOPPED;
}

bool IdleManipulatorTask::_taskFinished() const {
  return false;
}

ManipulatorCommand ZeroCurrentManipulatorTask::GetCommand(const JointsState& new_state) {
  Eigen::VectorXd tau(5);
  tau << 0, 0, 0, 0, 0;
  return ManipulatorCommand(ManipulatorCommand::JOINT_TORQUE, tau);
}

ManipulatorTask::TaskType ZeroCurrentManipulatorTask::GetType() const {
  return ManipulatorTask::ZERO_CURRENT;
}

bool ZeroCurrentManipulatorTask::_taskFinished() const {
  return false;
}