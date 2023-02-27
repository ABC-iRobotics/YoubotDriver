#include "MTask.hpp"
#include "Time.hpp"

using namespace youbot;

std::string youbot::MTask::Type2String(TaskType type) {
  switch (type)
  {
  case youbot::MTask::COMMUTATION:
    return "COMMUTATION";
  case youbot::MTask::STOPPED:
    return "STOPPED";
  case youbot::MTask::RAW_CONSTANT_JOINTSPEED:
    return "CONSTANT_JOINTSPEED";
  default:
    return "Conversion not implemented";
  }
}

youbot::ManipulatorCommand::ManipulatorCommand(BLDCCommand::Type type,
  const Eigen::VectorXd& value) {
  for (int i = 0; i < 5; i++)
    commands[i] = { type, value[i] };
}

youbot::ManipulatorCommand::ManipulatorCommand(BLDCCommand::Type type,
  const Eigen::VectorXi& value) {
  for (int i = 0; i < 5; i++)
    commands[i] = { type, value[i] };
}

youbot::ManipulatorCommand::ManipulatorCommand(const BLDCCommand& cmd0, const BLDCCommand& cmd1, const BLDCCommand& cmd2, const BLDCCommand& cmd3, const BLDCCommand& cmd4) {
  commands[0] = cmd0;
  commands[1] = cmd1;
  commands[2] = cmd2;
  commands[3] = cmd3;
  commands[4] = cmd4;
}

// Called by the motion layer

void youbot::MTask::Initialize(const JointsState& start_state_) { // Called by the motion layer
  start_state = start_state;
  started_at = std::chrono::steady_clock::now();
  started = true;
}

bool youbot::MTask::Finished() const {
  if (!started)
    return false;
  return _taskFinished();
}