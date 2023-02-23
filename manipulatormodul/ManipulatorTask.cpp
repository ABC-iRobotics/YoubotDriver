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
  return ManipulatorCommand(BLDCCommand::MOTOR_STOP, Eigen::VectorXd(5));
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
  return ManipulatorCommand(BLDCCommand::JOINT_TORQUE, tau);
}

ManipulatorTask::TaskType ZeroCurrentManipulatorTask::GetType() const {
  return ManipulatorTask::ZERO_CURRENT;
}

bool ZeroCurrentManipulatorTask::_taskFinished() const {
  return false;
}

ManipulatorCommand youbot::InitializeCommutationManipulatorTask::GetCommand(const JointsState& new_state) {
    // Check if the commutation should have finished - if not then run to error
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started_at).count() > 2000)
        throw std::runtime_error("Unsuccessful commutation too much time elapsed");
    // Check the joint states and call initialize/stop messages
    finished = true;
    BLDCCommand cmd[5];
    for (int i = 0; i < 5; i++) {
        bool inited = new_state.joint[i].status.value.Initialized();
        finished &= inited;
        if (inited)
            cmd[i] = BLDCCommand(BLDCCommand::MOTOR_STOP, 0);
        else
            cmd[i] = BLDCCommand(BLDCCommand::INITIALIZE_COMMUTATION, 0);
    }
    // Send out the resulting commands
    return { cmd[0], cmd[1], cmd[2], cmd[3], cmd[4] };
}

TaskType youbot::InitializeCommutationManipulatorTask::GetType() const {
  return INITIALIZATION;
}

bool youbot::InitializeCommutationManipulatorTask::_taskFinished() const {
  return finished;
}
