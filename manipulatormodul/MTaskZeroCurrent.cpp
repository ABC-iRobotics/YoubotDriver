#include "MTaskZeroCurrent.hpp"
#include "Time.hpp"

using namespace youbot;

ManipulatorCommand MTaskZeroCurrent::GetCommand(const JointsState& new_state) {
  Eigen::VectorXd tau(5);
  tau << 0, 0, 0, 0, 0;
  return ManipulatorCommand(BLDCCommand::JOINT_TORQUE, tau);
}

MTask::TaskType MTaskZeroCurrent::GetType() const {
  return MTask::ZERO_CURRENT;
}

bool MTaskZeroCurrent::_taskFinished() const {
  return false;
}