#include "MTaskStop.hpp"
#include "Time.hpp"

using namespace youbot;

ManipulatorCommand MTaskStop::GetCommand(const JointsState& new_state) {
  return ManipulatorCommand(BLDCCommand::MOTOR_STOP, Eigen::VectorXd(5));
}

MTask::TaskType MTaskStop::GetType() const {
  return MTask::STOPPED;
}

bool MTaskStop::_taskFinished() const {
  return false;
}