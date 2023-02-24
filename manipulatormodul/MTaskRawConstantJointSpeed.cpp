#include "MTaskRawConstantJointSpeed.hpp"
#include "Time.hpp"

using namespace youbot;

youbot::MTaskRawConstantJointSpeed::MTaskRawConstantJointSpeed(const Eigen::VectorXd& dq, double time_limit) :
  dq(dq), time_limit(time_limit) {}

ManipulatorCommand youbot::MTaskRawConstantJointSpeed::GetCommand(const JointsState& new_state) {
  return ManipulatorCommand(BLDCCommand::JOINT_VELOCITY, dq);
}

MTask::TaskType youbot::MTaskRawConstantJointSpeed::GetType() const {
  return TaskType::RAW_CONSTANT_JOINTSPEED;
}

bool youbot::MTaskRawConstantJointSpeed::_taskFinished() const {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started_at).count() >= time_limit * 1000.;
}
