#include "RawConstantJointSpeedTask.hpp"
#include "Time.hpp"

using namespace youbot;

youbot::RawConstantJointSpeedTask::RawConstantJointSpeedTask(const Eigen::VectorXd& dq, double time_limit) :
  dq(dq), time_limit(time_limit) {}

ManipulatorCommand youbot::RawConstantJointSpeedTask::GetCommand(const JointsState& new_state) {
  return ManipulatorCommand(ManipulatorCommand::JOINT_VELOCITY, dq);
}

ManipulatorTask::TaskType youbot::RawConstantJointSpeedTask::GetType() const {
  return TaskType::RAW_CONSTANT_JOINTSPEED;
}

bool youbot::RawConstantJointSpeedTask::_taskFinished() const {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started_at).count() >= time_limit * 1000.;
}
