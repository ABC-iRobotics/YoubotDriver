#include "MTaskZeroCurrent.hpp"
#include "Time.hpp"
#include "MTaskRawConstantJointPosition.hpp"

using namespace youbot;

youbot::MTaskRawConstantJointPosition::MTaskRawConstantJointPosition(
  const Eigen::VectorXd& q_required_rad) : q_required_rad(q_required_rad) {
  // PParameterFirstParametersPositionControl
  P_constants = Eigen::VectorXd(5);
  for (int i = 0; i < 5; i++)
    P_constants[i] = 500. / 4000. * 60.;
  P_constants *= 0.2;
}
#include <iostream>
#include "Logger.hpp"
ManipulatorCommand youbot::MTaskRawConstantJointPosition::GetCommand(const JointsState& new_state) {

  Eigen::VectorXd q(5);
  for (int i = 0; i < 5; i++)
    q[i] = new_state.joint[i].q.value;
  Eigen::VectorXd e = q_required_rad - q;
  //e.cwiseAbs
  finished = (e.cwiseAbs().maxCoeff() < 0.01);
  Eigen::VectorXd v_d = P_constants.cwiseProduct(e);
  log(Log::info, "Move to position: ");
  for (int i = 0; i < 5; i++)
    log(Log::info, std::to_string(v_d[i]));
  return ManipulatorCommand(BLDCCommand::JOINT_VELOCITY, v_d);
}

MTask::TaskType youbot::MTaskRawConstantJointPosition::GetType() const {
  return RAW_CONSTANT_JOINTPOSITON;
}

bool youbot::MTaskRawConstantJointPosition::_taskFinished() const {
  return finished;
}
