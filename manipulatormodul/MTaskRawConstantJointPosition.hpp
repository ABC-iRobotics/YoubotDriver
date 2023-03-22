#ifndef MANIPULATOR_TASK_CONSTANT_JOINT_POSITION_HPP
#define MANIPULATOR_TASK_CONSTANT_JOINT_POSITION_HPP

#include "Config.hpp"
#include "MTask.hpp"
#include "Eigen/dense"

namespace youbot {
  /// <summary>
  /// Task that controls the manipulator to the desired joint coordinates via simple P control
  /// </summary>
  class MTaskRawConstantJointPosition : public MTask {
    const Eigen::VectorXd q_required_rad;
    Eigen::VectorXd P_constants;
    bool finished;

  public:
    MTaskRawConstantJointPosition(const Eigen::VectorXd& q_required_rad);

    ManipulatorCommand GetCommand(const JointsState& new_state) override;

    TaskType GetType() const override;

  protected:
    bool _taskFinished() const override;
  };
}
#endif