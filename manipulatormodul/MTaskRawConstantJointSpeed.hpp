#ifndef RAWCONSTANTJOINTSPEED_TASK_HPP
#define RAWCONSTANTJOINTSPEED_TASK_HPP

#include "MTask.hpp"

namespace youbot {

  /// <summary>
  /// Task that communicates the same joint speed constantly
  /// </summary>
  class MTaskRawConstantJointSpeed : public MTask {
  public:
    MTaskRawConstantJointSpeed(const Eigen::VectorXd& dq, double time_limit);;

    ManipulatorCommand GetCommand(
      const JointsState& new_state) override;

    TaskType GetType() const override;

  private:
    bool _taskFinished() const override;

    const Eigen::VectorXd dq;
    const double time_limit;
  };
}
#endif
