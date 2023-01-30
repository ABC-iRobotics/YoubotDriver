#ifndef RAWCONSTANTJOINTSPEED_TASK_HPP
#define RAWCONSTANTJOINTSPEED_TASK_HPP

#include "ManipulatorTask.hpp"

namespace youbot {

  class RawConstantJointSpeedTask : public ManipulatorTask {
  public:
    RawConstantJointSpeedTask(const Eigen::VectorXd& dq, double time_limit);;

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
