#ifndef MANIPULATOR_TASK_STOP_HPP
#define MANIPULATOR_TASK_STOP_HPP

#include "MTask.hpp"
#include "Eigen/dense"

namespace youbot {

  /// <summary>
  /// Task that returns zero velocity command
  /// </summary>
  class MTaskStop : public MTask {
  public:
    ManipulatorCommand GetCommand(const JointsState& new_state) override;

    TaskType GetType() const override;

  protected:
    bool _taskFinished() const override;
  };
}
#endif