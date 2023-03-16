#ifndef MANIPULATOR_TASK_ZERO_CURRENT_HPP
#define MANIPULATOR_TASK_ZERO_CURRENT_HPP

#include "MTask.hpp"
#include "Eigen/dense"

namespace youbot {
  /// <summary>
  /// Task that returns zero current commands
  /// </summary>
  class MTaskZeroCurrent : public MTask {
  public:
    ManipulatorCommand GetCommand(const JointsState& new_state) override;

    TaskType GetType() const override;

  protected:
    bool _taskFinished() const override;
  };
}
#endif