#ifndef INIT_MAN_COMMUTATION_TASK_HPP
#define INIT_MAN_COMMUTATION_TASK_HPP

#include "MTask.hpp"
#include "Eigen/dense"

#include "Time.hpp"

namespace youbot {
  /// <summary>
  /// Task that send out commutation initialization command and finishes as all of the joints are initialized
  /// </summary>
  class MTaskCommutation : public MTask {
    bool finished = false;

  public:
    ManipulatorCommand GetCommand(const JointsState& new_state) override;

    TaskType GetType() const override;

  protected:
    bool _taskFinished() const override;
  };
}
#endif