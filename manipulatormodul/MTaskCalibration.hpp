#ifndef CALIBRATE_MANIPULATOR_TASK_HPP
#define CALIBRATE_MANIPULATOR_TASK_HPP

#include "MTask.hpp"
#include "Eigen/dense"

namespace youbot {
  /// <summary>
 /// Task that send out commutation initialization command and finishes as all of the joints are initialized
 /// 
 /// TODO!!
 /// </summary>
  class MTaskCalibration : public MTask {
    bool finished = false;

  public:
    ManipulatorCommand GetCommand(const JointsState& new_state) override {
      return ManipulatorCommand(BLDCCommand::MOTOR_STOP,Eigen::VectorXd(5));
    }

    TaskType GetType() const override {
      return CALIBRATION;
    }

  protected:
    bool _taskFinished() const override {
      return true;
    }
  };
}
#endif