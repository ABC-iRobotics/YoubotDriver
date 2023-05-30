#ifndef MANIPULATOR_TASK_GENERIC_RAW_CONSTANT_HPP
#define MANIPULATOR_TASK_GENERIC_RAW_CONSTANT_HPP

#include "MTask.hpp"
#include <vector>

namespace youbot {
  /// <summary>
  /// Task that allows arbitrary control type (position via P control...)
  /// </summary>
  class MTaskGenericRawConstant : public MTask {
  public:
    enum CmdType : uint8_t {
      STOP = 0,
      POSITION_MOTOR_TICK = 1,
      POSITION_MOTOR_RAD = 2,
      VELOCITY_MOTOR_RPM = 3,
      TORQUE_MOTOR_MA = 4,
      TORQUE_MOTOR_NM = 5,
      POSITION_JOINT_RAD = 10,
      VELOCITY_JOINT_RADPERSEC = 11,
      TORQUE_JOINT_NM = 12,
    };

    struct Cmd {
      CmdType type;
      double value;
    };
  private:
    const std::vector<Cmd> cmd;
    const double P_constant;
  public:
    MTaskGenericRawConstant(const std::vector<Cmd>& cmd,
      double P_constant = 500. / 4000. * 60. * 0.1);

    ManipulatorCommand GetCommand(const JointsState& new_state) override;

    TaskType GetType() const override;

  protected:
    bool _taskFinished() const override;
  };
}
#endif