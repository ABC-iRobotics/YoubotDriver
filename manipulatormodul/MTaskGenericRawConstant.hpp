#ifndef MANIPULATOR_TASK_GENERIC_RAW_CONSTANT_HPP
#define MANIPULATOR_TASK_GENERIC_RAW_CONSTANT_HPP

#include "MTask.hpp"
#include <vector>
#include <mutex>

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
      Cmd(CmdType type_, double value_) :type(type_), value(value_) {}
      Cmd() : type(CmdType::STOP), value(0) {};
    };
  private:
    Cmd target_cmd[5] = { {}, {}, {}, {}, {} };
    std::chrono::system_clock::time_point target_time = std::chrono::system_clock::now();
    std::mutex target_mutex;
    const double P_constant;

  public:
    MTaskGenericRawConstant(double P_constant = 500. / 4000. * 60. * 0.1);

    void SetCommand(const std::vector<Cmd>& cmd, double time_limit_s);

    ManipulatorCommand GetCommand(const JointsState& new_state) override;

    TaskType GetType() const override;

  protected:
    bool _taskFinished() const override;
  };
}
#endif