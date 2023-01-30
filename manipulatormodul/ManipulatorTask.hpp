#ifndef MANIPULATOR_TASK_HPP
#define MANIPULATOR_TASK_HPP

#include "YoubotManipulator.hpp"
#include "Eigen/dense"

#include "Time.hpp"

namespace youbot {

  struct ManipulatorCommand {
    enum Type {
      JOINT_POSITION, //forbidden
      JOINT_VELOCITY,
      JOINT_TORQUE,
      ENCODER_SET_REFERENCE
    } type;
    Eigen::VectorXd value;

    ManipulatorCommand(Type type, const Eigen::VectorXd& value);
  };

  class ManipulatorTask {
  public:
    enum TaskType {
      INITIALIZATION,
      STOPPED,
      RAW_CONSTANT_JOINTSPEED
    };
    static std::string Type2String(TaskType type);
    
    void Initialize(const JointsState& start_state_); // Called by the motion layer

    virtual ManipulatorCommand GetCommand(const JointsState& new_state) = 0; // performs the task after it zero velocity commands

    virtual TaskType GetType() const = 0;

    bool Finished() const;

    typedef std::shared_ptr<ManipulatorTask> Ptr;

  protected:
    virtual bool _taskFinished() const = 0;

    JointsState start_state; // set by initialize
    bool started = false; // set by initialize
    std::chrono::steady_clock::time_point started_at; // set by initialize
  };
}
#endif


/*


    // Longer operations
    void MoveToPosition_RawPID(const Eigen::VectorXd& target, double timelimit) { // todo a stop mechanism??

    }

    void MoveToPosition_PIDwSpeedRampant(const double target[5], const double maxspeed[5]) {

    }

    void MoveToPosition_JointInterpolated(const double target[5]) {

    }*/