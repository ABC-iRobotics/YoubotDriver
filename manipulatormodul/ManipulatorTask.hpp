#ifndef MANIPULATOR_TASK_HPP
#define MANIPULATOR_TASK_HPP

#include "Manipulator.hpp"
#include "Eigen/dense"

#include "Time.hpp"

namespace youbot {

  /// <summary>
  /// The ManipulatorCommand is a low level settings for the manipulator, e.g. use given joint velocity
  /// it is defined by a vector (e.g. joint velocities) and the type specifier e.g. "joint velocity"
  /// </summary>
  struct ManipulatorCommand {
    /// <summary>
    /// Command types currently used and handled in the MotionLayer
    /// </summary>
    enum Type {
      JOINT_POSITION, //forbidden
      JOINT_VELOCITY,
      JOINT_TORQUE,
      ENCODER_SET_REFERENCE
    } type;
    Eigen::VectorXd value;

    ManipulatorCommand(Type type, const Eigen::VectorXd& value); ///< Constructor
  };

  /// <summary>
  /// The manipulator task is a user defined (overridden) object that provides ManipulatorCommands
  /// for the current time moment and current manipulator status
  /// </summary>
  class ManipulatorTask {
  public:
    /// <summary>
    /// Task types currently implemented
    /// </summary>
    enum TaskType {
      NOT_DEFINED,
      INITIALIZATION,
      STOPPED,
      ZERO_CURRENT,
      RAW_CONSTANT_JOINTSPEED // currently these types are defined
    };

    static std::string Type2String(TaskType type); ///< Generate string from task type

    virtual TaskType GetType() const = 0; ///< Returns the type of the task

    void Initialize(const JointsState& start_state_); ///< Called by the motion layer at the first usage with the start state

    virtual ManipulatorCommand GetCommand(const JointsState& new_state) = 0;
    ///< Called again and again by the thread to give commands to the manipulator. Must contain the control logic.

    bool Finished() const; ///< Returns if the task is finished

    typedef std::shared_ptr<ManipulatorTask> Ptr;

  protected:
    virtual bool _taskFinished() const = 0; 

    JointsState start_state; // set by initialize
    bool started = false; // set by initialize
    std::chrono::steady_clock::time_point started_at; // set by initialize
  };
  
  /// <summary>
  /// Task that returns zero velocity command
  /// </summary>
  class IdleManipulatorTask : public ManipulatorTask {
  public:
    ManipulatorCommand GetCommand(const JointsState& new_state) override;

    TaskType GetType() const override;

  protected:
    bool _taskFinished() const override;
  };

  /// <summary>
  /// Task that returns zero current commands
  /// </summary>
  class ZeroCurrentManipulatorTask : public ManipulatorTask {
  public:
    ManipulatorCommand GetCommand(const JointsState& new_state) override;

    TaskType GetType() const override;

  protected:
    bool _taskFinished() const override;
  };
}
#endif