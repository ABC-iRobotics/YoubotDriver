#ifndef MANIPULATOR_TASK_HPP
#define MANIPULATOR_TASK_HPP

#include "Manipulator.hpp"
#include "Eigen/dense"

#include "Time.hpp"

namespace youbot {

  /// <summary>
/// Command that can be forwarded to the BLDC controllers
/// 
/// Templates are used to allow to use it with double and int as well.
/// </summary>
  class BLDCCommand {
  public:
    /// <summary>
    /// Command types currently used and handled in the MotionLayer
    /// 
    /// "int" for MOTOR_TICK, MOTOR_RPM, MOTOR_CURRENT_MA, ENCODER_SET_REFERENCE;
    /// "double" for JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE
    /// </summary>
    enum Type {
      NOT_DEFINED, // will cause std::runtime_error
      JOINT_POSITION, //forbidden for TMCM-1610 firmware 1.48
      JOINT_VELOCITY,
      JOINT_TORQUE,
      MOTOR_TICK,
      MOTOR_RPM,
      MOTOR_CURRENT_MA,
      MOTOR_VOLTAGE,
      MOTOR_STOP,
      ENCODER_SET_REFERENCE, // be careful
      INITIALIZE // be careful
    };

    BLDCCommand() : type(NOT_DEFINED), i_value(0), d_value(0) {}; ///! Empty constructor

    template <class T>
    BLDCCommand(Type type_, T value_) : type(type_), i_value(value_), d_value(value_) {} ///! constructor for double input

    template <class T>
    void Set(T value_) { i_value = value_; d_value = value_; } ///! Set the value

    /// <summary>
    /// Get the current value
    /// </summary>
    /// <typeparam name="T"> double/int according to the aims </typeparam>
    /// <returns></returns>
    template <class T>
    T Get() const {
      if constexpr (std::is_same_v<T, int>)
        return i_value;
      if constexpr (std::is_same_v<T, double>)
        return d_value;
      throw std::runtime_error("not allowed input");
    }

    Type GetType() const { return type; } ///! Get the command type

  private:
    Type type;
    int i_value;
    double d_value;
  };

  /// <summary>
  /// The ManipulatorCommand is a low level settings for the manipulator, e.g. use given joint velocity
  /// it is defined by a vector (e.g. joint velocities) and the type specifier e.g. "joint velocity"
  /// </summary>
  struct ManipulatorCommand {
    BLDCCommand commands[5];

    ManipulatorCommand(BLDCCommand::Type type, const Eigen::VectorXd& value); ///< Constructor for same type and double values 

    ManipulatorCommand(BLDCCommand::Type type, const Eigen::VectorXi& value); ///< Constructor for same type and int values 

    ManipulatorCommand(const BLDCCommand& cmd0, const BLDCCommand& cmd1, const BLDCCommand& cmd2,
      const BLDCCommand& cmd3, const BLDCCommand& cmd4); ///< Constructor to use different command types
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
      CALIBRATION,
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