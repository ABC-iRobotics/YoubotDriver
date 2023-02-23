#ifndef MOTION_LAYER_HPP
#define MOTION_LAYER_HPP

#include "JointState.hpp"
#include "ManipulatorTask.hpp"
#include "Eigen/dense"

namespace youbot {
  /// <summary>
  /// Class that provides functionality to perform tasks with the Manipulator, the Gripper and the Platform
  /// 
  /// Currently only one task can be managed - revision is coming soon.
  /// </summary>
  class MotionLayer {
  public:
    MotionLayer() = delete; ///< Not available constructor
    MotionLayer(MotionLayer&) = delete; ///< Not available constructor
    MotionLayer(const MotionLayer&) = delete; ///< Not available constructor

    MotionLayer(const std::string& configfilepath, bool virtual_ = false);  ///< Constructor: only saves the input

    /// <summary>
    /// Calls constructor of the manipulator, configurate it, initializes the commutation and calibrate the robot arm
    /// </summary>
    void Initialize(); 

    typedef ManipulatorTask::TaskType TaskType;

    /// <summary>
    /// Status of the manipulator: status of the joints and the type of the current task
    /// </summary>
    struct Status : JointsState {
      TaskType motion;
      void LogStatus() const;
    }; // Can be constructed by getting atomic structs

    /// <summary>
    /// Get status in a thread-safe way
    /// </summary>
    /// <returns></returns>
    Status GetStatus();

    /// <summary>
    /// Get true status in a thread-safe way of virtual manipulator for monitoring purposes
    /// </summary>
    /// <returns></returns>
    Eigen::VectorXd GetTrueStatus() const;

    /// <summary>
    /// Do manipulator task
    /// </summary>
    /// <param name="task"> a manipulator task to be performed</param>
    /// <param name="time_limit"> time limit </param>
    void DoManipulatorTask(ManipulatorTask::Ptr task, double time_limit);

    /// <summary>
    /// To check if a task is running - thread-safe
    /// </summary>
    /// <returns></returns>
    bool IsManipulatorTaskRunning() const;

    /// <summary>
    /// Stop the current manipulator task - thread-safe
    /// </summary>
    void StopManipulatorTask();
    
  private:
    /// <summary>
    /// Function that is called to restrain the given command to avoid going to the limit.
    /// 
    /// It can modify the given command before it is used according to the current joints state
    /// 
    /// Draft version of softlimits, must be developed later...
    /// </summary>
    /// <param name="cmd"> command that can be modified</param>
    /// <param name="status"> latest known manipulator state </param>
    void _SoftLimit(ManipulatorCommand& cmd, const JointsState& status) const;

    EtherCATMaster::Ptr center; ///< Virtual or physical ethercatbus handler
    std::atomic<ManipulatorTask::TaskType> motionStatus; ///< latest motion status
    std::unique_ptr<Manipulator> man = NULL; ///< initialized manipulator handler
    bool taskrunning = false;
    bool stoptask = false;
    const std::string configfilepath;
    bool virtual_;
  };
}
#endif






