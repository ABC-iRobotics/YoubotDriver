#ifndef YOUBOT_MANIPULATOR_MOTION_LAYER_HPP
#define YOUBOT_MANIPULATOR_MOTION_LAYER_HPP

#include "YoubotManipulator.hpp"
#include "ManipulatorTask.hpp"
#include "Eigen/dense"

namespace youbot {

  class YoubotManipulatorMotionLayer {
  public:
    // Constructor
    YoubotManipulatorMotionLayer(const std::string& configfilepath, bool virtual_ = false);

    void Initialize();// Special task, that initializes the commutation and calibrate the robot arm

    typedef ManipulatorTask::TaskType TaskType;
    struct Status {
      struct Joint {
        Data<double> q, dq, tau;
        Data<YoubotJoint::JointStatus> status;
        Joint() : status(0) {};
      } joint[5];
      TaskType motion;
      void LogStatus() const;
    }; // Can be constructed by getting atomic structs

    // Get status in a threadsafe way
    Status GetStatus(); // threadsafe getters by default
    Eigen::VectorXd GetTrueStatus() const; // threadsafe getters by default

    // Task related methods
    bool IsRunning() const; // threadsafe
    void StopTask(); // threadsafe
    
    // Tasks
    void DoTask(ManipulatorTask::Ptr task, double time_limit);

  private:
    EtherCATMaster::Ptr center; // Virtual or physical ethercatbus handler
    std::atomic<ManipulatorTask::TaskType> motionStatus;
    std::unique_ptr<YoubotManipulator> man = NULL; // initialized manipulator handler
    bool taskrunning = false;
    bool stoptask = false;
    const std::string configfilepath;
    bool virtual_;
  };
}
#endif






