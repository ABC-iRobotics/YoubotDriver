#ifndef YOUBOT_MANIPULATOR_MOTION_LAYER_HPP
#define YOUBOT_MANIPULATOR_MOTION_LAYER_HPP

#include "YoubotManipulator.hpp"
#include "Eigen/dense"

namespace youbot {
  class YoubotManipulatorMotionLayer {
  public:
    // Constructor
    YoubotManipulatorMotionLayer(const std::string& configfilepath, bool virtual_ = false);

    void Initialize();

    // Status related types
    enum MotionType {
      INITIALIZATION,
      STOPPED,
      CONSTANT_JOINTSPEED
    };
    static std::string to_string(MotionType type);

    struct Status {
      struct Joint {
        Data<double> q, dq, tau;
        Data<YoubotJoint::JointStatus> status;
        Joint() : status(0) {};
      } joint[5];
      MotionType motion;
      void LogStatus() const;
    }; // Can be constructed by getting atomic structs

    // Get status in a threadsafe way
    Status GetStatus(); // threadsafe getters by default
    Eigen::VectorXd GetTrueStatus() const; // threadsafe getters by default

    // Task related methods
    bool IsRunning() const; // threadsafe
    void StopTask(); // threadsafe
    
    // Tasks
    void ConstantJointSpeed(const Eigen::VectorXd& dq, double time_limit);

  private:
    EtherCATMaster::Ptr center; // Virtual of physical ethercatbus handler
    std::atomic<MotionType> motionStatus;
    std::unique_ptr<YoubotManipulator> man = NULL; // initialized manipulator handler
    bool taskrunning = false;
    bool stoptask = false;
    const std::string configfilepath;
    bool virtual_;
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





