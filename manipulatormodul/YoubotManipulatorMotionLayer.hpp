#ifndef YOUBOT_MANIPULATOR_MOTION_LAYER_HPP
#define YOUBOT_MANIPULATOR_MOTION_LAYER_HPP

#include "YoubotManipulator.hpp"
#include "ManipulatorTask.hpp"
#include "Eigen/dense"

namespace youbot {

  class YoubotManipulatorMotionLayer {
  public:
    const double limitZoneDeg = 8;
    const double corrjointspeed = 0.5;

    // Not available constructors
    YoubotManipulatorMotionLayer() = delete;
    YoubotManipulatorMotionLayer(YoubotManipulatorMotionLayer&) = delete;
    YoubotManipulatorMotionLayer(const YoubotManipulatorMotionLayer&) = delete;

    // Constructor
    YoubotManipulatorMotionLayer(const std::string& configfilepath, bool virtual_ = false);

    void Initialize();// Special task, that initializes the commutation and calibrate the robot arm

    typedef ManipulatorTask::TaskType TaskType;
    struct Status : JointsState {
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
    void _SoftLimit(ManipulatorCommand& cmd, const JointsState& status) const {
      static double limitZoneRad = limitZoneDeg / 180 * M_PI;
      for (int i = 0; i < 5; i++) {
        auto q = status.joint[i].q.value;
        auto p = man->GetJoint(i)->GetParameters();
        auto qmin_lim = p.qMinDeg / 180 * M_PI + limitZoneRad;
        auto qmax_lim = p.qMaxDeg / 180 * M_PI - limitZoneRad;
        if (q < qmin_lim) {
          switch (cmd.type) {
          case cmd.JOINT_POSITION:
            if (cmd.value(i) < qmin_lim)
              cmd.value(i) = qmin_lim;
            break;
          case cmd.JOINT_VELOCITY:
            if (cmd.value(i) < corrjointspeed)
              cmd.value(i) = corrjointspeed;
            break;
          case cmd.JOINT_TORQUE:
            if (cmd.value(i) < 0)
              cmd.value(i) = 0;
            break;
          default:
            break;
          }
        }
        if (q > qmax_lim) {
          switch (cmd.type) {
          case cmd.JOINT_POSITION:
            if (cmd.value(i) > qmax_lim)
              cmd.value(i) = qmax_lim;
            break;
          case cmd.JOINT_VELOCITY:
            if (cmd.value(i) > -corrjointspeed)
              cmd.value(i) = -corrjointspeed;
            break;
          case cmd.JOINT_TORQUE:
            if (cmd.value(i) > 0)
              cmd.value(i) = 0;
            break;
          default:
            break;
          }
        }
      }
    }

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






