#ifndef YOUBOT_MANIPULATOR_MODUL_HPP
#define YOUBOT_MANIPULATOR_MODUL_HPP

#include "YoubotManipulator.hpp"
#include <mutex>
#include "Eigen/dense" // just to test it...

namespace youbot {

  class YoubotManipulatorModul {
  public:
    struct Command {
      enum Type {
        STOP,
        JOINT_VELOCITY,
        JOINT_POSITION_RAW,
        JOINT_POSITION_INT,
        JOINT_TORQUE,
        CARTESIAN_POSITION_RAW,
        CARTESIAN_POSITION_INT,
        CARTESIAN_VELOCITY,
        CARTESIAN_FORCETORQUE
      } type = STOP;
      double params[7];
      double timelimit = 10;

      Command() {};
      Command(Type type_, double timelimit_=100) : type(type_), timelimit(timelimit_) {};
    };

    struct Status {
      struct Joint {
        double q, dq, tau;
        YoubotJoint::JointStatus status;
        Joint() : status(0), q(0), dq(0), tau(0) {};
      } joint[5];
      enum ManipulatorStatus {

      };
    }; // Get methods updates it

    YoubotManipulatorModul(const std::string& configfilepath, bool virtual_ = false);

    void AddCommand(const Command& c);
    Status GetStatus();
    void GetTrueStatus(double& q0, double& q1, double& q2, double& q3, double& q4);

    // Longer operations
    void MoveToPosition_RawPID(const double target[5]) {

    }

    void MoveToPosition_PIDwSpeedRampant(const double target[5], const double maxspeed[5]) {

    }

    void MoveToPosition_JointInterpolated(const double target[5]) {

    }

  private:
    EtherCATMaster::Ptr center;
    std::unique_ptr<YoubotManipulator> man;
    std::vector<Command> saved_commands; // saved by mutex
    std::mutex command_mutex, status_mutex;
    std::vector<Command> to_perform; // flipped
    Status status;

    void _processReturnsIntoStatus() { // must be called after ExchangeMsg // how can I get the status during calibration/mailbox messages, etc....
      std::lock_guard<std::mutex> lock(status_mutex);
      for (int i = 0; i < 5; i++) {
        auto j = man->GetJoint(i);
        status.joint[i].q = j->GetQLatestRad().value;
        status.joint[i].dq = j->GetDQLatestRad().value;
        status.joint[i].tau = j->GetTauLatestNm().value;
        status.joint[i].status = j->GetStatusLatest().value;
      }
    }
  };
}
#endif
