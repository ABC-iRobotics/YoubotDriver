#ifndef YOUBOT_MANIPULATOR_MODUL_HPP
#define YOUBOT_MANIPULATOR_MODUL_HPP

#include "YoubotManipulatorMotionLayer.hpp"
#include <thread>

#include "RawConstantJointSpeedTask.hpp"

namespace youbot {

  class YoubotManipulatorModul {
  public:
    // Not available constructors
    YoubotManipulatorModul() = delete;
    YoubotManipulatorModul(YoubotManipulatorModul&) = delete;
    YoubotManipulatorModul(const YoubotManipulatorModul&) = delete;

    YoubotManipulatorModul(const std::string& configfilepath, bool virtual_ = false); // Constructor - does nothing
    ~YoubotManipulatorModul();

    YoubotManipulatorMotionLayer::Status GetStatus() const;
    Eigen::VectorXd GetTrueStatus() const;

    void StartThreadAndInitialize();
    void StopThread(bool waitin = true);

  private:
    void _thread(const std::string& configfilepath, bool virtual_ = false) {
      threadtostop = false;
      if (!man)
        man = std::make_unique<YoubotManipulatorMotionLayer>(configfilepath, virtual_);
      man->Initialize();

      // Command based operation, checking stop...
      //while (!threadtostop) {
        Eigen::VectorXd dq(5);
        dq << 0.1, 0.1, -0.1, -0.1, 0.1;
        ManipulatorTask::Ptr task = std::make_shared<RawConstantJointSpeedTask>(dq, 10);
        man->DoTask(task, 5);
        //man->ConstantJointSpeed(dq,10);
      //}
      threadrunning = false;
    }

    std::thread t;
    bool threadrunning = false, threadtostop = false;

    std::unique_ptr<YoubotManipulatorMotionLayer> man = NULL;
    std::string configfilepath;
    bool virtual_;
  };
}
#endif

/*

    void Stop() {

      // conditional_variable?? - if thread running, otherwise
    }
    
void youbot::YoubotManipulatorModul::AddCommand(const Command& c) {
  std::lock_guard<std::mutex> lock(command_mutex);
  commands.push_back(c);
}

    void AddCommand(const Command& c);

  typedef YoubotManipulatorMotionLayer::MotionType MotionType;
    struct Command {
      MotionType type;
      Eigen::VectorXd params = {};
      double timelimit = 10;
      bool preemptive = true;

      Command() {};
      Command(MotionType type_, const Eigen::VectorXd& params, bool preemptive = true, double timelimit_=100) :
        preemptive(preemptive), type(type_), timelimit(timelimit_), params(params) {};
    };

    std::mutex command_mutex;
    std::vector<Command> commands;

*/





