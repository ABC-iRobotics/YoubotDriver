#ifndef YOUBOT_MANIPULATOR_MODUL_HPP
#define YOUBOT_MANIPULATOR_MODUL_HPP

#include "YoubotManipulatorMotionLayer.hpp"
#include <thread>
#include <mutex>

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

    void NewManipulatorTask(ManipulatorTask::Ptr task, double time_limit);

  private:
    struct NewTask {
      ManipulatorTask::Ptr ptr;
      double time_limit;
      NewTask() : ptr(NULL), time_limit(0) {}
      NewTask(ManipulatorTask::Ptr ptr_, double time_limit_) : ptr(ptr_), time_limit(time_limit_) {}
    } new_man_task = {};
    std::mutex new_task_mutex;

    void _thread(const std::string& configfilepath, bool virtual_ = false);

    std::thread t;
    bool threadrunning = false, threadtostop = false;

    std::unique_ptr<YoubotManipulatorMotionLayer> man = NULL;
    std::string configfilepath;
    bool virtual_;
  };
}
#endif
