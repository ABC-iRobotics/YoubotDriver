#ifndef MODULE_HPP
#define MODULE_HPP

#include "MotionLayer.hpp"
#include <thread>
#include <mutex>

namespace youbot {

  class Module {
  public:
    // Not available constructors
    Module() = delete;
    Module(Module&) = delete;
    Module(const Module&) = delete;

    Module(const std::string& configfilepath, bool virtual_ = false); // Constructor - does nothing
    ~Module();

    MotionLayer::Status GetStatus() const;
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

    std::unique_ptr<MotionLayer> man = NULL;
    std::string configfilepath;
    bool virtual_;
  };
}
#endif
