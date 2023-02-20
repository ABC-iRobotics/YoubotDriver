#ifndef MANAGER_HPP
#define MANAGER_HPP

#include "MotionLayer.hpp"
#include <thread>
#include <mutex>

namespace youbot {
  /// <summary>
  /// Class to manage the motion layer, start it on a detached thread start/stop tasks
  /// </summary>
  class Manager {
  public:
    Manager() = delete; ///< Not available constructors
    Manager(Manager&) = delete; ///< Not available constructors
    Manager(const Manager&) = delete; ///< Not available constructors

    /// <summary>
    /// Constructor, only saves the input
    /// </summary>
    /// <param name="configfilepath"></param>
    /// <param name="virtual_"></param>
    Manager(const std::string& configfilepath, bool virtual_ = false);
    ~Manager(); ///< Destructor

    MotionLayer::Status GetStatus() const; ///< Get status of the manipulator
    Eigen::VectorXd GetTrueStatus() const; ///< Get true joint vector of the virtual manipuator

    void StartThreadAndInitialize(); ///< Start the motion-layer on a detached thread, initialize the manipulator and start running
    void StopThread(bool waitin = true); ///< Stop thread, destruct related instances

    void NewManipulatorTask(ManipulatorTask::Ptr task, double time_limit); ///< Give new manipulator task

  private:
    struct NewTask {
      ManipulatorTask::Ptr ptr;
      double time_limit;
      NewTask() : ptr(NULL), time_limit(0) {}
      NewTask(ManipulatorTask::Ptr ptr_, double time_limit_) : ptr(ptr_), time_limit(time_limit_) {}
    } new_man_task = {};
    std::mutex new_task_mutex;

    void _thread(const std::string& configfilepath, bool virtual_ = false); ///< function of the detached thread

    std::thread t;
    bool threadrunning = false, threadtostop = false;

    std::unique_ptr<MotionLayer> man = NULL;
    std::string configfilepath;
    bool virtual_;
  };
}
#endif
