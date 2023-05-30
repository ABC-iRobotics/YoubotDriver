#ifndef MANAGER_WRAPPER_HPP
#define MANAGER_WRAPPER_HPP

#include <pybind11/pybind11.h>
#include "pybind11/numpy.h"

#include "Manager.hpp"

namespace py = pybind11;

class ManagerWrapper : private youbot::Manager {
  youbot::MTask::Ptr bridged_ptr;

public:
  ManagerWrapper() = delete; ///< Not available constructors
  ManagerWrapper(ManagerWrapper&) = delete; ///< Not available constructors
  ManagerWrapper(const ManagerWrapper&) = delete; ///< Not available constructors

  ManagerWrapper(const std::string& configfilepath, bool virtual_ = false);
  ~ManagerWrapper();

  py::dict GetStatus() const;

  py::array GetTrueStatus() const;

  void StartThreadAndInitialize();
  void StopThread(bool waitin = true);

  void StopTask();

  void ZeroCurrent(double time_limit);

  void JointVelocity(py::array_t<double> dq_, double time_limit);

  void JointPosition(py::array_t<double> q_, double time_limit);

  void Commutate();

  void Calibration();

  // pointer to the bridged task

  void StartBridgedTask();

  void SetBridgeTarget(py::array_t<int> mode, py::array_t<double> target, double time_limit);
};

#endif