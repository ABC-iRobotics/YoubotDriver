#ifndef MANAGER_WRAPPER_HPP
#define MANAGER_WRAPPER_HPP

#include <pybind11/pybind11.h>
#include "pybind11/numpy.h"

#include "Manager.hpp"

namespace py = pybind11;

class ManagerWrapper {
  MTask::Ptr bridged_ptr = NULL;

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

  void JointVelocity(py::array dq_, double time_limit);

  void JointPosition(py::array q_, double time_limit);

  void Commutate();

  void Calibration();

  // pointer to the bridged task

  void StartBridgedTask() {
	//bridged_ptr
	// init if NULL
	// and set it
  }

  void SetBridgeTarget(py::array mode, py::array target, double time_limit) {
	//bridged_ptr todo test if not zero...
  }
};

#endif