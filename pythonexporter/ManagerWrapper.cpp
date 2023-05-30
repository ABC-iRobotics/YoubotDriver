#include <pybind11/stl.h>
#include <iostream>

#include "ManagerWrapper.hpp"
#include "MTaskCommutation.hpp"
#include "MTaskCalibration.hpp"
#include "MTaskZeroCurrent.hpp"
#include "MTaskStop.hpp"
#include "MTaskRawConstantJointSpeed.hpp"
#include "MTaskRawConstantJointPosition.hpp"

#include <pybind11/stl.h>
#include <iostream>

using namespace youbot;
namespace py = pybind11;

ManagerWrapper::ManagerWrapper(const std::string& configfilepath, bool virtual_) {
  std::cout << configfilepath << std::endl;
  std::cout << (virtual_ ? "virtual" : "physical") << std::endl;
}

ManagerWrapper::~ManagerWrapper() {
  std::cout << "Destructor called" << std::endl;
}

py::dict ManagerWrapper::GetStatus() const {
  youbot::MotionLayer::Status status;
  status.joint[2].tau.value = 10.;
  py::dict out;
  // arrays
  double* q = new double[5];
  double* dq = new double[5];
  double* tau = new double[5];
  double* ticks = new double[5];
  double* RPM = new double[5];
  for (int i = 0; i < 5; i++) {
    q[i] = status.joint[i].q.value;
    dq[i] = status.joint[i].dq.value;
    tau[i] = status.joint[i].tau.value;
    ticks[i] = status.joint[i].motorticks.value;
    RPM[i] = status.joint[i].motorRPM.value;
  }
  out["q"] = py::array(5, q);
  out["dq"] = py::array(5, dq);
  out["tau"] = py::array(5, tau);
  out["ticks"] = py::array(5, ticks);
  out["RPM"] = py::array(5, RPM);

  // Create status strings in an array
  using np_str_t = std::array<char, 160>;
  pybind11::array_t<np_str_t> status_array(5);
  np_str_t* array_of_cstr_ptr = reinterpret_cast<np_str_t*>(status_array.request().ptr);
  for (int i = 0; i < 5; i++) {
    std::strncpy(array_of_cstr_ptr->data(), status.joint[i].status.value.toString().data(), array_of_cstr_ptr->size());
    array_of_cstr_ptr++;
  }
  out["joint_status"] = status_array;

  // Task type
  out["task"] = youbot::MTask::Type2String(status.motion);

  // Manipulator status
  out["manipulator_status"] = status.manipulatorStatus.ToString();

  return out;
}

py::array ManagerWrapper::GetTrueStatus() const {
  double* q = new double[5];
  for (int i = 0; i < 5; i++)
    q[i] = 0;
  return py::array(5, q);
}

void ManagerWrapper::StartThreadAndInitialize() {

}

void ManagerWrapper::StopThread(bool waitin) {

}

void ManagerWrapper::StopTask() {
  //youbot::Manager m;
  //MTask::Ptr ptr = std::make_shared<MTaskStop>();
  //m.NewManipulatorTask(ptr, 1);
}

void ManagerWrapper::ZeroCurrent(double time_limit) {
  //youbot::Manager m;

  //MTask::Ptr ptr = std::make_shared<MTaskZeroCurrent>();
  //m.NewManipulatorTask(ptr, time_limit);
}

void ManagerWrapper::JointVelocity(py::array dq_, double time_limit) {
  Eigen::VectorXd dq(5);
  for (int i = 0; i < 5; i++)
    dq[i] = *(double*)dq_.data(i);

  //MTask::Ptr ptr = std::make_shared<MTaskRawConstantJointSpeed>();
  //m.NewManipulatorTask(ptr, 1);
}

void ManagerWrapper::JointPosition(py::array q_, double time_limit) {
  Eigen::VectorXd q(5);
  for (int i = 0; i < 5; i++)
    q[i] = *(double*)q_.data(i);

  //MTask::Ptr ptr = std::make_shared<MTaskRawConstantJointPosition>();
  //m.NewManipulatorTask(ptr, 1);
}

void ManagerWrapper::Commutate() {
  //MTask::Ptr ptr = std::make_shared<MTaskCommutation>();
  //m.NewManipulatorTask(ptr, 1);
}

void ManagerWrapper::Calibration() {
  //MTask::Ptr ptr = std::make_shared<MTaskCalibration>();
  //m.NewManipulatorTask(ptr, 1);
}
