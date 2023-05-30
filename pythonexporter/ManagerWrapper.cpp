#include <pybind11/stl.h>
#include <iostream>

#include "ManagerWrapper.hpp"
#include "MTaskCommutation.hpp"
#include "MTaskCalibration.hpp"
#include "MTaskZeroCurrent.hpp"
#include "MTaskStop.hpp"
#include "MTaskRawConstantJointSpeed.hpp"
#include "MTaskRawConstantJointPosition.hpp"
#include "MTaskGenericRawConstant.hpp"

#include <pybind11/stl.h>
#include <iostream>

using namespace youbot;
namespace py = pybind11;

ManagerWrapper::ManagerWrapper(const std::string& configfilepath, bool virtual_) :
  Manager(configfilepath, virtual_),
  bridged_ptr(std::make_shared<MTaskGenericRawConstant>()) {}

ManagerWrapper::~ManagerWrapper() {}

py::dict ManagerWrapper::GetStatus() const {
  youbot::MotionLayer::Status status = Manager::GetStatus();
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
  auto q_ = Manager::GetTrueStatus();
  double* q = new double[5];
  for (int i = 0; i < 5; i++)
    q[i] = q_[i];
  return py::array(5, q);
}

void ManagerWrapper::StartThreadAndInitialize() {
  Manager::StartThreadAndInitialize();
}

void ManagerWrapper::StopThread(bool waitin) {
  Manager::StopThread(waitin);
}

void ManagerWrapper::StopTask() {
  MTask::Ptr ptr = std::make_shared<MTaskStop>();
  Manager::NewManipulatorTask(ptr, 1);
}

void ManagerWrapper::ZeroCurrent(double time_limit) {
  MTask::Ptr ptr = std::make_shared<MTaskZeroCurrent>();
  Manager::NewManipulatorTask(ptr, time_limit);
}

void ManagerWrapper::JointVelocity(py::array_t<double> dq_, double time_limit) {
  Eigen::VectorXd dq(5);
  for (int i = 0; i < 5; i++)
    dq[i] = *dq_.data(i);

  MTask::Ptr ptr = std::make_shared<MTaskRawConstantJointSpeed>(dq, time_limit);
  Manager::NewManipulatorTask(ptr, time_limit);
}

void ManagerWrapper::JointPosition(py::array_t<double> q_, double time_limit) {
  Eigen::VectorXd q(5);
  for (int i = 0; i < 5; i++)
    q[i] = *q_.data(i);

  MTask::Ptr ptr = std::make_shared<MTaskRawConstantJointPosition>(q);
  Manager::NewManipulatorTask(ptr, time_limit);
}

void ManagerWrapper::Commutate() {
  MTask::Ptr ptr = std::make_shared<MTaskCommutation>();
  Manager::NewManipulatorTask(ptr, 1e9);
}

void ManagerWrapper::Calibration() {
  MTask::Ptr ptr = std::make_shared<MTaskCalibration>();
  Manager::NewManipulatorTask(ptr, 1e9);
}

void ManagerWrapper::StartBridgedTask() {
  Manager::NewManipulatorTask(bridged_ptr, 1e9);
}

void ManagerWrapper::SetBridgeTarget(py::array_t<int> mode, py::array_t<double> target, double time_limit) {
  std::vector<MTaskGenericRawConstant::Cmd> cmds;
  for (int i = 0; i < 5; i++)
    cmds.push_back({ MTaskGenericRawConstant::CmdType(*mode.data(i)), *target.data(i) });
  std::static_pointer_cast<MTaskGenericRawConstant>(bridged_ptr)->SetCommand(cmds, time_limit);
}