#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "motorcycle.hpp"

#include "Manager.hpp"

#include <iostream>

#include "pybind11/numpy.h"

#include "MTaskCommutation.hpp"
#include "MTaskCalibration.hpp"
#include "MTaskZeroCurrent.hpp"
#include "MTaskStop.hpp"
#include "MTaskRawConstantJointSpeed.hpp"

using namespace youbot;
namespace py = pybind11;

void init_motorcycle(py::module &);

class ManagerWrapper {
public:
  ManagerWrapper() = delete; ///< Not available constructors
  ManagerWrapper(ManagerWrapper&) = delete; ///< Not available constructors
  ManagerWrapper(const ManagerWrapper&) = delete; ///< Not available constructors

  /// <summary>
  /// Constructor, only saves the input
  /// </summary>
  /// <param name="configfilepath"></param>
  /// <param name="virtual_"></param>
  ManagerWrapper(const std::string& configfilepath, bool virtual_ = false) {
    std::cout << configfilepath << std::endl;
    std::cout << (virtual_ ? "virtual" : "physical") << std::endl;
  }
  ~ManagerWrapper() {
    std::cout << "Destructor called" << std::endl;
  }

  py::dict GetStatus() const {
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

  py::array GetTrueStatus() const {
    double* q = new double[5];
    for (int i = 0; i < 5; i++)
      q[i] = 0;
    return py::array(5, q);
  }

  void StartThreadAndInitialize() {

  }
  void StopThread(bool waitin = true) {

  }

  void StopTask() {
    //youbot::Manager m;
    //MTask::Ptr ptr = std::make_shared<MTaskStop>();
    //m.NewManipulatorTask(ptr, 1);
  }

  void ZeroCurrent(double time_limit) {
    //youbot::Manager m;

    //MTask::Ptr ptr = std::make_shared<MTaskZeroCurrent>();
    //m.NewManipulatorTask(ptr, time_limit);
  }


  void JointVelocity(py::array dq_, double time_limit) {
    Eigen::VectorXd dq(5);
    for (int i = 0; i < 5; i++)
      dq[i] = *(double*)dq_.data(i);
    
    //MTask::Ptr ptr = std::make_shared<MTaskRawConstantJointSpeed>();
    //m.NewManipulatorTask(ptr, 1);
  }

  void JointPosition(py::array q_, double time_limit) {
    Eigen::VectorXd q(5);
    for (int i = 0; i < 5; i++)
      q[i] = *(double*)q_.data(i);

    //MTask::Ptr ptr = std::make_shared<MTaskRawConstantJointSpeed>();
    //m.NewManipulatorTask(ptr, 1);
  }

  void Commutate() {
    //MTask::Ptr ptr = std::make_shared<MTaskCommutation>();
    //m.NewManipulatorTask(ptr, 1);
  }

  void Calibration() {
    //MTask::Ptr ptr = std::make_shared<MTaskCalibration>();
    //m.NewManipulatorTask(ptr, 1);
  }

  void BridgedTask() {

  }

  void SetBridgedTarget(py::array mode, py::array target) {

  }
};


namespace mcl {

PYBIND11_MODULE(youbotpython, m) {
    // Optional docstring
    m.doc() = "Automobile library";
    
    py::class_<vehicles::Motorcycle>(m, "Motorcycle")
      .def(py::init<std::string>(), py::arg("name"))
      .def("get_name",
        py::overload_cast<>(&vehicles::Motorcycle::get_name, py::const_))
      .def("ride",
        py::overload_cast<std::string>(&vehicles::Motorcycle::ride, py::const_),
        py::arg("road"));

    py::class_<ManagerWrapper>(m, "youbot")
      .def(py::init<std::string,bool>(), py::arg("configfile"), py::arg("isVirtual"))
      .def("get_status",
        py::overload_cast<>(&ManagerWrapper::GetStatus, py::const_))
      .def("get_true_q",
        py::overload_cast<>(&ManagerWrapper::GetTrueStatus, py::const_))
      .def("start_thread_and_init",
        py::overload_cast<>(&ManagerWrapper::StartThreadAndInitialize))
      .def("stop_thread",
        py::overload_cast<bool>(&ManagerWrapper::StopThread), py::arg("waitin"));
}
}
