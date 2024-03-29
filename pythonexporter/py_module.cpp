#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ManagerWrapper.hpp"

using namespace youbot;
namespace py = pybind11;

void init_motorcycle(py::module &);

namespace mcl {

PYBIND11_MODULE(youbotpython, m) {
    // Optional docstring
    m.doc() = "Automobile library";

    py::class_<ManagerWrapper>(m, "youbot")
      .def(py::init<std::string,bool>(), py::arg("configfile"), py::arg("isVirtual"))
      .def("get_status",
        py::overload_cast<>(&ManagerWrapper::GetStatus, py::const_))
      .def("get_true_q",
        py::overload_cast<>(&ManagerWrapper::GetTrueStatus, py::const_))
      .def("start_thread_and_init",
        py::overload_cast<>(&ManagerWrapper::StartThreadAndInitialize))
      .def("stop_thread",
        py::overload_cast<bool>(&ManagerWrapper::StopThread), py::arg("waitin"))
      .def("stop_task",
        py::overload_cast<>(&ManagerWrapper::StopTask))
      .def("zero_current",
        py::overload_cast<double>(&ManagerWrapper::ZeroCurrent),py::arg("time_limit"))
      .def("joint_velocity",
        py::overload_cast<py::array_t<double>,double>(&ManagerWrapper::JointVelocity),py::arg("dqpdt"),py::arg("time_limit"))
      .def("joint_position",
        py::overload_cast<py::array_t<double>, double>(&ManagerWrapper::JointPosition), py::arg("q"), py::arg("time_limit"))
      .def("commute",
        py::overload_cast<>(&ManagerWrapper::Commutate))
      .def("calibrate",
        py::overload_cast<>(&ManagerWrapper::Calibration))
      .def("start_bridged_task",
        py::overload_cast<>(&ManagerWrapper::StartBridgedTask))
      .def("set_bridged_task",
        py::overload_cast<py::array_t<int>, py::array_t<double>, double>(&ManagerWrapper::SetBridgeTarget),
        py::arg("target_type"), py::arg("target"), py::arg("time_limit"));
}
}
