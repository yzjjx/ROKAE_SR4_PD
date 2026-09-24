#include "PD_con.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(rokae_pd, module)
{
    module.doc() = "ROKAE SR4 C++ PD controller";

    py::class_<PD_con>(module, "PDController")
        .def(py::init<>())
        .def("compute_control", &PD_con::PD_controller,
             py::arg("q_real"), py::arg("dq_real"),
             py::arg("q_ideal"), py::arg("dq_ideal"));
}
