#include <pybind11/pybind11.h>

#include "novaphy/novaphy.h"

namespace py = pybind11;

// Forward declarations for sub-module binders
void bind_math(py::module_& m);
void bind_core(py::module_& m);
void bind_collision(py::module_& m);
void bind_sim(py::module_& m);
void bind_dynamics(py::module_& m);

PYBIND11_MODULE(_core, m) {
    m.doc() = R"pbdoc(
        NovaPhy core Python extension module.

        This module exposes math types, collision detection, free-body simulation,
        and articulated-body dynamics routines implemented in C++.
    )pbdoc";

    m.def("version", &novaphy::version, R"pbdoc(
        Returns the NovaPhy library version string.

        Returns:
            str: Semantic version string.
    )pbdoc");

    bind_math(m);
    bind_core(m);
    bind_collision(m);
    bind_sim(m);
    bind_dynamics(m);
}
