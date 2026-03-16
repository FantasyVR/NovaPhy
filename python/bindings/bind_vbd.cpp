#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <limits>

#include "novaphy/vbd/vbd_config.h"
#include "novaphy/vbd/vbd_world.h"

namespace py = pybind11;

void bind_vbd(py::module_& m) {
    using namespace novaphy;

    py::enum_<VbdBackend>(m, "VbdBackend", R"pbdoc(
        Backend selection for the VBD/AVBD solver.
    )pbdoc")
        .value("CPU", VbdBackend::CPU)
        .value("CUDA", VbdBackend::CUDA)
        .export_values();

    py::class_<VBDConfig>(m, "VBDConfig", R"pbdoc(
        VBD/AVBD configuration aligned with avbd-demo3d (dt/gravity/iterations/alpha/gamma/beta_*).
    )pbdoc")
        .def(py::init<>())
        .def_readwrite("dt", &VBDConfig::dt)
        .def_readwrite("gravity", &VBDConfig::gravity)
        .def_readwrite("iterations", &VBDConfig::iterations)
        .def_readwrite("max_contacts_per_pair", &VBDConfig::max_contacts_per_pair)
        .def_readwrite("alpha", &VBDConfig::alpha)
        .def_readwrite("gamma", &VBDConfig::gamma)
        .def_readwrite("beta_linear", &VBDConfig::beta_linear,
            "接触约束 penalty 增长系数 (demo: betaLin=10000).")
        .def_readwrite("beta_angular", &VBDConfig::beta_angular,
            "关节约束 penalty 增长系数 (demo: betaAng=100).")
        .def_readwrite("initial_penalty", &VBDConfig::initial_penalty)
        .def_readwrite("velocity_smoothing", &VBDConfig::velocity_smoothing)
        .def_readwrite("primal_relaxation", &VBDConfig::primal_relaxation)
        .def_readwrite("lhs_regularization", &VBDConfig::lhs_regularization)
        .def_readwrite("backend", &VBDConfig::backend,
            "Backend selection for the AVBD solver (CPU or CUDA).")
        .def("__repr__", [](const VBDConfig& c) {
            return "<VBDConfig dt=" + std::to_string(c.dt) +
                   " iterations=" + std::to_string(c.iterations) + ">";
        });

    py::class_<VBDWorld>(m, "VBDWorld", R"pbdoc(
        VBD/AVBD simulation world.
    )pbdoc")
        .def(py::init<const Model&, const VBDConfig&>(),
             py::arg("model"),
             py::arg("config") = VBDConfig{})
        .def("step", &VBDWorld::step)
        .def("clear_forces", &VBDWorld::clear_forces)
        .def("add_ignore_collision", &VBDWorld::add_ignore_collision,
             py::arg("body_a"), py::arg("body_b"))
        .def("add_joint", &VBDWorld::add_joint,
             py::arg("body_a"), py::arg("body_b"),
             py::arg("rA"), py::arg("rB"),
             py::arg("stiffnessLin") = std::numeric_limits<float>::infinity(),
             py::arg("stiffnessAng") = 0.0f,
             py::arg("fracture") = std::numeric_limits<float>::infinity())
        .def("add_spring", &VBDWorld::add_spring,
             py::arg("body_a"), py::arg("body_b"),
             py::arg("rA"), py::arg("rB"),
             py::arg("stiffness"),
             py::arg("rest") = -1.0f)
        .def_property_readonly("state", py::overload_cast<>(&VBDWorld::state, py::const_),
             py::return_value_policy::reference_internal)
        .def_property_readonly("state_mut", py::overload_cast<>(&VBDWorld::state),
             py::return_value_policy::reference_internal,
             "Mutable state view (allows setting initial velocities).")
        .def_property_readonly("model", &VBDWorld::model,
             py::return_value_policy::reference_internal)
        .def_property_readonly("config", &VBDWorld::config,
             py::return_value_policy::reference_internal)
        .def("__repr__", [](const VBDWorld& w) {
            return "<VBDWorld bodies=" + std::to_string(w.model().num_bodies()) + ">";
        });
}
