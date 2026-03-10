#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include "novaphy/vbd/vbd_config.h"
#include "novaphy/vbd/vbd_world.h"

namespace py = pybind11;

void bind_vbd(py::module_& m) {
    using namespace novaphy;

    py::class_<VBDConfig>(m, "VBDConfig", R"pbdoc(
        VBD/AVBD 配置，与 avbd-demo3d 的 dt/gravity/iterations/alpha/gamma/beta_linear/beta_angular 对齐。
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
        .def("__repr__", [](const VBDConfig& c) {
            return "<VBDConfig dt=" + std::to_string(c.dt) +
                   " iterations=" + std::to_string(c.iterations) + ">";
        });

    py::class_<VBDWorld>(m, "VBDWorld", R"pbdoc(
        VBD/AVBD 仿真世界。
    )pbdoc")
        .def(py::init<const Model&, const VBDConfig&>(),
             py::arg("model"),
             py::arg("config") = VBDConfig{})
        .def("step", &VBDWorld::step)
        .def_property_readonly("state", py::overload_cast<>(&VBDWorld::state, py::const_),
             py::return_value_policy::reference_internal)
        .def_property_readonly("model", &VBDWorld::model,
             py::return_value_policy::reference_internal)
        .def_property_readonly("config", &VBDWorld::config,
             py::return_value_policy::reference_internal)
        .def("__repr__", [](const VBDWorld& w) {
            return "<VBDWorld bodies=" + std::to_string(w.model().num_bodies()) + ">";
        });
}
