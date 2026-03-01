#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "novaphy/ipc/ipc_config.h"
#include "novaphy/ipc/ipc_world.h"

namespace py = pybind11;

void bind_ipc(py::module_& m) {
    using namespace novaphy;

    py::class_<IPCConfig>(m, "IPCConfig", R"pbdoc(
        Configuration for IPC (Incremental Potential Contact) simulation.

        Controls libuipc backend parameters including timestep, contact
        tolerances, friction, and Newton solver settings.
    )pbdoc")
        .def(py::init<>())
        .def_readwrite("dt", &IPCConfig::dt,
            "Timestep in seconds (default: 0.01)")
        .def_readwrite("gravity", &IPCConfig::gravity,
            "Gravity acceleration vector in m/s^2")
        .def_readwrite("d_hat", &IPCConfig::d_hat,
            "Contact distance threshold in meters, mapped to scene contact/d_hat (default: 0.01)")
        .def_readwrite("kappa", &IPCConfig::kappa,
            "Barrier/contact stiffness in Pa used for default contact resistance (default: 1e8)")
        .def_readwrite("friction", &IPCConfig::friction,
            "Default friction coefficient (default: 0.5)")
        .def_readwrite("contact_resistance", &IPCConfig::contact_resistance,
            "Optional contact-resistance override in Pa (default: 1e9)")
        .def_readwrite("body_kappa", &IPCConfig::body_kappa,
            "Affine body stiffness in Pa (default: 1e8)")
        .def_readwrite("mass_density", &IPCConfig::mass_density,
            "Default mass density in kg/m^3 (default: 1e3)")
        .def_readwrite("newton_max_iter", &IPCConfig::newton_max_iter,
            "Max Newton iterations per step, mapped to scene newton/max_iter (default: 100)")
        .def_readwrite("newton_tol", &IPCConfig::newton_tol,
            "Newton tolerance mapped to scene newton/velocity_tol and newton/transrate_tol (default: 1e-2)")
        .def("__repr__", [](const IPCConfig& c) {
            return "<IPCConfig dt=" + std::to_string(c.dt) +
                   " friction=" + std::to_string(c.friction) +
                   " kappa=" + std::to_string(c.kappa) + ">";
        });

    py::class_<IPCWorld>(m, "IPCWorld", R"pbdoc(
        IPC-based simulation world using libuipc backend.

        Provides the same high-level interface as World, but delegates
        contact resolution to libuipc's GPU-accelerated IPC solver,
        which mathematically guarantees no interpenetration.

        Example::

            model = builder.build()
            config = novaphy.IPCConfig()
            config.dt = 0.01
            world = novaphy.IPCWorld(model, config)
            world.step()
    )pbdoc")
        .def(py::init<const Model&, const IPCConfig&>(),
             py::arg("model"),
             py::arg("config") = IPCConfig{},
             "Construct an IPC world from a NovaPhy model.")
        .def("step", &IPCWorld::step,
            "Advance the IPC simulation by one timestep.")
        .def("state", py::overload_cast<>(&IPCWorld::state, py::const_),
             py::return_value_policy::reference_internal,
            "Access current simulation state (read-only).")
        .def("model", &IPCWorld::model,
             py::return_value_policy::reference_internal,
            "Access the model used to build this world.")
        .def("config", &IPCWorld::config,
             py::return_value_policy::reference_internal,
            "Get the IPC configuration.")
        .def("frame", &IPCWorld::frame,
            "Get the current simulation frame number.")
        .def("__repr__", [](const IPCWorld& w) {
            return "<IPCWorld bodies=" + std::to_string(w.model().num_bodies()) +
                   " frame=" + std::to_string(w.frame()) + ">";
        });
}
