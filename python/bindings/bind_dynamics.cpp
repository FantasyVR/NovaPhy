#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "novaphy/core/articulation.h"
#include "novaphy/core/joint.h"
#include "novaphy/dynamics/articulated_solver.h"
#include "novaphy/dynamics/featherstone.h"

namespace py = pybind11;
using namespace novaphy;

void bind_dynamics(py::module_& m) {
    // --- JointType ---
    py::enum_<JointType>(m, "JointType", R"pbdoc(
        Joint model type used by articulated-body dynamics.
    )pbdoc")
        .value("Revolute", JointType::Revolute, R"pbdoc(
            One rotational degree of freedom around `axis` (radians).
        )pbdoc")
        .value("Fixed", JointType::Fixed, R"pbdoc(
            Zero degree-of-freedom rigid attachment.
        )pbdoc")
        .value("Free", JointType::Free, R"pbdoc(
            Six degree-of-freedom floating base joint.
        )pbdoc")
        .value("Slide", JointType::Slide, R"pbdoc(
            One translational degree of freedom along `axis` (meters).
        )pbdoc")
        .value("Ball", JointType::Ball, R"pbdoc(
            Three rotational degrees of freedom for orientation.
        )pbdoc");

    // --- Joint ---
    py::class_<Joint>(m, "Joint", R"pbdoc(
        Joint metadata and kinematic parameters for one articulation link.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates a default joint descriptor.
        )pbdoc")
        .def_readwrite("type", &Joint::type, R"pbdoc(
            Joint type enumeration controlling motion subspace.
        )pbdoc")
        .def_readwrite("axis", &Joint::axis, R"pbdoc(
            Joint axis vector used by revolute/slide joints in local coordinates.
        )pbdoc")
        .def_readwrite("parent", &Joint::parent, R"pbdoc(
            Parent link index (`-1` denotes root/world parent).
        )pbdoc")
        .def_readwrite("parent_to_joint", &Joint::parent_to_joint, R"pbdoc(
            Transform from parent-link frame to joint frame.
        )pbdoc")
        .def("num_q", &Joint::num_q, R"pbdoc(
            Returns the number of generalized position coordinates for this joint.

            Returns:
                int: Number of position coordinates.
        )pbdoc")
        .def("num_qd", &Joint::num_qd, R"pbdoc(
            Returns the number of generalized velocity coordinates for this joint.

            Returns:
                int: Number of velocity coordinates.
        )pbdoc");

    // --- Articulation ---
    py::class_<Articulation>(m, "Articulation", R"pbdoc(
        Tree-structured multibody model used by Featherstone dynamics routines.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates an empty articulation model.
        )pbdoc")
        .def_readwrite("joints", &Articulation::joints, R"pbdoc(
            Ordered joint list (one joint per link).
        )pbdoc")
        .def_readwrite("bodies", &Articulation::bodies, R"pbdoc(
            Per-link rigid-body inertial properties.
        )pbdoc")
        .def("num_links", &Articulation::num_links, R"pbdoc(
            Returns the number of links in the articulation.

            Returns:
                int: Number of links.
        )pbdoc")
        .def("total_q", &Articulation::total_q, R"pbdoc(
            Returns the total generalized-position dimension.

            Returns:
                int: Size of the full `q` vector.
        )pbdoc")
        .def("total_qd", &Articulation::total_qd, R"pbdoc(
            Returns the total generalized-velocity dimension.

            Returns:
                int: Size of the full `qd` vector.
        )pbdoc")
        .def("q_start", &Articulation::q_start, py::arg("link"), R"pbdoc(
            Returns the starting index of a link's position block in `q`.

            Args:
                link (int): Link index in the articulation.

            Returns:
                int: Start index into the generalized-position vector.
        )pbdoc")
        .def("qd_start", &Articulation::qd_start, py::arg("link"), R"pbdoc(
            Returns the starting index of a link's velocity block in `qd`.

            Args:
                link (int): Link index in the articulation.

            Returns:
                int: Start index into the generalized-velocity vector.
        )pbdoc")
        .def("build_spatial_inertias", &Articulation::build_spatial_inertias, R"pbdoc(
            Builds per-link spatial inertia matrices from rigid-body properties.

            Returns:
                None
        )pbdoc");

    // --- Featherstone algorithms ---
    m.def("forward_kinematics", [](const Articulation& model, const VecXf& q) {
        std::vector<SpatialTransform> X_J, X_up;
        std::vector<Transform> X_world;
        featherstone::forward_kinematics(model, q, X_J, X_up, X_world);
        return X_world;
    }, py::arg("model"), py::arg("q"),
    R"pbdoc(
        Computes world-frame transforms for all links from generalized positions.

        Args:
            model (Articulation): Articulation model with joints and bodies.
            q (ndarray): Generalized positions.

        Returns:
            list[Transform]: World transform per link.
    )pbdoc");

    m.def("inverse_dynamics", [](const Articulation& model,
                                 const VecXf& q, const VecXf& qd,
                                 const VecXf& qdd, const Vec3f& gravity) {
        return featherstone::inverse_dynamics(model, q, qd, qdd, gravity);
    }, py::arg("model"), py::arg("q"), py::arg("qd"),
       py::arg("qdd"), py::arg("gravity"),
    R"pbdoc(
        Runs Recursive Newton-Euler inverse dynamics.

        Args:
            model (Articulation): Articulation model.
            q (ndarray): Generalized positions.
            qd (ndarray): Generalized velocities.
            qdd (ndarray): Generalized accelerations.
            gravity (Vector3): Gravity vector in world coordinates (m/s^2).

        Returns:
            ndarray: Required joint efforts (torques/forces).
    )pbdoc");

    m.def("mass_matrix_crba", [](const Articulation& model, const VecXf& q) {
        return featherstone::mass_matrix(model, q);
    }, py::arg("model"), py::arg("q"),
    R"pbdoc(
        Computes the joint-space mass matrix using CRBA.

        Args:
            model (Articulation): Articulation model.
            q (ndarray): Generalized positions.

        Returns:
            ndarray: Symmetric positive-definite mass matrix H(q).
    )pbdoc");

    m.def("forward_dynamics", [](const Articulation& model,
                                 const VecXf& q, const VecXf& qd,
                                 const VecXf& tau, const Vec3f& gravity) {
        return featherstone::forward_dynamics(model, q, qd, tau, gravity);
    }, py::arg("model"), py::arg("q"), py::arg("qd"),
       py::arg("tau"), py::arg("gravity"),
    R"pbdoc(
        Computes generalized accelerations from applied joint efforts.

        Args:
            model (Articulation): Articulation model.
            q (ndarray): Generalized positions.
            qd (ndarray): Generalized velocities.
            tau (ndarray): Applied joint efforts.
            gravity (Vector3): Gravity vector in world coordinates (m/s^2).

        Returns:
            ndarray: Generalized accelerations.
    )pbdoc");

    // --- ArticulatedSolver ---
    py::class_<ArticulatedSolver>(m, "ArticulatedSolver", R"pbdoc(
        Stateful articulated-body integrator built on Featherstone dynamics.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates a solver with default integration settings.
        )pbdoc")
        .def("step", [](ArticulatedSolver& self, const Articulation& model,
                        VecXf q, VecXf qd, const VecXf& tau,
                        const Vec3f& gravity, float dt) {
            self.step(model, q, qd, tau, gravity, dt);
            return std::make_pair(q, qd);
        }, py::arg("model"), py::arg("q"), py::arg("qd"),
           py::arg("tau"), py::arg("gravity"), py::arg("dt"),
        R"pbdoc(
            Advances the articulated system by one time step.

            Args:
                model (Articulation): Articulation model.
                q (ndarray): Current generalized positions.
                qd (ndarray): Current generalized velocities.
                tau (ndarray): Applied joint efforts.
                gravity (Vector3): Gravity vector in world coordinates (m/s^2).
                dt (float): Integration time step in seconds.

            Returns:
                tuple[ndarray, ndarray]: Updated `(q, qd)` state.
        )pbdoc");
}
