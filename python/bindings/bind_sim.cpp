#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "novaphy/core/model.h"
#include "novaphy/core/model_builder.h"
#include "novaphy/sim/state.h"
#include "novaphy/sim/world.h"

namespace py = pybind11;
using namespace novaphy;

void bind_sim(py::module_& m) {
    // --- ModelBuilder ---
    py::class_<ModelBuilder>(m, "ModelBuilder", R"pbdoc(
        Builder for constructing an immutable simulation model.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates an empty model builder.
        )pbdoc")
        .def("add_body", &ModelBuilder::add_body,
             py::arg("body"),
             py::arg("transform") = Transform::identity(),
             R"pbdoc(
                 Adds a rigid body and returns its index.

                 Args:
                     body (RigidBody): Body mass and inertia properties.
                     transform (Transform): Initial world transform.

                 Returns:
                     int: New body index.
             )pbdoc")
        .def("add_shape", &ModelBuilder::add_shape,
             py::arg("shape"),
             R"pbdoc(
                 Adds a collision shape and returns its index.

                 Args:
                     shape (CollisionShape): Shape attached to a body or world.

                 Returns:
                     int: New shape index.
             )pbdoc")
        .def("add_ground_plane", &ModelBuilder::add_ground_plane,
             py::arg("y") = 0.0f,
             py::arg("friction") = 0.5f,
             py::arg("restitution") = 0.0f,
             R"pbdoc(
                 Adds an infinite ground plane shape.

                 Args:
                     y (float): Plane offset along +Y world axis in meters.
                     friction (float): Friction coefficient used by contact solver.
                     restitution (float): Restitution coefficient in [0, 1].

                 Returns:
                     int: New shape index.
             )pbdoc")
        .def("build", &ModelBuilder::build, R"pbdoc(
            Builds an immutable `Model` from accumulated bodies and shapes.

            Returns:
                Model: Immutable model object.
        )pbdoc")
        .def_property_readonly("num_bodies", &ModelBuilder::num_bodies, R"pbdoc(
            int: Number of currently added bodies.
        )pbdoc")
        .def_property_readonly("num_shapes", &ModelBuilder::num_shapes, R"pbdoc(
            int: Number of currently added shapes.
        )pbdoc");

    // --- Model ---
    py::class_<Model>(m, "Model", R"pbdoc(
        Immutable collection of rigid bodies and collision shapes.
    )pbdoc")
        .def_property_readonly("num_bodies", &Model::num_bodies, R"pbdoc(
            int: Number of bodies in the model.
        )pbdoc")
        .def_property_readonly("num_shapes", &Model::num_shapes, R"pbdoc(
            int: Number of shapes in the model.
        )pbdoc")
        .def_readonly("bodies", &Model::bodies, R"pbdoc(
            list[RigidBody]: Body inertial properties.
        )pbdoc")
        .def_readonly("shapes", &Model::shapes, R"pbdoc(
            list[CollisionShape]: Collision shape definitions.
        )pbdoc");

    // --- SolverSettings ---
    py::class_<SolverSettings>(m, "SolverSettings", R"pbdoc(
        Configuration for the contact solver iteration and stabilization.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates solver settings with default stable values.
        )pbdoc")
        .def_readwrite("velocity_iterations", &SolverSettings::velocity_iterations, R"pbdoc(
            int: Number of PGS velocity iterations per time step.
        )pbdoc")
        .def_readwrite("baumgarte", &SolverSettings::baumgarte, R"pbdoc(
            float: Position-error correction factor (dimensionless).
        )pbdoc")
        .def_readwrite("slop", &SolverSettings::slop, R"pbdoc(
            float: Penetration allowance before correction (meters).
        )pbdoc")
        .def_readwrite("warm_starting", &SolverSettings::warm_starting, R"pbdoc(
            bool: Reuse previous-frame impulses for faster convergence.
        )pbdoc");

    // --- SimState ---
    py::class_<SimState>(m, "SimState", R"pbdoc(
        Mutable simulation state for all bodies in the world.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates an empty simulation state.
        )pbdoc")
        .def_readonly("transforms", &SimState::transforms, R"pbdoc(
            list[Transform]: World transforms per body.
        )pbdoc")
        .def_readonly("linear_velocities", &SimState::linear_velocities, R"pbdoc(
            list[Vector3]: Linear velocities in world coordinates (m/s).
        )pbdoc")
        .def_readonly("angular_velocities", &SimState::angular_velocities, R"pbdoc(
            list[Vector3]: Angular velocities in world coordinates (rad/s).
        )pbdoc")
        .def("set_linear_velocity", &SimState::set_linear_velocity,
             py::arg("body_index"), py::arg("velocity"),
             R"pbdoc(
                 Sets one body's linear velocity.

                 Args:
                     body_index (int): Body index.
                     velocity (Vector3): Velocity in world coordinates (m/s).

                 Returns:
                     None
             )pbdoc")
        .def("set_angular_velocity", &SimState::set_angular_velocity,
             py::arg("body_index"), py::arg("velocity"),
             R"pbdoc(
                 Sets one body's angular velocity.

                 Args:
                     body_index (int): Body index.
                     velocity (Vector3): Angular velocity in world coordinates (rad/s).

                 Returns:
                     None
             )pbdoc")
        .def("apply_force", &SimState::apply_force, py::arg("body_index"), py::arg("force"),
             R"pbdoc(
                 Accumulates an external force at body center of mass.

                 Args:
                     body_index (int): Body index.
                     force (Vector3): Force in world coordinates (N).

                 Returns:
                     None
             )pbdoc")
        .def("apply_torque", &SimState::apply_torque, py::arg("body_index"), py::arg("torque"),
             R"pbdoc(
                 Accumulates an external torque on a body.

                 Args:
                     body_index (int): Body index.
                     torque (Vector3): Torque in world coordinates (N*m).

                 Returns:
                     None
             )pbdoc");

    // --- World ---
    py::class_<World>(m, "World", R"pbdoc(
        Top-level container that advances free-rigid-body simulation.
    )pbdoc")
        .def(py::init<const Model&, SolverSettings>(),
             py::arg("model"),
             py::arg("solver_settings") = SolverSettings{},
             R"pbdoc(
                 Creates a simulation world from an immutable model.

                 Args:
                     model (Model): Immutable model definition.
                     solver_settings (SolverSettings): Contact solver parameters.
             )pbdoc")
        .def("step", &World::step, py::arg("dt"),
             R"pbdoc(
                 Advances simulation by one fixed time step.

                 Args:
                     dt (float): Time step in seconds.

                 Returns:
                     None
             )pbdoc")
        .def("set_gravity", &World::set_gravity, py::arg("gravity"),
             R"pbdoc(
                 Sets the world gravity vector.

                 Args:
                     gravity (Vector3): Gravity in world coordinates (m/s^2).

                 Returns:
                     None
             )pbdoc")
        .def_property_readonly("gravity", &World::gravity, R"pbdoc(
            Vector3: Current gravity vector in world coordinates (m/s^2).
        )pbdoc")
        .def_property_readonly("state", py::overload_cast<>(&World::state),
             py::return_value_policy::reference_internal,
             R"pbdoc(
                 SimState: Mutable world state (reference).
             )pbdoc")
        .def_property_readonly("model", &World::model,
             py::return_value_policy::reference_internal,
             R"pbdoc(
                 Model: Immutable world model (reference).
             )pbdoc")
        .def_property_readonly("contacts", &World::contacts,
             py::return_value_policy::reference_internal,
             R"pbdoc(
                 list[ContactPoint]: Contact points generated during last step.
             )pbdoc")
        .def("apply_force", &World::apply_force,
             py::arg("body_index"), py::arg("force"),
             R"pbdoc(
                 Applies an external force for the next simulation step.

                 Args:
                     body_index (int): Body index.
                     force (Vector3): Force in world coordinates (N).

                 Returns:
                     None
             )pbdoc")
        .def("apply_torque", &World::apply_torque,
             py::arg("body_index"), py::arg("torque"),
             R"pbdoc(
                 Applies an external torque for the next simulation step.

                 Args:
                     body_index (int): Body index.
                     torque (Vector3): Torque in world coordinates (N*m).

                 Returns:
                     None
             )pbdoc");
}
