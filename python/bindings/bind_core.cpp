#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "novaphy/core/aabb.h"
#include "novaphy/core/body.h"
#include "novaphy/core/contact.h"
#include "novaphy/core/shape.h"

namespace py = pybind11;
using namespace novaphy;

void bind_core(py::module_& m) {
    // --- ShapeType enum ---
    py::enum_<ShapeType>(m, "ShapeType", R"pbdoc(
        Collision shape primitive type.
    )pbdoc")
        .value("Box", ShapeType::Box, R"pbdoc(
            Oriented box primitive.
        )pbdoc")
        .value("Sphere", ShapeType::Sphere, R"pbdoc(
            Sphere primitive.
        )pbdoc")
        .value("Plane", ShapeType::Plane, R"pbdoc(
            Infinite plane primitive.
        )pbdoc");

    // --- RigidBody ---
    py::class_<RigidBody>(m, "RigidBody", R"pbdoc(
        Rigid-body inertial parameters in body-local coordinates.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates a rigid body with default unit mass and identity inertia.
        )pbdoc")
        .def_readwrite("mass", &RigidBody::mass, R"pbdoc(
            float: Body mass in kilograms.
        )pbdoc")
        .def_readwrite("inertia", &RigidBody::inertia, R"pbdoc(
            Matrix3: Inertia tensor in body-local frame about center of mass (kg*m^2).
        )pbdoc")
        .def_readwrite("com", &RigidBody::com, R"pbdoc(
            Vector3: Center of mass in body-local coordinates (m).
        )pbdoc")
        .def("inv_mass", &RigidBody::inv_mass, R"pbdoc(
            Returns inverse mass.

            Returns:
                float: Inverse mass (kg^-1), or 0 for static bodies.
        )pbdoc")
        .def("inv_inertia", &RigidBody::inv_inertia, R"pbdoc(
            Returns inverse inertia tensor in body-local coordinates.

            Returns:
                Matrix3: Inverse inertia tensor, or zero matrix for static bodies.
        )pbdoc")
        .def("is_static", &RigidBody::is_static, R"pbdoc(
            Returns whether the body is static.

            Returns:
                bool: True when body mass is non-positive.
        )pbdoc")
        .def_static("from_box", &RigidBody::from_box,
                     py::arg("mass"), py::arg("half_extents"),
                     R"pbdoc(
                         Constructs a rigid body with solid-box inertia.

                         Args:
                             mass (float): Body mass in kilograms.
                             half_extents (Vector3): Box half extents in meters.

                         Returns:
                             RigidBody: Body with diagonal box inertia tensor.
                     )pbdoc")
        .def_static("from_sphere", &RigidBody::from_sphere,
                     py::arg("mass"), py::arg("radius"),
                     R"pbdoc(
                         Constructs a rigid body with solid-sphere inertia.

                         Args:
                             mass (float): Body mass in kilograms.
                             radius (float): Sphere radius in meters.

                         Returns:
                             RigidBody: Body with isotropic inertia tensor.
                     )pbdoc")
        .def_static("make_static", &RigidBody::make_static, R"pbdoc(
            Creates an immovable rigid body.

            Returns:
                RigidBody: Static body with zero mass and inertia.
        )pbdoc");

    // --- AABB ---
    py::class_<AABB>(m, "AABB", R"pbdoc(
        Axis-aligned bounding box in world coordinates.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates an empty/zero AABB.
        )pbdoc")
        .def(py::init<const Vec3f&, const Vec3f&>(), py::arg("min"), py::arg("max"),
             R"pbdoc(
                 Creates an AABB from min and max corners.

                 Args:
                     min (Vector3): Minimum corner in world coordinates.
                     max (Vector3): Maximum corner in world coordinates.
             )pbdoc")
        .def_readwrite("min", &AABB::min, R"pbdoc(
            Vector3: Minimum corner.
        )pbdoc")
        .def_readwrite("max", &AABB::max, R"pbdoc(
            Vector3: Maximum corner.
        )pbdoc")
        .def("overlaps", &AABB::overlaps, py::arg("other"), R"pbdoc(
            Tests overlap against another AABB.

            Args:
                other (AABB): Other axis-aligned box.

            Returns:
                bool: True if boxes overlap.
        )pbdoc")
        .def("center", &AABB::center, R"pbdoc(
            Returns box center.

            Returns:
                Vector3: Center point.
        )pbdoc")
        .def("half_extents", &AABB::half_extents, R"pbdoc(
            Returns box half extents.

            Returns:
                Vector3: Half extents along x/y/z.
        )pbdoc")
        .def("surface_area", &AABB::surface_area, R"pbdoc(
            Returns AABB surface area.

            Returns:
                float: Surface area in square meters.
        )pbdoc")
        .def("is_valid", &AABB::is_valid, R"pbdoc(
            Returns whether min corner is component-wise <= max corner.

            Returns:
                bool: True if AABB is valid.
        )pbdoc")
        .def_static("from_sphere", &AABB::from_sphere,
                     py::arg("center"), py::arg("radius"),
                     R"pbdoc(
                         Creates an AABB that bounds a sphere.

                         Args:
                             center (Vector3): Sphere center in world coordinates.
                             radius (float): Sphere radius in meters.

                         Returns:
                             AABB: Bounding box of the sphere.
                     )pbdoc");

    // --- CollisionShape ---
    py::class_<CollisionShape>(m, "CollisionShape", R"pbdoc(
        Collision shape descriptor with material and local pose information.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates a default box collision shape.
        )pbdoc")
        .def_readwrite("type", &CollisionShape::type, R"pbdoc(
            ShapeType: Active primitive type.
        )pbdoc")
        .def_readwrite("local_transform", &CollisionShape::local_transform, R"pbdoc(
            Transform: Local shape pose relative to owning body frame.
        )pbdoc")
        .def_readwrite("friction", &CollisionShape::friction, R"pbdoc(
            float: Friction coefficient used by contact solver.
        )pbdoc")
        .def_readwrite("restitution", &CollisionShape::restitution, R"pbdoc(
            float: Restitution coefficient in [0, 1].
        )pbdoc")
        .def_readwrite("body_index", &CollisionShape::body_index, R"pbdoc(
            int: Owning body index, or -1 for world-owned shapes.
        )pbdoc")
        .def_property("box_half_extents",
            [](const CollisionShape& s) { return s.box.half_extents; },
            [](CollisionShape& s, const Vec3f& v) { s.box.half_extents = v; },
            R"pbdoc(
                Vector3: Box half extents in local coordinates (m).
            )pbdoc")
        .def_property("sphere_radius",
            [](const CollisionShape& s) { return s.sphere.radius; },
            [](CollisionShape& s, float r) { s.sphere.radius = r; },
            R"pbdoc(
                float: Sphere radius in meters.
            )pbdoc")
        .def_property("plane_normal",
            [](const CollisionShape& s) { return s.plane.normal; },
            [](CollisionShape& s, const Vec3f& n) { s.plane.normal = n; },
            R"pbdoc(
                Vector3: Plane normal vector.
            )pbdoc")
        .def_property("plane_offset",
            [](const CollisionShape& s) { return s.plane.offset; },
            [](CollisionShape& s, float d) { s.plane.offset = d; },
            R"pbdoc(
                float: Plane offset in meters along normal direction.
            )pbdoc")
        .def_static("make_box", &CollisionShape::make_box,
                     py::arg("half_extents"), py::arg("body_idx"),
                     py::arg("local") = Transform::identity(),
                     py::arg("friction") = 0.5f, py::arg("restitution") = 0.3f,
                     R"pbdoc(
                         Creates a box collision shape.

                         Args:
                             half_extents (Vector3): Box half extents (m).
                             body_idx (int): Owning body index.
                             local (Transform): Local shape pose in body frame.
                             friction (float): Friction coefficient.
                             restitution (float): Restitution coefficient.

                         Returns:
                             CollisionShape: Box shape descriptor.
                     )pbdoc")
        .def_static("make_sphere", &CollisionShape::make_sphere,
                     py::arg("radius"), py::arg("body_idx"),
                     py::arg("local") = Transform::identity(),
                     py::arg("friction") = 0.5f, py::arg("restitution") = 0.3f,
                     R"pbdoc(
                         Creates a sphere collision shape.

                         Args:
                             radius (float): Sphere radius in meters.
                             body_idx (int): Owning body index.
                             local (Transform): Local shape pose in body frame.
                             friction (float): Friction coefficient.
                             restitution (float): Restitution coefficient.

                         Returns:
                             CollisionShape: Sphere shape descriptor.
                     )pbdoc")
        .def_static("make_plane", &CollisionShape::make_plane,
                     py::arg("normal"), py::arg("offset"),
                     py::arg("friction") = 0.5f, py::arg("restitution") = 0.0f,
                     R"pbdoc(
                         Creates an infinite plane collision shape.

                         Args:
                             normal (Vector3): Plane normal vector.
                             offset (float): Plane offset along normal (m).
                             friction (float): Friction coefficient.
                             restitution (float): Restitution coefficient.

                         Returns:
                             CollisionShape: Plane shape descriptor.
                     )pbdoc")
        .def("compute_aabb", &CollisionShape::compute_aabb, py::arg("body_transform"), R"pbdoc(
            Computes world-space AABB for the shape.

            Args:
                body_transform (Transform): Owning body transform in world frame.

            Returns:
                AABB: World-space bounding box.
        )pbdoc");

    // --- ContactPoint ---
    py::class_<ContactPoint>(m, "ContactPoint", R"pbdoc(
        One world-space contact sample between two bodies/shapes.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates an empty contact point.
        )pbdoc")
        .def_readwrite("position", &ContactPoint::position, R"pbdoc(
            Vector3: Contact position in world coordinates (m).
        )pbdoc")
        .def_readwrite("normal", &ContactPoint::normal, R"pbdoc(
            Vector3: Contact normal from body A toward body B.
        )pbdoc")
        .def_readwrite("penetration", &ContactPoint::penetration, R"pbdoc(
            float: Penetration depth in meters.
        )pbdoc")
        .def_readwrite("body_a", &ContactPoint::body_a, R"pbdoc(
            int: First body index (`-1` may denote world).
        )pbdoc")
        .def_readwrite("body_b", &ContactPoint::body_b, R"pbdoc(
            int: Second body index (`-1` may denote world).
        )pbdoc")
        .def_readwrite("friction", &ContactPoint::friction, R"pbdoc(
            float: Combined friction coefficient at the contact.
        )pbdoc")
        .def_readwrite("restitution", &ContactPoint::restitution, R"pbdoc(
            float: Combined restitution coefficient at the contact.
        )pbdoc");
}
