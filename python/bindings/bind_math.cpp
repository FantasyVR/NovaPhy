#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include "novaphy/math/math_types.h"
#include "novaphy/math/math_utils.h"
#include "novaphy/math/spatial.h"

namespace py = pybind11;
using namespace novaphy;

void bind_math(py::module_& m) {
    // --- Transform ---
    py::class_<Transform>(m, "Transform", R"pbdoc(
        Rigid transform represented by translation and quaternion orientation.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates an identity transform.
        )pbdoc")
        .def(py::init([](const Vec3f& pos, const Vec4f& rot) {
            // rot = [x, y, z, w] (Eigen convention for coeffs())
            return Transform(pos, Quatf(rot[3], rot[0], rot[1], rot[2]));
        }), py::arg("position"), py::arg("rotation"),
            R"pbdoc(
                Creates a transform from position and quaternion.

                Args:
                    position (Vector3): Translation in world coordinates (m).
                    rotation (Vector4): Quaternion as [x, y, z, w].
            )pbdoc")
        .def_readwrite("position", &Transform::position, R"pbdoc(
            Vector3: Translation component in meters.
        )pbdoc")
        .def_property("rotation",
            [](const Transform& t) -> Vec4f {
                // Return as [x, y, z, w]
                return Vec4f(t.rotation.x(), t.rotation.y(),
                             t.rotation.z(), t.rotation.w());
            },
            [](Transform& t, const Vec4f& q) {
                t.rotation = Quatf(q[3], q[0], q[1], q[2]);
            },
            R"pbdoc(
                Quaternion orientation as [x, y, z, w].
            )pbdoc")
        .def("transform_point", &Transform::transform_point, py::arg("point"), R"pbdoc(
            Transforms a point from local frame to world frame.

            Args:
                point (Vector3): Local-space point.

            Returns:
                Vector3: World-space point (m).
        )pbdoc")
        .def("transform_vector", &Transform::transform_vector, py::arg("vector"), R"pbdoc(
            Rotates a direction vector without translation.

            Args:
                vector (Vector3): Local-space direction vector.

            Returns:
                Vector3: Rotated world-space direction.
        )pbdoc")
        .def("inverse", &Transform::inverse, R"pbdoc(
            Computes inverse rigid transform.

            Returns:
                Transform: Inverse transform.
        )pbdoc")
        .def("rotation_matrix", &Transform::rotation_matrix, R"pbdoc(
            Returns 3x3 rotation matrix corresponding to quaternion orientation.

            Returns:
                Matrix3: Rotation matrix.
        )pbdoc")
        .def("__mul__", &Transform::operator*, py::arg("other"))
        .def_static("identity", &Transform::identity, R"pbdoc(
            Creates identity transform.

            Returns:
                Transform: Identity transform.
        )pbdoc")
        .def_static("from_translation", &Transform::from_translation, py::arg("t"), R"pbdoc(
            Creates translation-only transform.

            Args:
                t (Vector3): Translation vector (m).

            Returns:
                Transform: Transform with identity rotation.
        )pbdoc")
        .def_static("from_rotation", [](const Vec4f& q) {
            return Transform::from_rotation(Quatf(q[3], q[0], q[1], q[2]));
        }, py::arg("q"), R"pbdoc(
            Creates rotation-only transform.

            Args:
                q (Vector4): Quaternion as [x, y, z, w].

            Returns:
                Transform: Transform with zero translation.
        )pbdoc")
        .def_static("from_axis_angle", &Transform::from_axis_angle,
                     py::arg("axis"), py::arg("angle"), R"pbdoc(
                         Creates rotation-only transform from axis-angle representation.

                         Args:
                             axis (Vector3): Rotation axis.
                             angle (float): Rotation angle in radians.

                         Returns:
                             Transform: Rotation-only transform.
                     )pbdoc")
        .def("__repr__", [](const Transform& t) {
            return "Transform(pos=[" + std::to_string(t.position.x()) + ", " +
                   std::to_string(t.position.y()) + ", " +
                   std::to_string(t.position.z()) + "], rot=[" +
                   std::to_string(t.rotation.w()) + ", " +
                   std::to_string(t.rotation.x()) + ", " +
                   std::to_string(t.rotation.y()) + ", " +
                   std::to_string(t.rotation.z()) + "])";
        });

    // --- SpatialTransform ---
    py::class_<SpatialTransform>(m, "SpatialTransform", R"pbdoc(
        Spatial transform for Featherstone-style spatial vectors.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates identity spatial transform.
        )pbdoc")
        .def(py::init<const Mat3f&, const Vec3f&>(), py::arg("E"), py::arg("r"), R"pbdoc(
            Creates spatial transform X_{A<-B}.

            Args:
                E (Matrix3): Rotation from frame B to frame A.
                r (Vector3): Origin of B expressed in A (m).
        )pbdoc")
        .def_readwrite("E", &SpatialTransform::E, R"pbdoc(
            Matrix3: Rotation from frame B to frame A.
        )pbdoc")
        .def_readwrite("r", &SpatialTransform::r, R"pbdoc(
            Vector3: Origin of frame B expressed in frame A (m).
        )pbdoc")
        .def("apply_motion", &SpatialTransform::apply_motion, py::arg("v"), R"pbdoc(
            Applies this transform to a spatial motion vector.

            Args:
                v (ndarray): Spatial motion vector [angular; linear].

            Returns:
                ndarray: Transformed spatial motion vector.
        )pbdoc")
        .def("apply_force", &SpatialTransform::apply_force, py::arg("f"), R"pbdoc(
            Applies dual transform to a spatial force vector.

            Args:
                f (ndarray): Spatial force vector [moment; force].

            Returns:
                ndarray: Transformed spatial force vector.
        )pbdoc")
        .def("to_matrix", &SpatialTransform::to_matrix, R"pbdoc(
            Converts transform to 6x6 matrix form.

            Returns:
                ndarray: Spatial transform matrix.
        )pbdoc")
        .def("inverse", &SpatialTransform::inverse, R"pbdoc(
            Computes inverse spatial transform.

            Returns:
                SpatialTransform: Inverse transform.
        )pbdoc")
        .def("__mul__", &SpatialTransform::operator*, py::arg("other"))
        .def_static("from_transform", &SpatialTransform::from_transform, py::arg("t"), R"pbdoc(
            Converts rigid `Transform` to `SpatialTransform`.

            Args:
                t (Transform): Rigid transform.

            Returns:
                SpatialTransform: Equivalent spatial transform.
        )pbdoc")
        .def_static("identity", &SpatialTransform::identity, R"pbdoc(
            Creates identity spatial transform.

            Returns:
                SpatialTransform: Identity transform.
        )pbdoc");

    // --- Free functions ---
    m.def("skew", &skew, py::arg("v"), R"pbdoc(
        Builds skew-symmetric matrix [v]_x for cross-product operations.

        Args:
            v (Vector3): Input vector.

        Returns:
            Matrix3: Skew-symmetric matrix.
    )pbdoc");
    m.def("spatial_cross_motion", &spatial_cross_motion, py::arg("v"), py::arg("u"),
          R"pbdoc(
              Computes spatial cross product for motion vectors.

              Args:
                  v (ndarray): Left spatial motion vector [angular; linear].
                  u (ndarray): Right spatial motion vector [angular; linear].

              Returns:
                  ndarray: Motion cross product.
          )pbdoc");
    m.def("spatial_cross_force", &spatial_cross_force, py::arg("v"), py::arg("f"),
          R"pbdoc(
              Computes spatial cross product for force vectors.

              Args:
                  v (ndarray): Spatial motion vector [angular; linear].
                  f (ndarray): Spatial force vector [moment; force].

              Returns:
                  ndarray: Force cross product.
          )pbdoc");
    m.def("spatial_inertia_matrix", &spatial_inertia_matrix,
          py::arg("mass"), py::arg("com"), py::arg("I_rot"),
          R"pbdoc(
              Builds 6x6 spatial inertia matrix from rigid-body parameters.

              Args:
                  mass (float): Body mass in kilograms.
                  com (Vector3): Center of mass in body coordinates (m).
                  I_rot (Matrix3): Rotational inertia about CoM (kg*m^2).

              Returns:
                  ndarray: Spatial inertia matrix.
          )pbdoc");
    m.def("deg2rad", &deg2rad, py::arg("deg"), R"pbdoc(
        Converts degrees to radians.

        Args:
            deg (float): Angle in degrees.

        Returns:
            float: Angle in radians.
    )pbdoc");
    m.def("rad2deg", &rad2deg, py::arg("rad"), R"pbdoc(
        Converts radians to degrees.

        Args:
            rad (float): Angle in radians.

        Returns:
            float: Angle in degrees.
    )pbdoc");
}
