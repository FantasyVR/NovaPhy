#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "novaphy/collision/broadphase.h"
#include "novaphy/collision/narrowphase.h"

namespace py = pybind11;
using namespace novaphy;

void bind_collision(py::module_& m) {
    // --- BroadPhasePair ---
    py::class_<BroadPhasePair>(m, "BroadPhasePair", R"pbdoc(
        Candidate overlap pair produced by broadphase culling.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates an empty broadphase pair.
        )pbdoc")
        .def_readwrite("body_a", &BroadPhasePair::body_a, R"pbdoc(
            int: First body/shape index of the overlap pair.
        )pbdoc")
        .def_readwrite("body_b", &BroadPhasePair::body_b, R"pbdoc(
            int: Second body/shape index of the overlap pair.
        )pbdoc");

    // --- SweepAndPrune ---
    py::class_<SweepAndPrune>(m, "SweepAndPrune", R"pbdoc(
        Sweep-and-Prune broadphase collision detector.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates an empty broadphase accelerator.
        )pbdoc")
        .def("update", &SweepAndPrune::update,
             py::arg("body_aabbs"), py::arg("static_mask"),
             R"pbdoc(
                 Updates overlap candidates from current world-space AABBs.

                 Args:
                     body_aabbs (list[AABB]): Per-body world-space AABBs.
                     static_mask (list[bool]): Static-body mask for pruning static-static pairs.

                 Returns:
                     None
             )pbdoc")
        .def("get_pairs", &SweepAndPrune::get_pairs, R"pbdoc(
            Returns candidate overlap pairs from the latest update.

            Returns:
                list[BroadPhasePair]: Potentially overlapping body/shape pairs.
        )pbdoc");

    // --- Collision dispatcher ---
    // Return (bool, list[ContactPoint]) tuple instead of passing contacts by reference
    m.def("collide_shapes",
          [](const CollisionShape& a, const Transform& ta,
             const CollisionShape& b, const Transform& tb) {
              std::vector<ContactPoint> contacts;
              bool result = collide_shapes(a, ta, b, tb, contacts);
              return std::make_pair(result, contacts);
          },
          py::arg("shape_a"), py::arg("transform_a"),
          py::arg("shape_b"), py::arg("transform_b"),
          R"pbdoc(
              Tests collision between two shapes and returns contact data.

              Args:
                  shape_a (CollisionShape): First shape.
                  transform_a (Transform): World transform for shape A parent body.
                  shape_b (CollisionShape): Second shape.
                  transform_b (Transform): World transform for shape B parent body.

              Returns:
                  tuple[bool, list[ContactPoint]]: `(hit, contacts)` where `hit`
                  indicates overlap and `contacts` stores world-space manifold points.
          )pbdoc");
}
