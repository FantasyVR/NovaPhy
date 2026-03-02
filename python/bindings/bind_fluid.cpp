#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "novaphy/fluid/neighbor_search.h"
#include "novaphy/fluid/particle_state.h"
#include "novaphy/fluid/sph_kernel.h"

namespace py = pybind11;
using namespace novaphy;

void bind_fluid(py::module_& m) {
    // --- SPH Kernels ---
    py::class_<SPHKernels>(m, "SPHKernels", R"pbdoc(
        SPH smoothing kernel functions for fluid simulation.
    )pbdoc")
        .def_static("poly6", &SPHKernels::poly6,
                     py::arg("r_sq"), py::arg("h"),
                     R"pbdoc(
                         Poly6 kernel value for density estimation.

                         Args:
                             r_sq (float): Squared distance between particles.
                             h (float): Support radius.

                         Returns:
                             float: Kernel value W(r, h).
                     )pbdoc")
        .def_static("spiky_grad", &SPHKernels::spiky_grad,
                     py::arg("r"), py::arg("h"),
                     R"pbdoc(
                         Gradient of Spiky kernel for pressure forces.

                         Args:
                             r (Vector3): Vector from particle j to particle i.
                             h (float): Support radius.

                         Returns:
                             Vector3: Gradient of spiky kernel.
                     )pbdoc")
        .def_static("spiky", &SPHKernels::spiky,
                     py::arg("r_len"), py::arg("h"),
                     R"pbdoc(
                         Spiky kernel value.

                         Args:
                             r_len (float): Distance between particles.
                             h (float): Support radius.

                         Returns:
                             float: Kernel value W(r, h).
                     )pbdoc")
        .def_static("cubic_spline", &SPHKernels::cubic_spline,
                     py::arg("r_len"), py::arg("h"),
                     R"pbdoc(
                         Cubic spline kernel value.

                         Args:
                             r_len (float): Distance between particles.
                             h (float): Support radius.

                         Returns:
                             float: Kernel value W(r, h).
                     )pbdoc");

    // --- FluidBlockDef ---
    py::class_<FluidBlockDef>(m, "FluidBlockDef", R"pbdoc(
        Definition of a rectangular block of fluid particles.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates a default fluid block definition.
        )pbdoc")
        .def_readwrite("lower", &FluidBlockDef::lower, R"pbdoc(
            Vector3: Lower corner of the fluid block (m).
        )pbdoc")
        .def_readwrite("upper", &FluidBlockDef::upper, R"pbdoc(
            Vector3: Upper corner of the fluid block (m).
        )pbdoc")
        .def_readwrite("particle_spacing", &FluidBlockDef::particle_spacing, R"pbdoc(
            float: Inter-particle spacing (m).
        )pbdoc")
        .def_readwrite("rest_density", &FluidBlockDef::rest_density, R"pbdoc(
            float: Rest density of the fluid (kg/m^3).
        )pbdoc")
        .def_readwrite("initial_velocity", &FluidBlockDef::initial_velocity, R"pbdoc(
            Vector3: Initial velocity for all particles (m/s).
        )pbdoc");

    // --- ParticleState ---
    py::class_<ParticleState>(m, "ParticleState", R"pbdoc(
        SOA storage for fluid particle simulation state.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates an empty particle state.
        )pbdoc")
        .def("init", &ParticleState::init,
             py::arg("initial_positions"),
             py::arg("initial_velocity") = Vec3f::Zero(),
             R"pbdoc(
                 Initialize particle arrays from positions.

                 Args:
                     initial_positions (list[Vector3]): Starting positions.
                     initial_velocity (Vector3): Initial velocity for all particles.
             )pbdoc")
        .def("clear", &ParticleState::clear, R"pbdoc(
            Clear all particle data.
        )pbdoc")
        .def_property_readonly("num_particles", &ParticleState::num_particles, R"pbdoc(
            int: Number of particles.
        )pbdoc")
        .def_readonly("positions", &ParticleState::positions, R"pbdoc(
            list[Vector3]: Particle positions in world frame (m).
        )pbdoc")
        .def_readonly("velocities", &ParticleState::velocities, R"pbdoc(
            list[Vector3]: Particle velocities in world frame (m/s).
        )pbdoc")
        .def_readonly("densities", &ParticleState::densities, R"pbdoc(
            list[float]: Per-particle densities (kg/m^3).
        )pbdoc")
        .def_readonly("lambdas", &ParticleState::lambdas, R"pbdoc(
            list[float]: PBF constraint multipliers.
        )pbdoc");

    // --- SpatialHashGrid ---
    py::class_<SpatialHashGrid>(m, "SpatialHashGrid", R"pbdoc(
        Uniform spatial hash grid for SPH neighbor queries.
    )pbdoc")
        .def(py::init<float>(),
             py::arg("cell_size") = 0.1f,
             R"pbdoc(
                 Creates a spatial hash grid.

                 Args:
                     cell_size (float): Grid cell size (should match kernel radius).
             )pbdoc")
        .def("build", &SpatialHashGrid::build, py::arg("positions"),
             R"pbdoc(
                 Build grid from particle positions.

                 Args:
                     positions (list[Vector3]): Particle positions.
             )pbdoc")
        .def("clear", &SpatialHashGrid::clear, R"pbdoc(
            Clear all grid data.
        )pbdoc")
        .def("query_neighbors",
             [](const SpatialHashGrid& grid, const Vec3f& point, float radius) {
                 std::vector<int> neighbors;
                 grid.query_neighbors(point, radius, neighbors);
                 return neighbors;
             },
             py::arg("point"), py::arg("radius"),
             R"pbdoc(
                 Query neighbor particle indices within radius.

                 Args:
                     point (Vector3): Query point.
                     radius (float): Search radius.

                 Returns:
                     list[int]: Indices of neighbor particles.
             )pbdoc")
        .def_property("cell_size",
                      &SpatialHashGrid::cell_size,
                      &SpatialHashGrid::set_cell_size,
                      R"pbdoc(
                          float: Grid cell size in meters.
                      )pbdoc");

    // --- Free functions ---
    m.def("generate_fluid_block", &generate_fluid_block,
          py::arg("block_def"),
          R"pbdoc(
              Generate particle positions filling a rectangular block.

              Args:
                  block_def (FluidBlockDef): Block definition.

              Returns:
                  list[Vector3]: Generated particle positions.
          )pbdoc");
}
