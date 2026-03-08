#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "novaphy/fluid/boundary.h"
#include "novaphy/fluid/fluid_world.h"
#include "novaphy/fluid/neighbor_search.h"
#include "novaphy/fluid/particle_state.h"
#include "novaphy/fluid/pbf_solver.h"
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
        .def("init",
             [](ParticleState& self,
                const std::vector<Vec3f>& initial_positions,
                const Vec3f& initial_velocity) {
                 self.init(initial_positions, initial_velocity);
             },
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
        .def("build",
             [](SpatialHashGrid& grid, const std::vector<Vec3f>& positions) {
                 grid.build(positions);
             },
             py::arg("positions"),
             R"pbdoc(
                 Build grid from particle positions.

                 Args:
                     positions (list[Vector3]): Particle positions.
             )pbdoc")
        .def("clear", &SpatialHashGrid::clear, R"pbdoc(
            Clear all grid data.
        )pbdoc")
        .def("query_neighbors",
             py::overload_cast<const Vec3f&, float>(&SpatialHashGrid::query_neighbors, py::const_),
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

    // --- PBFSettings ---
    py::class_<PBFSettings>(m, "PBFSettings", R"pbdoc(
        Configuration for the PBF fluid solver.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Creates PBF settings with default values.
        )pbdoc")
        .def_readwrite("rest_density", &PBFSettings::rest_density, R"pbdoc(
            float: Rest density rho_0 (kg/m^3).
        )pbdoc")
        .def_readwrite("kernel_radius", &PBFSettings::kernel_radius, R"pbdoc(
            float: SPH kernel support radius h (m).
        )pbdoc")
        .def_readwrite("particle_radius", &PBFSettings::particle_radius, R"pbdoc(
            float: Particle visual/collision radius (m).
        )pbdoc")
        .def_readwrite("solver_iterations", &PBFSettings::solver_iterations, R"pbdoc(
            int: Number of PBF constraint iterations.
        )pbdoc")
        .def_readwrite("epsilon", &PBFSettings::epsilon, R"pbdoc(
            float: Relaxation parameter for constraint (CFM).
        )pbdoc")
        .def_readwrite("xsph_viscosity", &PBFSettings::xsph_viscosity, R"pbdoc(
            float: XSPH artificial viscosity coefficient.
        )pbdoc")
        .def_readwrite("use_domain_bounds", &PBFSettings::use_domain_bounds, R"pbdoc(
            bool: Whether to clamp particles to domain AABB.
        )pbdoc")
        .def_readwrite("domain_lower", &PBFSettings::domain_lower, R"pbdoc(
            Vector3: Domain lower bound (m).
        )pbdoc")
        .def_readwrite("domain_upper", &PBFSettings::domain_upper, R"pbdoc(
            Vector3: Domain upper bound (m).
        )pbdoc")
        .def("particle_mass", &PBFSettings::particle_mass,
             py::arg("spacing"),
             R"pbdoc(
                 Compute particle mass from rest density and spacing.

                 Args:
                     spacing (float): Inter-particle spacing (m).

                 Returns:
                     float: Particle mass (kg).
             )pbdoc");

    // --- PBFSolver ---
    py::class_<PBFSolver>(m, "PBFSolver", R"pbdoc(
        Position Based Fluids solver (Macklin & Muller, SIGGRAPH 2013).
    )pbdoc")
        .def(py::init<const PBFSettings&>(),
             py::arg("settings") = PBFSettings{},
             R"pbdoc(
                 Creates a PBF solver.

                 Args:
                     settings (PBFSettings): Solver configuration.
             )pbdoc")
        .def("step", &PBFSolver::step,
             py::arg("state"), py::arg("dt"), py::arg("gravity"),
             py::arg("particle_mass"),
             R"pbdoc(
                 Run one PBF step on a particle state.

                 Args:
                     state (ParticleState): Mutable particle state.
                     dt (float): Time step (s).
                     gravity (Vector3): Gravity vector (m/s^2).
                     particle_mass (float): Mass per particle (kg).
             )pbdoc")
        .def_property_readonly("settings",
             py::overload_cast<>(&PBFSolver::settings),
             py::return_value_policy::reference_internal,
             R"pbdoc(
                 PBFSettings: Solver settings (reference).
             )pbdoc");

    // --- BoundaryParticle ---
    py::class_<BoundaryParticle>(m, "BoundaryParticle", R"pbdoc(
        A boundary particle sampled from a rigid body surface.
    )pbdoc")
        .def(py::init<>())
        .def_readwrite("local_position", &BoundaryParticle::local_position, R"pbdoc(
            Vector3: Position in body-local frame (m).
        )pbdoc")
        .def_readwrite("body_index", &BoundaryParticle::body_index, R"pbdoc(
            int: Owning rigid body index (-1 = world-owned).
        )pbdoc")
        .def_readwrite("volume", &BoundaryParticle::volume, R"pbdoc(
            float: Akinci volume psi for density contribution.
        )pbdoc");

    // --- FluidWorld ---
    py::class_<FluidWorld, World>(m, "FluidWorld", R"pbdoc(
        Extended simulation world with PBF fluid and rigid-fluid coupling.
    )pbdoc")
        .def(py::init<const Model&, const std::vector<FluidBlockDef>&,
                       SolverSettings, PBFSettings, float>(),
             py::arg("model"),
             py::arg("fluid_blocks") = std::vector<FluidBlockDef>{},
             py::arg("solver_settings") = SolverSettings{},
             py::arg("pbf_settings") = PBFSettings{},
             py::arg("boundary_extent") = 1.0f,
             R"pbdoc(
                 Creates a fluid simulation world with rigid-fluid coupling.

                 Args:
                     model (Model): Rigid-body model.
                     fluid_blocks (list[FluidBlockDef]): Fluid block definitions.
                     solver_settings (SolverSettings): Contact solver settings.
                     pbf_settings (PBFSettings): PBF solver settings.
                     boundary_extent (float): Half-extent for plane boundary sampling (m).
             )pbdoc")
        .def("step", &FluidWorld::step, py::arg("dt"),
             R"pbdoc(
                 Advance rigid-body and fluid simulation by one time step.

                 Args:
                     dt (float): Time step in seconds.
             )pbdoc")
        .def_property_readonly("fluid_state",
             py::overload_cast<>(&FluidWorld::fluid_state),
             py::return_value_policy::reference_internal,
             R"pbdoc(
                 ParticleState: Fluid particle state (reference).
             )pbdoc")
        .def_property_readonly("num_particles", &FluidWorld::num_particles, R"pbdoc(
            int: Number of fluid particles.
        )pbdoc")
        .def_property_readonly("num_boundary_particles", &FluidWorld::num_boundary_particles,
             R"pbdoc(
                 int: Number of boundary particles.
             )pbdoc")
        .def_property_readonly("boundary_particles", &FluidWorld::boundary_particles,
             py::return_value_policy::reference_internal,
             R"pbdoc(
                 list[BoundaryParticle]: Boundary particles for rigid-fluid coupling.
             )pbdoc")
        .def_property_readonly("pbf_settings",
             py::overload_cast<>(&FluidWorld::pbf_settings),
             py::return_value_policy::reference_internal,
             R"pbdoc(
                 PBFSettings: PBF solver settings (reference).
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

    m.def("sample_model_boundaries", &sample_model_boundaries,
          py::arg("model"), py::arg("spacing"),
          py::arg("plane_extent") = 1.0f,
          R"pbdoc(
              Sample boundary particles from all shapes in a model.

              Args:
                  model (Model): Scene model.
                  spacing (float): Boundary particle spacing (m).
                  plane_extent (float): Half-extent for plane boundaries (m).

              Returns:
                  list[BoundaryParticle]: Boundary particles.
          )pbdoc");
}
