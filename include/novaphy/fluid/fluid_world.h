#pragma once

#include "novaphy/fluid/boundary.h"
#include "novaphy/fluid/particle_state.h"
#include "novaphy/fluid/pbf_solver.h"
#include "novaphy/sim/world.h"

namespace novaphy {

/**
 * @brief Extended simulation world with fluid (PBF) and rigid-fluid coupling.
 *
 * @details Inherits rigid-body simulation from World and adds
 * a PBF fluid solver with Akinci boundary particle coupling.
 * The step() method runs rigid-body, fluid, and coupling solvers.
 */
class FluidWorld : public World {
public:
    /**
     * @brief Construct a fluid world from a model and fluid definitions.
     *
     * @param[in] model Rigid-body model.
     * @param[in] fluid_blocks Fluid block definitions for particle emission.
     * @param[in] solver_settings Rigid-body contact solver settings.
     * @param[in] pbf_settings PBF fluid solver settings.
     * @param[in] boundary_extent Half-extent for plane boundary sampling (m).
     */
    FluidWorld(const Model& model,
               const std::vector<FluidBlockDef>& fluid_blocks = {},
               SolverSettings solver_settings = {},
               PBFSettings pbf_settings = {},
               float boundary_extent = 1.0f);

    /**
     * @brief Advance both rigid-body and fluid simulation by one time step.
     *
     * @param[in] dt Time step (s).
     */
    void step(float dt);

    /**
     * @brief Read-only access to fluid particle state.
     *
     * @return Const reference to ParticleState.
     */
    const ParticleState& fluid_state() const { return fluid_state_; }

    /**
     * @brief Mutable access to fluid particle state.
     *
     * @return Reference to ParticleState.
     */
    ParticleState& fluid_state() { return fluid_state_; }

    /**
     * @brief Mutable access to PBF solver settings.
     *
     * @return Reference to PBFSettings.
     */
    PBFSettings& pbf_settings() { return pbf_solver_.settings(); }

    /**
     * @brief Read-only access to PBF solver settings.
     *
     * @return Const reference to PBFSettings.
     */
    const PBFSettings& pbf_settings() const { return pbf_solver_.settings(); }

    /**
     * @brief Get number of fluid particles.
     *
     * @return Particle count.
     */
    int num_particles() const { return fluid_state_.num_particles(); }

    /**
     * @brief Get number of boundary particles.
     *
     * @return Boundary particle count.
     */
    int num_boundary_particles() const { return static_cast<int>(boundary_particles_.size()); }

    /**
     * @brief Read-only access to boundary particles.
     *
     * @return Const reference to boundary particle vector.
     */
    const std::vector<BoundaryParticle>& boundary_particles() const { return boundary_particles_; }

private:
    /**
     * @brief Apply boundary density contribution to fluid particles.
     */
    void apply_boundary_density(const std::vector<Vec3f>& boundary_world_pos);

    /**
     * @brief Compute coupling forces from fluid onto rigid bodies.
     */
    void apply_coupling_forces(const std::vector<Vec3f>& boundary_world_pos, float dt);

    ParticleState fluid_state_;
    PBFSolver pbf_solver_;
    std::vector<BoundaryParticle> boundary_particles_;
    SpatialHashGrid boundary_grid_;
    float particle_mass_ = 0.0f;
    float particle_spacing_ = 0.02f;
};

}  // namespace novaphy
