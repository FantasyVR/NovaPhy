/**
 * @file fluid_world.cpp
 * @brief Extended simulation world with PBF fluid and rigid-fluid coupling.
 */
#include "novaphy/fluid/fluid_world.h"

#include "novaphy/fluid/sph_kernel.h"

namespace novaphy {

FluidWorld::FluidWorld(const Model& model,
                       const std::vector<FluidBlockDef>& fluid_blocks,
                       SolverSettings solver_settings,
                       PBFSettings pbf_settings,
                       float boundary_extent)
    : World(model, solver_settings),
      pbf_solver_(pbf_settings),
      boundary_grid_(pbf_settings.kernel_radius) {
    // Generate fluid particles from all blocks
    std::vector<Vec3f> all_positions;
    Vec3f initial_vel = Vec3f::Zero();

    for (const auto& block : fluid_blocks) {
        auto block_positions = generate_fluid_block(block);
        all_positions.insert(all_positions.end(),
                             block_positions.begin(), block_positions.end());
        initial_vel = block.initial_velocity;
        particle_spacing_ = block.particle_spacing;
    }

    if (!all_positions.empty()) {
        fluid_state_.init(all_positions, initial_vel);
        particle_mass_ = pbf_settings.particle_mass(particle_spacing_);
    }

    // Generate boundary particles from rigid body shapes
    boundary_particles_ = sample_model_boundaries(model, particle_spacing_, boundary_extent);

    // Compute initial boundary volumes (Akinci)
    if (!boundary_particles_.empty()) {
        auto bpos = boundary_world_positions(boundary_particles_, state().transforms);
        compute_boundary_volumes(boundary_particles_, bpos, pbf_settings.kernel_radius);
    }
}

void FluidWorld::step(float dt) {
    PerformanceMonitor& monitor = performance_monitor();
    monitor.begin_frame();
    detail::ScopedPerformanceCaptureContext capture_context(&monitor);

    {
        detail::PerformancePhaseScope total_scope(&monitor, "fluid.total");

        int n_fluid = fluid_state_.num_particles();
        int n_boundary = static_cast<int>(boundary_particles_.size());

        if (n_fluid > 0) {
            std::vector<Vec3f> boundary_pos;
            if (n_boundary > 0) {
                detail::PerformancePhaseScope phase_scope(&monitor,
                                                          "fluid.boundary.positions");
                boundary_pos =
                    boundary_world_positions(boundary_particles_, state().transforms);
            }

            {
                detail::PerformancePhaseScope phase_scope(&monitor, "fluid.pbf.total");
                pbf_solver_.step(fluid_state_, dt, gravity(), particle_mass_);
            }

            if (n_boundary > 0) {
                detail::PerformancePhaseScope phase_scope(&monitor,
                                                          "fluid.boundary_density");
                apply_boundary_density(boundary_pos);
            }

            if (n_boundary > 0) {
                detail::PerformancePhaseScope phase_scope(&monitor, "fluid.coupling.total");
                apply_coupling_forces(boundary_pos, dt);
            }
        }

        {
            detail::PerformancePhaseScope phase_scope(&monitor, "fluid.rigid_step");
            step_rigid_pipeline(dt);
        }

        monitor.record_metric("fluid_particles",
                              static_cast<double>(fluid_state_.num_particles()));
        monitor.record_metric("boundary_particles",
                              static_cast<double>(boundary_particles_.size()));
        monitor.record_metric("pbf_solver_iterations",
                              static_cast<double>(pbf_solver_.settings().solver_iterations));
        monitor.record_metric("kernel_radius",
                              static_cast<double>(pbf_solver_.settings().kernel_radius));
    }

    monitor.end_frame();
}

void FluidWorld::apply_boundary_density(const std::vector<Vec3f>& boundary_world_pos) {
    int n_fluid = fluid_state_.num_particles();
    float h = pbf_solver_.settings().kernel_radius;

    // Build grid for boundary particles
    boundary_grid_.set_cell_size(h);
    boundary_grid_.build(boundary_world_pos);

    float rho0 = pbf_solver_.settings().rest_density;
    for (int i = 0; i < n_fluid; ++i) {
        float boundary_density = 0.0f;
        const std::vector<int> neighbors =
            boundary_grid_.query_neighbors(fluid_state_.positions[i], h);
        for (int j : neighbors) {
            Vec3f r = fluid_state_.positions[i] - boundary_world_pos[j];
            float r_sq = r.squaredNorm();
            // Akinci density contribution: rho0 * psi_b * W
            boundary_density += rho0 * boundary_particles_[j].volume *
                                SPHKernels::poly6(r_sq, h);
        }
        fluid_state_.densities[i] += boundary_density;
    }
}

void FluidWorld::apply_coupling_forces(const std::vector<Vec3f>& boundary_world_pos,
                                        float dt) {
    int n_fluid = fluid_state_.num_particles();
    int n_boundary = static_cast<int>(boundary_particles_.size());
    float h = pbf_solver_.settings().kernel_radius;
    float rho0 = pbf_solver_.settings().rest_density;

    // Build grid for fluid particles
    SpatialHashGrid fluid_grid(h);
    fluid_grid.build(fluid_state_.positions);

    // For each boundary particle, compute pressure force from nearby fluid
    for (int b = 0; b < n_boundary; ++b) {
        int body_idx = boundary_particles_[b].body_index;
        if (body_idx < 0) continue;  // skip world-owned (planes)
        if (model().bodies[body_idx].is_static()) continue;

        Vec3f force = Vec3f::Zero();
        float psi_b = boundary_particles_[b].volume;

        const std::vector<int> neighbors =
            fluid_grid.query_neighbors(boundary_world_pos[b], h);
        for (int f : neighbors) {
            Vec3f r = boundary_world_pos[b] - fluid_state_.positions[f];
            float r_sq = r.squaredNorm();
            if (r_sq >= h * h) continue;

            // Akinci pressure force: f_b = -m_f * (p_f / rho_f^2) * rho0 * psi_b * grad W
            // Simplified using PBF lambda: use density-based pressure
            float rho_f = std::max(fluid_state_.densities[f], 1.0f);
            float pressure = std::max(0.0f, rho_f - rho0);
            float pressure_coeff = particle_mass_ * pressure / (rho_f * rho_f);

            Vec3f grad = SPHKernels::spiky_grad(r, h);
            force += pressure_coeff * rho0 * psi_b * grad;
        }

        // Apply accumulated force to rigid body
        if (force.squaredNorm() > 1e-20f) {
            // Force at boundary particle position (creates torque about CoM)
            World::apply_force(body_idx, force);

            // Torque from off-center force application
            Vec3f body_com = state().transforms[body_idx].position;
            Vec3f r_arm = boundary_world_pos[b] - body_com;
            Vec3f torque = r_arm.cross(force);
            World::apply_torque(body_idx, torque);
        }
    }
}

}  // namespace novaphy
