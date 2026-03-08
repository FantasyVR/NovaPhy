/**
 * @file world.cpp
 * @brief High-level simulation world stepping pipeline.
 */
#include "novaphy/sim/world.h"

namespace novaphy {

/**
 * @brief Constructs a simulation world from an immutable model.
 * @param[in] model Rigid-body and collision-shape model definition.
 * @param[in] solver_settings Contact solver tuning parameters.
 */
World::World(const Model& model, SolverSettings solver_settings)
    : model_(model), solver_(solver_settings) {
    state_.init(model_.initial_transforms);
}

/**
 * @brief Advances the world by one fixed time step.
 * @param[in] dt Simulation step size in seconds.
 */
void World::step(float dt) {
    performance_monitor_.begin_frame();
    detail::ScopedPerformanceCaptureContext capture_context(&performance_monitor_);
    step_rigid_pipeline(dt);
    performance_monitor_.end_frame();
}

void World::step_rigid_pipeline(float dt) {
    detail::PerformancePhaseScope world_total_scope(&performance_monitor_, "world.total");

    const int n = model_.num_bodies();
    int dynamic_body_count = 0;

    {
        detail::PerformancePhaseScope phase_scope(&performance_monitor_,
                                                  "world.integrate_velocity");
        for (int i = 0; i < n; ++i) {
            if (model_.bodies[i].is_static()) continue;
            dynamic_body_count += 1;
            SymplecticEuler::integrate_velocity(
                state_.linear_velocities[i],
                state_.angular_velocities[i],
                state_.forces[i],
                state_.torques[i],
                model_.bodies[i].inv_mass(),
                model_.bodies[i].inv_inertia(),
                gravity_, dt);
        }
    }

    const int num_shapes = model_.num_shapes();
    std::vector<AABB> shape_aabbs(num_shapes);
    std::vector<bool> shape_static(num_shapes);

    {
        detail::PerformancePhaseScope phase_scope(&performance_monitor_,
                                                  "world.broadphase.build_aabbs");
        for (int i = 0; i < num_shapes; ++i) {
            const auto& shape = model_.shapes[i];
            if (shape.body_index >= 0) {
                shape_aabbs[i] = shape.compute_aabb(state_.transforms[shape.body_index]);
                shape_static[i] = model_.bodies[shape.body_index].is_static();
            } else {
                shape_aabbs[i] = shape.compute_aabb(Transform::identity());
                shape_static[i] = true;
            }
        }
    }

    {
        detail::PerformancePhaseScope phase_scope(&performance_monitor_,
                                                  "world.broadphase.sap");
        broadphase_.update(shape_aabbs, shape_static);
    }

    contacts_.clear();
    {
        detail::PerformancePhaseScope phase_scope(&performance_monitor_,
                                                  "world.narrowphase.total");
        const auto& pairs = broadphase_.get_pairs();
        for (const auto& pair : pairs) {
            const auto& sa = model_.shapes[pair.body_a];
            const auto& sb = model_.shapes[pair.body_b];

            Transform ta = (sa.body_index >= 0) ?
                state_.transforms[sa.body_index] :
                Transform::identity();
            Transform tb = (sb.body_index >= 0) ?
                state_.transforms[sb.body_index] :
                Transform::identity();

            std::vector<ContactPoint> new_contacts;
            if (collide_shapes(sa, ta, sb, tb, new_contacts)) {
                for (auto& cp : new_contacts) {
                    cp.friction = combine_friction(sa.friction, sb.friction);
                    cp.restitution = combine_restitution(sa.restitution, sb.restitution);
                    contacts_.push_back(cp);
                }
            }
        }
    }

    {
        detail::PerformancePhaseScope phase_scope(&performance_monitor_, "world.solver.total");
        solver_.solve(contacts_, model_.bodies, state_.transforms,
                      state_.linear_velocities, state_.angular_velocities, dt);
    }

    {
        detail::PerformancePhaseScope phase_scope(&performance_monitor_,
                                                  "world.integrate_position");
        for (int i = 0; i < n; ++i) {
            if (model_.bodies[i].is_static()) continue;
            SymplecticEuler::integrate_position(
                state_.transforms[i],
                state_.linear_velocities[i],
                state_.angular_velocities[i], dt);
        }
    }

    {
        detail::PerformancePhaseScope phase_scope(&performance_monitor_, "world.clear_forces");
        state_.clear_forces();
    }

    const int candidate_pair_count = static_cast<int>(broadphase_.get_pairs().size());
    record_world_metrics(dynamic_body_count, candidate_pair_count,
                         static_cast<int>(contacts_.size()));
}

/**
 * @brief Applies an external force to a body for the next step.
 * @param[in] body_index Body index in model order.
 * @param[in] force Force vector in world frame (N).
 */
void World::apply_force(int body_index, const Vec3f& force) {
    state_.apply_force(body_index, force);
}

/**
 * @brief Applies an external torque to a body for the next step.
 * @param[in] body_index Body index in model order.
 * @param[in] torque Torque vector in world frame (N*m).
 */
void World::apply_torque(int body_index, const Vec3f& torque) {
    state_.apply_torque(body_index, torque);
}

void World::record_world_metrics(int dynamic_body_count,
                                 int candidate_pair_count,
                                 int contact_count) {
    performance_monitor_.record_metric("bodies", static_cast<double>(model_.num_bodies()));
    performance_monitor_.record_metric("dynamic_bodies", static_cast<double>(dynamic_body_count));
    performance_monitor_.record_metric("shapes", static_cast<double>(model_.num_shapes()));
    performance_monitor_.record_metric("candidate_pairs",
                                       static_cast<double>(candidate_pair_count));
    performance_monitor_.record_metric("contacts", static_cast<double>(contact_count));
    performance_monitor_.record_metric("solver_iterations",
                                       static_cast<double>(solver_.settings().velocity_iterations));
}

}  // namespace novaphy
