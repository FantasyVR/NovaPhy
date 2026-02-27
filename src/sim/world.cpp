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
    state_.init(model_.num_bodies(), model_.initial_transforms);
}

/**
 * @brief Advances the world by one fixed time step.
 * @param[in] dt Simulation step size in seconds.
 */
void World::step(float dt) {
    int n = model_.num_bodies();

    // 1. Integrate velocities (apply gravity + external forces)
    for (int i = 0; i < n; ++i) {
        if (model_.bodies[i].is_static()) continue;
        SymplecticEuler::integrate_velocity(
            state_.linear_velocities[i],
            state_.angular_velocities[i],
            state_.forces[i],
            state_.torques[i],
            model_.bodies[i].inv_mass(),
            model_.bodies[i].inv_inertia(),
            gravity_, dt);
    }

    // 2. Broadphase collision detection
    //    Build AABBs for all shapes, then run SAP
    int num_shapes = model_.num_shapes();
    std::vector<AABB> shape_aabbs(num_shapes);
    std::vector<bool> shape_static(num_shapes);
    for (int i = 0; i < num_shapes; ++i) {
        const auto& shape = model_.shapes[i];
        if (shape.body_index >= 0) {
            shape_aabbs[i] = shape.compute_aabb(state_.transforms[shape.body_index]);
            shape_static[i] = model_.bodies[shape.body_index].is_static();
        } else {
            // World-owned shapes (like planes) use identity transform
            shape_aabbs[i] = shape.compute_aabb(Transform::identity());
            shape_static[i] = true;  // planes are always static
        }
    }
    broadphase_.update(shape_aabbs, shape_static);

    // 3. Narrowphase collision detection
    contacts_.clear();
    const auto& pairs = broadphase_.get_pairs();
    for (const auto& pair : pairs) {
        const auto& sa = model_.shapes[pair.body_a];  // broadphase uses shape indices
        const auto& sb = model_.shapes[pair.body_b];

        // Pass body transforms; narrowphase handles local_transform internally
        Transform ta = (sa.body_index >= 0) ?
            state_.transforms[sa.body_index] :
            Transform::identity();
        Transform tb = (sb.body_index >= 0) ?
            state_.transforms[sb.body_index] :
            Transform::identity();

        std::vector<ContactPoint> new_contacts;
        if (collide_shapes(sa, ta, sb, tb, new_contacts)) {
            for (auto& cp : new_contacts) {
                // Narrowphase already sets body_a/body_b from shape.body_index
                // Override friction/restitution with combined values
                cp.friction = combine_friction(sa.friction, sb.friction);
                cp.restitution = combine_restitution(sa.restitution, sb.restitution);
                contacts_.push_back(cp);
            }
        }
    }

    // 4. Solve contact constraints
    solver_.solve(contacts_, model_.bodies, state_.transforms,
                  state_.linear_velocities, state_.angular_velocities, dt);

    // 5. Integrate positions
    for (int i = 0; i < n; ++i) {
        if (model_.bodies[i].is_static()) continue;
        SymplecticEuler::integrate_position(
            state_.transforms[i],
            state_.linear_velocities[i],
            state_.angular_velocities[i], dt);
    }

    // 6. Clear forces for next step
    state_.clear_forces();
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

}  // namespace novaphy
