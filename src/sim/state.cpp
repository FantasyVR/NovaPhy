/**
 * @file state.cpp
 * @brief Mutable simulation state storage for body poses and velocities.
 */
#include "novaphy/sim/state.h"

namespace novaphy {

/**
 * @brief Initializes state arrays for a fixed number of bodies.
 * @param[in] n Number of bodies in the simulation model.
 * @param[in] initial_transforms Initial world transforms for each body.
 */
void SimState::init(int n, const std::vector<Transform>& initial_transforms) {
    transforms = initial_transforms;
    linear_velocities.assign(n, Vec3f::Zero());
    angular_velocities.assign(n, Vec3f::Zero());
    forces.assign(n, Vec3f::Zero());
    torques.assign(n, Vec3f::Zero());
}

/**
 * @brief Clears all accumulated external forces and torques.
 */
void SimState::clear_forces() {
    for (auto& f : forces) f = Vec3f::Zero();
    for (auto& t : torques) t = Vec3f::Zero();
}

/**
 * @brief Sets linear velocity for one body index.
 * @param[in] body_index Body index in model order.
 * @param[in] vel Target linear velocity in world frame (m/s).
 */
void SimState::set_linear_velocity(int body_index, const Vec3f& vel) {
    if (body_index >= 0 && body_index < static_cast<int>(linear_velocities.size())) {
        linear_velocities[body_index] = vel;
    }
}

/**
 * @brief Sets angular velocity for one body index.
 * @param[in] body_index Body index in model order.
 * @param[in] vel Target angular velocity in world frame (rad/s).
 */
void SimState::set_angular_velocity(int body_index, const Vec3f& vel) {
    if (body_index >= 0 && body_index < static_cast<int>(angular_velocities.size())) {
        angular_velocities[body_index] = vel;
    }
}

/**
 * @brief Accumulates an external force on one body.
 * @param[in] body_index Body index in model order.
 * @param[in] force Force vector in world frame (N).
 */
void SimState::apply_force(int body_index, const Vec3f& force) {
    if (body_index >= 0 && body_index < static_cast<int>(forces.size())) {
        forces[body_index] += force;
    }
}

/**
 * @brief Accumulates an external torque on one body.
 * @param[in] body_index Body index in model order.
 * @param[in] torque Torque vector in world frame (N*m).
 */
void SimState::apply_torque(int body_index, const Vec3f& torque) {
    if (body_index >= 0 && body_index < static_cast<int>(torques.size())) {
        torques[body_index] += torque;
    }
}

}  // namespace novaphy
