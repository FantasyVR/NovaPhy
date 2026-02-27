#pragma once

#include <vector>

#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief Per-body simulation state buffers for free rigid-body stepping.
 *
 * @details Stores pose, velocity, and accumulated wrench terms for each body.
 * Linear quantities are in world coordinates (m, m/s, N), angular quantities
 * are in world coordinates (rad/s, N*m).
 */
struct SimState {
    std::vector<Transform> transforms;        /**< Body world transforms (position + quaternion orientation). */
    std::vector<Vec3f> linear_velocities;     /**< Body linear velocities in world frame (m/s). */
    std::vector<Vec3f> angular_velocities;    /**< Body angular velocities in world frame (rad/s). */
    std::vector<Vec3f> forces;                /**< Accumulated external forces at CoM in world frame (N). */
    std::vector<Vec3f> torques;               /**< Accumulated external torques in world frame (N*m). */

    /**
     * @brief Initialize all state arrays for a model with n bodies.
     *
     * @param [in] n Number of bodies.
     * @param [in] initial_transforms Initial world transforms, one per body.
     * @return void
     */
    void init(int n, const std::vector<Transform>& initial_transforms);

    /**
     * @brief Clear accumulated external forces and torques.
     *
     * @details Called once per simulation step after integration.
     *
     * @return void
     */
    void clear_forces();

    /**
     * @brief Set linear velocity for one body.
     *
     * @param [in] body_index Body index.
     * @param [in] vel Target linear velocity in world frame (m/s).
     * @return void
     */
    void set_linear_velocity(int body_index, const Vec3f& vel);

    /**
     * @brief Set angular velocity for one body.
     *
     * @param [in] body_index Body index.
     * @param [in] vel Target angular velocity in world frame (rad/s).
     * @return void
     */
    void set_angular_velocity(int body_index, const Vec3f& vel);

    /**
     * @brief Accumulate an external force at a body's center of mass.
     *
     * @param [in] body_index Body index.
     * @param [in] force Force vector in world frame (N).
     * @return void
     */
    void apply_force(int body_index, const Vec3f& force);

    /**
     * @brief Accumulate an external torque on a body.
     *
     * @param [in] body_index Body index.
     * @param [in] torque Torque vector in world frame (N*m).
     * @return void
     */
    void apply_torque(int body_index, const Vec3f& torque);
};

}  // namespace novaphy
