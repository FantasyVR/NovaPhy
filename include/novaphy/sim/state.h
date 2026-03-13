#pragma once

#include <span>
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
    std::vector<int> sleeping;                /**< Per-body sleep flag (1 if body is frozen, 0 otherwise). */
    std::vector<float> sleep_timer;           /**< Time accumulated below threshold for each body (seconds). */
    std::vector<float> smoothed_energy;       /**< Per-body EMA-smoothed kinetic energy. */
    std::vector<int> island_id;               /**< Island assignment for each body (-1 if not assigned). */

    /**
     * @brief Initialize all state arrays from the model's initial transforms.
     *
     * @param [in] initial_transforms Initial world transforms, one per body.
     * @return void
     */
    void init(std::span<const Transform> initial_transforms);

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

    /**
     * @brief Wake a sleeping body, making it dynamic again.
     *
     * @details Clears sleep flag and resets sleep timer. Also zeros velocities
     * to ensure clean start from rest. Propagates wake to all bodies in the
     * same island.
     *
     * @param [in] body_index Body index.
     * @return void
     */
    void wake_body(int body_index);

    /**
     * @brief Check if a body is currently sleeping.
     *
     * @param [in] body_index Body index.
     * @return true if body is sleeping, false otherwise.
     */
    bool is_sleeping(int body_index) const;

    /**
     * @brief Update smoothed kinetic energy for a body using EMA.
     *
     * @param [in] body_index Body index.
     * @param [in] energy Current kinetic energy (unsmoothed).
     * @param [in] alpha EMA smoothing factor (0-1).
     * @return void
     */
    void update_energy(int body_index, float energy, float alpha);

    /**
     * @brief Get smoothed kinetic energy for a body.
     *
     * @param [in] body_index Body index.
     * @return Smoothed energy value.
     */
    float get_smoothed_energy(int body_index) const;

    /**
     * @brief Build islands from contact points using Union-Find.
     *
     * @details Constructs connected components of bodies via contacts.
     * Each island is assigned a unique ID in island_id array.
     *
     * @param [in] contacts Contact points for the current step.
     * @return void
     */
    void build_islands(const std::vector<class ContactPoint>& contacts);

    /**
     * @brief Evaluate sleep state based on energy thresholds.
     *
     * @details Uses island-level logic: all bodies in an island must have
     * low energy for the duration threshold before sleeping together.
     *
     * @param [in] dt Time step in seconds.
     * @param [in] energy_threshold Energy threshold for sleep consideration.
     * @param [in] time_threshold Time below threshold before sleeping (seconds).
     * @return void
     */
    void evaluate_sleep(float dt, float energy_threshold, float time_threshold);

    /**
     * @brief Propagate wake state to all bodies in the same island.
     *
     * @param [in] body_index Body that was woken up.
     * @return void
     */
    void propagate_wake_through_island(int body_index);
};

}  // namespace novaphy
