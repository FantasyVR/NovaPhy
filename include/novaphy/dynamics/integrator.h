#pragma once

#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief Symplectic-Euler integrator for free rigid-body states.
 *
 * @details Uses semi-implicit integration: velocities are updated first from
 * forces, then positions/orientations are advanced using updated velocities.
 */
struct SymplecticEuler {
    /**
     * @brief Integrate linear and angular velocities for one time step.
     *
     * @param [in,out] linear_vel Linear velocity in world frame (m/s).
     * @param [in,out] angular_vel Angular velocity in world frame (rad/s).
     * @param [in] force External force at center of mass in world frame (N).
     * @param [in] torque External torque in world frame (N*m).
     * @param [in] inv_mass Inverse mass (kg^-1).
     * @param [in] inv_inertia Inverse inertia tensor.
     * @param [in] gravity Gravity vector in world frame (m/s^2).
     * @param [in] dt Time step in seconds.
     * @return void
     */
    static void integrate_velocity(Vec3f& linear_vel, Vec3f& angular_vel,
                                   const Vec3f& force, const Vec3f& torque,
                                   float inv_mass, const Mat3f& inv_inertia,
                                   const Vec3f& gravity, float dt);

    /**
     * @brief Integrate pose from current velocities for one time step.
     *
     * @param [in,out] transform Body transform (position + quaternion).
     * @param [in] linear_vel Linear velocity in world frame (m/s).
     * @param [in] angular_vel Angular velocity in world frame (rad/s).
     * @param [in] dt Time step in seconds.
     * @return void
     */
    static void integrate_position(Transform& transform,
                                   const Vec3f& linear_vel,
                                   const Vec3f& angular_vel, float dt);
};

}  // namespace novaphy
