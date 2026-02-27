/**
 * @file integrator.cpp
 * @brief Time integration routines for free rigid-body states.
 */
#include "novaphy/dynamics/integrator.h"

namespace novaphy {

/**
 * @brief Integrates linear and angular velocity using symplectic Euler.
 * @param[in,out] linear_vel Linear velocity in world frame (m/s).
 * @param[in,out] angular_vel Angular velocity in world frame (rad/s).
 * @param[in] force Net external force in world frame (N).
 * @param[in] torque Net external torque in world frame (N*m).
 * @param[in] inv_mass Inverse mass (`0` for static/infinite mass).
 * @param[in] inv_inertia Inverse world-frame inertia tensor.
 * @param[in] gravity World gravity acceleration (m/s^2).
 * @param[in] dt Time step (s).
 */
void SymplecticEuler::integrate_velocity(Vec3f& linear_vel, Vec3f& angular_vel,
                                          const Vec3f& force, const Vec3f& torque,
                                          float inv_mass, const Mat3f& inv_inertia,
                                          const Vec3f& gravity, float dt) {
    // Semi-implicit Euler: v(t+dt) = v(t) + a * dt
    linear_vel += (gravity + force * inv_mass) * dt;
    angular_vel += inv_inertia * torque * dt;

    // Gentle angular damping to prevent numerical drift
    angular_vel *= 0.999f;
}

/**
 * @brief Integrates position and orientation using updated velocities.
 * @param[in,out] transform Pose in world frame.
 * @param[in] linear_vel Linear velocity in world frame (m/s).
 * @param[in] angular_vel Angular velocity in world frame (rad/s).
 * @param[in] dt Time step (s).
 */
void SymplecticEuler::integrate_position(Transform& transform,
                                          const Vec3f& linear_vel,
                                          const Vec3f& angular_vel, float dt) {
    // Position update
    transform.position += linear_vel * dt;

    // Orientation update via quaternion derivative
    // dq/dt = 0.5 * omega * q
    Quatf& q = transform.rotation;
    Quatf omega_quat(0, angular_vel.x(), angular_vel.y(), angular_vel.z());
    Quatf dq;
    dq.w() = 0.5f * (-angular_vel.x() * q.x() - angular_vel.y() * q.y() - angular_vel.z() * q.z());
    dq.x() = 0.5f * (angular_vel.x() * q.w() + angular_vel.z() * q.y() - angular_vel.y() * q.z());
    dq.y() = 0.5f * (angular_vel.y() * q.w() - angular_vel.z() * q.x() + angular_vel.x() * q.z());
    dq.z() = 0.5f * (angular_vel.z() * q.w() + angular_vel.y() * q.x() - angular_vel.x() * q.y());

    q.w() += dq.w() * dt;
    q.x() += dq.x() * dt;
    q.y() += dq.y() * dt;
    q.z() += dq.z() * dt;
    q.normalize();
}

}  // namespace novaphy
