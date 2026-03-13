#pragma once

#include <span>
#include <vector>

#include "novaphy/core/body.h"
#include "novaphy/core/contact.h"
#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief Runtime tuning parameters for the free-body contact solver.
 *
 * @details These parameters control convergence and stability of the
 * Projected Gauss-Seidel (Sequential Impulse) contact solve in SI units.
 * For dense stacks, increasing iteration count and using moderate Baumgarte
 * stabilization typically reduces jitter and drift.
 */
struct SolverSettings {
    int velocity_iterations = 30;  /**< Number of PGS velocity iterations per time step. */
    float baumgarte = 0.3f;        /**< Fraction of position error corrected per step (dimensionless). */
    float slop = 0.005f;           /**< Penetration allowance before correction in meters. */
    bool warm_starting = true;     /**< Reuse cached impulses from previous frame to speed convergence. */

    // Sleep mechanism settings
    bool sleep_enabled = false;                    /**< Enable sleep mechanism to freeze stationary bodies. */
    float sleep_energy_threshold = 0.01f;          /**< Kinetic energy threshold for sleep (universal, all scenarios). */
    float sleep_time_required = 0.5f;              /**< Time below threshold before sleeping (seconds). */
    float sleep_ema_alpha = 0.8f;                  /**< EMA smoothing factor for energy (0-1). */
};

/**
 * @brief Sequential Impulse contact solver for unconstrained rigid bodies.
 *
 * @details Resolves normal and friction contact constraints using a
 * Projected Gauss-Seidel loop with accumulated impulse clamping. Contact
 * impulses are computed in world coordinates and applied to linear velocity
 * (m/s) and angular velocity (rad/s).
 *
 * @note This solver is intended for a single-threaded update loop.
 * @warning Inputs must be physically consistent (mass in kg, inertia in kg*m^2,
 * time step in seconds) to avoid instability.
 */
class FreeBodySolver {
public:
    /**
     * @brief Construct a contact solver with user-defined settings.
     *
     * @param [in] settings Solver iteration and stabilization parameters.
     */
    explicit FreeBodySolver(SolverSettings settings = {});

    /**
     * @brief Resolve contact constraints for the current simulation step.
     *
     * @details This function precomputes effective masses/bias terms and then
     * iteratively applies impulses to satisfy non-penetration and Coulomb
     * friction constraints.
     *
     * @param [in,out] contacts Contact points and warm-start impulse cache.
     * @param [in] bodies Rigid-body mass and inertia properties.
     * @param [in] transforms Body transforms in world coordinates.
     * @param [in,out] linear_velocities Body linear velocities in world frame (m/s).
     * @param [in,out] angular_velocities Body angular velocities in world frame (rad/s).
     * @param [in] sleeping Per-body sleep flags.
     * @param [in] dt Simulation time step in seconds.
     * @return void
     */
    void solve(std::span<ContactPoint> contacts,
               std::span<const RigidBody> bodies,
               std::span<const Transform> transforms,
               std::span<Vec3f> linear_velocities,
               std::span<Vec3f> angular_velocities,
               std::span<const int> sleeping,
               float dt);

    /**
     * @brief Mutable access to current solver settings.
     *
     * @return Reference to the internal SolverSettings.
     */
    SolverSettings& settings() { return settings_; }

private:
    SolverSettings settings_;

    /**
     * @brief Cached per-contact quantities reused across solver iterations.
     */
    struct ConstraintData {
        Vec3f r_a, r_b;           /**< Contact offset from each body CoM in world frame (m). */
        float effective_mass_n;   /**< Scalar effective mass along contact normal (kg^-1 inverse form). */
        float effective_mass_t1;  /**< Scalar effective mass along first friction tangent. */
        float effective_mass_t2;  /**< Scalar effective mass along second friction tangent. */
        Vec3f tangent1, tangent2; /**< Orthonormal tangents spanning the contact plane. */
        float bias;               /**< Velocity-level Baumgarte/restitution bias term (m/s). */
        Mat3f inv_I_a_world;      /**< Body A inverse inertia tensor in world frame ((kg*m^2)^-1). */
        Mat3f inv_I_b_world;      /**< Body B inverse inertia tensor in world frame ((kg*m^2)^-1). */
    };

    std::vector<ConstraintData> constraint_data_;

    /**
     * @brief Precompute contact-space quantities before the PGS loop.
     *
     * @param [in,out] contacts Contact set for the current step.
     * @param [in] bodies Rigid-body mass properties.
     * @param [in] transforms Body transforms in world coordinates.
     * @param [in] linear_velocities Current body linear velocities in world frame (m/s).
     * @param [in] angular_velocities Current body angular velocities in world frame (rad/s).
     * @param [in] sleeping Per-body sleep flags.
     * @param [in] dt Simulation time step in seconds.
     * @return void
     */
    void pre_step(std::span<ContactPoint> contacts,
                  std::span<const RigidBody> bodies,
                  std::span<const Transform> transforms,
                  std::span<const Vec3f> linear_velocities,
                  std::span<const Vec3f> angular_velocities,
                  std::span<const int> sleeping,
                  float dt);

    /**
     * @brief Execute one PGS sweep over all contact constraints.
     *
     * @param [in,out] contacts Contact set with accumulated impulses.
     * @param [in] bodies Rigid-body mass properties.
     * @param [in,out] linear_velocities Body linear velocities in world frame (m/s).
     * @param [in,out] angular_velocities Body angular velocities in world frame (rad/s).
     * @param [in] sleeping Per-body sleep flags.
     * @return void
     */
    void solve_velocity(std::span<ContactPoint> contacts,
                        std::span<const RigidBody> bodies,
                        std::span<Vec3f> linear_velocities,
                        std::span<Vec3f> angular_velocities,
                        std::span<const int> sleeping);
};

}  // namespace novaphy
