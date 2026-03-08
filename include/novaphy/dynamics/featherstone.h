#pragma once

#include <span>
#include <vector>

#include "novaphy/core/articulation.h"
#include "novaphy/math/math_types.h"
#include "novaphy/math/spatial.h"

namespace novaphy {

/**
 * @brief Featherstone-family rigid-body dynamics algorithms.
 *
 * @details This namespace provides forward kinematics, inverse dynamics
 * (RNEA), mass-matrix assembly (CRBA), and forward dynamics for a single
 * tree-structured articulation.
 *
 * Spatial-vector convention is [angular; linear], and computations follow
 * standard rigid-body dynamics in SI units (m, kg, s, N, rad).
 */
namespace featherstone {

struct ForwardKinematicsResult {
    std::vector<SpatialTransform> joint_transforms;
    std::vector<SpatialTransform> parent_transforms;
    std::vector<Transform> world_transforms;
};

/**
 * @brief Compute per-link poses from generalized positions.
 *
 * @details Performs a kinematic traversal from root to leaves to assemble:
 * joint transforms, parent-relative transforms, and world transforms for each
 * link in the articulation.
 *
 * @param [in] model Articulation graph and joint definitions.
 * @param [in] q Generalized position vector, length = model.total_q().
 * @return Joint transforms, parent-relative spatial transforms, and world-frame
 * rigid transforms for every articulation link.
 *
 * @note Output transforms are expressed in world coordinates for rendering
 * and downstream dynamics calls.
 * @warning Input dimensions must match model topology.
 */
ForwardKinematicsResult forward_kinematics(const Articulation& model,
                                           const VecXf& q);

/**
 * @brief Compute generalized forces using RNEA (inverse dynamics).
 *
 * @details Recursive Newton-Euler Algorithm evaluates the joint efforts
 * required to realize a prescribed motion state (q, qd, qdd), optionally
 * including external link forces.
 *
 * @param [in] model Articulation graph and inertial properties.
 * @param [in] q Generalized positions.
 * @param [in] qd Generalized velocities.
 * @param [in] qdd Generalized accelerations.
 * @param [in] gravity Gravity vector in world frame (m/s^2).
 * @param [in] f_ext Optional external spatial forces per link in link-local frame (N, N*m).
 * @return Generalized force vector tau (joint torques/forces in SI units).
 *
 * @note Time complexity is O(n) for n links.
 * @warning If provided, f_ext must contain one entry per link.
 */
VecXf inverse_dynamics(const Articulation& model,
                       const VecXf& q,
                       const VecXf& qd,
                       const VecXf& qdd,
                       const Vec3f& gravity,
                       std::span<const SpatialVector> f_ext = {});

/**
 * @brief Assemble the joint-space inertia matrix using CRBA.
 *
 * @details Composite Rigid Body Algorithm computes H(q), the configuration-
 * dependent mass matrix used in forward dynamics and model-based control.
 *
 * @param [in] model Articulation graph and inertial properties.
 * @param [in] q Generalized positions.
 * @return Joint-space inertia matrix H(q), size nv x nv, symmetric positive definite.
 *
 * @note Typical complexity is O(n^2) for tree articulations.
 */
MatXf mass_matrix(const Articulation& model,
                  const VecXf& q);

/**
 * @brief Compute generalized accelerations from applied efforts.
 *
 * @details Solves
 * qdd = H(q)^-1 * (tau - C(q, qd)),
 * where C is obtained from inverse dynamics with zero acceleration.
 *
 * @param [in] model Articulation graph and inertial properties.
 * @param [in] q Generalized positions.
 * @param [in] qd Generalized velocities.
 * @param [in] tau Applied joint efforts (torques/forces).
 * @param [in] gravity Gravity vector in world frame (m/s^2).
 * @param [in] f_ext Optional external spatial forces per link in link-local frame (N, N*m).
 * @return Generalized acceleration vector qdd.
 *
 * @note This API is commonly used for simulation stepping and inverse-control validation.
 */
VecXf forward_dynamics(const Articulation& model,
                       const VecXf& q,
                       const VecXf& qd,
                       const VecXf& tau,
                       const Vec3f& gravity,
                       std::span<const SpatialVector> f_ext = {});

}  // namespace featherstone

}  // namespace novaphy
