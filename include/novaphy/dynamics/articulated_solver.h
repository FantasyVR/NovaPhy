#pragma once

#include "novaphy/core/articulation.h"
#include "novaphy/dynamics/featherstone.h"
#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief Time-stepping solver for articulated-body dynamics.
 *
 * @details Wraps Featherstone forward dynamics and advances generalized state
 * using semi-implicit integration. Quaternion state components are normalized
 * after integration for free/ball joints.
 */
class ArticulatedSolver {
public:
    /** @brief Construct articulated solver with default settings. */
    ArticulatedSolver() = default;

    /**
     * @brief Advance articulated state by one time step.
     *
     * @param [in] model Articulation model.
     * @param [in,out] q Generalized positions, updated in place.
     * @param [in,out] qd Generalized velocities, updated in place.
     * @param [in] tau Applied joint efforts (torques/forces).
     * @param [in] gravity Gravity vector in world frame (m/s^2).
     * @param [in] dt Time step in seconds.
     * @return void
     */
    void step(const Articulation& model,
              VecXf& q,
              VecXf& qd,
              const VecXf& tau,
              const Vec3f& gravity,
              float dt);

private:
    /**
     * @brief Normalize quaternion state blocks after integration.
     *
     * @param [in] model Articulation model.
     * @param [in,out] q Generalized position vector.
     * @return void
     */
    void normalize_quaternions(const Articulation& model, VecXf& q);
};

}  // namespace novaphy
