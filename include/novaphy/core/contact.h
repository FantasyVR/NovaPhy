#pragma once

#include "novaphy/math/math_types.h"
#include <vector>

namespace novaphy {

/**
 * @brief One world-space contact sample between two colliders.
 *
 * @details Contact normal points from body A toward body B. Penetration depth
 * is positive when shapes overlap. Units follow SI conventions.
 */
struct ContactPoint {
    Vec3f position = Vec3f::Zero();  /**< Contact position in world coordinates (m). */
    Vec3f normal = Vec3f::Zero();    /**< Contact normal from body A toward body B. */
    float penetration = 0.0f;        /**< Penetration depth in meters; positive means overlap. */

    int body_a = -1;   /**< Body index for side A, or -1 for world/static shape. */
    int body_b = -1;   /**< Body index for side B, or -1 for world/static shape. */
    int shape_a = -1;  /**< Shape index for side A. */
    int shape_b = -1;  /**< Shape index for side B. */

    float friction = 0.5f;     /**< Combined friction coefficient (dimensionless). */
    float restitution = 0.0f;  /**< Combined restitution coefficient (dimensionless). */

    float accumulated_normal_impulse = 0.0f;      /**< Cached normal impulse for warm starting (N*s). */
    float accumulated_tangent_impulse_1 = 0.0f;   /**< Cached first tangent impulse for warm starting (N*s). */
    float accumulated_tangent_impulse_2 = 0.0f;   /**< Cached second tangent impulse for warm starting (N*s). */
};

/**
 * @brief Contact manifold grouping points for one shape pair.
 *
 * @details Multiple points from the same interacting pair improve stacking
 * stability and friction torque approximation.
 */
struct ContactManifold {
    int shape_a = -1;                  /**< Shape index for side A. */
    int shape_b = -1;                  /**< Shape index for side B. */
    int body_a = -1;                   /**< Body index for side A. */
    int body_b = -1;                   /**< Body index for side B. */
    std::vector<ContactPoint> points;  /**< Contact points in this manifold. */
};

/**
 * @brief Combine two friction coefficients.
 *
 * @param [in] a Friction coefficient from material A.
 * @param [in] b Friction coefficient from material B.
 * @return Geometric-mean combined friction coefficient.
 */
inline float combine_friction(float a, float b) {
    return std::sqrt(a * b);
}

/**
 * @brief Combine two restitution coefficients.
 *
 * @param [in] a Restitution coefficient from material A.
 * @param [in] b Restitution coefficient from material B.
 * @return Maximum of both coefficients.
 */
inline float combine_restitution(float a, float b) {
    return std::max(a, b);
}

}  // namespace novaphy
