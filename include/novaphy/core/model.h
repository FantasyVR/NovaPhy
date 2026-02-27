#pragma once

#include <vector>

#include "novaphy/core/body.h"
#include "novaphy/core/shape.h"
#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief Immutable free-body scene model.
 *
 * @details Describes rigid bodies, their initial world transforms, and
 * collision shapes. Instances are typically created with `ModelBuilder`.
 */
struct Model {
    std::vector<RigidBody> bodies;            /**< Rigid-body inertial properties. */
    std::vector<Transform> initial_transforms;  /**< Initial body transforms in world coordinates. */
    std::vector<CollisionShape> shapes;       /**< Collision shapes attached to bodies/world. */

    /**
     * @brief Get number of bodies in the model.
     *
     * @return Body count.
     */
    int num_bodies() const { return static_cast<int>(bodies.size()); }

    /**
     * @brief Get number of collision shapes in the model.
     *
     * @return Shape count.
     */
    int num_shapes() const { return static_cast<int>(shapes.size()); }
};

}  // namespace novaphy
