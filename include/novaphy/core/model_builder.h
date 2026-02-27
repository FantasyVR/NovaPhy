#pragma once

#include <string>
#include <vector>

#include "novaphy/core/body.h"
#include "novaphy/core/shape.h"
#include "novaphy/math/math_types.h"

namespace novaphy {

/** @brief Forward declaration of immutable model container. */
struct Model;

/**
 * @brief Builder utility for constructing immutable free-body scene models.
 *
 * @details Collects rigid bodies, their initial transforms, and collision
 * shapes, then emits a `Model` snapshot suitable for simulation.
 */
class ModelBuilder {
public:
    /** @brief Construct an empty model builder. */
    ModelBuilder() = default;

    /**
     * @brief Add a rigid body with initial world transform.
     *
     * @param [in] body Body inertial properties.
     * @param [in] transform Initial world transform.
     * @return Index of inserted body.
     */
    int add_body(const RigidBody& body, const Transform& transform = Transform::identity());

    /**
     * @brief Add a collision shape.
     *
     * @param [in] shape Collision shape descriptor.
     * @return Index of inserted shape.
     */
    int add_shape(const CollisionShape& shape);

    /**
     * @brief Add infinite horizontal ground plane.
     *
     * @param [in] y Plane offset along +Y world axis in meters.
     * @param [in] friction Friction coefficient.
     * @param [in] restitution Restitution coefficient.
     * @return Index of inserted plane shape.
     */
    int add_ground_plane(float y = 0.0f, float friction = 0.5f, float restitution = 0.0f);

    /**
     * @brief Build immutable `Model` from accumulated entities.
     *
     * @return Immutable scene model.
     */
    Model build() const;

    /**
     * @brief Get current number of added bodies.
     *
     * @return Body count.
     */
    int num_bodies() const { return static_cast<int>(bodies_.size()); }

    /**
     * @brief Get current number of added shapes.
     *
     * @return Shape count.
     */
    int num_shapes() const { return static_cast<int>(shapes_.size()); }

private:
    std::vector<RigidBody> bodies_;            /**< Staged body list. */
    std::vector<Transform> initial_transforms_;  /**< Staged initial transforms. */
    std::vector<CollisionShape> shapes_;       /**< Staged collision shapes. */
};

}  // namespace novaphy
