/**
 * @file model_builder.cpp
 * @brief Builder utilities for assembling simulation models.
 */
#include "novaphy/core/model_builder.h"

#include "novaphy/core/model.h"

namespace novaphy {

/**
 * @brief Adds a rigid body with an initial world transform.
 * @param[in] body Body inertial and kinematic properties in SI units.
 * @param[in] transform Initial world transform of the body frame.
 * @return Index of the inserted body in the model body array.
 */
int ModelBuilder::add_body(const RigidBody& body, const Transform& transform) {
    int idx = static_cast<int>(bodies_.size());
    bodies_.push_back(body);
    initial_transforms_.push_back(transform);
    return idx;
}

/**
 * @brief Adds a collision shape descriptor to the model.
 * @param[in] shape Shape geometry and contact material parameters.
 * @return Index of the inserted shape in the model shape array.
 */
int ModelBuilder::add_shape(const CollisionShape& shape) {
    int idx = static_cast<int>(shapes_.size());
    shapes_.push_back(shape);
    return idx;
}

/**
 * @brief Creates and adds an infinite horizontal ground plane.
 * @param[in] y Plane offset along world Y in meters.
 * @param[in] friction Coulomb friction coefficient.
 * @param[in] restitution Coefficient of restitution in `[0, 1]`.
 * @return Index of the inserted plane shape.
 */
int ModelBuilder::add_ground_plane(float y, float friction, float restitution) {
    CollisionShape plane = CollisionShape::make_plane(
        Vec3f(0.0f, 1.0f, 0.0f), y, friction, restitution);
    return add_shape(plane);
}

/**
 * @brief Materializes a `Model` from the accumulated builder state.
 * @return Copy of all configured bodies, transforms, and shapes.
 */
Model ModelBuilder::build() const {
    Model m;
    m.bodies = bodies_;
    m.initial_transforms = initial_transforms_;
    m.shapes = shapes_;
    return m;
}

}  // namespace novaphy
