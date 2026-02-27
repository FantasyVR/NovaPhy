#pragma once

#include <vector>

#include "novaphy/core/contact.h"
#include "novaphy/core/shape.h"

namespace novaphy {
namespace narrowphase {

/**
 * @brief Detect and generate contacts for sphere-sphere intersection.
 *
 * @param [in] a First sphere shape.
 * @param [in] ta World transform of first shape's parent body.
 * @param [in] b Second sphere shape.
 * @param [in] tb World transform of second shape's parent body.
 * @param [out] contacts Generated world-space contact points.
 * @return True if shapes overlap.
 */
bool collide_sphere_sphere(const CollisionShape& a, const Transform& ta,
                           const CollisionShape& b, const Transform& tb,
                           std::vector<ContactPoint>& contacts);

/**
 * @brief Detect and generate contacts for sphere-plane intersection.
 *
 * @param [in] sphere Sphere collision shape.
 * @param [in] ts World transform of sphere parent body.
 * @param [in] plane Infinite plane shape.
 * @param [out] contacts Generated world-space contact points.
 * @return True if the sphere penetrates or touches the plane.
 */
bool collide_sphere_plane(const CollisionShape& sphere, const Transform& ts,
                          const CollisionShape& plane,
                          std::vector<ContactPoint>& contacts);

/**
 * @brief Detect and generate contacts for oriented box-sphere intersection.
 *
 * @param [in] box Box collision shape.
 * @param [in] tb World transform of box parent body.
 * @param [in] sphere Sphere collision shape.
 * @param [in] ts World transform of sphere parent body.
 * @param [out] contacts Generated world-space contact points.
 * @return True if shapes overlap.
 */
bool collide_box_sphere(const CollisionShape& box, const Transform& tb,
                        const CollisionShape& sphere, const Transform& ts,
                        std::vector<ContactPoint>& contacts);

/**
 * @brief Detect and generate contacts for oriented box-plane intersection.
 *
 * @param [in] box Box collision shape.
 * @param [in] tb World transform of box parent body.
 * @param [in] plane Infinite plane shape.
 * @param [out] contacts Generated world-space contact points.
 * @return True if the box intersects the plane.
 */
bool collide_box_plane(const CollisionShape& box, const Transform& tb,
                       const CollisionShape& plane,
                       std::vector<ContactPoint>& contacts);

/**
 * @brief Detect and generate contacts for oriented box-box intersection.
 *
 * @details Uses a Separating Axis Theorem (SAT) test and then builds a
 * contact manifold for impulse-based solving.
 *
 * @param [in] a First box shape.
 * @param [in] ta World transform of first box parent body.
 * @param [in] b Second box shape.
 * @param [in] tb World transform of second box parent body.
 * @param [out] contacts Generated world-space contact manifold points.
 * @return True if boxes overlap.
 */
bool collide_box_box(const CollisionShape& a, const Transform& ta,
                     const CollisionShape& b, const Transform& tb,
                     std::vector<ContactPoint>& contacts);

}  // namespace narrowphase

/**
 * @brief Dispatch collision test based on shape type pair.
 *
 * @details Routes to the corresponding narrowphase routine and appends
 * generated world-space contact points for subsequent constraint solving.
 *
 * @param [in] a First collision shape.
 * @param [in] ta World transform of first shape parent body.
 * @param [in] b Second collision shape.
 * @param [in] tb World transform of second shape parent body.
 * @param [out] contacts Generated contact points.
 * @return True if a collision is detected.
 */
bool collide_shapes(const CollisionShape& a, const Transform& ta,
                    const CollisionShape& b, const Transform& tb,
                    std::vector<ContactPoint>& contacts);

}  // namespace novaphy
