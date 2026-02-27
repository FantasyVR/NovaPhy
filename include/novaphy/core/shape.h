#pragma once

#include "novaphy/core/aabb.h"
#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief Supported collision-shape primitives.
 */
enum class ShapeType { Box, Sphere, Plane };

/**
 * @brief Box primitive described by local half extents.
 */
struct BoxShape {
    Vec3f half_extents = Vec3f(0.5f, 0.5f, 0.5f);  /**< Half lengths along local x/y/z axes in meters. */
};

/**
 * @brief Sphere primitive described by radius.
 */
struct SphereShape {
    float radius = 0.5f;  /**< Sphere radius in meters. */
};

/**
 * @brief Infinite plane primitive in Hessian form.
 *
 * @details Plane equation is n.dot(x) = offset where n is unit length.
 */
struct PlaneShape {
    Vec3f normal = Vec3f(0.0f, 1.0f, 0.0f);  /**< Unit normal in world/local frame according to usage. */
    float offset = 0.0f;                     /**< Signed distance from origin along normal (m). */
};

/**
 * @brief Collision-shape descriptor attached to a body or world.
 *
 * @details Stores primitive geometry, local pose offset, and contact material
 * parameters used by the collision and contact-solver pipeline.
 */
struct CollisionShape {
    ShapeType type = ShapeType::Box;  /**< Active primitive type. */

    BoxShape box;        /**< Box primitive payload. */
    SphereShape sphere;  /**< Sphere primitive payload. */
    PlaneShape plane;    /**< Plane primitive payload. */

    Transform local_transform = Transform::identity();  /**< Shape pose in parent-body local frame. */
    float friction = 0.5f;                              /**< Coulomb friction coefficient (dimensionless). */
    float restitution = 0.3f;                           /**< Restitution coefficient in [0, 1]. */

    int body_index = -1;  /**< Owning body index, or -1 for world-owned shapes (e.g., planes). */

    /**
     * @brief Compute a world-space AABB for this shape.
     *
     * @param [in] body_transform Parent body world transform.
     * @return Axis-aligned bounds in world coordinates.
     */
    AABB compute_aabb(const Transform& body_transform) const {
        Transform world = body_transform * local_transform;
        switch (type) {
            case ShapeType::Box:
                return AABB::from_oriented_box(box.half_extents, world);
            case ShapeType::Sphere:
                return AABB::from_sphere(world.position, sphere.radius);
            case ShapeType::Plane:
                // Planes are infinite; use a very large AABB
                return AABB(Vec3f::Constant(-1e6f), Vec3f::Constant(1e6f));
        }
        return AABB();
    }

    /**
     * @brief Create a box collision shape.
     *
     * @param [in] half_extents Local half extents in meters.
     * @param [in] body_idx Owning body index.
     * @param [in] local Local transform from body frame to shape frame.
     * @param [in] friction Friction coefficient.
     * @param [in] restitution Restitution coefficient.
     * @return Constructed box shape descriptor.
     */
    static CollisionShape make_box(const Vec3f& half_extents, int body_idx,
                                   const Transform& local = Transform::identity(),
                                   float friction = 0.5f, float restitution = 0.3f) {
        CollisionShape s;
        s.type = ShapeType::Box;
        s.box.half_extents = half_extents;
        s.body_index = body_idx;
        s.local_transform = local;
        s.friction = friction;
        s.restitution = restitution;
        return s;
    }

    /**
     * @brief Create a sphere collision shape.
     *
     * @param [in] radius Sphere radius in meters.
     * @param [in] body_idx Owning body index.
     * @param [in] local Local transform from body frame to shape frame.
     * @param [in] friction Friction coefficient.
     * @param [in] restitution Restitution coefficient.
     * @return Constructed sphere shape descriptor.
     */
    static CollisionShape make_sphere(float radius, int body_idx,
                                      const Transform& local = Transform::identity(),
                                      float friction = 0.5f, float restitution = 0.3f) {
        CollisionShape s;
        s.type = ShapeType::Sphere;
        s.sphere.radius = radius;
        s.body_index = body_idx;
        s.local_transform = local;
        s.friction = friction;
        s.restitution = restitution;
        return s;
    }

    /**
     * @brief Create an infinite plane collision shape.
     *
     * @param [in] normal Plane normal vector (normalized internally).
     * @param [in] offset Plane offset along normal in meters.
     * @param [in] friction Friction coefficient.
     * @param [in] restitution Restitution coefficient.
     * @return Constructed plane shape descriptor.
     */
    static CollisionShape make_plane(const Vec3f& normal, float offset,
                                     float friction = 0.5f, float restitution = 0.0f) {
        CollisionShape s;
        s.type = ShapeType::Plane;
        s.plane.normal = normal.normalized();
        s.plane.offset = offset;
        s.body_index = -1;  // planes are typically world-owned
        s.friction = friction;
        s.restitution = restitution;
        return s;
    }
};

}  // namespace novaphy
