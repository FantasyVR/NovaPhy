#pragma once

#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief Axis-aligned bounding box in 3D.
 *
 * @details Used by broadphase collision culling and spatial queries.
 * Coordinates are expressed in world units (meters).
 */
struct AABB {
    Vec3f min = Vec3f::Constant(std::numeric_limits<float>::max());   /**< Minimum corner. */
    Vec3f max = Vec3f::Constant(-std::numeric_limits<float>::max());  /**< Maximum corner. */

    /** @brief Construct an invalid/empty AABB. */
    AABB() = default;

    /**
     * @brief Construct AABB from corners.
     *
     * @param [in] min_pt Minimum corner.
     * @param [in] max_pt Maximum corner.
     */
    AABB(const Vec3f& min_pt, const Vec3f& max_pt) : min(min_pt), max(max_pt) {}

    /**
     * @brief Test overlap with another AABB.
     *
     * @param [in] other Other AABB.
     * @return True if boxes overlap on all axes.
     */
    bool overlaps(const AABB& other) const {
        return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
               (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
               (min.z() <= other.max.z() && max.z() >= other.min.z());
    }

    /**
     * @brief Expand this AABB to include a point.
     *
     * @param [in] point Point in world coordinates.
     * @return void
     */
    void expand(const Vec3f& point) {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }

    /**
     * @brief Return merged AABB containing this and another box.
     *
     * @param [in] other Other AABB.
     * @return Union AABB.
     */
    AABB merged(const AABB& other) const {
        return AABB(min.cwiseMin(other.min), max.cwiseMax(other.max));
    }

    /**
     * @brief Return AABB expanded by uniform margin.
     *
     * @param [in] margin Margin in meters.
     * @return Expanded AABB.
     */
    AABB expanded(float margin) const {
        Vec3f m(margin, margin, margin);
        return AABB(min - m, max + m);
    }

    /**
     * @brief Compute center point.
     *
     * @return Center of AABB.
     */
    Vec3f center() const { return 0.5f * (min + max); }

    /**
     * @brief Compute half extents along each axis.
     *
     * @return Half extents vector.
     */
    Vec3f half_extents() const { return 0.5f * (max - min); }

    /**
     * @brief Compute surface area.
     *
     * @return Surface area in square meters.
     */
    float surface_area() const {
        Vec3f d = max - min;
        return 2.0f * (d.x() * d.y() + d.y() * d.z() + d.z() * d.x());
    }

    /**
     * @brief Check if AABB has valid corner ordering.
     *
     * @return True if min <= max for all axes.
     */
    bool is_valid() const {
        return min.x() <= max.x() && min.y() <= max.y() && min.z() <= max.z();
    }

    /**
     * @brief Create AABB from center and half extents.
     *
     * @param [in] center Center point.
     * @param [in] half Half extents.
     * @return AABB defined by center and extents.
     */
    static AABB from_center_half_extents(const Vec3f& center, const Vec3f& half) {
        return AABB(center - half, center + half);
    }

    /**
     * @brief Create AABB enclosing a sphere.
     *
     * @param [in] center Sphere center.
     * @param [in] radius Sphere radius in meters.
     * @return Sphere-bounding AABB.
     */
    static AABB from_sphere(const Vec3f& center, float radius) {
        Vec3f r(radius, radius, radius);
        return AABB(center - r, center + r);
    }

    /**
     * @brief Create AABB enclosing an oriented box.
     *
     * @param [in] half_extents Local half extents of box.
     * @param [in] t Box world transform.
     * @return World-space AABB that contains all eight transformed corners.
     */
    static AABB from_oriented_box(const Vec3f& half_extents, const Transform& t) {
        AABB result;
        // Transform each of the 8 corners
        for (int i = 0; i < 8; ++i) {
            Vec3f corner(
                (i & 1) ? half_extents.x() : -half_extents.x(),
                (i & 2) ? half_extents.y() : -half_extents.y(),
                (i & 4) ? half_extents.z() : -half_extents.z());
            result.expand(t.transform_point(corner));
        }
        return result;
    }
};

}  // namespace novaphy
