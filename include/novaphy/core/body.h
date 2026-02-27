#pragma once

#include "novaphy/math/math_types.h"
#include "novaphy/math/spatial.h"

namespace novaphy {

/**
 * @brief Rigid-body inertial properties.
 *
 * @details Stores mass, center of mass, and inertia tensor in body-local
 * coordinates. These quantities are used by free-body and articulated
 * dynamics solvers in SI units.
 */
struct RigidBody {
    float mass = 1.0f;             /**< Body mass in kilograms (kg). */
    Mat3f inertia = Mat3f::Identity();  /**< Body-frame inertia tensor about CoM (kg*m^2). */
    Vec3f com = Vec3f::Zero();          /**< Center of mass in body-local coordinates (m). */

    /**
     * @brief Get inverse mass.
     *
     * @return Inverse mass (kg^-1), or zero for static bodies.
     */
    float inv_mass() const { return mass > 0.0f ? 1.0f / mass : 0.0f; }

    /**
     * @brief Get inverse inertia tensor in body coordinates.
     *
     * @return Inverse inertia tensor ((kg*m^2)^-1), or zero matrix for static bodies.
     */
    Mat3f inv_inertia() const {
        if (mass <= 0.0f) return Mat3f::Zero();
        return inertia.inverse();
    }

    /**
     * @brief Check whether the body is static.
     *
     * @return True if mass is non-positive and body is treated as immovable.
     */
    bool is_static() const { return mass <= 0.0f; }

    /**
     * @brief Build 6x6 spatial inertia matrix.
     *
     * @details Uses body mass, local CoM, and local inertia tensor to produce
     * the spatial inertia at the body-frame origin.
     *
     * @return Spatial inertia matrix in body coordinates.
     */
    SpatialMatrix spatial_inertia() const {
        return spatial_inertia_matrix(mass, com, inertia);
    }

    /**
     * @brief Construct a rigid body with box inertia.
     *
     * @param [in] m Body mass in kilograms.
     * @param [in] half_extents Box half extents in meters in local coordinates.
     * @return RigidBody with diagonal inertia for a solid box.
     */
    static RigidBody from_box(float m, const Vec3f& half_extents) {
        RigidBody b;
        b.mass = m;
        float w = 2.0f * half_extents.x();
        float h = 2.0f * half_extents.y();
        float d = 2.0f * half_extents.z();
        float c = m / 12.0f;
        b.inertia = Mat3f::Zero();
        b.inertia(0, 0) = c * (h * h + d * d);
        b.inertia(1, 1) = c * (w * w + d * d);
        b.inertia(2, 2) = c * (w * w + h * h);
        return b;
    }

    /**
     * @brief Construct a rigid body with solid-sphere inertia.
     *
     * @param [in] m Body mass in kilograms.
     * @param [in] radius Sphere radius in meters.
     * @return RigidBody with isotropic inertia tensor.
     */
    static RigidBody from_sphere(float m, float radius) {
        RigidBody b;
        b.mass = m;
        float I = (2.0f / 5.0f) * m * radius * radius;
        b.inertia = Mat3f::Identity() * I;
        return b;
    }

    /**
     * @brief Construct a static rigid body.
     *
     * @return RigidBody with zero mass/inertia that is treated as immovable.
     */
    static RigidBody make_static() {
        RigidBody b;
        b.mass = 0.0f;
        b.inertia = Mat3f::Zero();
        return b;
    }
};

}  // namespace novaphy
