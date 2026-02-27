#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include "novaphy/novaphy_types.h"

namespace novaphy {

/** @brief 2D float vector type. */
using Vec2f = Eigen::Vector2f;
/** @brief 3D float vector type. */
using Vec3f = Eigen::Vector3f;
/** @brief 4D float vector type. */
using Vec4f = Eigen::Vector4f;
/** @brief Dynamic-size float vector type. */
using VecXf = Eigen::VectorXf;

/** @brief 3x3 float matrix type. */
using Mat3f = Eigen::Matrix3f;
/** @brief 4x4 float matrix type. */
using Mat4f = Eigen::Matrix4f;
/** @brief Dynamic-size float matrix type. */
using MatXf = Eigen::MatrixXf;
/** @brief 6x6 float matrix type. */
using Mat6f = Eigen::Matrix<float, 6, 6>;

/** @brief Unit quaternion type used for 3D orientation. */
using Quatf = Eigen::Quaternionf;

/**
 * @brief Rigid transform represented by translation and quaternion orientation.
 *
 * @details `position` is expressed in world coordinates (m). `rotation` stores
 * attitude as a unit quaternion for stable composition and interpolation.
 */
struct Transform {
    Vec3f position = Vec3f::Zero();   /**< Translation component in meters. */
    Quatf rotation = Quatf::Identity();  /**< Unit quaternion orientation. */

    /** @brief Construct identity transform. */
    Transform() = default;

    /**
     * @brief Construct transform from translation and quaternion.
     *
     * @param [in] p Translation component in meters.
     * @param [in] r Orientation quaternion (normalized internally).
     */
    Transform(const Vec3f& p, const Quatf& r) : position(p), rotation(r.normalized()) {}

    /**
     * @brief Compose two transforms.
     *
     * @param [in] other Right-hand transform to apply after this one.
     * @return Composed transform.
     */
    Transform operator*(const Transform& other) const {
        return Transform(position + rotation * other.position,
                         rotation * other.rotation);
    }

    /**
     * @brief Compute inverse rigid transform.
     *
     * @return Inverse transform.
     */
    Transform inverse() const {
        Quatf inv_rot = rotation.conjugate();
        return Transform(inv_rot * (-position), inv_rot);
    }

    /**
     * @brief Transform a point from local to world coordinates.
     *
     * @param [in] p Local-space point.
     * @return World-space point in meters.
     */
    Vec3f transform_point(const Vec3f& p) const { return position + rotation * p; }

    /**
     * @brief Transform a direction vector (rotation only).
     *
     * @param [in] v Local-space direction.
     * @return Rotated world-space direction.
     */
    Vec3f transform_vector(const Vec3f& v) const { return rotation * v; }

    /**
     * @brief Get rotation matrix equivalent of quaternion orientation.
     *
     * @return 3x3 rotation matrix.
     */
    Mat3f rotation_matrix() const { return rotation.toRotationMatrix(); }

    /**
     * @brief Create identity transform.
     *
     * @return Identity transform.
     */
    static Transform identity() { return Transform(); }

    /**
     * @brief Create translation-only transform.
     *
     * @param [in] t Translation in meters.
     * @return Transform with identity rotation.
     */
    static Transform from_translation(const Vec3f& t) {
        return Transform(t, Quatf::Identity());
    }

    /**
     * @brief Create rotation-only transform.
     *
     * @param [in] q Orientation quaternion.
     * @return Transform with zero translation.
     */
    static Transform from_rotation(const Quatf& q) {
        return Transform(Vec3f::Zero(), q);
    }

    /**
     * @brief Create transform from axis-angle rotation.
     *
     * @param [in] axis Rotation axis.
     * @param [in] angle Rotation angle in radians.
     * @return Rotation-only transform.
     */
    static Transform from_axis_angle(const Vec3f& axis, float angle) {
        return Transform(Vec3f::Zero(),
                         Quatf(Eigen::AngleAxisf(angle, axis.normalized())));
    }
};

}  // namespace novaphy
