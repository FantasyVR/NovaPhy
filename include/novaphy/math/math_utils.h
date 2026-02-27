#pragma once

#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief Builds the skew-symmetric cross-product matrix from a 3D vector.
 * @details Returns `[v]x` such that `skew(a) * b == a.cross(b)`, which is
 *          commonly used in rigid-body spatial algebra.
 * @param[in] v Input vector in either local or world coordinates.
 * @return 3x3 skew-symmetric matrix representing the cross-product operator.
 */
inline Mat3f skew(const Vec3f& v) {
    Mat3f m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    return m;
}

/**
 * @brief Creates a unit quaternion from axis-angle representation.
 * @param[in] axis Rotation axis (direction only; normalized internally).
 * @param[in] angle Rotation magnitude in radians.
 * @return Quaternion representing the same 3D orientation increment.
 */
inline Quatf quat_from_axis_angle(const Vec3f& axis, float angle) {
    return Quatf(Eigen::AngleAxisf(angle, axis.normalized()));
}

/**
 * @brief Clamps a scalar to the closed interval `[lo, hi]`.
 * @param[in] val Input scalar value.
 * @param[in] lo Lower bound.
 * @param[in] hi Upper bound.
 * @return Clamped value within `[lo, hi]`.
 */
inline float clampf(float val, float lo, float hi) {
    return std::max(lo, std::min(val, hi));
}

/**
 * @brief Linearly interpolates between two scalar values.
 * @param[in] a Start value at `t = 0`.
 * @param[in] b End value at `t = 1`.
 * @param[in] t Interpolation parameter, typically in `[0, 1]`.
 * @return Interpolated scalar `a + t * (b - a)`.
 */
inline float lerpf(float a, float b, float t) { return a + t * (b - a); }

/** @brief Mathematical constant pi in single precision. */
constexpr float PI = 3.14159265358979323846f;

/**
 * @brief Converts an angle from degrees to radians.
 * @param[in] deg Angle in degrees.
 * @return Angle in radians.
 */
inline float deg2rad(float deg) { return deg * PI / 180.0f; }

/**
 * @brief Converts an angle from radians to degrees.
 * @param[in] rad Angle in radians.
 * @return Angle in degrees.
 */
inline float rad2deg(float rad) { return rad * 180.0f / PI; }

}  // namespace novaphy
