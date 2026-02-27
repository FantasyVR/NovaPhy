#pragma once

#include "novaphy/math/math_types.h"
#include "novaphy/math/math_utils.h"

namespace novaphy {

/**
 * @brief 6D spatial vector using Featherstone ordering [angular; linear].
 */
using SpatialVector = Eigen::Matrix<float, 6, 1>;
/**
 * @brief 6x6 spatial matrix used for inertia and spatial transforms.
 */
using SpatialMatrix = Eigen::Matrix<float, 6, 6>;

/**
 * @brief Extract angular component from a spatial vector.
 *
 * @param [in] v Spatial vector [w; v].
 * @return Angular part (top 3).
 */
inline Vec3f spatial_angular(const SpatialVector& v) { return v.head<3>(); }
/**
 * @brief Extract linear component from a spatial vector.
 *
 * @param [in] v Spatial vector [w; v].
 * @return Linear part (bottom 3).
 */
inline Vec3f spatial_linear(const SpatialVector& v) { return v.tail<3>(); }

/**
 * @brief Construct a spatial vector from angular and linear components.
 *
 * @param [in] angular Angular component.
 * @param [in] linear Linear component.
 * @return Spatial vector [angular; linear].
 */
inline SpatialVector make_spatial(const Vec3f& angular, const Vec3f& linear) {
    SpatialVector v;
    v.head<3>() = angular;
    v.tail<3>() = linear;
    return v;
}

/**
 * @brief Spatial cross product for motion vectors.
 *
 * @details For v=[w; v_l] and u=[w'; v'], returns
 * [w x w'; w x v' + v_l x w'].
 *
 * @param [in] v Left motion vector.
 * @param [in] u Right motion vector.
 * @return Motion cross product.
 */
inline SpatialVector spatial_cross_motion(const SpatialVector& v, const SpatialVector& u) {
    Vec3f w = spatial_angular(v);
    Vec3f vl = spatial_linear(v);
    Vec3f wp = spatial_angular(u);
    Vec3f vp = spatial_linear(u);
    return make_spatial(w.cross(wp), w.cross(vp) + vl.cross(wp));
}

/**
 * @brief Spatial cross product for force vectors.
 *
 * @details For motion v=[w; v_l] and force f=[n; f_l], returns
 * [w x n + v_l x f_l; w x f_l].
 *
 * @param [in] v Motion vector.
 * @param [in] f Force vector.
 * @return Force cross product.
 */
inline SpatialVector spatial_cross_force(const SpatialVector& v, const SpatialVector& f) {
    Vec3f w = spatial_angular(v);
    Vec3f vl = spatial_linear(v);
    Vec3f n = spatial_angular(f);
    Vec3f fl = spatial_linear(f);
    return make_spatial(w.cross(n) + vl.cross(fl), w.cross(fl));
}

/**
 * @brief Spatial rigid transform between coordinate frames.
 *
 * @details Represents X_{A<-B} with rotation `E` (B to A) and translation `r`
 * (origin of B expressed in A, in meters).
 */
struct SpatialTransform {
    Mat3f E = Mat3f::Identity();  /**< Rotation from frame B to frame A. */
    Vec3f r = Vec3f::Zero();      /**< Translation from A origin to B origin, expressed in A (m). */

    /** @brief Construct identity spatial transform. */
    SpatialTransform() = default;

    /**
     * @brief Construct spatial transform from rotation and translation.
     *
     * @param [in] rot Rotation from B to A.
     * @param [in] trans Origin of B expressed in A (m).
     */
    SpatialTransform(const Mat3f& rot, const Vec3f& trans) : E(rot), r(trans) {}

    /**
     * @brief Apply transform to a spatial motion vector.
     *
     * @param [in] v Motion vector in frame B.
     * @return Motion vector expressed in frame A.
     */
    SpatialVector apply_motion(const SpatialVector& v) const {
        Vec3f w = spatial_angular(v);
        Vec3f vl = spatial_linear(v);
        Vec3f w_new = E * w;
        Vec3f v_new = E * vl + r.cross(E * w);
        return make_spatial(w_new, v_new);
    }

    /**
     * @brief Apply dual transform to a spatial force vector.
     *
     * @param [in] f Force vector in frame A dual space.
     * @return Force vector mapped consistently with frame transform.
     */
    SpatialVector apply_force(const SpatialVector& f) const {
        Vec3f n = spatial_angular(f);
        Vec3f fl = spatial_linear(f);
        Vec3f fl_new = E.transpose() * fl;
        Vec3f n_new = E.transpose() * (n - r.cross(fl));
        return make_spatial(n_new, fl_new);
    }

    /**
     * @brief Convert to 6x6 matrix representation.
     *
     * @return Spatial transform matrix.
     */
    SpatialMatrix to_matrix() const {
        Mat3f rx = skew(r);
        SpatialMatrix X;
        X.topLeftCorner<3, 3>() = E;
        X.topRightCorner<3, 3>() = Mat3f::Zero();
        X.bottomLeftCorner<3, 3>() = rx * E;
        X.bottomRightCorner<3, 3>() = E;
        return X;
    }

    /**
     * @brief Compute inverse spatial transform.
     *
     * @return Inverse transform X_{B<-A}.
     */
    SpatialTransform inverse() const {
        Mat3f Et = E.transpose();
        return SpatialTransform(Et, -Et * r);
    }

    /**
     * @brief Compose two spatial transforms.
     *
     * @param [in] other Right-side transform.
     * @return Composed transform.
     */
    SpatialTransform operator*(const SpatialTransform& other) const {
        return SpatialTransform(E * other.E, r + E * other.r);
    }

    /**
     * @brief Create spatial transform from rigid transform.
     *
     * @param [in] t Rigid transform with quaternion rotation and translation.
     * @return Spatial transform with matching pose.
     */
    static SpatialTransform from_transform(const Transform& t) {
        return SpatialTransform(t.rotation_matrix(), t.position);
    }

    /**
     * @brief Create identity spatial transform.
     *
     * @return Identity transform.
     */
    static SpatialTransform identity() { return SpatialTransform(); }
};

/**
 * @brief Build a 6x6 spatial inertia matrix.
 *
 * @details Inputs are body mass, center of mass in body frame, and rotational
 * inertia about CoM in body frame.
 *
 * I_spatial = [ I_rot + m * [c]_x^T * [c]_x,  m * [c]_x^T ]
 *             [ m * [c]_x,                     m * I_3     ]
 *
 * @param [in] mass Body mass in kilograms.
 * @param [in] com Center of mass in body coordinates (m).
 * @param [in] I_rot Rotational inertia about CoM in body coordinates (kg*m^2).
 * @return Spatial inertia matrix in body coordinates.
 */
inline SpatialMatrix spatial_inertia_matrix(float mass, const Vec3f& com,
                                            const Mat3f& I_rot) {
    Mat3f cx = skew(com);
    SpatialMatrix I;
    I.topLeftCorner<3, 3>() = I_rot + mass * cx.transpose() * cx;
    I.topRightCorner<3, 3>() = mass * cx.transpose();
    I.bottomLeftCorner<3, 3>() = mass * cx;
    I.bottomRightCorner<3, 3>() = mass * Mat3f::Identity();
    return I;
}

/**
 * @brief Transform spatial inertia matrix by spatial transform.
 *
 * @param [in] X Spatial transform.
 * @param [in] I Input spatial inertia matrix.
 * @return Transformed spatial inertia matrix.
 */
inline SpatialMatrix transform_spatial_inertia(const SpatialTransform& X,
                                               const SpatialMatrix& I) {
    SpatialMatrix Xm = X.to_matrix();
    return Xm * I * Xm.transpose();
}

}  // namespace novaphy
