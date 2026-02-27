/**
 * @file joint.cpp
 * @brief Joint kinematics and motion-subspace helper implementations.
 */
#include "novaphy/core/joint.h"

#include <cmath>

namespace novaphy {

/**
 * @brief Computes a joint transform from generalized coordinates.
 * @param[in] q Pointer to the joint configuration vector for this joint type.
 * @return Rigid transform mapping the parent joint frame to the child frame.
 */
Transform Joint::joint_transform(const float* q) const {
    switch (type) {
        case JointType::Revolute: {
            float angle = q[0];
            return Transform::from_axis_angle(axis, angle);
        }
        case JointType::Fixed:
            return Transform::identity();
        case JointType::Free: {
            // q = [px, py, pz, qx, qy, qz, qw].
            Vec3f pos(q[0], q[1], q[2]);
            Quatf rot(q[6], q[3], q[4], q[5]);  // Eigen order: w, x, y, z.
            rot.normalize();
            return Transform(pos, rot);
        }
        case JointType::Slide: {
            // q = [distance], translation along the normalized joint axis.
            Vec3f translation = axis.normalized() * q[0];
            return Transform::from_translation(translation);
        }
        case JointType::Ball: {
            // q = [qx, qy, qz, qw], pure rotational parameterization.
            Quatf rot(q[3], q[0], q[1], q[2]);  // Eigen order: w, x, y, z.
            rot.normalize();
            return Transform(Vec3f::Zero(), rot);
        }
    }
    return Transform::identity();
}

/**
 * @brief Fills the joint motion-subspace basis vectors.
 * @param[out] S_cols Output spatial basis columns, sized by `dof()`.
 */
void Joint::motion_subspace(SpatialVector* S_cols) const {
    switch (type) {
        case JointType::Revolute:
            // Single column: rotation about axis [angular; 0].
            S_cols[0] = make_spatial(axis, Vec3f::Zero());
            break;
        case JointType::Fixed:
            // No motion columns for fixed joints.
            break;
        case JointType::Free:
            // 6 columns: identity basis (angular first, then linear).
            for (int i = 0; i < 6; ++i) {
                S_cols[i] = SpatialVector::Zero();
                S_cols[i](i) = 1.0f;
            }
            break;
        case JointType::Slide:
            // Single column: translation along axis [0; axis].
            S_cols[0] = make_spatial(Vec3f::Zero(), axis.normalized());
            break;
        case JointType::Ball:
            // 3 columns: angular identity basis [e_i; 0].
            S_cols[0] = make_spatial(Vec3f(1, 0, 0), Vec3f::Zero());
            S_cols[1] = make_spatial(Vec3f(0, 1, 0), Vec3f::Zero());
            S_cols[2] = make_spatial(Vec3f(0, 0, 1), Vec3f::Zero());
            break;
    }
}

}  // namespace novaphy
