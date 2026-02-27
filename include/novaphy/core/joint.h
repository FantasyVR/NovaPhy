#pragma once

#include "novaphy/math/math_types.h"
#include "novaphy/math/spatial.h"

namespace novaphy {

/**
 * @brief Joint types supported by articulation dynamics.
 */
enum class JointType {
    Revolute,  /**< 1 DOF rotation about a single axis (hinge). */
    Fixed,     /**< 0 DOF rigid attachment (weld). */
    Free,      /**< 6 DOF free-floating base with quaternion position state. */
    Slide,     /**< 1 DOF translation along a single axis. */
    Ball,      /**< 3 rotational DOFs represented by quaternion state. */
};

/**
 * @brief Joint descriptor connecting one link to its parent.
 *
 * @details Defines joint kinematics and motion subspace used by Featherstone
 * forward/inverse dynamics. Axes are expressed in the joint-local frame.
 */
struct Joint {
    JointType type = JointType::Revolute;  /**< Joint type. */
    Vec3f axis = Vec3f(0, 0, 1);           /**< Joint axis for revolute/slide joints. */
    int parent = -1;                       /**< Parent link index, or -1 for world root. */

    Transform parent_to_joint = Transform::identity();  /**< Constant transform from parent frame to joint frame. */

    /**
     * @brief Get number of generalized position coordinates for this joint.
     *
     * @return Position DOF count.
     */
    int num_q() const {
        switch (type) {
            case JointType::Revolute: return 1;
            case JointType::Fixed:    return 0;
            case JointType::Free:     return 7;  // pos(3) + quat(4)
            case JointType::Slide:    return 1;
            case JointType::Ball:     return 4;  // quat(4): [qx, qy, qz, qw]
        }
        return 0;
    }

    /**
     * @brief Get number of generalized velocity coordinates for this joint.
     *
     * @return Velocity DOF count.
     */
    int num_qd() const {
        switch (type) {
            case JointType::Revolute: return 1;
            case JointType::Fixed:    return 0;
            case JointType::Free:     return 6;  // angular(3) + linear(3)
            case JointType::Slide:    return 1;
            case JointType::Ball:     return 3;  // angular(3)
        }
        return 0;
    }

    /**
     * @brief Compute joint transform from generalized position coordinates.
     *
     * @param [in] q Pointer to this joint's coordinate block in the global q vector.
     * @return Joint transform from joint frame to child-link frame.
     *
     * @note `q` layout depends on joint type:
     * revolute/slide: scalar, ball: quaternion, free: translation + quaternion.
     */
    Transform joint_transform(const float* q) const;

    /**
     * @brief Compute motion subspace columns for this joint.
     *
     * @param [out] S_cols Output array of spatial motion columns, length = num_qd().
     * @return void
     *
     * @details Columns follow Featherstone convention [angular; linear].
     */
    void motion_subspace(SpatialVector* S_cols) const;
};

}  // namespace novaphy
