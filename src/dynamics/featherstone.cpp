/**
 * @file featherstone.cpp
 * @brief Featherstone-style articulated rigid-body dynamics algorithms.
 */
#include "novaphy/dynamics/featherstone.h"

#include <Eigen/Dense>
#include <cmath>

namespace novaphy {
namespace featherstone {

/**
 * @brief Computes articulated link transforms from generalized positions.
 * @param[in] model Articulation model containing joint tree topology.
 * @param[in] q Generalized coordinates.
 * @return Joint, parent-relative, and world transforms for all links.
 */
ForwardKinematicsResult forward_kinematics(const Articulation& model,
                                           const VecXf& q) {
    const int n = model.num_links();
    ForwardKinematicsResult result;
    result.joint_transforms.resize(n);
    result.parent_transforms.resize(n);
    result.world_transforms.resize(n);

    for (int i = 0; i < n; ++i) {
        const auto& joint = model.joints[i];
        const int qi = model.q_start(i);
        const Transform joint_transform = joint.joint_transform(q.data() + qi);
        const Transform local_transform = joint.parent_to_joint * joint_transform;

        result.joint_transforms[i] = SpatialTransform::from_transform(joint_transform);
        result.parent_transforms[i] =
            result.joint_transforms[i] * SpatialTransform::from_transform(joint.parent_to_joint);

        if (joint.parent < 0) {
            result.world_transforms[i] = local_transform;
        } else {
            result.world_transforms[i] = result.world_transforms[joint.parent] * local_transform;
        }
    }

    return result;
}

/**
 * @brief Computes inverse dynamics using recursive Newton-Euler passes.
 * @param[in] model Articulation model.
 * @param[in] q Generalized coordinates.
 * @param[in] qd Generalized velocities.
 * @param[in] qdd Generalized accelerations.
 * @param[in] gravity World gravity vector in m/s^2.
 * @param[in] f_ext Optional external spatial forces per link.
 * @return Joint generalized forces/torques in SI units.
 */
VecXf inverse_dynamics(const Articulation& model,
                       const VecXf& q,
                       const VecXf& qd,
                       const VecXf& qdd,
                       const Vec3f& gravity,
                       std::span<const SpatialVector> f_ext) {
    const int n = model.num_links();
    const int nv = model.total_qd();
    const auto kinematics = forward_kinematics(model, q);

    // Forward pass: compute velocities and accelerations
    std::vector<SpatialVector> v(n), a(n);

    // Spatial acceleration due to gravity (expressed in world frame)
    // a_gravity = [0; -g] (the base acceleration is -gravity for the recursive formulation)
    SpatialVector a_grav = make_spatial(Vec3f::Zero(), -gravity);

    for (int i = 0; i < n; ++i) {
        const auto& joint = model.joints[i];
        const int qdi = model.qd_start(i);
        const int nv_i = joint.num_qd();

        // Motion subspace
        SpatialVector S_cols[6];
        joint.motion_subspace(S_cols);

        // Joint velocity: vJ = S * qd_i
        SpatialVector vJ = SpatialVector::Zero();
        for (int k = 0; k < nv_i; ++k) {
            vJ += S_cols[k] * qd(qdi + k);
        }

        if (joint.parent < 0) {
            v[i] = vJ;
            a[i] = kinematics.parent_transforms[i].apply_motion(a_grav);
        } else {
            v[i] = kinematics.parent_transforms[i].apply_motion(v[joint.parent]) + vJ;
            a[i] = kinematics.parent_transforms[i].apply_motion(a[joint.parent]);
        }

        // Joint acceleration
        SpatialVector aJ = SpatialVector::Zero();
        for (int k = 0; k < nv_i; ++k) {
            aJ += S_cols[k] * qdd(qdi + k);
        }

        a[i] += aJ + spatial_cross_motion(v[i], vJ);

    }

    // Backward pass: compute forces and project onto joint axes
    std::vector<SpatialVector> f(n);
    for (int i = 0; i < n; ++i) {
        f[i] = model.I_body[i] * a[i] + spatial_cross_force(v[i], model.I_body[i] * v[i]);
        if (!f_ext.empty() && i < static_cast<int>(f_ext.size())) {
            f[i] -= f_ext[i];
        }
    }

    VecXf tau = VecXf::Zero(nv);

    for (int i = n - 1; i >= 0; --i) {
        const auto& joint = model.joints[i];
        int qdi = model.qd_start(i);
        int nv_i = joint.num_qd();

        // Project onto joint axes
        SpatialVector S_cols[6];
        joint.motion_subspace(S_cols);
        for (int k = 0; k < nv_i; ++k) {
            tau(qdi + k) = S_cols[k].dot(f[i]);
        }

        // Propagate force to parent
        if (joint.parent >= 0) {
            f[joint.parent] += kinematics.parent_transforms[i].apply_force(f[i]);
        }
    }

    return tau;
}

/**
 * @brief Computes the articulated mass matrix using CRBA.
 * @param[in] model Articulation model.
 * @param[in] q Generalized coordinates.
 * @return Symmetric positive-definite mass matrix `H(q)`.
 */
MatXf mass_matrix(const Articulation& model,
                  const VecXf& q) {
    const int n = model.num_links();
    const int nv = model.total_qd();
    const auto kinematics = forward_kinematics(model, q);

    // Composite rigid body algorithm
    std::vector<SpatialMatrix> I_c(n);
    for (int i = 0; i < n; ++i) {
        I_c[i] = model.I_body[i];
    }

    // Backward pass: composite inertia
    for (int i = n - 1; i >= 0; --i) {
        if (model.joints[i].parent >= 0) {
            // I_c[parent] += X_up[i]^T * I_c[i] * X_up[i]
            const SpatialMatrix Xm = kinematics.parent_transforms[i].to_matrix();
            I_c[model.joints[i].parent] += Xm.transpose() * I_c[i] * Xm;
        }
    }

    // Compute mass matrix H
    MatXf H = MatXf::Zero(nv, nv);

    for (int i = 0; i < n; ++i) {
        const auto& joint_i = model.joints[i];
        int qdi = model.qd_start(i);
        int nv_i = joint_i.num_qd();
        if (nv_i == 0) continue;

        SpatialVector S_cols_i[6];
        joint_i.motion_subspace(S_cols_i);

        // F = I_c[i] * S_i
        for (int k = 0; k < nv_i; ++k) {
            SpatialVector F_k = I_c[i] * S_cols_i[k];

            // H[i,i] block: S_i^T * F
            for (int l = 0; l < nv_i; ++l) {
                H(qdi + k, qdi + l) = S_cols_i[l].dot(F_k);
            }

            // Propagate F up the tree to fill off-diagonal blocks
            SpatialVector F_prop = F_k;
            int j = i;
            while (model.joints[j].parent >= 0) {
                F_prop = kinematics.parent_transforms[j].apply_force(F_prop);
                j = model.joints[j].parent;

                const auto& joint_j = model.joints[j];
                int qdj = model.qd_start(j);
                int nv_j = joint_j.num_qd();

                SpatialVector S_cols_j[6];
                joint_j.motion_subspace(S_cols_j);

                for (int l = 0; l < nv_j; ++l) {
                    float val = S_cols_j[l].dot(F_prop);
                    H(qdi + k, qdj + l) = val;
                    H(qdj + l, qdi + k) = val;  // symmetric
                }
            }
        }
    }

    return H;
}

/**
 * @brief Solves forward dynamics `qdd = H^{-1}(tau - C)`.
 * @param[in] model Articulation model.
 * @param[in] q Generalized coordinates.
 * @param[in] qd Generalized velocities.
 * @param[in] tau Applied generalized forces.
 * @param[in] gravity World gravity vector in m/s^2.
 * @param[in] f_ext Optional external spatial forces per link.
 * @return Generalized accelerations.
 */
VecXf forward_dynamics(const Articulation& model,
                       const VecXf& q,
                       const VecXf& qd,
                       const VecXf& tau,
                       const Vec3f& gravity,
                       std::span<const SpatialVector> f_ext) {
    const int nv = model.total_qd();

    // H = CRBA(q)
    MatXf H = mass_matrix(model, q);

    // C = RNEA(q, qd, 0), bias forces (gravity + Coriolis + centrifugal).
    VecXf qdd_zero = VecXf::Zero(nv);
    VecXf C = inverse_dynamics(model, q, qd, qdd_zero, gravity, f_ext);

    // qdd = H^{-1} * (tau - C)
    // Use Cholesky decomposition for SPD matrix
    Eigen::LLT<MatXf> llt(H);
    VecXf qdd = llt.solve(tau - C);

    return qdd;
}

}  // namespace featherstone
}  // namespace novaphy

