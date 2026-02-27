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
 * @param[out] X_J Per-link joint transforms.
 * @param[out] X_up Per-link parent transforms in spatial form.
 * @param[out] X_world Per-link world transforms for visualization/queries.
 */
void forward_kinematics(const Articulation& model,
                        const VecXf& q,
                        std::vector<SpatialTransform>& X_J,
                        std::vector<SpatialTransform>& X_up,
                        std::vector<Transform>& X_world) {
    int n = model.num_links();
    X_J.resize(n);
    X_up.resize(n);
    X_world.resize(n);

    for (int i = 0; i < n; ++i) {
        const auto& joint = model.joints[i];
        int qi = model.q_start(i);

        // Joint transform
        Transform T_J = joint.joint_transform(q.data() + qi);
        X_J[i] = SpatialTransform::from_transform(T_J);

        // Transform from link i's frame to parent's frame:
        // X_up[i] = X_J[i] * X_tree[i]  where X_tree = parent_to_joint
        SpatialTransform X_tree = SpatialTransform::from_transform(joint.parent_to_joint);
        X_up[i] = X_J[i] * X_tree;

        // World transform (for rendering/FK output)
        Transform T_link = joint.parent_to_joint * T_J;
        if (joint.parent >= 0) {
            X_world[i] = X_world[joint.parent] * T_link.inverse();
            // Actually we want the world-frame position of each link
            // X_world[i] = parent_world * parent_to_joint * joint_transform
            // But let's compute it from transforms directly
        }
    }

    // Recompute world transforms from Transform chain (clearer)
    for (int i = 0; i < n; ++i) {
        const auto& joint = model.joints[i];
        int qi = model.q_start(i);
        Transform T_J = joint.joint_transform(q.data() + qi);
        Transform T_local = joint.parent_to_joint * T_J;

        if (joint.parent < 0) {
            X_world[i] = T_local;
        } else {
            X_world[i] = X_world[joint.parent] * T_local;
        }
    }
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
                       const std::vector<SpatialVector>& f_ext) {
    int n = model.num_links();
    int nv = model.total_qd();

    // Forward pass: compute velocities and accelerations
    std::vector<SpatialTransform> X_J(n), X_up(n);
    std::vector<Transform> X_world(n);
    std::vector<SpatialVector> v(n), a(n);
    std::vector<SpatialVector> S(n);  // motion subspace (stored as single column for revolute)

    // Spatial acceleration due to gravity (expressed in world frame)
    // a_gravity = [0; -g] (the base acceleration is -gravity for the recursive formulation)
    SpatialVector a_grav = make_spatial(Vec3f::Zero(), -gravity);

    for (int i = 0; i < n; ++i) {
        const auto& joint = model.joints[i];
        int qi = model.q_start(i);
        int qdi = model.qd_start(i);
        int nv_i = joint.num_qd();

        // Joint transform
        Transform T_J = joint.joint_transform(q.data() + qi);
        X_J[i] = SpatialTransform::from_transform(T_J);
        SpatialTransform X_tree = SpatialTransform::from_transform(joint.parent_to_joint);
        X_up[i] = X_J[i] * X_tree;

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
            a[i] = X_up[i].apply_motion(a_grav);
        } else {
            v[i] = X_up[i].apply_motion(v[joint.parent]) + vJ;
            a[i] = X_up[i].apply_motion(a[joint.parent]);
        }

        // Joint acceleration
        SpatialVector aJ = SpatialVector::Zero();
        for (int k = 0; k < nv_i; ++k) {
            aJ += S_cols[k] * qdd(qdi + k);
        }

        a[i] += aJ + spatial_cross_motion(v[i], vJ);

        // Store first motion subspace column for tau computation
        if (nv_i > 0) S[i] = S_cols[0];
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
            f[joint.parent] += X_up[i].apply_force(f[i]);
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
    int n = model.num_links();
    int nv = model.total_qd();

    // Forward pass: compute transforms
    std::vector<SpatialTransform> X_J(n), X_up(n);
    std::vector<Transform> X_world(n);

    for (int i = 0; i < n; ++i) {
        const auto& joint = model.joints[i];
        int qi = model.q_start(i);

        Transform T_J = joint.joint_transform(q.data() + qi);
        X_J[i] = SpatialTransform::from_transform(T_J);
        SpatialTransform X_tree = SpatialTransform::from_transform(joint.parent_to_joint);
        X_up[i] = X_J[i] * X_tree;
    }

    // Composite rigid body algorithm
    std::vector<SpatialMatrix> I_c(n);
    for (int i = 0; i < n; ++i) {
        I_c[i] = model.I_body[i];
    }

    // Backward pass: composite inertia
    for (int i = n - 1; i >= 0; --i) {
        if (model.joints[i].parent >= 0) {
            // I_c[parent] += X_up[i]^T * I_c[i] * X_up[i]
            SpatialMatrix Xm = X_up[i].to_matrix();
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
                F_prop = X_up[j].apply_force(F_prop);
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
                       const std::vector<SpatialVector>& f_ext) {
    int nv = model.total_qd();

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

