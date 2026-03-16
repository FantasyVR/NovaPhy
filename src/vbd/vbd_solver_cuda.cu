/**
 * @file vbd_solver_cuda.cu
 * @brief CUDA backend for 3D AVBD: collision → vertex coloring → inertial init
 *        → per-color iteration (build LHS/RHS → LDL solve → update position)
 *        → update dual (lambda/penalty) → velocity update.
 *
 * Vertex coloring: bodies that share a contact get different colors; same-color
 * bodies update in parallel (no data race); different colors run in sequence.
 * Augmented Lagrangian handles hard constraints; coloring gives maximum GPU
 * parallelism while avoiding constraint conflicts.
 */

#include "novaphy/vbd/vbd_solver.h"
#include "novaphy/core/shape.h"

#include <cuda_runtime.h>
#include <cooperative_groups.h>
#include <cub/cub.cuh>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <set>
#include <vector>

namespace cg = cooperative_groups;

namespace novaphy {

namespace {

// Constants copied from the CPU implementation to keep AVBD behavior aligned.
constexpr float PENALTY_MIN = 1.0f;
constexpr float PENALTY_MAX = 10000000000.0f;
constexpr float STICK_THRESH = 0.00001f;
constexpr float COLLISION_MARGIN = 0.01f;

// Simple POD types used on the device side. We deliberately avoid exposing
// Eigen types to CUDA kernels to keep the device code minimal and portable.
struct DeviceVec3 {
    float x, y, z;
};

struct DeviceQuat {
    float w, x, y, z;
};

struct DeviceBody {
    float mass;
    float inertia_diag[3];
    int   is_static;
};

struct DeviceContact {
    int body_a;
    int body_b;
    DeviceVec3 rA;
    DeviceVec3 rB;
    float basis[9];   // row-major 3x3 matrix
    float C0[3];
    float penalty[3];
    float lambda[3];
    float friction;
    int   feature_id;
    int   stick;
};

struct DeviceJoint {
    int body_a;
    int body_b;
    DeviceVec3 rA;
    DeviceVec3 rB;
    float C0Lin[3];
    float C0Ang[3];
    float penaltyLin[3];
    float penaltyAng[3];
    float lambdaLin[3];
    float lambdaAng[3];
    float stiffnessLin;
    float stiffnessAng;
    float torqueArm;
    float fracture;
    int   broken;
};

struct DeviceSpring {
    int body_a;
    int body_b;
    DeviceVec3 rA;
    DeviceVec3 rB;
    float rest;
    float stiffness;
};

// GPU collision: AABB and shape for broadphase.
struct DeviceAABB {
    float minx, miny, minz;
    float maxx, maxy, maxz;
};

struct DeviceShape {
    int body_index;
    int type;   // 0 = box, 1 = plane, 2 = sphere
    int is_static;
    float half[3];
    float radius;
    float plane_n[3];
    float plane_d;
    float local_pos[3];
    float local_quat[4];
    float friction;
};

// --- small helpers (host + device where needed) ---

inline DeviceVec3 to_device(const Vec3f& v) {
    return DeviceVec3{v.x(), v.y(), v.z()};
}

inline Vec3f to_host(const DeviceVec3& v) {
    return Vec3f(v.x, v.y, v.z);
}

inline DeviceQuat to_device(const Quatf& q) {
    return DeviceQuat{q.w(), q.x(), q.y(), q.z()};
}

__host__ __device__ inline DeviceVec3 make_vec3(float x, float y, float z) {
    DeviceVec3 r; r.x = x; r.y = y; r.z = z; return r;
}

__host__ __device__ inline DeviceVec3 add(const DeviceVec3& a, const DeviceVec3& b) {
    return make_vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__host__ __device__ inline DeviceVec3 sub(const DeviceVec3& a, const DeviceVec3& b) {
    return make_vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__host__ __device__ inline DeviceVec3 mul(const DeviceVec3& a, float s) {
    return make_vec3(a.x * s, a.y * s, a.z * s);
}

__host__ __device__ inline DeviceVec3 mul_componentwise(const DeviceVec3& a, const DeviceVec3& b) {
    return make_vec3(a.x * b.x, a.y * b.y, a.z * b.z);
}

__host__ __device__ inline float dot(const DeviceVec3& a, const DeviceVec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__host__ __device__ inline DeviceVec3 cross(const DeviceVec3& a, const DeviceVec3& b) {
    return make_vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

__host__ __device__ inline float length(const DeviceVec3& a) {
    return sqrtf(dot(a, a));
}

__host__ __device__ inline DeviceVec3 mat3_row(const float m[9], int row) {
    return make_vec3(m[3 * row + 0], m[3 * row + 1], m[3 * row + 2]);
}

__host__ __device__ inline DeviceVec3 mat3_mul_vec(const float m[9], const DeviceVec3& v) {
    return make_vec3(
        m[0] * v.x + m[1] * v.y + m[2] * v.z,
        m[3] * v.x + m[4] * v.y + m[5] * v.z,
        m[6] * v.x + m[7] * v.y + m[8] * v.z
    );
}

__host__ __device__ inline DeviceVec3 quat_rotate(const DeviceQuat& q, const DeviceVec3& v) {
    // q * [0,v] * q^{-1}
    DeviceVec3 qv = make_vec3(q.x, q.y, q.z);
    DeviceVec3 t = mul(cross(qv, v), 2.0f);
    return add(v, add(mul(t, q.w), cross(qv, t)));
}

__host__ __device__ inline DeviceVec3 quat_small_angle_diff_vec(const DeviceQuat& q,
                                                                const DeviceQuat& q0) {
    // Approximate small-angle difference between q and q0.
    // Convert to full quaternions and reuse the same formula as CPU, but in
    // practice for small steps this is close enough for AVBD debug purposes.
    // Note: we do not normalize here to keep math simple in the kernel.
    // dq = q * q0^{-1}
    float inv_norm_sq = 1.0f / (q0.w * q0.w + q0.x * q0.x + q0.y * q0.y + q0.z * q0.z);
    DeviceQuat q0_inv{
        q0.w * inv_norm_sq,
        -q0.x * inv_norm_sq,
        -q0.y * inv_norm_sq,
        -q0.z * inv_norm_sq
    };
    DeviceQuat dq;
    dq.w = q.w * q0_inv.w - q.x * q0_inv.x - q.y * q0_inv.y - q.z * q0_inv.z;
    dq.x = q.w * q0_inv.x + q.x * q0_inv.w + q.y * q0_inv.z - q.z * q0_inv.y;
    dq.y = q.w * q0_inv.y - q.x * q0_inv.z + q.y * q0_inv.w + q.z * q0_inv.x;
    dq.z = q.w * q0_inv.z + q.x * q0_inv.y - q.y * q0_inv.x + q.z * q0_inv.w;
    if (dq.w < 0.0f) {
        dq.w = -dq.w;
        dq.x = -dq.x;
        dq.y = -dq.y;
        dq.z = -dq.z;
    }
    return make_vec3(2.0f * dq.x, 2.0f * dq.y, 2.0f * dq.z);
}

// Skew-symmetric matrix for cross product: skew(v)*u = cross(v,u). Row-major 3x3.
__host__ __device__ inline void skew_matrix(const DeviceVec3& v, float M[9]) {
    M[0] = 0.0f;   M[1] = -v.z; M[2] = v.y;
    M[3] = v.z;   M[4] = 0.0f;  M[5] = -v.x;
    M[6] = -v.y;  M[7] = v.x;   M[8] = 0.0f;
}

__host__ __device__ inline DeviceVec3 world_point_device(const DeviceVec3* positions,
                                                        const DeviceQuat* rotations,
                                                        int body,
                                                        int nBodies,
                                                        const DeviceVec3& r_local_or_world) {
    if (body < 0 || body >= nBodies) return r_local_or_world;
    return add(positions[body], quat_rotate(rotations[body], r_local_or_world));
}

// Quaternion multiply: qa * qb (for combining transforms).
__host__ __device__ inline DeviceQuat quat_mul(const DeviceQuat& a, const DeviceQuat& b) {
    DeviceQuat r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

__host__ __device__ inline DeviceVec3 quat_transform_point(const DeviceQuat& q, const DeviceVec3& p) {
    return quat_rotate(q, p);
}

__host__ __device__ inline DeviceQuat quat_conjugate(const DeviceQuat& q) {
    return DeviceQuat{q.w, -q.x, -q.y, -q.z};
}

__host__ __device__ inline DeviceVec3 normalize_vec3(const DeviceVec3& a) {
    float L = length(a);
    if (L < 1e-12f) return a;
    return mul(a, 1.0f / L);
}

// Geometric stiffness for ball-socket (demo3d): column k is -v[k]*e_k + v.
__host__ __device__ inline void geometric_stiffness_ball_socket(int k, const DeviceVec3& v, float H[9]) {
    for (int i = 0; i < 9; ++i) H[i] = 0.0f;
    H[0] = -v.x; H[4] = -v.y; H[8] = -v.z;
    if (k == 0) { H[0] += v.x; H[3] += v.y; H[6] += v.z; }
    else if (k == 1) { H[1] += v.x; H[4] += v.y; H[7] += v.z; }
    else { H[2] += v.x; H[5] += v.y; H[8] += v.z; }
}

// Diagonalize: diagonal matrix with column norms of 3x3 stored row-major in m.
__host__ __device__ inline void diagonalize_device(const float m[9], float d[9]) {
    float c0 = sqrtf(m[0] * m[0] + m[3] * m[3] + m[6] * m[6]);
    float c1 = sqrtf(m[1] * m[1] + m[4] * m[4] + m[7] * m[7]);
    float c2 = sqrtf(m[2] * m[2] + m[5] * m[5] + m[8] * m[8]);
    d[0] = c0; d[1] = 0.0f; d[2] = 0.0f;
    d[3] = 0.0f; d[4] = c1; d[5] = 0.0f;
    d[6] = 0.0f; d[7] = 0.0f; d[8] = c2;
}

__host__ __device__ inline float penalty_norm_sq(const float p[3]) {
    return p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
}

// Host-side helpers mirroring the CPU implementation in vbd_solver.cpp.
// These are used by VbdSolver::step_cuda() when running on the host.
inline Vec3f world_point_host(const SimState& state, int body, const Vec3f& r_local_or_world) {
    if (body < 0) return r_local_or_world;
    return state.transforms[body].transform_point(r_local_or_world);
}

inline Vec3f quat_diff_vec_demo3d_host(const Quatf& a, const Quatf& b) {
    Quatf dq = a * b.inverse();
    dq.normalize();
    return 2.0f * Vec3f(dq.x(), dq.y(), dq.z());
}

inline Quatf quat_add_omega_dt_host(const Quatf& q, const Vec3f& omega, float dt) {
    if (dt <= 0.0f) return q;
    const Quatf omega_q(0.0f, omega.x(), omega.y(), omega.z());
    Quatf q_new = q;
    q_new.coeffs() += (0.5f * dt) * (omega_q * q).coeffs();
    q_new.normalize();
    return q_new;
}

inline Vec3f angular_velocity_from_quat_diff_host(const Quatf& q_now, const Quatf& q_prev, float dt) {
    if (dt <= 0.0f) return Vec3f::Zero();
    Quatf dq = q_now * q_prev.inverse();
    dq.normalize();
    if (dq.w() < 0.0f) dq.coeffs() *= -1.0f;
    Vec3f v(dq.x(), dq.y(), dq.z());
    return (2.0f * v) / dt;
}

// Small dense 6x6 linear solver using Gaussian elimination with partial pivoting.
// Solves A * x = b in-place. On numerical failure, x is set to zero.
__device__ void solve6x6(float A[36], float b[6], float x[6]) {
    // Augment A with b to form [A|b] of size 6x7.
    float M[6][7];
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {
            M[r][c] = A[r * 6 + c];
        }
        M[r][6] = b[r];
    }

    // Forward elimination
    for (int k = 0; k < 6; ++k) {
        // Pivot
        int pivot = k;
        float max_val = fabsf(M[k][k]);
        for (int r = k + 1; r < 6; ++r) {
            float v = fabsf(M[r][k]);
            if (v > max_val) {
                max_val = v;
                pivot = r;
            }
        }
        if (max_val < 1e-12f) {
            // Singular or ill-conditioned; return zero correction.
            for (int i = 0; i < 6; ++i) x[i] = 0.0f;
            return;
        }
        if (pivot != k) {
            for (int c = k; c < 7; ++c) {
                float tmp = M[k][c];
                M[k][c] = M[pivot][c];
                M[pivot][c] = tmp;
            }
        }

        float diag = M[k][k];
        for (int c = k; c < 7; ++c) {
            M[k][c] /= diag;
        }

        // Eliminate
        for (int r = k + 1; r < 6; ++r) {
            float factor = M[r][k];
            if (fabsf(factor) < 1e-12f) continue;
            for (int c = k; c < 7; ++c) {
                M[r][c] -= factor * M[k][c];
            }
        }
    }

    // Back substitution
    for (int r = 5; r >= 0; --r) {
        float sum = M[r][6];
        for (int c = r + 1; c < 6; ++c) {
            sum -= M[r][c] * x[c];
        }
        x[r] = sum;  // M[r][r] is 1 after normalization
    }
}

// CUDA kernel for per-body primal update (6x6 AVBD step).
// When vertex coloring is used, only bodies with body_color[bi] == current_color
// perform the update; others return immediately. Same-color bodies run in
// parallel with no data conflict; different colors are launched in sequence.
__global__ void avbd_primal_contacts_kernel(
    int nBodies,
    const DeviceBody* bodies,
    const int* body_color,
    int current_color,
    const DeviceVec3* initial_pos,
    const DeviceQuat* initial_rot,
    const DeviceVec3* inertial_pos,
    const DeviceQuat* inertial_rot,
    DeviceVec3* positions,
    DeviceQuat* rotations,
    const DeviceContact* contacts,
    int nContacts,
    const DeviceJoint* joints,
    int nJoints,
    const DeviceSpring* springs,
    int nSprings,
    float dt,
    float alpha,
    float lhs_regularization,
    float primal_relaxation) {
    int bi = blockIdx.x * blockDim.x + threadIdx.x;
    if (bi >= nBodies) return;
    if (body_color != nullptr && body_color[bi] != current_color) return;

    const DeviceBody& body = bodies[bi];
    if (body.is_static) return;

    const float dt2 = dt * dt;
    if (dt2 < 1e-12f) return;

    DeviceVec3 pos = positions[bi];
    DeviceQuat rot = rotations[bi];
    DeviceVec3 dqLin = sub(pos, initial_pos[bi]);
    DeviceVec3 dqAng = quat_small_angle_diff_vec(rot, initial_rot[bi]);

    // Mass and inertia blocks.
    float MLin = body.mass;
    DeviceVec3 MAng_diag = make_vec3(body.inertia_diag[0], body.inertia_diag[1], body.inertia_diag[2]);

    // 3x3 blocks for LHS and 3x1 RHS vectors.
    float lhsLin[9] = {
        MLin / dt2, 0.0f, 0.0f,
        0.0f, MLin / dt2, 0.0f,
        0.0f, 0.0f, MLin / dt2
    };
    float lhsAng[9] = {
        MAng_diag.x / dt2, 0.0f, 0.0f,
        0.0f, MAng_diag.y / dt2, 0.0f,
        0.0f, 0.0f, MAng_diag.z / dt2
    };
    float lhsCross[9] = {
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f
    };

    DeviceVec3 rhsLin = mul(sub(pos, inertial_pos[bi]), MLin / dt2);
    DeviceVec3 rot_err = quat_small_angle_diff_vec(rot, inertial_rot[bi]);
    DeviceVec3 rhsAng = make_vec3(
        MAng_diag.x / dt2 * rot_err.x,
        MAng_diag.y / dt2 * rot_err.y,
        MAng_diag.z / dt2 * rot_err.z);

    // Accumulate contact constraints affecting body bi.
    for (int ci = 0; ci < nContacts; ++ci) {
        const DeviceContact& ac = contacts[ci];
        bool onA = (ac.body_a == bi);
        bool onB = (ac.body_b == bi);
        if (!onA && !onB) continue;

        int ia = ac.body_a;
        int ib = ac.body_b;

        // World-space contact arms.
        DeviceVec3 rA_w = make_vec3(0.0f, 0.0f, 0.0f);
        DeviceVec3 rB_w = make_vec3(0.0f, 0.0f, 0.0f);
        if (ia >= 0 && ia < nBodies) rA_w = quat_rotate(rotations[ia], ac.rA);
        if (ib >= 0 && ib < nBodies) rB_w = quat_rotate(rotations[ib], ac.rB);

        // Basis rows.
        DeviceVec3 n0 = mat3_row(ac.basis, 0);
        DeviceVec3 t1 = mat3_row(ac.basis, 1);
        DeviceVec3 t2 = mat3_row(ac.basis, 2);

        DeviceVec3 jALin_row0 = n0;
        DeviceVec3 jALin_row1 = t1;
        DeviceVec3 jALin_row2 = t2;
        DeviceVec3 jBLin_row0 = mul(jALin_row0, -1.0f);
        DeviceVec3 jBLin_row1 = mul(jALin_row1, -1.0f);
        DeviceVec3 jBLin_row2 = mul(jALin_row2, -1.0f);

        // dq for both bodies.
        DeviceVec3 dqALin = (ia == bi) ? dqLin : make_vec3(0.0f, 0.0f, 0.0f);
        DeviceVec3 dqBLin = (ib == bi) ? dqLin : make_vec3(0.0f, 0.0f, 0.0f);
        DeviceVec3 dqAAng = (ia == bi) ? dqAng : make_vec3(0.0f, 0.0f, 0.0f);
        DeviceVec3 dqBAng = (ib == bi) ? dqAng : make_vec3(0.0f, 0.0f, 0.0f);
        if (ia != bi && ia >= 0 && ia < nBodies) {
            dqALin = sub(positions[ia], initial_pos[ia]);
            dqAAng = quat_small_angle_diff_vec(rotations[ia], initial_rot[ia]);
        }
        if (ib != bi && ib >= 0 && ib < nBodies) {
            dqBLin = sub(positions[ib], initial_pos[ib]);
            dqBAng = quat_small_angle_diff_vec(rotations[ib], initial_rot[ib]);
        }

        DeviceVec3 C = make_vec3(
            ac.C0[0] * (1.0f - alpha),
            ac.C0[1] * (1.0f - alpha),
            ac.C0[2] * (1.0f - alpha));

        // Linear contribution to C.
        C.x += dot(jALin_row0, dqALin) + dot(jBLin_row0, dqBLin);
        C.y += dot(jALin_row1, dqALin) + dot(jBLin_row1, dqBLin);
        C.z += dot(jALin_row2, dqALin) + dot(jBLin_row2, dqBLin);

        // Angular contribution to C.
        DeviceVec3 jAAng_row0 = cross(rA_w, n0);
        DeviceVec3 jAAng_row1 = cross(rA_w, t1);
        DeviceVec3 jAAng_row2 = cross(rA_w, t2);
        DeviceVec3 jBAng_row0 = mul(cross(rB_w, n0), -1.0f);
        DeviceVec3 jBAng_row1 = mul(cross(rB_w, t1), -1.0f);
        DeviceVec3 jBAng_row2 = mul(cross(rB_w, t2), -1.0f);

        C.x += dot(jAAng_row0, dqAAng) + dot(jBAng_row0, dqBAng);
        C.y += dot(jAAng_row1, dqAAng) + dot(jBAng_row1, dqBAng);
        C.z += dot(jAAng_row2, dqAAng) + dot(jBAng_row2, dqBAng);

        DeviceVec3 Kdiag = make_vec3(ac.penalty[0], ac.penalty[1], ac.penalty[2]);
        DeviceVec3 F = make_vec3(
            Kdiag.x * C.x + ac.lambda[0],
            Kdiag.y * C.y + ac.lambda[1],
            Kdiag.z * C.z + ac.lambda[2]);

        if (F.x > 0.0f) F.x = 0.0f;

        float bounds = fabsf(F.x) * ac.friction;
        float ft_len = sqrtf(F.y * F.y + F.z * F.z);
        if (ft_len > bounds && ft_len > 1e-12f) {
            float scale = bounds / ft_len;
            F.y *= scale;
            F.z *= scale;
        }

        // Choose jacobians for this body.
        DeviceVec3 jLin_row0 = onA ? jALin_row0 : jBLin_row0;
        DeviceVec3 jLin_row1 = onA ? jALin_row1 : jBLin_row1;
        DeviceVec3 jLin_row2 = onA ? jALin_row2 : jBLin_row2;
        DeviceVec3 jAng_row0 = onA ? jAAng_row0 : jBAng_row0;
        DeviceVec3 jAng_row1 = onA ? jAAng_row1 : jBAng_row1;
        DeviceVec3 jAng_row2 = onA ? jAAng_row2 : jBAng_row2;

        // Accumulate J^T K J and J^T F contributions.
        // lhsLin += J_lin^T * K * J_lin
        // lhsAng += J_ang^T * K * J_ang
        // lhsCross += J_ang^T * K * J_lin
        DeviceVec3 kF = Kdiag;

        // Helper lambda to accumulate outer product scaled by K.
        auto accum_outer = [](float M[9], const DeviceVec3& a, const DeviceVec3& b, float k) {
            M[0] += k * a.x * b.x;
            M[1] += k * a.x * b.y;
            M[2] += k * a.x * b.z;
            M[3] += k * a.y * b.x;
            M[4] += k * a.y * b.y;
            M[5] += k * a.y * b.z;
            M[6] += k * a.z * b.x;
            M[7] += k * a.z * b.y;
            M[8] += k * a.z * b.z;
        };

        accum_outer(lhsLin, jLin_row0, jLin_row0, kF.x);
        accum_outer(lhsLin, jLin_row1, jLin_row1, kF.y);
        accum_outer(lhsLin, jLin_row2, jLin_row2, kF.z);

        accum_outer(lhsAng, jAng_row0, jAng_row0, kF.x);
        accum_outer(lhsAng, jAng_row1, jAng_row1, kF.y);
        accum_outer(lhsAng, jAng_row2, jAng_row2, kF.z);

        accum_outer(lhsCross, jAng_row0, jLin_row0, kF.x);
        accum_outer(lhsCross, jAng_row1, jLin_row1, kF.y);
        accum_outer(lhsCross, jAng_row2, jLin_row2, kF.z);

        // RHS contributions: rhsLin += J_lin^T * F, rhsAng += J_ang^T * F.
        rhsLin.x += jLin_row0.x * F.x + jLin_row1.x * F.y + jLin_row2.x * F.z;
        rhsLin.y += jLin_row0.y * F.x + jLin_row1.y * F.y + jLin_row2.y * F.z;
        rhsLin.z += jLin_row0.z * F.x + jLin_row1.z * F.y + jLin_row2.z * F.z;

        rhsAng.x += jAng_row0.x * F.x + jAng_row1.x * F.y + jAng_row2.x * F.z;
        rhsAng.y += jAng_row0.y * F.x + jAng_row1.y * F.y + jAng_row2.y * F.z;
        rhsAng.z += jAng_row0.z * F.x + jAng_row1.z * F.y + jAng_row2.z * F.z;
    }

    // Helper to accumulate outer products (used by joints/springs).
    auto accum_outer = [](float M[9], const DeviceVec3& a, const DeviceVec3& b, float k) {
        M[0] += k * a.x * b.x; M[1] += k * a.x * b.y; M[2] += k * a.x * b.z;
        M[3] += k * a.y * b.x; M[4] += k * a.y * b.y; M[5] += k * a.y * b.z;
        M[6] += k * a.z * b.x; M[7] += k * a.z * b.y; M[8] += k * a.z * b.z;
    };

    // --- Joint constraints ---
    for (int ji = 0; ji < nJoints; ++ji) {
        const DeviceJoint& j = joints[ji];
        if (j.broken) continue;
        bool onA = (j.body_a == bi);
        bool onB = (j.body_b == bi);
        if (!onA && !onB) continue;

        int ia = j.body_a;
        int ib = j.body_b;

        if (penalty_norm_sq(j.penaltyLin) > 1e-18f) {
            DeviceVec3 xA = world_point_device(positions, rotations, ia, nBodies, j.rA);
            DeviceVec3 xB = world_point_device(positions, rotations, ib, nBodies, j.rB);
            DeviceVec3 C = sub(xA, xB);
            const float inf = 1e20f;
            if (j.stiffnessLin >= inf) {
                C.x -= j.C0Lin[0] * alpha;
                C.y -= j.C0Lin[1] * alpha;
                C.z -= j.C0Lin[2] * alpha;
            }
            DeviceVec3 K = make_vec3(j.penaltyLin[0], j.penaltyLin[1], j.penaltyLin[2]);
            DeviceVec3 F = add(mul_componentwise(K, C), make_vec3(j.lambdaLin[0], j.lambdaLin[1], j.lambdaLin[2]));

            DeviceVec3 rA_w = (ia >= 0 && ia < nBodies) ? quat_rotate(rotations[ia], j.rA) : j.rA;
            DeviceVec3 rB_w = (ib >= 0 && ib < nBodies) ? quat_rotate(rotations[ib], j.rB) : j.rB;

            float jAng_rm[9];
            if (onA) skew_matrix(make_vec3(-rA_w.x, -rA_w.y, -rA_w.z), jAng_rm);
            else skew_matrix(rB_w, jAng_rm);

            DeviceVec3 jLin_row0 = onA ? make_vec3(1.0f, 0.0f, 0.0f) : make_vec3(-1.0f, 0.0f, 0.0f);
            DeviceVec3 jLin_row1 = onA ? make_vec3(0.0f, 1.0f, 0.0f) : make_vec3(0.0f, -1.0f, 0.0f);
            DeviceVec3 jLin_row2 = onA ? make_vec3(0.0f, 0.0f, 1.0f) : make_vec3(0.0f, 0.0f, -1.0f);
            DeviceVec3 jAng_row0 = make_vec3(jAng_rm[0], jAng_rm[1], jAng_rm[2]);
            DeviceVec3 jAng_row1 = make_vec3(jAng_rm[3], jAng_rm[4], jAng_rm[5]);
            DeviceVec3 jAng_row2 = make_vec3(jAng_rm[6], jAng_rm[7], jAng_rm[8]);

            accum_outer(lhsLin, jLin_row0, jLin_row0, K.x);
            accum_outer(lhsLin, jLin_row1, jLin_row1, K.y);
            accum_outer(lhsLin, jLin_row2, jLin_row2, K.z);
            accum_outer(lhsAng, jAng_row0, jAng_row0, K.x);
            accum_outer(lhsAng, jAng_row1, jAng_row1, K.y);
            accum_outer(lhsAng, jAng_row2, jAng_row2, K.z);
            accum_outer(lhsCross, jAng_row0, jLin_row0, K.x);
            accum_outer(lhsCross, jAng_row1, jLin_row1, K.y);
            accum_outer(lhsCross, jAng_row2, jLin_row2, K.z);

            DeviceVec3 r = onA ? rA_w : make_vec3(-rB_w.x, -rB_w.y, -rB_w.z);
            float H[9] = {0.0f};
            float G[9];
            geometric_stiffness_ball_socket(0, r, G);
            for (int i = 0; i < 9; ++i) H[i] = G[i] * F.x;
            geometric_stiffness_ball_socket(1, r, G);
            for (int i = 0; i < 9; ++i) H[i] += G[i] * F.y;
            geometric_stiffness_ball_socket(2, r, G);
            for (int i = 0; i < 9; ++i) H[i] += G[i] * F.z;
            float Hdiag[9];
            diagonalize_device(H, Hdiag);
            lhsAng[0] += Hdiag[0];
            lhsAng[4] += Hdiag[4];
            lhsAng[8] += Hdiag[8];

            rhsLin.x += jLin_row0.x * F.x + jLin_row1.x * F.y + jLin_row2.x * F.z;
            rhsLin.y += jLin_row0.y * F.x + jLin_row1.y * F.y + jLin_row2.y * F.z;
            rhsLin.z += jLin_row0.z * F.x + jLin_row1.z * F.y + jLin_row2.z * F.z;
            rhsAng.x += jAng_row0.x * F.x + jAng_row1.x * F.y + jAng_row2.x * F.z;
            rhsAng.y += jAng_row0.y * F.x + jAng_row1.y * F.y + jAng_row2.y * F.z;
            rhsAng.z += jAng_row0.z * F.x + jAng_row1.z * F.y + jAng_row2.z * F.z;
        }

        if (penalty_norm_sq(j.penaltyAng) > 1e-18f) {
            DeviceQuat qA = (ia >= 0 && ia < nBodies) ? rotations[ia] : DeviceQuat{1.0f, 0.0f, 0.0f, 0.0f};
            DeviceQuat qB = (ib >= 0 && ib < nBodies) ? rotations[ib] : DeviceQuat{1.0f, 0.0f, 0.0f, 0.0f};
            DeviceVec3 C = mul(quat_small_angle_diff_vec(qA, qB), j.torqueArm);
            const float inf = 1e20f;
            if (j.stiffnessAng >= inf) {
                C.x -= j.C0Ang[0] * alpha;
                C.y -= j.C0Ang[1] * alpha;
                C.z -= j.C0Ang[2] * alpha;
            }
            DeviceVec3 K = make_vec3(j.penaltyAng[0], j.penaltyAng[1], j.penaltyAng[2]);
            DeviceVec3 F = add(mul_componentwise(K, C), make_vec3(j.lambdaAng[0], j.lambdaAng[1], j.lambdaAng[2]));

            float s = (onA ? 1.0f : -1.0f) * j.torqueArm;
            DeviceVec3 jAng_row0 = make_vec3(s, 0.0f, 0.0f);
            DeviceVec3 jAng_row1 = make_vec3(0.0f, s, 0.0f);
            DeviceVec3 jAng_row2 = make_vec3(0.0f, 0.0f, s);
            accum_outer(lhsAng, jAng_row0, jAng_row0, K.x);
            accum_outer(lhsAng, jAng_row1, jAng_row1, K.y);
            accum_outer(lhsAng, jAng_row2, jAng_row2, K.z);
            rhsAng.x += jAng_row0.x * F.x + jAng_row1.x * F.y + jAng_row2.x * F.z;
            rhsAng.y += jAng_row0.y * F.x + jAng_row1.y * F.y + jAng_row2.y * F.z;
            rhsAng.z += jAng_row0.z * F.x + jAng_row1.z * F.y + jAng_row2.z * F.z;
        }
    }

    // --- Spring forces (linearized stiffness; no dual) ---
    for (int si = 0; si < nSprings; ++si) {
        const DeviceSpring& s = springs[si];
        bool onA = (s.body_a == bi);
        bool onB = (s.body_b == bi);
        if (!onA && !onB) continue;
        if (s.body_a < 0 || s.body_b < 0) continue;

        DeviceVec3 pA = world_point_device(positions, rotations, s.body_a, nBodies, s.rA);
        DeviceVec3 pB = world_point_device(positions, rotations, s.body_b, nBodies, s.rB);
        DeviceVec3 d = sub(pA, pB);
        float dLen = length(d);
        if (dLen <= 1e-6f) continue;
        DeviceVec3 nrm = mul(d, 1.0f / dLen);
        float rest = s.rest;
        if (rest < 0.0f) rest = dLen;
        float C = dLen - rest;
        float f = s.stiffness * C;

        DeviceVec3 rWorld = (onA ? quat_rotate(rotations[s.body_a], s.rA) : quat_rotate(rotations[s.body_b], s.rB));
        DeviceVec3 jLin_v = onA ? nrm : mul(nrm, -1.0f);
        DeviceVec3 jAng_v = onA ? cross(rWorld, nrm) : mul(cross(rWorld, nrm), -1.0f);

        DeviceVec3 F_lin = mul(jLin_v, f);
        DeviceVec3 Tau = mul(jAng_v, f);
        float k = s.stiffness;
        accum_outer(lhsLin, jLin_v, jLin_v, k);
        accum_outer(lhsAng, jAng_v, jAng_v, k);
        accum_outer(lhsCross, jAng_v, jLin_v, k);
        rhsLin.x += F_lin.x; rhsLin.y += F_lin.y; rhsLin.z += F_lin.z;
        rhsAng.x += Tau.x; rhsAng.y += Tau.y; rhsAng.z += Tau.z;
    }

    // Assemble 6x6 system LHS * dq = RHS (with RHS = -rhs).
    float LHS[36] = {0.0f};
    // Top-left 3x3: lhsLin
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            LHS[r * 6 + c] = lhsLin[r * 3 + c];
    // Top-right 3x3: lhsCross^T
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            LHS[r * 6 + (3 + c)] = lhsCross[c * 3 + r];
    // Bottom-left 3x3: lhsCross
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            LHS[(3 + r) * 6 + c] = lhsCross[r * 3 + c];
    // Bottom-right 3x3: lhsAng
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            LHS[(3 + r) * 6 + (3 + c)] = lhsAng[r * 3 + c];

    // Regularization.
    float reg = lhs_regularization > 0.0f ? lhs_regularization : 0.0f;
    if (reg > 0.0f) {
        LHS[0] += reg;  LHS[7] += reg;  LHS[14] += reg;
        LHS[21] += reg; LHS[28] += reg; LHS[35] += reg;
    }

    float RHS6[6] = {
        -rhsLin.x, -rhsLin.y, -rhsLin.z,
        -rhsAng.x, -rhsAng.y, -rhsAng.z
    };
    float dq6[6];
    solve6x6(LHS, RHS6, dq6);

    // Clamp relaxation factor into a reasonable range.
    float relax = primal_relaxation;
    if (relax < 0.01f) relax = 0.01f;
    if (relax > 1.0f) relax = 1.0f;
    DeviceVec3 dxLin = make_vec3(dq6[0] * relax, dq6[1] * relax, dq6[2] * relax);
    DeviceVec3 dxAng = make_vec3(dq6[3] * relax, dq6[4] * relax, dq6[5] * relax);

    // Update position; for rotation we approximate small-angle update by
    // rotating by dxAng in world space as if it were an angular velocity
    // over a unit time step.
    positions[bi] = add(positions[bi], dxLin);
    // First-order quaternion update: q_new = q + 0.5 * [0, dxAng] * q.
    DeviceQuat q = rotations[bi];
    DeviceQuat omega_q{0.0f, dxAng.x, dxAng.y, dxAng.z};
    DeviceQuat dq;
    dq.w = 0.5f * (omega_q.w * q.w - omega_q.x * q.x - omega_q.y * q.y - omega_q.z * q.z);
    dq.x = 0.5f * (omega_q.w * q.x + omega_q.x * q.w + omega_q.y * q.z - omega_q.z * q.y);
    dq.y = 0.5f * (omega_q.w * q.y - omega_q.x * q.z + omega_q.y * q.w + omega_q.z * q.x);
    dq.z = 0.5f * (omega_q.w * q.z + omega_q.x * q.y - omega_q.y * q.x + omega_q.z * q.w);
    q.w += dq.w;
    q.x += dq.x;
    q.y += dq.y;
    q.z += dq.z;
    float inv_len = rsqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q.w *= inv_len; q.x *= inv_len; q.y *= inv_len; q.z *= inv_len;
    rotations[bi] = q;
}

// Same primal logic but runs all colors in one launch with grid sync to cut kernel
// launch overhead (critical for 100k+ bodies: num_colors can be 50+, iterations 10
// -> 500+ launches/step otherwise).
__global__ void avbd_primal_all_colors_kernel(
    int nBodies,
    const DeviceBody* bodies,
    const int* body_color,
    int num_colors,
    const DeviceVec3* initial_pos,
    const DeviceQuat* initial_rot,
    const DeviceVec3* inertial_pos,
    const DeviceQuat* inertial_rot,
    DeviceVec3* positions,
    DeviceQuat* rotations,
    const DeviceContact* contacts,
    int nContacts,
    const DeviceJoint* joints,
    int nJoints,
    const DeviceSpring* springs,
    int nSprings,
    float dt,
    float alpha,
    float lhs_regularization,
    float primal_relaxation) {
    cg::grid_group grid = cg::this_grid();
    int bi = blockIdx.x * blockDim.x + threadIdx.x;

    for (int current_color = 0; current_color < num_colors; ++current_color) {
        if (bi >= nBodies) { grid.sync(); continue; }
        if (body_color != nullptr && body_color[bi] != current_color) { grid.sync(); continue; }
        const DeviceBody& body = bodies[bi];
        if (body.is_static) { grid.sync(); continue; }

        const float dt2 = dt * dt;
        if (dt2 < 1e-12f) { grid.sync(); continue; }

        DeviceVec3 pos = positions[bi];
        DeviceQuat rot = rotations[bi];
        DeviceVec3 dqLin = sub(pos, initial_pos[bi]);
        DeviceVec3 dqAng = quat_small_angle_diff_vec(rot, initial_rot[bi]);

        float MLin = body.mass;
        DeviceVec3 MAng_diag = make_vec3(body.inertia_diag[0], body.inertia_diag[1], body.inertia_diag[2]);
        float lhsLin[9] = { MLin / dt2, 0.0f, 0.0f, 0.0f, MLin / dt2, 0.0f, 0.0f, 0.0f, MLin / dt2 };
        float lhsAng[9] = { MAng_diag.x / dt2, 0.0f, 0.0f, 0.0f, MAng_diag.y / dt2, 0.0f, 0.0f, 0.0f, MAng_diag.z / dt2 };
        float lhsCross[9] = {0};

        DeviceVec3 rhsLin = mul(sub(pos, inertial_pos[bi]), MLin / dt2);
        DeviceVec3 rot_err = quat_small_angle_diff_vec(rot, inertial_rot[bi]);
        DeviceVec3 rhsAng = make_vec3(MAng_diag.x / dt2 * rot_err.x, MAng_diag.y / dt2 * rot_err.y, MAng_diag.z / dt2 * rot_err.z);

        for (int ci = 0; ci < nContacts; ++ci) {
            const DeviceContact& ac = contacts[ci];
            bool onA = (ac.body_a == bi);
            bool onB = (ac.body_b == bi);
            if (!onA && !onB) continue;
            int ia = ac.body_a, ib = ac.body_b;
            DeviceVec3 rA_w = (ia >= 0 && ia < nBodies) ? quat_rotate(rotations[ia], ac.rA) : make_vec3(0,0,0);
            DeviceVec3 rB_w = (ib >= 0 && ib < nBodies) ? quat_rotate(rotations[ib], ac.rB) : make_vec3(0,0,0);
            DeviceVec3 n0 = mat3_row(ac.basis, 0), t1 = mat3_row(ac.basis, 1), t2 = mat3_row(ac.basis, 2);
            DeviceVec3 jALin_row0 = n0, jALin_row1 = t1, jALin_row2 = t2;
            DeviceVec3 jBLin_row0 = mul(n0,-1.0f), jBLin_row1 = mul(t1,-1.0f), jBLin_row2 = mul(t2,-1.0f);
            DeviceVec3 dqALin = (ia == bi) ? dqLin : make_vec3(0,0,0);
            DeviceVec3 dqBLin = (ib == bi) ? dqLin : make_vec3(0,0,0);
            DeviceVec3 dqAAng = (ia == bi) ? dqAng : make_vec3(0,0,0);
            DeviceVec3 dqBAng = (ib == bi) ? dqAng : make_vec3(0,0,0);
            if (ia != bi && ia >= 0 && ia < nBodies) { dqALin = sub(positions[ia], initial_pos[ia]); dqAAng = quat_small_angle_diff_vec(rotations[ia], initial_rot[ia]); }
            if (ib != bi && ib >= 0 && ib < nBodies) { dqBLin = sub(positions[ib], initial_pos[ib]); dqBAng = quat_small_angle_diff_vec(rotations[ib], initial_rot[ib]); }
            DeviceVec3 C = make_vec3(ac.C0[0]*(1.0f-alpha), ac.C0[1]*(1.0f-alpha), ac.C0[2]*(1.0f-alpha));
            C.x += dot(jALin_row0,dqALin)+dot(jBLin_row0,dqBLin); C.y += dot(jALin_row1,dqALin)+dot(jBLin_row1,dqBLin); C.z += dot(jALin_row2,dqALin)+dot(jBLin_row2,dqBLin);
            DeviceVec3 jAAng_row0 = cross(rA_w,n0), jAAng_row1 = cross(rA_w,t1), jAAng_row2 = cross(rA_w,t2);
            DeviceVec3 jBAng_row0 = mul(cross(rB_w,n0),-1.0f), jBAng_row1 = mul(cross(rB_w,t1),-1.0f), jBAng_row2 = mul(cross(rB_w,t2),-1.0f);
            C.x += dot(jAAng_row0,dqAAng)+dot(jBAng_row0,dqBAng); C.y += dot(jAAng_row1,dqAAng)+dot(jBAng_row1,dqBAng); C.z += dot(jAAng_row2,dqAAng)+dot(jBAng_row2,dqBAng);
            DeviceVec3 Kdiag = make_vec3(ac.penalty[0],ac.penalty[1],ac.penalty[2]);
            DeviceVec3 F = make_vec3(Kdiag.x*C.x+ac.lambda[0], Kdiag.y*C.y+ac.lambda[1], Kdiag.z*C.z+ac.lambda[2]);
            if (F.x > 0.0f) F.x = 0.0f;
            float bounds = fabsf(F.x)*ac.friction;
            float ft_len = sqrtf(F.y*F.y+F.z*F.z);
            if (ft_len > bounds && ft_len > 1e-12f) { float scale = bounds/ft_len; F.y*=scale; F.z*=scale; }
            DeviceVec3 jLin_row0 = onA ? jALin_row0 : jBLin_row0, jLin_row1 = onA ? jALin_row1 : jBLin_row1, jLin_row2 = onA ? jALin_row2 : jBLin_row2;
            DeviceVec3 jAng_row0 = onA ? jAAng_row0 : jBAng_row0, jAng_row1 = onA ? jAAng_row1 : jBAng_row1, jAng_row2 = onA ? jAAng_row2 : jBAng_row2;
            auto accum = [](float M[9], const DeviceVec3& a, const DeviceVec3& b, float k) {
                M[0]+=k*a.x*b.x; M[1]+=k*a.x*b.y; M[2]+=k*a.x*b.z; M[3]+=k*a.y*b.x; M[4]+=k*a.y*b.y; M[5]+=k*a.y*b.z; M[6]+=k*a.z*b.x; M[7]+=k*a.z*b.y; M[8]+=k*a.z*b.z;
            };
            accum(lhsLin,jLin_row0,jLin_row0,Kdiag.x); accum(lhsLin,jLin_row1,jLin_row1,Kdiag.y); accum(lhsLin,jLin_row2,jLin_row2,Kdiag.z);
            accum(lhsAng,jAng_row0,jAng_row0,Kdiag.x); accum(lhsAng,jAng_row1,jAng_row1,Kdiag.y); accum(lhsAng,jAng_row2,jAng_row2,Kdiag.z);
            accum(lhsCross,jAng_row0,jLin_row0,Kdiag.x); accum(lhsCross,jAng_row1,jLin_row1,Kdiag.y); accum(lhsCross,jAng_row2,jLin_row2,Kdiag.z);
            rhsLin.x += jLin_row0.x*F.x+jLin_row1.x*F.y+jLin_row2.x*F.z; rhsLin.y += jLin_row0.y*F.x+jLin_row1.y*F.y+jLin_row2.y*F.z; rhsLin.z += jLin_row0.z*F.x+jLin_row1.z*F.y+jLin_row2.z*F.z;
            rhsAng.x += jAng_row0.x*F.x+jAng_row1.x*F.y+jAng_row2.x*F.z; rhsAng.y += jAng_row0.y*F.x+jAng_row1.y*F.y+jAng_row2.y*F.z; rhsAng.z += jAng_row0.z*F.x+jAng_row1.z*F.y+jAng_row2.z*F.z;
        }

        auto accum_outer = [](float M[9], const DeviceVec3& a, const DeviceVec3& b, float k) {
            M[0]+=k*a.x*b.x; M[1]+=k*a.x*b.y; M[2]+=k*a.x*b.z; M[3]+=k*a.y*b.x; M[4]+=k*a.y*b.y; M[5]+=k*a.y*b.z; M[6]+=k*a.z*b.x; M[7]+=k*a.z*b.y; M[8]+=k*a.z*b.z;
        };
        for (int ji = 0; ji < nJoints; ++ji) {
            const DeviceJoint& j = joints[ji];
            if (j.broken) continue;
            bool onA = (j.body_a == bi), onB = (j.body_b == bi);
            if (!onA && !onB) continue;
            int ia = j.body_a, ib = j.body_b;
            if (penalty_norm_sq(j.penaltyLin) > 1e-18f) {
                DeviceVec3 xA = world_point_device(positions, rotations, ia, nBodies, j.rA);
                DeviceVec3 xB = world_point_device(positions, rotations, ib, nBodies, j.rB);
                DeviceVec3 C = sub(xA, xB);
                if (j.stiffnessLin >= 1e20f) { C.x -= j.C0Lin[0]*alpha; C.y -= j.C0Lin[1]*alpha; C.z -= j.C0Lin[2]*alpha; }
                DeviceVec3 K = make_vec3(j.penaltyLin[0],j.penaltyLin[1],j.penaltyLin[2]);
                DeviceVec3 F = add(mul_componentwise(K,C), make_vec3(j.lambdaLin[0],j.lambdaLin[1],j.lambdaLin[2]));
                DeviceVec3 rA_w = (ia>=0&&ia<nBodies) ? quat_rotate(rotations[ia],j.rA) : j.rA;
                DeviceVec3 rB_w = (ib>=0&&ib<nBodies) ? quat_rotate(rotations[ib],j.rB) : j.rB;
                float jAng_rm[9];
                if (onA) skew_matrix(make_vec3(-rA_w.x,-rA_w.y,-rA_w.z), jAng_rm);
                else skew_matrix(rB_w, jAng_rm);
                DeviceVec3 jLin_row0 = onA ? make_vec3(1,0,0) : make_vec3(-1,0,0), jLin_row1 = onA ? make_vec3(0,1,0) : make_vec3(0,-1,0), jLin_row2 = onA ? make_vec3(0,0,1) : make_vec3(0,0,-1);
                DeviceVec3 jAng_row0 = make_vec3(jAng_rm[0],jAng_rm[1],jAng_rm[2]), jAng_row1 = make_vec3(jAng_rm[3],jAng_rm[4],jAng_rm[5]), jAng_row2 = make_vec3(jAng_rm[6],jAng_rm[7],jAng_rm[8]);
                accum_outer(lhsLin,jLin_row0,jLin_row0,K.x); accum_outer(lhsLin,jLin_row1,jLin_row1,K.y); accum_outer(lhsLin,jLin_row2,jLin_row2,K.z);
                accum_outer(lhsAng,jAng_row0,jAng_row0,K.x); accum_outer(lhsAng,jAng_row1,jAng_row1,K.y); accum_outer(lhsAng,jAng_row2,jAng_row2,K.z);
                accum_outer(lhsCross,jAng_row0,jLin_row0,K.x); accum_outer(lhsCross,jAng_row1,jLin_row1,K.y); accum_outer(lhsCross,jAng_row2,jLin_row2,K.z);
                DeviceVec3 r = onA ? rA_w : make_vec3(-rB_w.x,-rB_w.y,-rB_w.z);
                float H[9]={0}, G[9];
                geometric_stiffness_ball_socket(0,r,G); for(int i=0;i<9;i++) H[i]=G[i]*F.x;
                geometric_stiffness_ball_socket(1,r,G); for(int i=0;i<9;i++) H[i]+=G[i]*F.y;
                geometric_stiffness_ball_socket(2,r,G); for(int i=0;i<9;i++) H[i]+=G[i]*F.z;
                float Hdiag[9]; diagonalize_device(H, Hdiag);
                lhsAng[0]+=Hdiag[0]; lhsAng[4]+=Hdiag[4]; lhsAng[8]+=Hdiag[8];
                rhsLin.x += jLin_row0.x*F.x+jLin_row1.x*F.y+jLin_row2.x*F.z; rhsLin.y += jLin_row0.y*F.x+jLin_row1.y*F.y+jLin_row2.y*F.z; rhsLin.z += jLin_row0.z*F.x+jLin_row1.z*F.y+jLin_row2.z*F.z;
                rhsAng.x += jAng_row0.x*F.x+jAng_row1.x*F.y+jAng_row2.x*F.z; rhsAng.y += jAng_row0.y*F.x+jAng_row1.y*F.y+jAng_row2.y*F.z; rhsAng.z += jAng_row0.z*F.x+jAng_row1.z*F.y+jAng_row2.z*F.z;
            }
            if (penalty_norm_sq(j.penaltyAng) > 1e-18f) {
                DeviceQuat qA = (ia>=0&&ia<nBodies) ? rotations[ia] : DeviceQuat{1,0,0,0};
                DeviceQuat qB = (ib>=0&&ib<nBodies) ? rotations[ib] : DeviceQuat{1,0,0,0};
                DeviceVec3 C = mul(quat_small_angle_diff_vec(qA,qB), j.torqueArm);
                if (j.stiffnessAng >= 1e20f) { C.x -= j.C0Ang[0]*alpha; C.y -= j.C0Ang[1]*alpha; C.z -= j.C0Ang[2]*alpha; }
                DeviceVec3 K = make_vec3(j.penaltyAng[0],j.penaltyAng[1],j.penaltyAng[2]);
                DeviceVec3 F = add(mul_componentwise(K,C), make_vec3(j.lambdaAng[0],j.lambdaAng[1],j.lambdaAng[2]));
                float s = (onA ? 1.0f : -1.0f) * j.torqueArm;
                DeviceVec3 jAng_row0 = make_vec3(s,0,0), jAng_row1 = make_vec3(0,s,0), jAng_row2 = make_vec3(0,0,s);
                accum_outer(lhsAng,jAng_row0,jAng_row0,K.x); accum_outer(lhsAng,jAng_row1,jAng_row1,K.y); accum_outer(lhsAng,jAng_row2,jAng_row2,K.z);
                rhsAng.x += jAng_row0.x*F.x+jAng_row1.x*F.y+jAng_row2.x*F.z; rhsAng.y += jAng_row0.y*F.x+jAng_row1.y*F.y+jAng_row2.y*F.z; rhsAng.z += jAng_row0.z*F.x+jAng_row1.z*F.y+jAng_row2.z*F.z;
            }
        }
        for (int si = 0; si < nSprings; ++si) {
            const DeviceSpring& s = springs[si];
            bool onA = (s.body_a == bi), onB = (s.body_b == bi);
            if (!onA && !onB) continue;
            if (s.body_a < 0 || s.body_b < 0) continue;
            DeviceVec3 pA = world_point_device(positions, rotations, s.body_a, nBodies, s.rA);
            DeviceVec3 pB = world_point_device(positions, rotations, s.body_b, nBodies, s.rB);
            DeviceVec3 d = sub(pA,pB);
            float dLen = length(d);
            if (dLen <= 1e-6f) continue;
            DeviceVec3 nrm = mul(d, 1.0f/dLen);
            float rest = s.rest; if (rest < 0.0f) rest = dLen;
            float C = dLen - rest;
            float f = s.stiffness * C;
            DeviceVec3 rWorld = onA ? quat_rotate(rotations[s.body_a],s.rA) : quat_rotate(rotations[s.body_b],s.rB);
            DeviceVec3 jLin_v = onA ? nrm : mul(nrm,-1.0f);
            DeviceVec3 jAng_v = onA ? cross(rWorld,nrm) : mul(cross(rWorld,nrm),-1.0f);
            accum_outer(lhsLin,jLin_v,jLin_v,s.stiffness);
            accum_outer(lhsAng,jAng_v,jAng_v,s.stiffness);
            accum_outer(lhsCross,jAng_v,jLin_v,s.stiffness);
            rhsLin.x += jLin_v.x*f; rhsLin.y += jLin_v.y*f; rhsLin.z += jLin_v.z*f;
            rhsAng.x += jAng_v.x*f; rhsAng.y += jAng_v.y*f; rhsAng.z += jAng_v.z*f;
        }

        float LHS[36] = {0};
        for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) LHS[r*6+c] = lhsLin[r*3+c];
        for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) LHS[r*6+(3+c)] = lhsCross[c*3+r];
        for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) LHS[(3+r)*6+c] = lhsCross[r*3+c];
        for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) LHS[(3+r)*6+(3+c)] = lhsAng[r*3+c];
        if (lhs_regularization > 0.0f) { float reg = lhs_regularization; LHS[0]+=reg; LHS[7]+=reg; LHS[14]+=reg; LHS[21]+=reg; LHS[28]+=reg; LHS[35]+=reg; }
        float RHS6[6] = { -rhsLin.x, -rhsLin.y, -rhsLin.z, -rhsAng.x, -rhsAng.y, -rhsAng.z };
        float dq6[6];
        solve6x6(LHS, RHS6, dq6);
        float relax = (primal_relaxation < 0.01f) ? 0.01f : ((primal_relaxation > 1.0f) ? 1.0f : primal_relaxation);
        DeviceVec3 dxLin = make_vec3(dq6[0]*relax, dq6[1]*relax, dq6[2]*relax);
        DeviceVec3 dxAng = make_vec3(dq6[3]*relax, dq6[4]*relax, dq6[5]*relax);
        positions[bi] = add(positions[bi], dxLin);
        DeviceQuat q = rotations[bi];
        DeviceQuat omega_q{0.0f, dxAng.x, dxAng.y, dxAng.z};
        DeviceQuat dq;
        dq.w = 0.5f*(omega_q.w*q.w - omega_q.x*q.x - omega_q.y*q.y - omega_q.z*q.z);
        dq.x = 0.5f*(omega_q.w*q.x + omega_q.x*q.w + omega_q.y*q.z - omega_q.z*q.y);
        dq.y = 0.5f*(omega_q.w*q.y - omega_q.x*q.z + omega_q.y*q.w + omega_q.z*q.x);
        dq.z = 0.5f*(omega_q.w*q.z + omega_q.x*q.y - omega_q.y*q.x + omega_q.z*q.w);
        q.w += dq.w; q.x += dq.x; q.y += dq.y; q.z += dq.z;
        float inv_len = rsqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
        q.w*=inv_len; q.x*=inv_len; q.y*=inv_len; q.z*=inv_len;
        rotations[bi] = q;

        grid.sync();
    }
}

// CUDA kernel for per-contact dual update. This mirrors the CPU implementation
// in VbdSolver::avbd_dual for contacts, but runs each contact in parallel.
__global__ void avbd_dual_contacts_kernel(
    int nContacts,
    DeviceContact* contacts,
    const DeviceVec3* initial_pos,
    const DeviceQuat* initial_rot,
    const DeviceVec3* positions,
    const DeviceQuat* rotations,
    const DeviceBody* bodies,
    int nBodies,
    float alpha,
    float beta_linear) {
    int ci = blockIdx.x * blockDim.x + threadIdx.x;
    if (ci >= nContacts) return;

    DeviceContact& ac = contacts[ci];

    int ia = ac.body_a;
    int ib = ac.body_b;

    bool dynA = (ia >= 0 && ia < nBodies) && (bodies[ia].is_static == 0);
    bool dynB = (ib >= 0 && ib < nBodies) && (bodies[ib].is_static == 0);
    if (!dynA && !dynB) return;

    DeviceVec3 dqALin = dynA ? sub(positions[ia], initial_pos[ia]) : make_vec3(0.0f, 0.0f, 0.0f);
    DeviceVec3 dqBLin = dynB ? sub(positions[ib], initial_pos[ib]) : make_vec3(0.0f, 0.0f, 0.0f);

    DeviceVec3 dqAAng = make_vec3(0.0f, 0.0f, 0.0f);
    DeviceVec3 dqBAng = make_vec3(0.0f, 0.0f, 0.0f);
    if (dynA) {
        dqAAng = quat_small_angle_diff_vec(rotations[ia], initial_rot[ia]);
    }
    if (dynB) {
        dqBAng = quat_small_angle_diff_vec(rotations[ib], initial_rot[ib]);
    }

    DeviceVec3 rA_w = make_vec3(0.0f, 0.0f, 0.0f);
    DeviceVec3 rB_w = make_vec3(0.0f, 0.0f, 0.0f);
    if (ia >= 0 && ia < nBodies) {
        rA_w = quat_rotate(rotations[ia], ac.rA);
    }
    if (ib >= 0 && ib < nBodies) {
        rB_w = quat_rotate(rotations[ib], ac.rB);
    }

    // jALin = basis, jBLin = -basis
    // Build C = C0*(1-alpha) + J*dq (both linear and angular parts).
    DeviceVec3 C = make_vec3(
        ac.C0[0] * (1.0f - alpha),
        ac.C0[1] * (1.0f - alpha),
        ac.C0[2] * (1.0f - alpha));

    // Linear contribution
    DeviceVec3 jALin_row0 = mat3_row(ac.basis, 0);
    DeviceVec3 jALin_row1 = mat3_row(ac.basis, 1);
    DeviceVec3 jALin_row2 = mat3_row(ac.basis, 2);
    DeviceVec3 jBLin_row0 = mul(jALin_row0, -1.0f);
    DeviceVec3 jBLin_row1 = mul(jALin_row1, -1.0f);
    DeviceVec3 jBLin_row2 = mul(jALin_row2, -1.0f);

    C.x += dot(jALin_row0, dqALin) + dot(jBLin_row0, dqBLin);
    C.y += dot(jALin_row1, dqALin) + dot(jBLin_row1, dqBLin);
    C.z += dot(jALin_row2, dqALin) + dot(jBLin_row2, dqBLin);

    // Angular contribution
    DeviceVec3 jAAng_row0 = cross(rA_w, jALin_row0);
    DeviceVec3 jAAng_row1 = cross(rA_w, jALin_row1);
    DeviceVec3 jAAng_row2 = cross(rA_w, jALin_row2);
    DeviceVec3 jBAng_row0 = mul(cross(rB_w, jALin_row0), -1.0f);
    DeviceVec3 jBAng_row1 = mul(cross(rB_w, jALin_row1), -1.0f);
    DeviceVec3 jBAng_row2 = mul(cross(rB_w, jALin_row2), -1.0f);

    C.x += dot(jAAng_row0, dqAAng) + dot(jBAng_row0, dqBAng);
    C.y += dot(jAAng_row1, dqAAng) + dot(jBAng_row1, dqBAng);
    C.z += dot(jAAng_row2, dqAAng) + dot(jBAng_row2, dqBAng);

    // Penalty and lambda update (contact only; joints are handled on CPU).
    DeviceVec3 Kdiag = make_vec3(ac.penalty[0], ac.penalty[1], ac.penalty[2]);
    DeviceVec3 F = make_vec3(
        Kdiag.x * C.x + ac.lambda[0],
        Kdiag.y * C.y + ac.lambda[1],
        Kdiag.z * C.z + ac.lambda[2]);

    // Normal force must be non-positive (supporting contact).
    if (F.x > 0.0f) F.x = 0.0f;

    float bounds = fabsf(F.x) * ac.friction;
    float ft_len = sqrtf(F.y * F.y + F.z * F.z);
    if (ft_len > bounds && ft_len > 1e-12f) {
        float scale = bounds / ft_len;
        F.y *= scale;
        F.z *= scale;
    }

    ac.lambda[0] = F.x;
    ac.lambda[1] = F.y;
    ac.lambda[2] = F.z;

    if (F.x < 0.0f) {
        ac.penalty[0] = fminf(ac.penalty[0] + beta_linear * fabsf(C.x), PENALTY_MAX);
    }
    if (ft_len <= bounds) {
        ac.penalty[1] = fminf(ac.penalty[1] + beta_linear * fabsf(C.y), PENALTY_MAX);
        ac.penalty[2] = fminf(ac.penalty[2] + beta_linear * fabsf(C.z), PENALTY_MAX);
        float tang_norm = sqrtf(C.y * C.y + C.z * C.z);
        ac.stick = (tang_norm < STICK_THRESH) ? 1 : 0;
    }
}

// Per-body dual kernel: each thread is a body and updates only contacts where
// this body is the "owner" (body_a, or body_b when body_a is static). This
// gives a deterministic update order by body index, closer to the HLSL
// strategy and CPU Gauss–Seidel ordering, reducing trajectory divergence.
__global__ void avbd_dual_contacts_per_body_kernel(
    int nBodies,
    const DeviceBody* bodies,
    int nContacts,
    DeviceContact* contacts,
    const DeviceVec3* initial_pos,
    const DeviceQuat* initial_rot,
    const DeviceVec3* positions,
    const DeviceQuat* rotations,
    float alpha,
    float beta_linear) {
    int bi = blockIdx.x * blockDim.x + threadIdx.x;
    if (bi >= nBodies) return;

    for (int ci = 0; ci < nContacts; ++ci) {
        DeviceContact& ac = contacts[ci];
        int ia = ac.body_a;
        int ib = ac.body_b;
        // Assign each contact to exactly one thread: body_a if valid, else body_b.
        int owner = (ia >= 0 && ia < nBodies) ? ia : ib;
        if (owner != bi) continue;

        bool dynA = (ia >= 0 && ia < nBodies) && (bodies[ia].is_static == 0);
        bool dynB = (ib >= 0 && ib < nBodies) && (bodies[ib].is_static == 0);
        if (!dynA && !dynB) continue;

        DeviceVec3 dqALin = dynA ? sub(positions[ia], initial_pos[ia]) : make_vec3(0.0f, 0.0f, 0.0f);
        DeviceVec3 dqBLin = dynB ? sub(positions[ib], initial_pos[ib]) : make_vec3(0.0f, 0.0f, 0.0f);
        DeviceVec3 dqAAng = make_vec3(0.0f, 0.0f, 0.0f);
        DeviceVec3 dqBAng = make_vec3(0.0f, 0.0f, 0.0f);
        if (dynA) dqAAng = quat_small_angle_diff_vec(rotations[ia], initial_rot[ia]);
        if (dynB) dqBAng = quat_small_angle_diff_vec(rotations[ib], initial_rot[ib]);

        DeviceVec3 rA_w = (ia >= 0 && ia < nBodies) ? quat_rotate(rotations[ia], ac.rA) : make_vec3(0.0f, 0.0f, 0.0f);
        DeviceVec3 rB_w = (ib >= 0 && ib < nBodies) ? quat_rotate(rotations[ib], ac.rB) : make_vec3(0.0f, 0.0f, 0.0f);

        DeviceVec3 C = make_vec3(
            ac.C0[0] * (1.0f - alpha),
            ac.C0[1] * (1.0f - alpha),
            ac.C0[2] * (1.0f - alpha));

        DeviceVec3 jALin_row0 = mat3_row(ac.basis, 0);
        DeviceVec3 jALin_row1 = mat3_row(ac.basis, 1);
        DeviceVec3 jALin_row2 = mat3_row(ac.basis, 2);
        DeviceVec3 jBLin_row0 = mul(jALin_row0, -1.0f);
        DeviceVec3 jBLin_row1 = mul(jALin_row1, -1.0f);
        DeviceVec3 jBLin_row2 = mul(jALin_row2, -1.0f);

        C.x += dot(jALin_row0, dqALin) + dot(jBLin_row0, dqBLin);
        C.y += dot(jALin_row1, dqALin) + dot(jBLin_row1, dqBLin);
        C.z += dot(jALin_row2, dqALin) + dot(jBLin_row2, dqBLin);

        DeviceVec3 jAAng_row0 = cross(rA_w, jALin_row0);
        DeviceVec3 jAAng_row1 = cross(rA_w, jALin_row1);
        DeviceVec3 jAAng_row2 = cross(rA_w, jALin_row2);
        DeviceVec3 jBAng_row0 = mul(cross(rB_w, jALin_row0), -1.0f);
        DeviceVec3 jBAng_row1 = mul(cross(rB_w, jALin_row1), -1.0f);
        DeviceVec3 jBAng_row2 = mul(cross(rB_w, jALin_row2), -1.0f);

        C.x += dot(jAAng_row0, dqAAng) + dot(jBAng_row0, dqBAng);
        C.y += dot(jAAng_row1, dqAAng) + dot(jBAng_row1, dqBAng);
        C.z += dot(jAAng_row2, dqAAng) + dot(jBAng_row2, dqBAng);

        DeviceVec3 Kdiag = make_vec3(ac.penalty[0], ac.penalty[1], ac.penalty[2]);
        DeviceVec3 F = make_vec3(
            Kdiag.x * C.x + ac.lambda[0],
            Kdiag.y * C.y + ac.lambda[1],
            Kdiag.z * C.z + ac.lambda[2]);

        if (F.x > 0.0f) F.x = 0.0f;

        float bounds = fabsf(F.x) * ac.friction;
        float ft_len = sqrtf(F.y * F.y + F.z * F.z);
        if (ft_len > bounds && ft_len > 1e-12f) {
            float scale = bounds / ft_len;
            F.y *= scale;
            F.z *= scale;
        }

        ac.lambda[0] = F.x;
        ac.lambda[1] = F.y;
        ac.lambda[2] = F.z;

        if (F.x < 0.0f) {
            ac.penalty[0] = fminf(ac.penalty[0] + beta_linear * fabsf(C.x), PENALTY_MAX);
        }
        if (ft_len <= bounds) {
            ac.penalty[1] = fminf(ac.penalty[1] + beta_linear * fabsf(C.y), PENALTY_MAX);
            ac.penalty[2] = fminf(ac.penalty[2] + beta_linear * fabsf(C.z), PENALTY_MAX);
            float tang_norm = sqrtf(C.y * C.y + C.z * C.z);
            ac.stick = (tang_norm < STICK_THRESH) ? 1 : 0;
        }
    }
}

// Per-joint dual update: lambda and penalty (demo3d Joint::updateDual).
__global__ void avbd_dual_joints_kernel(
    int nJoints,
    DeviceJoint* joints,
    const DeviceVec3* positions,
    const DeviceQuat* rotations,
    const DeviceBody* bodies,
    int nBodies,
    float alpha,
    float beta_linear,
    float beta_angular) {
    int ji = blockIdx.x * blockDim.x + threadIdx.x;
    if (ji >= nJoints) return;

    DeviceJoint& j = joints[ji];
    if (j.broken) return;

    int ia = j.body_a;
    int ib = j.body_b;
    bool dynA = (ia >= 0 && ia < nBodies) && (bodies[ia].is_static == 0);
    bool dynB = (ib >= 0 && ib < nBodies) && (bodies[ib].is_static == 0);
    if (!dynA && !dynB) return;

    const float inf = 1e20f;

    if (penalty_norm_sq(j.penaltyLin) > 1e-18f) {
        DeviceVec3 xA = world_point_device(positions, rotations, ia, nBodies, j.rA);
        DeviceVec3 xB = world_point_device(positions, rotations, ib, nBodies, j.rB);
        DeviceVec3 C = sub(xA, xB);
        if (j.stiffnessLin >= inf) {
            C.x -= j.C0Lin[0] * alpha;
            C.y -= j.C0Lin[1] * alpha;
            C.z -= j.C0Lin[2] * alpha;
            DeviceVec3 K = make_vec3(j.penaltyLin[0], j.penaltyLin[1], j.penaltyLin[2]);
            DeviceVec3 F = add(mul_componentwise(K, C), make_vec3(j.lambdaLin[0], j.lambdaLin[1], j.lambdaLin[2]));
            j.lambdaLin[0] = F.x;
            j.lambdaLin[1] = F.y;
            j.lambdaLin[2] = F.z;
        }
        float absC0 = fabsf(C.x);
        float absC1 = fabsf(C.y);
        float absC2 = fabsf(C.z);
        float stiff = fminf(j.stiffnessLin, PENALTY_MAX);
        j.penaltyLin[0] = fminf(j.penaltyLin[0] + beta_linear * absC0, stiff);
        j.penaltyLin[1] = fminf(j.penaltyLin[1] + beta_linear * absC1, stiff);
        j.penaltyLin[2] = fminf(j.penaltyLin[2] + beta_linear * absC2, stiff);
    }

    if (penalty_norm_sq(j.penaltyAng) > 1e-18f) {
        DeviceQuat qA = (ia >= 0 && ia < nBodies) ? rotations[ia] : DeviceQuat{1.0f, 0.0f, 0.0f, 0.0f};
        DeviceQuat qB = (ib >= 0 && ib < nBodies) ? rotations[ib] : DeviceQuat{1.0f, 0.0f, 0.0f, 0.0f};
        DeviceVec3 C = mul(quat_small_angle_diff_vec(qA, qB), j.torqueArm);
        if (j.stiffnessAng >= inf) {
            C.x -= j.C0Ang[0] * alpha;
            C.y -= j.C0Ang[1] * alpha;
            C.z -= j.C0Ang[2] * alpha;
            DeviceVec3 K = make_vec3(j.penaltyAng[0], j.penaltyAng[1], j.penaltyAng[2]);
            DeviceVec3 F = add(mul_componentwise(K, C), make_vec3(j.lambdaAng[0], j.lambdaAng[1], j.lambdaAng[2]));
            j.lambdaAng[0] = F.x;
            j.lambdaAng[1] = F.y;
            j.lambdaAng[2] = F.z;
        }
        float absC0 = fabsf(C.x);
        float absC1 = fabsf(C.y);
        float absC2 = fabsf(C.z);
        float stiff = fminf(j.stiffnessAng, PENALTY_MAX);
        j.penaltyAng[0] = fminf(j.penaltyAng[0] + beta_angular * absC0, stiff);
        j.penaltyAng[1] = fminf(j.penaltyAng[1] + beta_angular * absC1, stiff);
        j.penaltyAng[2] = fminf(j.penaltyAng[2] + beta_angular * absC2, stiff);
    }

    float lamSq = j.lambdaAng[0] * j.lambdaAng[0] + j.lambdaAng[1] * j.lambdaAng[1] + j.lambdaAng[2] * j.lambdaAng[2];
    if (j.fracture > 0.0f && lamSq > j.fracture * j.fracture) {
        j.penaltyLin[0] = j.penaltyLin[1] = j.penaltyLin[2] = 0.0f;
        j.penaltyAng[0] = j.penaltyAng[1] = j.penaltyAng[2] = 0.0f;
        j.lambdaLin[0] = j.lambdaLin[1] = j.lambdaLin[2] = 0.0f;
        j.lambdaAng[0] = j.lambdaAng[1] = j.lambdaAng[2] = 0.0f;
        j.broken = 1;
    }
}

// -----------------------------------------------------------------------------
// GPU collision: AABB and broadphase (all-pairs). Narrowphase stays on host.
// -----------------------------------------------------------------------------
__global__ void compute_shape_aabbs_kernel(
    const DeviceShape* __restrict__ shapes,
    const DeviceVec3* __restrict__ positions,
    const DeviceQuat* __restrict__ rotations,
    int num_shapes,
    int nBodies,
    DeviceAABB* __restrict__ aabbs) {
    int si = blockIdx.x * blockDim.x + threadIdx.x;
    if (si >= num_shapes) return;

    const DeviceShape& s = shapes[si];
    if (s.type == 1) {
        // Plane: use large AABB so it overlaps everything
        aabbs[si].minx = aabbs[si].miny = aabbs[si].minz = -1e6f;
        aabbs[si].maxx = aabbs[si].maxy = aabbs[si].maxz = 1e6f;
        return;
    }
    if (s.type == 2) {
        // Sphere: world center = body*local; AABB = center +/- radius.
        DeviceVec3 world_pos;
        if (s.body_index >= 0 && s.body_index < nBodies) {
            DeviceVec3 local_p = make_vec3(s.local_pos[0], s.local_pos[1], s.local_pos[2]);
            world_pos = add(positions[s.body_index], quat_rotate(rotations[s.body_index], local_p));
        } else {
            world_pos = make_vec3(s.local_pos[0], s.local_pos[1], s.local_pos[2]);
        }
        float r = s.radius;
        aabbs[si].minx = world_pos.x - r;
        aabbs[si].miny = world_pos.y - r;
        aabbs[si].minz = world_pos.z - r;
        aabbs[si].maxx = world_pos.x + r;
        aabbs[si].maxy = world_pos.y + r;
        aabbs[si].maxz = world_pos.z + r;
        return;
    }

    // Box: world transform = body * local
    DeviceVec3 world_pos;
    DeviceQuat world_rot;
    if (s.body_index >= 0 && s.body_index < nBodies) {
        DeviceVec3 local_p = make_vec3(s.local_pos[0], s.local_pos[1], s.local_pos[2]);
        DeviceQuat local_q = {s.local_quat[0], s.local_quat[1], s.local_quat[2], s.local_quat[3]};
        world_pos = add(positions[s.body_index], quat_rotate(rotations[s.body_index], local_p));
        world_rot = quat_mul(rotations[s.body_index], local_q);
    } else {
        world_pos = make_vec3(s.local_pos[0], s.local_pos[1], s.local_pos[2]);
        world_rot = {s.local_quat[0], s.local_quat[1], s.local_quat[2], s.local_quat[3]};
    }

    DeviceVec3 half = make_vec3(s.half[0], s.half[1], s.half[2]);
    float minx = 1e9f, miny = 1e9f, minz = 1e9f;
    float maxx = -1e9f, maxy = -1e9f, maxz = -1e9f;
    for (int i = 0; i < 8; ++i) {
        DeviceVec3 corner = make_vec3(
            (i & 1) ? half.x : -half.x,
            (i & 2) ? half.y : -half.y,
            (i & 4) ? half.z : -half.z);
        DeviceVec3 w = add(world_pos, quat_rotate(world_rot, corner));
        if (w.x < minx) minx = w.x;
        if (w.y < miny) miny = w.y;
        if (w.z < minz) minz = w.z;
        if (w.x > maxx) maxx = w.x;
        if (w.y > maxy) maxy = w.y;
        if (w.z > maxz) maxz = w.z;
    }
    aabbs[si].minx = minx;
    aabbs[si].miny = miny;
    aabbs[si].minz = minz;
    aabbs[si].maxx = maxx;
    aabbs[si].maxy = maxy;
    aabbs[si].maxz = maxz;
}

__global__ void broadphase_pairs_kernel(
    const DeviceAABB* __restrict__ aabbs,
    const int* __restrict__ shape_static,
    int num_shapes,
    int* __restrict__ pair_count,
    int2* __restrict__ pairs) {
    // One thread per (i, j) with i < j
    int total_pairs = num_shapes * (num_shapes - 1) / 2;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= total_pairs) return;

    // Map linear index to (i, j): 0->(0,1), 1->(0,2), 2->(1,2), ...
    int i = 0, j = 0;
    int rem = idx;
    for (int ii = 0; ii < num_shapes; ++ii) {
        int row_size = num_shapes - 1 - ii;
        if (rem < row_size) {
            i = ii;
            j = ii + 1 + rem;
            break;
        }
        rem -= row_size;
    }

    if (shape_static[i] && shape_static[j]) return;

    const DeviceAABB& ai = aabbs[i];
    const DeviceAABB& aj = aabbs[j];
    if (ai.maxx < aj.minx || ai.minx > aj.maxx) return;
    if (ai.maxy < aj.miny || ai.miny > aj.maxy) return;
    if (ai.maxz < aj.minz || ai.minz > aj.maxz) return;

    int slot = atomicAdd(pair_count, 1);
    pairs[slot] = make_int2(i, j);
}

// -----------------------------------------------------------------------------
// GPU Broadphase (spatial hash / uniform grid)
// - Insert each shape AABB into all overlapped grid cells (no misses).
// - Sort (cell_key, shape_id) by cell_key.
// - For each cell run, emit all unique shape pairs within that cell.
// Planes are handled separately (paired with all non-plane shapes).
// -----------------------------------------------------------------------------
__host__ __device__ inline uint64_t hash_u64(uint64_t x) {
    // SplitMix64
    x += 0x9e3779b97f4a7c15ull;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
    return x ^ (x >> 31);
}

__host__ __device__ inline uint64_t cell_key_hash(int ix, int iy, int iz) {
    // Hash 3 ints into 64-bit key (collisions possible but acceptable: produces superset pairs).
    uint64_t x = static_cast<uint64_t>(static_cast<uint32_t>(ix));
    uint64_t y = static_cast<uint64_t>(static_cast<uint32_t>(iy));
    uint64_t z = static_cast<uint64_t>(static_cast<uint32_t>(iz));
    uint64_t h = 0x243f6a8885a308d3ull;
    h ^= hash_u64(x + 0x9e3779b97f4a7c15ull);
    h ^= hash_u64(y + 0xbf58476d1ce4e5b9ull);
    h ^= hash_u64(z + 0x94d049bb133111ebull);
    return hash_u64(h);
}

__device__ inline int fast_floor_div(float x, float inv_cell) {
    return static_cast<int>(floorf(x * inv_cell));
}

__global__ void count_cell_entries_kernel(
    const DeviceAABB* __restrict__ aabbs,
    const DeviceShape* __restrict__ shapes,
    int num_shapes,
    float inv_cell_size,
    int* __restrict__ out_counts) {
    int si = blockIdx.x * blockDim.x + threadIdx.x;
    if (si >= num_shapes) return;
    const DeviceShape& s = shapes[si];
    if (s.type == 1) { out_counts[si] = 0; return; }  // plane handled separately
    const DeviceAABB& a = aabbs[si];
    int ix0 = fast_floor_div(a.minx, inv_cell_size);
    int iy0 = fast_floor_div(a.miny, inv_cell_size);
    int iz0 = fast_floor_div(a.minz, inv_cell_size);
    int ix1 = fast_floor_div(a.maxx, inv_cell_size);
    int iy1 = fast_floor_div(a.maxy, inv_cell_size);
    int iz1 = fast_floor_div(a.maxz, inv_cell_size);
    int nx = ix1 - ix0 + 1;
    int ny = iy1 - iy0 + 1;
    int nz = iz1 - iz0 + 1;
    int cnt = nx * ny * nz;
    out_counts[si] = cnt > 0 ? cnt : 0;
}

__global__ void fill_cell_entries_kernel(
    const DeviceAABB* __restrict__ aabbs,
    const DeviceShape* __restrict__ shapes,
    int num_shapes,
    float inv_cell_size,
    const int* __restrict__ offsets,
    uint64_t* __restrict__ keys_out,
    int* __restrict__ shape_out) {
    int si = blockIdx.x * blockDim.x + threadIdx.x;
    if (si >= num_shapes) return;
    const DeviceShape& s = shapes[si];
    if (s.type == 1) return;
    const DeviceAABB& a = aabbs[si];
    int ix0 = fast_floor_div(a.minx, inv_cell_size);
    int iy0 = fast_floor_div(a.miny, inv_cell_size);
    int iz0 = fast_floor_div(a.minz, inv_cell_size);
    int ix1 = fast_floor_div(a.maxx, inv_cell_size);
    int iy1 = fast_floor_div(a.maxy, inv_cell_size);
    int iz1 = fast_floor_div(a.maxz, inv_cell_size);
    int base = offsets[si];
    int idx = 0;
    for (int iz = iz0; iz <= iz1; ++iz) {
        for (int iy = iy0; iy <= iy1; ++iy) {
            for (int ix = ix0; ix <= ix1; ++ix) {
                int out = base + idx++;
                keys_out[out] = cell_key_hash(ix, iy, iz);
                shape_out[out] = si;
            }
        }
    }
}

__global__ void emit_pairs_from_cells_kernel(
    const uint64_t* __restrict__ sorted_keys,
    const int* __restrict__ sorted_shapes,
    int n_entries,
    const int* __restrict__ shape_static,
    int* __restrict__ pair_count,
    int2* __restrict__ pairs,
    int max_pairs) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_entries) return;
    // Only segment starts emit.
    if (i > 0 && sorted_keys[i] == sorted_keys[i - 1]) return;
    // Find segment end.
    int j = i + 1;
    while (j < n_entries && sorted_keys[j] == sorted_keys[i]) ++j;
    // Emit all pairs within [i, j)
    for (int a = i; a < j; ++a) {
        int sa = sorted_shapes[a];
        for (int b = a + 1; b < j; ++b) {
            int sb = sorted_shapes[b];
            if (sa == sb) continue;
            if (shape_static[sa] && shape_static[sb]) continue;
            int lo = sa < sb ? sa : sb;
            int hi = sa < sb ? sb : sa;
            int slot = atomicAdd(pair_count, 1);
            if (slot < max_pairs) pairs[slot] = make_int2(lo, hi);
        }
    }
}

// -----------------------------------------------------------------------------
// GPU Narrowphase: box-box (SAT/manifold) and box-plane on device.
// Output: raw contacts (body_a, body_b, rA, rB, basis, friction, feature_id)
// for host to apply warmstart + C0 + sort.
// -----------------------------------------------------------------------------
struct RawContact {
    int body_a;
    int body_b;
    float rA[3];
    float rB[3];
    float basis[9];
    float friction;
    int feature_id;
};

// RawContact plus warmstart data (lambda, penalty, stick) filled by GPU warmstart lookup.
// Used for step 2: D2H and CPU does C0 + sort; or as intermediate for step 3.
struct RawContactWarmstart {
    RawContact base;
    float lambda[3];
    float penalty[3];
    int stick;
};

enum { AXIS_FACE_A = 0, AXIS_FACE_B = 1, AXIS_EDGE = 2 };
constexpr int MAX_CONTACTS = 8;
constexpr int MAX_POLY_VERTS = 16;
constexpr float SAT_AXIS_EPSILON = 1.0e-6f;
constexpr float PLANE_EPSILON = 1.0e-5f;
constexpr float CONTACT_MERGE_DIST_SQ = 1.0e-6f;

struct DeviceOBB {
    DeviceVec3 center;
    DeviceQuat rotation;
    DeviceVec3 half;
    DeviceVec3 axis[3];
};

// -----------------------------------------------------------------------------
// TODO（后续工作）：在 GPU 上完成 RawContact 的 key 生成 + 排序 / 去重 / 压缩
//
// Goal: move the \"dedup + sort\" part of host-side
// build_contact_constraints_from_raw_contacts onto the GPU, and only copy a
// compacted set of contacts back to the host. This comment block documents
// data structures and intended flow; parts of it are now implemented below.
//
// 一、数据结构（device 端）
// ---------------------------------------------------------------------------
// 1. RawContact 已存在：
//    struct RawContact {
//        int body_a, body_b;
//        float rA[3], rB[3];
//        float basis[9];
//        float friction;
//        int feature_id;
//    };
//
// 2. 新增 key 数组（与 RawContact 一一对应）：
//    - uint64_t* d_contact_keys;        // size = n_raw
//    - RawContact* d_raw_contacts;      // size = n_raw（已有）
//
// 3. 排序/去重用的缓冲：
//    - uint64_t* d_contact_keys_sorted;    // size = n_raw
//    - RawContact* d_raw_sorted;          // size = n_raw
//    - int* d_unique_indices;             // size = n_raw（可选，用于标记 first-of-key）
//    - cuda / CUB 临时缓冲 d_sort_temp。
//
// 二、key 设计（与 CPU 端 contact_key 对齐） / Key design (aligned with CPU)
// ---------------------------------------------------------------------------
// We implement the same FNV-1a based hash as the CPU-side contact_key() to keep
// behavior as close as possible while allowing GPU sorting:
//
__device__ __host__ inline uint64_t fnv1a_u64_gpu(uint64_t h, uint64_t v) {
    constexpr uint64_t kPrime = 1099511628211ull;
    h ^= v;
    h *= kPrime;
    return h;
}

__device__ __host__ inline uint64_t make_contact_key_gpu(int body_a, int body_b, int feature_id) {
    int lo = body_a < body_b ? body_a : body_b;
    int hi = body_a < body_b ? body_b : body_a;
    uint64_t h = 1469598103934665603ull;
    h = fnv1a_u64_gpu(h, static_cast<uint32_t>(lo + 2));
    h = fnv1a_u64_gpu(h, static_cast<uint32_t>(hi + 2));
    h = fnv1a_u64_gpu(h, static_cast<uint32_t>(feature_id));
    return h;
}

__device__ __host__ inline uint64_t make_pair_key_gpu(int a, int b) {
    int lo = a < b ? a : b;
    int hi = a < b ? b : a;
    uint64_t h = 1469598103934665603ull;
    h = fnv1a_u64_gpu(h, static_cast<uint32_t>(lo + 2));
    h = fnv1a_u64_gpu(h, static_cast<uint32_t>(hi + 2));
    return h;
}
//
// 三、GPU 排序 / 去重流程（以及下方实际实现）
// ---------------------------------------------------------------------------
// 假设 narrowphase_kernel 之后，我们有：
//   - int n_raw;                   // raw_contacts 个数
//   - RawContact* d_raw_contacts;  // size >= n_raw
//
// 1) 生成 key 数组：
//
__global__ void build_contact_keys_kernel(const RawContact* rc, int n, uint64_t* keys) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    const RawContact& c = rc[i];
    keys[i] = make_contact_key_gpu(c.body_a, c.body_b, c.feature_id);
}

__global__ void build_contact_keys_from_device_kernel(const DeviceContact* contacts, int n, uint64_t* keys) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    const DeviceContact& c = contacts[i];
    keys[i] = make_contact_key_gpu(c.body_a, c.body_b, c.feature_id);
}

__global__ void fill_indices_kernel(int* indices, int n) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    indices[i] = i;
}

__global__ void gather_contacts_kernel(const DeviceContact* src, const int* indices, int n, DeviceContact* dst) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    dst[i] = src[indices[i]];
}
//
// 2) 使用 CUB 做按 key 的稳定排序：
//
//   size_t sort_bytes = 0;
//   cub::DeviceRadixSort::SortPairs(
//       nullptr, sort_bytes,
//       d_contact_keys, d_contact_keys_sorted,
//       d_raw_contacts, d_raw_sorted,
//       n_raw);
//   // 分配 d_sort_temp（在 ensure_cuda_buffers 里已完成）
//   cub::DeviceRadixSort::SortPairs(
//       d_sort_temp, sort_bytes,
//       d_contact_keys, d_contact_keys_sorted,
//       d_raw_contacts, d_raw_sorted,
//       n_raw);
//
//   // 之后：d_contact_keys_sorted / d_raw_sorted 是按 key 排序好的结果。
//
// 3) 在 GPU 上做 unique / 压缩：
//
//   - 思路 A：用 `cub::DeviceSelect::Unique` 对 key 做 unique，得到：
//       * unique keys（每个 key 只保留第一条）
//       * mapping 数组（或者直接压缩 RawContact）
//   - 思路 B：写一个简单 kernel，扫描 sorted keys，标记 first-of-run：
//
__global__ void mark_unique_contact_flags_kernel(const uint64_t* keys, int n, int* flags) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    if (i == 0) {
        flags[i] = 1;
    } else {
        flags[i] = (keys[i] != keys[i - 1]) ? 1 : 0;
    }
}

__global__ void compact_sorted_contacts_kernel(
    const RawContact* sorted_contacts,
    const int* unique_flags,
    const int* unique_indices,
    int n,
    RawContact* out_compact) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    if (!unique_flags[i]) return;
    int dst = unique_indices[i];
    out_compact[dst] = sorted_contacts[i];
}

// Binary search: returns index j where keys[j] == key, or -1 if not found.
__device__ int binary_search_contact_key(const uint64_t* keys_sorted, int n, uint64_t key) {
    int lo = 0;
    int hi = n;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (keys_sorted[mid] < key)
            lo = mid + 1;
        else
            hi = mid;
    }
    if (lo < n && keys_sorted[lo] == key)
        return lo;
    return -1;
}

// Mark valid (1) or ignored (0) for each raw warmstart contact using ignore pair keys.
__global__ void filter_ignore_flags_kernel(
    const RawContactWarmstart* raw_warmstart,
    int n,
    const uint64_t* ignore_pair_keys,
    int n_ignore,
    int* valid_flags) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    const RawContact& r = raw_warmstart[i].base;
    uint64_t pk = make_pair_key_gpu(r.body_a, r.body_b);
    int valid = 1;
    for (int j = 0; j < n_ignore; ++j) {
        if (ignore_pair_keys[j] == pk) {
            valid = 0;
            break;
        }
    }
    valid_flags[i] = valid;
}

// Scatter valid RawContactWarmstart to compact buffer.
__global__ void compact_warmstart_kernel(
    const RawContactWarmstart* raw_warmstart,
    const int* valid_flags,
    const int* valid_indices,
    int n,
    RawContactWarmstart* out_compact) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n || !valid_flags[i]) return;
    int dst = valid_indices[i];
    out_compact[dst] = raw_warmstart[i];
}

// Build DeviceContact from RawContactWarmstart: compute C0 = basis*(xA-xB)+margin, apply alpha*gamma to lambda/penalty.
__global__ void build_contact_c0_kernel(
    const RawContactWarmstart* raw_warmstart,
    int n,
    const DeviceVec3* positions,
    const DeviceQuat* rotations,
    int n_bodies,
    float alpha,
    float gamma,
    float margin,
    DeviceContact* contacts) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    const RawContactWarmstart& rw = raw_warmstart[i];
    const RawContact& r = rw.base;
    DeviceContact& c = contacts[i];
    c.body_a = r.body_a;
    c.body_b = r.body_b;
    c.rA.x = r.rA[0]; c.rA.y = r.rA[1]; c.rA.z = r.rA[2];
    c.rB.x = r.rB[0]; c.rB.y = r.rB[1]; c.rB.z = r.rB[2];
    for (int k = 0; k < 9; ++k) c.basis[k] = r.basis[k];
    c.friction = r.friction;
    c.feature_id = r.feature_id;
    c.lambda[0] = rw.lambda[0]; c.lambda[1] = rw.lambda[1]; c.lambda[2] = rw.lambda[2];
    c.penalty[0] = rw.penalty[0]; c.penalty[1] = rw.penalty[1]; c.penalty[2] = rw.penalty[2];
    c.stick = rw.stick;

    DeviceVec3 xA = c.rA, xB = c.rB;
    if (r.body_a >= 0 && r.body_a < n_bodies) {
        xA = add(positions[r.body_a], quat_rotate(rotations[r.body_a], c.rA));
    }
    if (r.body_b >= 0 && r.body_b < n_bodies) {
        xB = add(positions[r.body_b], quat_rotate(rotations[r.body_b], c.rB));
    }
    DeviceVec3 dx = sub(xA, xB);
    c.C0[0] = (c.basis[0] * dx.x + c.basis[1] * dx.y + c.basis[2] * dx.z) + margin;
    c.C0[1] = (c.basis[3] * dx.x + c.basis[4] * dx.y + c.basis[5] * dx.z);
    c.C0[2] = (c.basis[6] * dx.x + c.basis[7] * dx.y + c.basis[8] * dx.z);

    c.lambda[0] *= alpha * gamma;
    c.lambda[1] *= alpha * gamma;
    c.lambda[2] *= alpha * gamma;
    float p0 = c.penalty[0] * gamma;
    float p1 = c.penalty[1] * gamma;
    float p2 = c.penalty[2] * gamma;
    c.penalty[0] = p0 < PENALTY_MIN ? PENALTY_MIN : (p0 > PENALTY_MAX ? PENALTY_MAX : p0);
    c.penalty[1] = p1 < PENALTY_MIN ? PENALTY_MIN : (p1 > PENALTY_MAX ? PENALTY_MAX : p1);
    c.penalty[2] = p2 < PENALTY_MIN ? PENALTY_MIN : (p2 > PENALTY_MAX ? PENALTY_MAX : p2);
}

// Fill warmstart data (lambda, penalty, stick, rA/rB when stick) from prev frame by key lookup.
__global__ void warmstart_lookup_kernel(
    const RawContact* raw_contacts,
    int n_raw,
    const uint64_t* prev_keys_sorted,
    const DeviceContact* prev_contacts_sorted,
    int n_prev,
    RawContactWarmstart* out_warmstart) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_raw) return;
    const RawContact& r = raw_contacts[i];
    RawContactWarmstart& w = out_warmstart[i];
    w.base = r;
    w.lambda[0] = 0.f;
    w.lambda[1] = 0.f;
    w.lambda[2] = 0.f;
    w.penalty[0] = PENALTY_MIN;
    w.penalty[1] = PENALTY_MIN;
    w.penalty[2] = PENALTY_MIN;
    w.stick = 0;
    if (n_prev <= 0) return;
    uint64_t key = make_contact_key_gpu(r.body_a, r.body_b, r.feature_id);
    int j = binary_search_contact_key(prev_keys_sorted, n_prev, key);
    if (j >= 0) {
        const DeviceContact& p = prev_contacts_sorted[j];
        w.lambda[0] = p.lambda[0]; w.lambda[1] = p.lambda[1]; w.lambda[2] = p.lambda[2];
        w.penalty[0] = p.penalty[0]; w.penalty[1] = p.penalty[1]; w.penalty[2] = p.penalty[2];
        w.stick = p.stick;
        if (p.stick) {
            w.base.rA[0] = p.rA.x; w.base.rA[1] = p.rA.y; w.base.rA[2] = p.rA.z;
            w.base.rB[0] = p.rB.x; w.base.rB[1] = p.rB.y; w.base.rB[2] = p.rB.z;
        }
    }
}

//   - 压缩完成后，得到：
//       * int n_compact;               // unique contacts 数量
//       * RawContact* d_raw_compact;   // size >= n_compact
//
// 4) 回传 CPU 只需要压缩后的 contacts：
//
//   - 现在 D2H 可以改为：
//       cudaMemcpy(h_raw, d_raw_compact, n_compact*sizeof(RawContact), D2H);
//       build_contact_constraints_from_raw_contacts(..., span(h_raw, n_compact));
//
//   - 这样 host 侧 hash / 去重 + C0 计算的工作量显著下降，
//     为下一步“连 warmstart + C0 + 排序一并搬到 GPU”打基础。
//
// 四、后续扩展（warmstart & 全 GPU build）
// ---------------------------------------------------------------------------
// - 上一步压缩后保留的 key（或由压缩 kernel 顺便写出）可以用来在 GPU 维护
//   上一帧的接触表，实现完全在设备端的 warmstart。那时 host 只读最终的
//   DeviceContact（或只在 debug/headless 对比时才读）。
//
// 目前先保留这些说明，后续真正实现时只需要沿着这里的数据结构和流程往下写即可。
// -----------------------------------------------------------------------------

__device__ DeviceOBB makeOBB_device(const DeviceVec3& pos, const DeviceQuat& q, const DeviceVec3& half) {
    DeviceOBB box;
    box.center = pos;
    box.rotation = q;
    box.half = half;
    box.axis[0] = quat_rotate(q, make_vec3(1.f, 0.f, 0.f));
    box.axis[1] = quat_rotate(q, make_vec3(0.f, 1.f, 0.f));
    box.axis[2] = quat_rotate(q, make_vec3(0.f, 0.f, 1.f));
    return box;
}

__device__ float absDot_device(const DeviceVec3& a, const DeviceVec3& b) {
    float d = dot(a, b);
    return d >= 0.f ? d : -d;
}

__device__ DeviceVec3 supportPoint_device(const DeviceOBB& box, const DeviceVec3& dir) {
    float sx = dot(dir, box.axis[0]) >= 0.f ? 1.f : -1.f;
    float sy = dot(dir, box.axis[1]) >= 0.f ? 1.f : -1.f;
    float sz = dot(dir, box.axis[2]) >= 0.f ? 1.f : -1.f;
    return add(box.center, add(mul(box.axis[0], box.half.x * sx),
            add(mul(box.axis[1], box.half.y * sy), mul(box.axis[2], box.half.z * sz))));
}

__device__ void getFaceAxes_device(const DeviceOBB& box, int axisIndex,
    DeviceVec3& u, DeviceVec3& v, float& extentU, float& extentV) {
    if (axisIndex == 0) {
        u = box.axis[1]; v = box.axis[2];
        extentU = box.half.y; extentV = box.half.z;
    } else if (axisIndex == 1) {
        u = box.axis[0]; v = box.axis[2];
        extentU = box.half.x; extentV = box.half.z;
    } else {
        u = box.axis[0]; v = box.axis[1];
        extentU = box.half.x; extentV = box.half.y;
    }
}

struct FaceFrameDevice {
    int axisIndex;
    DeviceVec3 normal, center, u, v;
    float extentU, extentV;
};

__device__ void buildFaceFrame_device(const DeviceOBB& box, int axisIndex,
    const DeviceVec3& outwardNormal, FaceFrameDevice& frame) {
    float s = dot(outwardNormal, box.axis[axisIndex]) >= 0.f ? 1.f : -1.f;
    frame.axisIndex = axisIndex;
    frame.normal = mul(box.axis[axisIndex], s);
    frame.center = add(box.center, mul(frame.normal, (axisIndex == 0 ? box.half.x : axisIndex == 1 ? box.half.y : box.half.z)));
    getFaceAxes_device(box, axisIndex, frame.u, frame.v, frame.extentU, frame.extentV);
}

__device__ int chooseIncidentFaceAxis_device(const DeviceOBB& box, const DeviceVec3& referenceNormal) {
    int axis = 0;
    float best = -1e30f;
    for (int i = 0; i < 3; ++i) {
        float d = absDot_device(box.axis[i], referenceNormal);
        if (d > best) { best = d; axis = i; }
    }
    return axis;
}

__device__ void buildIncidentFace_device(const DeviceOBB& box, int axisIndex,
    const DeviceVec3& referenceNormal, DeviceVec3 outVerts[4]) {
    float s = dot(box.axis[axisIndex], referenceNormal) > 0.f ? -1.f : 1.f;
    DeviceVec3 faceNormal = mul(box.axis[axisIndex], s);
    float h = (axisIndex == 0 ? box.half.x : axisIndex == 1 ? box.half.y : box.half.z);
    DeviceVec3 faceCenter = add(box.center, mul(faceNormal, h));
    DeviceVec3 u, v;
    float extentU, extentV;
    getFaceAxes_device(box, axisIndex, u, v, extentU, extentV);
    outVerts[0] = add(faceCenter, add(mul(u, extentU), mul(v, extentV)));
    outVerts[1] = add(faceCenter, add(mul(u, -extentU), mul(v, extentV)));
    outVerts[2] = add(faceCenter, add(mul(u, -extentU), mul(v, -extentV)));
    outVerts[3] = add(faceCenter, add(mul(u, extentU), mul(v, -extentV)));
}

__device__ float clamp_device(float x, float a, float b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

__device__ int clipPolygonAgainstPlane_device(const DeviceVec3* inVerts, int inCount,
    const DeviceVec3& planeNormal, float planeOffset, DeviceVec3* outVerts) {
    if (inCount <= 0) return 0;
    int outCount = 0;
    DeviceVec3 a = inVerts[inCount - 1];
    float da = dot(planeNormal, a) - planeOffset;
    for (int i = 0; i < inCount; ++i) {
        DeviceVec3 b = inVerts[i];
        float db = dot(planeNormal, b) - planeOffset;
        int aInside = (da <= PLANE_EPSILON) ? 1 : 0;
        int bInside = (db <= PLANE_EPSILON) ? 1 : 0;
        if (aInside != bInside) {
            float t = 0.f;
            float denom = da - db;
            if (fabsf(denom) > SAT_AXIS_EPSILON) t = clamp_device(da / denom, 0.f, 1.f);
            if (outCount < MAX_POLY_VERTS)
                outVerts[outCount++] = add(a, mul(sub(b, a), t));
        }
        if (bInside && outCount < MAX_POLY_VERTS) outVerts[outCount++] = b;
        a = b; da = db;
    }
    return outCount;
}

__device__ int addContact_device(const DeviceVec3& pa, const DeviceQuat& qa,
    const DeviceVec3& pb, const DeviceQuat& qb, DeviceVec3 xA, DeviceVec3 xB, int featureKey,
    DeviceVec3* contacts_rA, DeviceVec3* contacts_rB, int* featureKeys, DeviceVec3* midpoints, int n) {
    DeviceVec3 midpoint = mul(add(xA, xB), 0.5f);
    for (int i = 0; i < n; ++i) {
        DeviceVec3 d = sub(midpoint, midpoints[i]);
        if (dot(d, d) < CONTACT_MERGE_DIST_SQ) return n;
    }
    if (n >= MAX_CONTACTS) return n;
    DeviceQuat qa_inv = quat_conjugate(qa);
    DeviceQuat qb_inv = quat_conjugate(qb);
    contacts_rA[n] = quat_rotate(qa_inv, sub(xA, pa));
    contacts_rB[n] = quat_rotate(qb_inv, sub(xB, pb));
    featureKeys[n] = featureKey;
    midpoints[n] = midpoint;
    return n + 1;
}

struct SatAxisDevice {
    int type;
    int indexA, indexB;
    float separation;
    DeviceVec3 normalAB;
    int valid;
};

__device__ int testAxis_device(const DeviceOBB& boxA, const DeviceOBB& boxB, const DeviceVec3& delta,
    const DeviceVec3& axis, int type, int indexA, int indexB, SatAxisDevice* best) {
    float lenSq = dot(axis, axis);
    if (lenSq < SAT_AXIS_EPSILON) return 1;
    float invLen = 1.f / sqrtf(lenSq);
    DeviceVec3 n = mul(axis, invLen);
    if (dot(n, delta) < 0.f) n = mul(n, -1.f);
    float distance = fabsf(dot(delta, n));
    float rA = boxA.half.x * absDot_device(n, boxA.axis[0]) + boxA.half.y * absDot_device(n, boxA.axis[1]) + boxA.half.z * absDot_device(n, boxA.axis[2]);
    float rB = boxB.half.x * absDot_device(n, boxB.axis[0]) + boxB.half.y * absDot_device(n, boxB.axis[1]) + boxB.half.z * absDot_device(n, boxB.axis[2]);
    float separation = distance - (rA + rB);
    if (separation > 0.f) return 0;
    if (!best->valid || separation > best->separation) {
        best->valid = 1;
        best->type = type;
        best->indexA = indexA;
        best->indexB = indexB;
        best->separation = separation;
        best->normalAB = n;
    }
    return 1;
}

__device__ void supportEdge_device(const DeviceOBB& box, int axisIndex, const DeviceVec3& dir,
    DeviceVec3& edgeA, DeviceVec3& edgeB) {
    int axis1 = (axisIndex + 1) % 3;
    int axis2 = (axisIndex + 2) % 3;
    float s1 = dot(dir, box.axis[axis1]) >= 0.f ? 1.f : -1.f;
    float s2 = dot(dir, box.axis[axis2]) >= 0.f ? 1.f : -1.f;
    float h1 = axis1 == 0 ? box.half.x : axis1 == 1 ? box.half.y : box.half.z;
    float h2 = axis2 == 0 ? box.half.x : axis2 == 1 ? box.half.y : box.half.z;
    DeviceVec3 edgeCenter = add(box.center, add(mul(box.axis[axis1], h1 * s1), mul(box.axis[axis2], h2 * s2)));
    float ha = axisIndex == 0 ? box.half.x : axisIndex == 1 ? box.half.y : box.half.z;
    edgeA = sub(edgeCenter, mul(box.axis[axisIndex], ha));
    edgeB = add(edgeCenter, mul(box.axis[axisIndex], ha));
}

__device__ void closestPointsOnSegments_device(const DeviceVec3& p0, const DeviceVec3& p1,
    const DeviceVec3& q0, const DeviceVec3& q1, DeviceVec3& c0, DeviceVec3& c1) {
    DeviceVec3 d1 = sub(p1, p0), d2 = sub(q1, q0), r = sub(p0, q0);
    float a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
    float s = 0.f, t = 0.f;
    if (a <= SAT_AXIS_EPSILON && e <= SAT_AXIS_EPSILON) {
        c0 = p0; c1 = q0; return;
    }
    if (a <= SAT_AXIS_EPSILON) {
        t = clamp_device(f / e, 0.f, 1.f);
    } else {
        float c = dot(d1, r);
        if (e <= SAT_AXIS_EPSILON) {
            s = clamp_device(-c / a, 0.f, 1.f);
        } else {
            float b = dot(d1, d2);
            float denom = a * e - b * b;
            if (fabsf(denom) > SAT_AXIS_EPSILON) s = clamp_device((b * f - c * e) / denom, 0.f, 1.f);
            t = (b * s + f) / e;
            if (t < 0.f) { t = 0.f; s = clamp_device(-c / a, 0.f, 1.f); }
            else if (t > 1.f) { t = 1.f; s = clamp_device((b - c) / a, 0.f, 1.f); }
        }
    }
    c0 = add(p0, mul(d1, s));
    c1 = add(q0, mul(d2, t));
}

__device__ void orthonormalBasis_device(const DeviceVec3& normal, float basis[9]) {
    DeviceVec3 n = normalize_vec3(normal);
    DeviceVec3 t1 = (fabsf(n.x) > fabsf(n.z)) ? make_vec3(-n.y, n.x, 0.f) : make_vec3(0.f, -n.z, n.y);
    t1 = normalize_vec3(t1);
    DeviceVec3 t2 = cross(n, t1);
    t2 = normalize_vec3(t2);
    basis[0] = n.x; basis[1] = n.y; basis[2] = n.z;
    basis[3] = t1.x; basis[4] = t1.y; basis[5] = t1.z;
    basis[6] = t2.x; basis[7] = t2.y; basis[8] = t2.z;
}

__device__ int buildFaceManifold_device(const DeviceVec3& pa, const DeviceQuat& qa,
    const DeviceVec3& pb, const DeviceQuat& qb, const DeviceOBB& boxA, const DeviceOBB& boxB,
    int referenceIsA, int referenceAxis, const DeviceVec3& normalAB,
    DeviceVec3* out_rA, DeviceVec3* out_rB, int* out_feature, DeviceVec3* midpoints, int nOut) {
    const DeviceOBB& refBox = referenceIsA ? boxA : boxB;
    const DeviceOBB& incBox = referenceIsA ? boxB : boxA;
    DeviceVec3 refOutward = referenceIsA ? normalAB : mul(normalAB, -1.f);
    FaceFrameDevice refFace;
    buildFaceFrame_device(refBox, referenceAxis, refOutward, refFace);
    int incAxis = chooseIncidentFaceAxis_device(incBox, refFace.normal);
    DeviceVec3 clip0[MAX_POLY_VERTS], clip1[MAX_POLY_VERTS];
    buildIncidentFace_device(incBox, incAxis, refFace.normal, clip0);
    int count = 4;
    DeviceVec3 n0 = refFace.u;
    float o0 = dot(n0, refFace.center) + refFace.extentU;
    count = clipPolygonAgainstPlane_device(clip0, count, n0, o0, clip1);
    if (!count) return nOut;
    DeviceVec3 n1 = mul(refFace.u, -1.f);
    float o1 = dot(n1, refFace.center) + refFace.extentU;
    count = clipPolygonAgainstPlane_device(clip1, count, n1, o1, clip0);
    if (!count) return nOut;
    DeviceVec3 n2 = refFace.v;
    float o2 = dot(n2, refFace.center) + refFace.extentV;
    count = clipPolygonAgainstPlane_device(clip0, count, n2, o2, clip1);
    if (!count) return nOut;
    DeviceVec3 n3 = mul(refFace.v, -1.f);
    float o3 = dot(n3, refFace.center) + refFace.extentV;
    count = clipPolygonAgainstPlane_device(clip1, count, n3, o3, clip0);
    if (!count) return nOut;
    int featurePrefix = (referenceIsA ? AXIS_FACE_A : AXIS_FACE_B) << 24;
    featurePrefix |= (referenceAxis & 0xFF) << 16;
    featurePrefix |= (incAxis & 0xFF) << 8;
    for (int i = 0; i < count && nOut < MAX_CONTACTS; ++i) {
        DeviceVec3 pInc = clip0[i];
        float dist = dot(sub(pInc, refFace.center), refFace.normal);
        if (dist > PLANE_EPSILON) continue;
        DeviceVec3 pRef = sub(pInc, mul(refFace.normal, dist));
        DeviceVec3 xA = referenceIsA ? pRef : pInc;
        DeviceVec3 xB = referenceIsA ? pInc : pRef;
        nOut = addContact_device(pa, qa, pb, qb, xA, xB, featurePrefix | (i & 0xFF),
            out_rA, out_rB, out_feature, midpoints, nOut);
    }
    if (nOut == 0) {
        DeviceVec3 xA = supportPoint_device(boxA, normalAB);
        DeviceVec3 xB = supportPoint_device(boxB, mul(normalAB, -1.f));
        nOut = addContact_device(pa, qa, pb, qb, xA, xB, featurePrefix,
            out_rA, out_rB, out_feature, midpoints, 0);
    }
    return nOut;
}

__device__ int buildEdgeContact_device(const DeviceVec3& pa, const DeviceQuat& qa,
    const DeviceVec3& pb, const DeviceQuat& qb, const DeviceOBB& boxA, const DeviceOBB& boxB,
    int axisA, int axisB, const DeviceVec3& normalAB,
    DeviceVec3* out_rA, DeviceVec3* out_rB, int* out_feature, DeviceVec3* midpoints, int nOut) {
    DeviceVec3 a0, a1, b0, b1;
    supportEdge_device(boxA, axisA, normalAB, a0, a1);
    supportEdge_device(boxB, axisB, mul(normalAB, -1.f), b0, b1);
    DeviceVec3 xA, xB;
    closestPointsOnSegments_device(a0, a1, b0, b1, xA, xB);
    int featureKey = (AXIS_EDGE << 24) | ((axisA & 0xFF) << 8) | (axisB & 0xFF);
    nOut = addContact_device(pa, qa, pb, qb, xA, xB, featureKey, out_rA, out_rB, out_feature, midpoints, nOut);
    if (nOut == 0) {
        xA = supportPoint_device(boxA, normalAB);
        xB = supportPoint_device(boxB, mul(normalAB, -1.f));
        nOut = addContact_device(pa, qa, pb, qb, xA, xB, featureKey, out_rA, out_rB, out_feature, midpoints, 0);
    }
    return nOut;
}

__device__ int collide_box_box_device(const DeviceVec3& pa, const DeviceQuat& qa, const DeviceVec3& half_a,
    const DeviceVec3& pb, const DeviceQuat& qb, const DeviceVec3& half_b,
    DeviceVec3* out_rA, DeviceVec3* out_rB, int* out_feature, float basis_out[9]) {
    DeviceOBB boxA = makeOBB_device(pa, qa, half_a);
    DeviceOBB boxB = makeOBB_device(pb, qb, half_b);
    DeviceVec3 delta = sub(pb, pa);
    SatAxisDevice bestFace = {0, -1, -1, -1e30f, make_vec3(0,0,0), 0};
    SatAxisDevice bestEdge = {0, -1, -1, -1e30f, make_vec3(0,0,0), 0};
    for (int i = 0; i < 3; ++i) {
        if (!testAxis_device(boxA, boxB, delta, boxA.axis[i], AXIS_FACE_A, i, -1, &bestFace)) return 0;
    }
    for (int i = 0; i < 3; ++i) {
        if (!testAxis_device(boxA, boxB, delta, boxB.axis[i], AXIS_FACE_B, -1, i, &bestFace)) return 0;
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            DeviceVec3 axis = cross(boxA.axis[i], boxB.axis[j]);
            if (!testAxis_device(boxA, boxB, delta, axis, AXIS_EDGE, i, j, &bestEdge)) return 0;
        }
    }
    if (!bestFace.valid) return 0;
    SatAxisDevice best = bestFace;
    if (bestEdge.valid) {
        const float edgeRelTol = 0.95f, edgeAbsTol = 0.01f;
        if (edgeRelTol * bestEdge.separation > bestFace.separation + edgeAbsTol)
            best = bestEdge;
    }
    DeviceVec3 negNorm = mul(best.normalAB, -1.f);
    orthonormalBasis_device(negNorm, basis_out);
    DeviceVec3 midpoints[MAX_CONTACTS];
    int nOut = 0;
    if (best.type == AXIS_EDGE) {
        return buildEdgeContact_device(pa, qa, pb, qb, boxA, boxB, best.indexA, best.indexB, best.normalAB,
            out_rA, out_rB, out_feature, midpoints, 0);
    }
    if (best.type == AXIS_FACE_A) {
        return buildFaceManifold_device(pa, qa, pb, qb, boxA, boxB, 1, best.indexA, best.normalAB,
            out_rA, out_rB, out_feature, midpoints, 0);
    }
    return buildFaceManifold_device(pa, qa, pb, qb, boxA, boxB, 0, best.indexB, best.normalAB,
        out_rA, out_rB, out_feature, midpoints, 0);
}

__device__ int collide_box_plane_device(const DeviceVec3& n, float d,
    const DeviceVec3& pb, const DeviceQuat& qb, const DeviceVec3& half_b,
    DeviceVec3* out_rA, DeviceVec3* out_rB, int* out_feature, float basis_out[9]) {
    float len = sqrtf(dot(n, n));
    if (len < 1e-12f) return 0;
    DeviceVec3 nn = mul(n, 1.f / len);
    orthonormalBasis_device(nn, basis_out);
    int nOut = 0;
    DeviceQuat qb_inv = quat_conjugate(qb);
    for (int i = 0; i < 8; ++i) {
        DeviceVec3 corner = make_vec3(
            (i & 1) ? half_b.x : -half_b.x,
            (i & 2) ? half_b.y : -half_b.y,
            (i & 4) ? half_b.z : -half_b.z);
        DeviceVec3 worldCorner = add(pb, quat_rotate(qb, corner));
        float dist = dot(nn, worldCorner) - d;
        if (dist < 0.f) {
            out_rA[nOut] = add(worldCorner, mul(nn, -dist));
            out_rB[nOut] = quat_rotate(qb_inv, sub(worldCorner, pb));
            out_feature[nOut] = (AXIS_FACE_B << 24) | (i & 0xFF);
            nOut++;
            if (nOut >= MAX_CONTACTS) break;
        }
    }
    return nOut;
}

__device__ DeviceVec3 clamp_vec3(const DeviceVec3& v, const DeviceVec3& lo, const DeviceVec3& hi) {
    return make_vec3(
        clamp_device(v.x, lo.x, hi.x),
        clamp_device(v.y, lo.y, hi.y),
        clamp_device(v.z, lo.z, hi.z));
}

// Closest point on OBB to a world point p. Box pose (pb,qb), half extents.
__device__ DeviceVec3 closest_point_obb(const DeviceVec3& pb, const DeviceQuat& qb, const DeviceVec3& half, const DeviceVec3& p) {
    DeviceQuat qb_inv = quat_conjugate(qb);
    DeviceVec3 plocal = quat_rotate(qb_inv, sub(p, pb));
    DeviceVec3 clamped = clamp_vec3(plocal, mul(half, -1.f), half);
    return add(pb, quat_rotate(qb, clamped));
}

__device__ int collide_sphere_plane_device(const DeviceVec3& n, float d,
    const DeviceVec3& sphere_center, float radius,
    DeviceVec3* out_rA, DeviceVec3* out_rB, int* out_feature, float basis_out[9]) {
    float len = sqrtf(dot(n, n));
    if (len < 1e-12f) return 0;
    DeviceVec3 nn = mul(n, 1.f / len);
    orthonormalBasis_device(nn, basis_out);
    float dist = dot(nn, sphere_center) - d;  // signed distance along nn
    if (dist >= radius) return 0;
    // Contact: xA on plane, xB on sphere surface.
    DeviceVec3 xA = sub(sphere_center, mul(nn, dist));            // projection onto plane
    DeviceVec3 xB = sub(sphere_center, mul(nn, radius));          // sphere point toward plane along -nn
    // For sphere, we store rB in sphere-local coordinates: since sphere is rotation-invariant, use local frame = body frame
    out_rA[0] = xA;                  // plane contact stored as world (body=-1 style) by caller when needed
    out_rB[0] = xB;                  // caller will convert to local using qb_inv if sphere has rotation; we handle in kernel
    out_feature[0] = 0;
    return 1;
}

__device__ int collide_sphere_box_device(const DeviceVec3& box_p, const DeviceQuat& box_q, const DeviceVec3& half,
    const DeviceVec3& sphere_p, float radius,
    DeviceVec3* out_rA, DeviceVec3* out_rB, int* out_feature, float basis_out[9]) {
    DeviceVec3 closest = closest_point_obb(box_p, box_q, half, sphere_p);
    DeviceVec3 v = sub(closest, sphere_p);
    float dist2 = dot(v, v);
    if (dist2 > radius * radius) return 0;
    float dist = sqrtf(fmaxf(dist2, 1e-12f));
    DeviceVec3 n = mul(v, 1.0f / dist);  // from sphere toward box
    orthonormalBasis_device(n, basis_out);
    DeviceVec3 xA = closest;
    DeviceVec3 xB = add(sphere_p, mul(n, radius));
    out_rA[0] = xA;
    out_rB[0] = xB;
    out_feature[0] = 0;
    return 1;
}

// One thread per pair: run narrowphase and append raw contacts to global buffer.
__global__ void narrowphase_kernel(
    const int2* __restrict__ pairs,
    int n_pairs,
    const DeviceShape* __restrict__ shapes,
    const DeviceVec3* __restrict__ positions,
    const DeviceQuat* __restrict__ rotations,
    int nBodies,
    int* __restrict__ contact_count,
    RawContact* __restrict__ raw_contacts,
    int max_contacts) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n_pairs) return;

    int2 p = pairs[idx];
    int si = p.x, sj = p.y;
    const DeviceShape& sa = shapes[si];
    const DeviceShape& sb = shapes[sj];

    int ia = sa.body_index;
    int ib = sb.body_index;
    if ((sa.type == 0 || sa.type == 2) && (ia < 0 || ia >= nBodies)) return;
    if ((sb.type == 0 || sb.type == 2) && (ib < 0 || ib >= nBodies)) return;

    DeviceVec3 pa = make_vec3(0.f, 0.f, 0.f);
    DeviceQuat qa = {1.f, 0.f, 0.f, 0.f};
    DeviceVec3 pb = make_vec3(0.f, 0.f, 0.f);
    DeviceQuat qb = {1.f, 0.f, 0.f, 0.f};
    DeviceVec3 half_a = make_vec3(sa.half[0], sa.half[1], sa.half[2]);
    DeviceVec3 half_b = make_vec3(sb.half[0], sb.half[1], sb.half[2]);
    float rad_a = sa.radius;
    float rad_b = sb.radius;

    if ((sa.type == 0 || sa.type == 2) && ia >= 0 && ia < nBodies) {
        DeviceVec3 local_p = make_vec3(sa.local_pos[0], sa.local_pos[1], sa.local_pos[2]);
        DeviceQuat local_q = {sa.local_quat[0], sa.local_quat[1], sa.local_quat[2], sa.local_quat[3]};
        pa = add(positions[ia], quat_rotate(rotations[ia], local_p));
        qa = quat_mul(rotations[ia], local_q);
    }
    if ((sb.type == 0 || sb.type == 2) && ib >= 0 && ib < nBodies) {
        DeviceVec3 local_p = make_vec3(sb.local_pos[0], sb.local_pos[1], sb.local_pos[2]);
        DeviceQuat local_q = {sb.local_quat[0], sb.local_quat[1], sb.local_quat[2], sb.local_quat[3]};
        pb = add(positions[ib], quat_rotate(rotations[ib], local_p));
        qb = quat_mul(rotations[ib], local_q);
    }

    DeviceVec3 rA[MAX_CONTACTS], rB[MAX_CONTACTS];
    int featureIds[MAX_CONTACTS];
    float basis[9];
    int nc = 0;
    float friction = 0.5f;

    if (sa.type == 0 && sb.type == 0) {
        nc = collide_box_box_device(pa, qa, half_a, pb, qb, half_b, rA, rB, featureIds, basis);
        friction = sqrtf(sa.friction * sb.friction);
        if (friction < 0.01f) friction = 0.5f;
    } else if (sa.type == 1 && sb.type == 0) {
        DeviceVec3 plane_n = make_vec3(sa.plane_n[0], sa.plane_n[1], sa.plane_n[2]);
        nc = collide_box_plane_device(plane_n, sa.plane_d, pb, qb, half_b, rA, rB, featureIds, basis);
        friction = sqrtf(sa.friction * sb.friction);
        if (friction < 0.01f) friction = 0.5f;
    } else if (sa.type == 0 && sb.type == 1) {
        DeviceVec3 plane_n = make_vec3(-sb.plane_n[0], -sb.plane_n[1], -sb.plane_n[2]);
        nc = collide_box_plane_device(plane_n, -sb.plane_d, pa, qa, half_a, rA, rB, featureIds, basis);
        for (int k = 0; k < nc; ++k) {
            DeviceVec3 tmp = rA[k]; rA[k] = rB[k]; rB[k] = tmp;
        }
        friction = sqrtf(sa.friction * sb.friction);
        if (friction < 0.01f) friction = 0.5f;
    } else if (sa.type == 1 && sb.type == 2) {
        // plane (A) vs sphere (B)
        DeviceVec3 plane_n = make_vec3(sa.plane_n[0], sa.plane_n[1], sa.plane_n[2]);
        DeviceVec3 xA[MAX_CONTACTS], xB[MAX_CONTACTS];
        int feat[MAX_CONTACTS];
        nc = collide_sphere_plane_device(plane_n, sa.plane_d, pb, rad_b, xA, xB, feat, basis);
        if (nc > 0) {
            // Convert to rA/rB: plane uses world point as rA; sphere uses local (qb^{-1}*(xB-pb))
            DeviceQuat qb_inv = quat_conjugate(qb);
            rA[0] = xA[0];
            rB[0] = quat_rotate(qb_inv, sub(xB[0], pb));
            featureIds[0] = feat[0];
        }
        friction = sqrtf(sa.friction * sb.friction);
        if (friction < 0.01f) friction = 0.5f;
    } else if (sa.type == 2 && sb.type == 1) {
        // sphere (A) vs plane (B): flip plane normal/offset so normal points B->A
        DeviceVec3 plane_n = make_vec3(-sb.plane_n[0], -sb.plane_n[1], -sb.plane_n[2]);
        DeviceVec3 xA[MAX_CONTACTS], xB[MAX_CONTACTS];
        int feat[MAX_CONTACTS];
        nc = collide_sphere_plane_device(plane_n, -sb.plane_d, pa, rad_a, xA, xB, feat, basis);
        if (nc > 0) {
            DeviceQuat qa_inv = quat_conjugate(qa);
            // Here xA is on plane, xB on sphere surface; but our A is sphere, B is plane,
            // we must swap so xA_on_sphere becomes rA, xB_on_plane becomes rB.
            DeviceVec3 xPlane = xA[0];
            DeviceVec3 xSphere = xB[0];
            rA[0] = quat_rotate(qa_inv, sub(xSphere, pa));
            rB[0] = xPlane;
            featureIds[0] = feat[0];
        }
        friction = sqrtf(sa.friction * sb.friction);
        if (friction < 0.01f) friction = 0.5f;
    } else if (sa.type == 0 && sb.type == 2) {
        // box (A) vs sphere (B)
        DeviceVec3 xA[MAX_CONTACTS], xB[MAX_CONTACTS];
        int feat[MAX_CONTACTS];
        nc = collide_sphere_box_device(pa, qa, half_a, pb, rad_b, xA, xB, feat, basis);
        if (nc > 0) {
            DeviceQuat qa_inv = quat_conjugate(qa);
            DeviceQuat qb_inv = quat_conjugate(qb);
            rA[0] = quat_rotate(qa_inv, sub(xA[0], pa));
            rB[0] = quat_rotate(qb_inv, sub(xB[0], pb));
            featureIds[0] = feat[0];
        }
        friction = sqrtf(sa.friction * sb.friction);
        if (friction < 0.01f) friction = 0.5f;
    } else if (sa.type == 2 && sb.type == 0) {
        // sphere (A) vs box (B): compute as box(A') vs sphere(B') then swap
        DeviceVec3 xA[MAX_CONTACTS], xB[MAX_CONTACTS];
        int feat[MAX_CONTACTS];
        nc = collide_sphere_box_device(pb, qb, half_b, pa, rad_a, xA, xB, feat, basis);
        if (nc > 0) {
            DeviceQuat qa_inv = quat_conjugate(qa);
            DeviceQuat qb_inv = quat_conjugate(qb);
            // xA is on box(B), xB is on sphere(A). swap for (A sphere, B box)
            DeviceVec3 xBox = xA[0];
            DeviceVec3 xSphere = xB[0];
            rA[0] = quat_rotate(qa_inv, sub(xSphere, pa));
            rB[0] = quat_rotate(qb_inv, sub(xBox, pb));
            featureIds[0] = feat[0];
        }
        friction = sqrtf(sa.friction * sb.friction);
        if (friction < 0.01f) friction = 0.5f;
    }
    if (nc <= 0) return;

    int max_cp = nc > 8 ? 8 : nc;
    for (int k = 0; k < max_cp; ++k) {
        int slot = atomicAdd(contact_count, 1);
        if (slot >= max_contacts) return;
        RawContact& c = raw_contacts[slot];
        c.body_a = ia;
        c.body_b = ib;
        c.rA[0] = rA[k].x; c.rA[1] = rA[k].y; c.rA[2] = rA[k].z;
        c.rB[0] = rB[k].x; c.rB[1] = rB[k].y; c.rB[2] = rB[k].z;
        for (int i = 0; i < 9; ++i) c.basis[i] = basis[i];
        c.friction = friction;
        c.feature_id = featureIds[k];
    }
}

// -----------------------------------------------------------------------------
// Persistent CUDA device buffers: avoid cudaMalloc/cudaFree every step (major
// bottleneck for large scenes). Allocate once (or when size grows), reuse.
// -----------------------------------------------------------------------------
struct CudaBuffers {
    DeviceBody* d_bodies = nullptr;
    int* d_body_color = nullptr;
    DeviceJoint* d_joints = nullptr;
    DeviceSpring* d_springs = nullptr;
    DeviceVec3* d_initial_pos = nullptr;
    DeviceQuat* d_initial_rot = nullptr;
    DeviceVec3* d_inertial_pos = nullptr;
    DeviceQuat* d_inertial_rot = nullptr;
    DeviceVec3* d_positions = nullptr;
    DeviceQuat* d_rotations = nullptr;
    DeviceContact* d_contacts = nullptr;
    int cap_bodies = 0;
    int cap_contacts = 0;
    int cap_joints = 0;
    int cap_springs = 0;
    // GPU collision (broadphase + narrowphase)
    DeviceShape* d_shapes = nullptr;
    DeviceAABB* d_aabbs = nullptr;
    int* d_shape_static = nullptr;
    int2* d_pairs = nullptr;
    int* d_pair_count = nullptr;
    // GPU grid broadphase temp
    int* d_cell_counts = nullptr;
    int* d_cell_offsets = nullptr;
    uint64_t* d_cell_keys = nullptr;
    int* d_cell_shapes = nullptr;
    uint64_t* d_cell_keys_sorted = nullptr;
    int* d_cell_shapes_sorted = nullptr;
    void* d_scan_temp = nullptr;
    size_t scan_temp_bytes = 0;
    void* d_sort_temp = nullptr;
    size_t sort_temp_bytes = 0;
    int cap_cell_entries = 0;
    RawContact* d_raw_contacts = nullptr;
    int* d_narrowphase_contact_count = nullptr;
    // GPU contact key / sort / unique buffers
    uint64_t* d_contact_keys = nullptr;
    uint64_t* d_contact_keys_sorted = nullptr;
    RawContact* d_raw_sorted = nullptr;
    int* d_unique_flags = nullptr;
    int* d_unique_indices = nullptr;
    // Prev-frame for GPU warmstart (key + contact sorted by key)
    DeviceContact* d_prev_contacts = nullptr;
    uint64_t* d_prev_contact_keys = nullptr;
    uint64_t* d_prev_contact_keys_sorted = nullptr;
    DeviceContact* d_prev_contacts_sorted = nullptr;
    int* d_prev_contact_indices = nullptr;  // sort (key, index) then gather to avoid CUB large-value shared mem
    int n_prev_contacts = 0;
    int cap_prev_contacts = 0;
    // Warmstart output (step 2: D2H for CPU C0+sort; step 3: input to build DeviceContact)
    RawContactWarmstart* d_raw_contacts_warmstart = nullptr;
    RawContactWarmstart* h_pinned_raw_warmstart = nullptr;
    RawContactWarmstart* d_raw_warmstart_compact = nullptr;  // step 3: after ignore filter
    DeviceContact* d_contacts_sorted = nullptr;              // step 3: sort output then copy to d_contacts
    int* d_contact_indices = nullptr;                        // step 3: sort (key, index) then gather by index
    // Ignore pair keys for GPU filter (step 3)
    uint64_t* d_ignore_pair_keys = nullptr;
    int n_ignore_pair_keys = 0;
    int cap_ignore_pair_keys = 0;
    int cap_shapes = 0;
    int cap_pairs = 0;
    int cap_raw_contacts = 0;
    int cap_contact_keys = 0;
    int cap_cell_entries_host = 0;
    // Pinned host collision buffers (avoid per-step std::vector allocations)
    DeviceShape* h_pinned_shapes = nullptr;
    int* h_pinned_shape_static = nullptr;
    RawContact* h_pinned_raw_contacts = nullptr;
    // Pinned host memory for faster H2D/D2H (avoid pageable copy).
    DeviceBody* h_pinned_bodies = nullptr;
    DeviceVec3* h_pinned_initial_pos = nullptr;
    DeviceQuat* h_pinned_initial_rot = nullptr;
    DeviceVec3* h_pinned_inertial_pos = nullptr;
    DeviceQuat* h_pinned_inertial_rot = nullptr;
    DeviceVec3* h_pinned_positions = nullptr;
    DeviceQuat* h_pinned_rotations = nullptr;
    DeviceContact* h_pinned_contacts = nullptr;
    DeviceJoint* h_pinned_joints = nullptr;
    DeviceSpring* h_pinned_springs = nullptr;
};
static CudaBuffers s_cuda;

// Forward declaration: GPU-side raw contact compaction helper.
static void compress_raw_contacts_on_gpu(int n_raw, int* out_n_compact);

static void ensure_cuda_buffers(int nBodies, int nContacts, int nJoints, int nSprings, int num_shapes = 0) {
    if (nBodies > s_cuda.cap_bodies) {
        if (s_cuda.h_pinned_bodies) cudaFreeHost(s_cuda.h_pinned_bodies);
        if (s_cuda.h_pinned_initial_pos) cudaFreeHost(s_cuda.h_pinned_initial_pos);
        if (s_cuda.h_pinned_initial_rot) cudaFreeHost(s_cuda.h_pinned_initial_rot);
        if (s_cuda.h_pinned_inertial_pos) cudaFreeHost(s_cuda.h_pinned_inertial_pos);
        if (s_cuda.h_pinned_inertial_rot) cudaFreeHost(s_cuda.h_pinned_inertial_rot);
        if (s_cuda.h_pinned_positions) cudaFreeHost(s_cuda.h_pinned_positions);
        if (s_cuda.h_pinned_rotations) cudaFreeHost(s_cuda.h_pinned_rotations);
        s_cuda.h_pinned_bodies = nullptr;
        s_cuda.h_pinned_initial_pos = nullptr;
        s_cuda.h_pinned_initial_rot = nullptr;
        s_cuda.h_pinned_inertial_pos = nullptr;
        s_cuda.h_pinned_inertial_rot = nullptr;
        s_cuda.h_pinned_positions = nullptr;
        s_cuda.h_pinned_rotations = nullptr;
        if (s_cuda.d_bodies) cudaFree(s_cuda.d_bodies);
        if (s_cuda.d_body_color) cudaFree(s_cuda.d_body_color);
        if (s_cuda.d_initial_pos) cudaFree(s_cuda.d_initial_pos);
        if (s_cuda.d_initial_rot) cudaFree(s_cuda.d_initial_rot);
        if (s_cuda.d_inertial_pos) cudaFree(s_cuda.d_inertial_pos);
        if (s_cuda.d_inertial_rot) cudaFree(s_cuda.d_inertial_rot);
        if (s_cuda.d_positions) cudaFree(s_cuda.d_positions);
        if (s_cuda.d_rotations) cudaFree(s_cuda.d_rotations);
        s_cuda.d_bodies = nullptr;
        s_cuda.d_body_color = nullptr;
        s_cuda.d_initial_pos = nullptr;
        s_cuda.d_initial_rot = nullptr;
        s_cuda.d_inertial_pos = nullptr;
        s_cuda.d_inertial_rot = nullptr;
        s_cuda.d_positions = nullptr;
        s_cuda.d_rotations = nullptr;
        s_cuda.cap_bodies = 0;
        if (nBodies > 0) {
            cudaMalloc(&s_cuda.d_bodies, nBodies * sizeof(DeviceBody));
            cudaMalloc(&s_cuda.d_initial_pos, nBodies * sizeof(DeviceVec3));
            cudaMalloc(&s_cuda.d_initial_rot, nBodies * sizeof(DeviceQuat));
            cudaMalloc(&s_cuda.d_inertial_pos, nBodies * sizeof(DeviceVec3));
            cudaMalloc(&s_cuda.d_inertial_rot, nBodies * sizeof(DeviceQuat));
            cudaMalloc(&s_cuda.d_positions, nBodies * sizeof(DeviceVec3));
            cudaMalloc(&s_cuda.d_rotations, nBodies * sizeof(DeviceQuat));
            if (cudaMalloc(&s_cuda.d_body_color, nBodies * sizeof(int)) != cudaSuccess)
                s_cuda.d_body_color = nullptr;
            cudaMallocHost(&s_cuda.h_pinned_bodies, nBodies * sizeof(DeviceBody));
            cudaMallocHost(&s_cuda.h_pinned_initial_pos, nBodies * sizeof(DeviceVec3));
            cudaMallocHost(&s_cuda.h_pinned_initial_rot, nBodies * sizeof(DeviceQuat));
            cudaMallocHost(&s_cuda.h_pinned_inertial_pos, nBodies * sizeof(DeviceVec3));
            cudaMallocHost(&s_cuda.h_pinned_inertial_rot, nBodies * sizeof(DeviceQuat));
            cudaMallocHost(&s_cuda.h_pinned_positions, nBodies * sizeof(DeviceVec3));
            cudaMallocHost(&s_cuda.h_pinned_rotations, nBodies * sizeof(DeviceQuat));
            s_cuda.cap_bodies = nBodies;
        }
    }
    // Contact cap must be at least nContacts and, for GPU build path, at least cap_raw_contacts.
    int contact_cap = nContacts;
    if (s_cuda.cap_raw_contacts > contact_cap) contact_cap = s_cuda.cap_raw_contacts;
        if (contact_cap > s_cuda.cap_contacts) {
        if (s_cuda.h_pinned_contacts) cudaFreeHost(s_cuda.h_pinned_contacts);
        s_cuda.h_pinned_contacts = nullptr;
        if (s_cuda.d_contacts_sorted) cudaFree(s_cuda.d_contacts_sorted);
        if (s_cuda.d_contact_indices) cudaFree(s_cuda.d_contact_indices);
        s_cuda.d_contacts_sorted = nullptr;
        s_cuda.d_contact_indices = nullptr;
        if (s_cuda.d_contacts) cudaFree(s_cuda.d_contacts);
        s_cuda.d_contacts = nullptr;
        if (s_cuda.d_prev_contacts) cudaFree(s_cuda.d_prev_contacts);
        if (s_cuda.d_prev_contact_keys) cudaFree(s_cuda.d_prev_contact_keys);
        if (s_cuda.d_prev_contact_keys_sorted) cudaFree(s_cuda.d_prev_contact_keys_sorted);
        if (s_cuda.d_prev_contacts_sorted) cudaFree(s_cuda.d_prev_contacts_sorted);
        if (s_cuda.d_prev_contact_indices) cudaFree(s_cuda.d_prev_contact_indices);
        s_cuda.d_prev_contacts = nullptr;
        s_cuda.d_prev_contact_keys = nullptr;
        s_cuda.d_prev_contact_keys_sorted = nullptr;
        s_cuda.d_prev_contacts_sorted = nullptr;
        s_cuda.d_prev_contact_indices = nullptr;
        s_cuda.cap_contacts = 0;
        s_cuda.cap_prev_contacts = 0;
        if (contact_cap > 0) {
            cudaMalloc(&s_cuda.d_contacts, contact_cap * sizeof(DeviceContact));
            cudaMallocHost(&s_cuda.h_pinned_contacts, contact_cap * sizeof(DeviceContact));
            cudaMalloc(&s_cuda.d_contacts_sorted, contact_cap * sizeof(DeviceContact));
            cudaMalloc(&s_cuda.d_contact_indices, contact_cap * sizeof(int));
            cudaMalloc(&s_cuda.d_prev_contacts, contact_cap * sizeof(DeviceContact));
            cudaMalloc(&s_cuda.d_prev_contact_keys, contact_cap * sizeof(uint64_t));
            cudaMalloc(&s_cuda.d_prev_contact_keys_sorted, contact_cap * sizeof(uint64_t));
            cudaMalloc(&s_cuda.d_prev_contacts_sorted, contact_cap * sizeof(DeviceContact));
            cudaMalloc(&s_cuda.d_prev_contact_indices, contact_cap * sizeof(int));
            s_cuda.cap_contacts = contact_cap;
            s_cuda.cap_prev_contacts = contact_cap;
        }
    }
    if (nJoints > s_cuda.cap_joints) {
        if (s_cuda.h_pinned_joints) cudaFreeHost(s_cuda.h_pinned_joints);
        s_cuda.h_pinned_joints = nullptr;
        if (s_cuda.d_joints) cudaFree(s_cuda.d_joints);
        s_cuda.d_joints = nullptr;
        s_cuda.cap_joints = 0;
        if (nJoints > 0) {
            cudaMalloc(&s_cuda.d_joints, nJoints * sizeof(DeviceJoint));
            cudaMallocHost(&s_cuda.h_pinned_joints, nJoints * sizeof(DeviceJoint));
            s_cuda.cap_joints = nJoints;
        }
    }
    if (nSprings > s_cuda.cap_springs) {
        if (s_cuda.h_pinned_springs) cudaFreeHost(s_cuda.h_pinned_springs);
        s_cuda.h_pinned_springs = nullptr;
        if (s_cuda.d_springs) cudaFree(s_cuda.d_springs);
        s_cuda.d_springs = nullptr;
        s_cuda.cap_springs = 0;
        if (nSprings > 0) {
            cudaMalloc(&s_cuda.d_springs, nSprings * sizeof(DeviceSpring));
            cudaMallocHost(&s_cuda.h_pinned_springs, nSprings * sizeof(DeviceSpring));
            s_cuda.cap_springs = nSprings;
        }
    }

    if (num_shapes > 0) {
        if (num_shapes > s_cuda.cap_shapes) {
            if (s_cuda.h_pinned_shapes) cudaFreeHost(s_cuda.h_pinned_shapes);
            if (s_cuda.h_pinned_shape_static) cudaFreeHost(s_cuda.h_pinned_shape_static);
            s_cuda.h_pinned_shapes = nullptr;
            s_cuda.h_pinned_shape_static = nullptr;
            if (s_cuda.d_shapes) cudaFree(s_cuda.d_shapes);
            if (s_cuda.d_aabbs) cudaFree(s_cuda.d_aabbs);
            if (s_cuda.d_shape_static) cudaFree(s_cuda.d_shape_static);
            s_cuda.d_shapes = nullptr;
            s_cuda.d_aabbs = nullptr;
            s_cuda.d_shape_static = nullptr;
            s_cuda.cap_shapes = 0;
            cudaMalloc(&s_cuda.d_shapes, num_shapes * sizeof(DeviceShape));
            cudaMalloc(&s_cuda.d_aabbs, num_shapes * sizeof(DeviceAABB));
            cudaMalloc(&s_cuda.d_shape_static, num_shapes * sizeof(int));
            cudaMallocHost(&s_cuda.h_pinned_shapes, num_shapes * sizeof(DeviceShape));
            cudaMallocHost(&s_cuda.h_pinned_shape_static, num_shapes * sizeof(int));
            s_cuda.cap_shapes = num_shapes;
        }
        // Pair buffer capacity: heuristic upper bound for dense stacking (reduce reallocs).
        // We deliberately over-allocate to avoid rare overflow -> CPU fallback spikes.
        int max_pairs = num_shapes * 64;  // typical grid broadphase yields O(N) pairs
        if (max_pairs < 1024) max_pairs = 1024;
        int max_raw = max_pairs * 8;
        if (max_pairs > s_cuda.cap_pairs || max_raw > s_cuda.cap_raw_contacts) {
            if (s_cuda.h_pinned_raw_contacts) cudaFreeHost(s_cuda.h_pinned_raw_contacts);
            s_cuda.h_pinned_raw_contacts = nullptr;
            if (s_cuda.d_raw_contacts) cudaFree(s_cuda.d_raw_contacts);
            if (s_cuda.d_narrowphase_contact_count) cudaFree(s_cuda.d_narrowphase_contact_count);
            if (s_cuda.d_pairs) cudaFree(s_cuda.d_pairs);
            if (s_cuda.d_pair_count) cudaFree(s_cuda.d_pair_count);
            s_cuda.d_raw_contacts = nullptr;
            s_cuda.d_narrowphase_contact_count = nullptr;
            s_cuda.d_pairs = nullptr;
            s_cuda.d_pair_count = nullptr;
            s_cuda.cap_pairs = 0;
            s_cuda.cap_raw_contacts = 0;
            if (max_pairs > 0) {
                cudaMalloc(&s_cuda.d_pairs, max_pairs * sizeof(int2));
                cudaMalloc(&s_cuda.d_pair_count, sizeof(int));
                cudaMalloc(&s_cuda.d_raw_contacts, max_raw * sizeof(RawContact));
                cudaMalloc(&s_cuda.d_narrowphase_contact_count, sizeof(int));
                cudaMallocHost(&s_cuda.h_pinned_raw_contacts, max_raw * sizeof(RawContact));
                s_cuda.cap_pairs = max_pairs;
                s_cuda.cap_raw_contacts = max_raw;
            }
        }
        if (s_cuda.cap_raw_contacts > 0 && !s_cuda.d_raw_contacts_warmstart) {
            cudaMalloc(&s_cuda.d_raw_contacts_warmstart, s_cuda.cap_raw_contacts * sizeof(RawContactWarmstart));
            cudaMallocHost(&s_cuda.h_pinned_raw_warmstart, s_cuda.cap_raw_contacts * sizeof(RawContactWarmstart));
            cudaMalloc(&s_cuda.d_raw_warmstart_compact, s_cuda.cap_raw_contacts * sizeof(RawContactWarmstart));
        }
        // Contact buffers may not have been allocated yet (nContacts was 0); ensure cap_contacts >= cap_raw_contacts.
        if (s_cuda.cap_raw_contacts > 0 && s_cuda.cap_raw_contacts > s_cuda.cap_contacts) {
            int contact_cap = s_cuda.cap_raw_contacts;
            if (s_cuda.h_pinned_contacts) cudaFreeHost(s_cuda.h_pinned_contacts);
            s_cuda.h_pinned_contacts = nullptr;
            if (s_cuda.d_contacts_sorted) cudaFree(s_cuda.d_contacts_sorted);
            if (s_cuda.d_contact_indices) cudaFree(s_cuda.d_contact_indices);
            s_cuda.d_contacts_sorted = nullptr;
            s_cuda.d_contact_indices = nullptr;
            if (s_cuda.d_contacts) cudaFree(s_cuda.d_contacts);
            s_cuda.d_contacts = nullptr;
            if (s_cuda.d_prev_contacts) cudaFree(s_cuda.d_prev_contacts);
            if (s_cuda.d_prev_contact_keys) cudaFree(s_cuda.d_prev_contact_keys);
            if (s_cuda.d_prev_contact_keys_sorted) cudaFree(s_cuda.d_prev_contact_keys_sorted);
            if (s_cuda.d_prev_contacts_sorted) cudaFree(s_cuda.d_prev_contacts_sorted);
            if (s_cuda.d_prev_contact_indices) cudaFree(s_cuda.d_prev_contact_indices);
            s_cuda.d_prev_contacts = nullptr;
            s_cuda.d_prev_contact_keys = nullptr;
            s_cuda.d_prev_contact_keys_sorted = nullptr;
            s_cuda.d_prev_contacts_sorted = nullptr;
            s_cuda.d_prev_contact_indices = nullptr;
            cudaMalloc(&s_cuda.d_contacts, contact_cap * sizeof(DeviceContact));
            cudaMallocHost(&s_cuda.h_pinned_contacts, contact_cap * sizeof(DeviceContact));
            cudaMalloc(&s_cuda.d_contacts_sorted, contact_cap * sizeof(DeviceContact));
            cudaMalloc(&s_cuda.d_contact_indices, contact_cap * sizeof(int));
            cudaMalloc(&s_cuda.d_prev_contacts, contact_cap * sizeof(DeviceContact));
            cudaMalloc(&s_cuda.d_prev_contact_keys, contact_cap * sizeof(uint64_t));
            cudaMalloc(&s_cuda.d_prev_contact_keys_sorted, contact_cap * sizeof(uint64_t));
            cudaMalloc(&s_cuda.d_prev_contacts_sorted, contact_cap * sizeof(DeviceContact));
            cudaMalloc(&s_cuda.d_prev_contact_indices, contact_cap * sizeof(int));
            s_cuda.cap_contacts = contact_cap;
            s_cuda.cap_prev_contacts = contact_cap;
        }

        // Contact key / sort / unique buffers (size grows with cap_raw_contacts).
        int max_keys = s_cuda.cap_raw_contacts;
        if (max_keys > s_cuda.cap_contact_keys) {
            if (s_cuda.d_contact_keys) cudaFree(s_cuda.d_contact_keys);
            if (s_cuda.d_contact_keys_sorted) cudaFree(s_cuda.d_contact_keys_sorted);
            if (s_cuda.d_raw_sorted) cudaFree(s_cuda.d_raw_sorted);
            if (s_cuda.d_unique_flags) cudaFree(s_cuda.d_unique_flags);
            if (s_cuda.d_unique_indices) cudaFree(s_cuda.d_unique_indices);
            s_cuda.d_contact_keys = nullptr;
            s_cuda.d_contact_keys_sorted = nullptr;
            s_cuda.d_raw_sorted = nullptr;
            s_cuda.d_unique_flags = nullptr;
            s_cuda.d_unique_indices = nullptr;
            if (max_keys > 0) {
                cudaMalloc(&s_cuda.d_contact_keys, max_keys * sizeof(uint64_t));
                cudaMalloc(&s_cuda.d_contact_keys_sorted, max_keys * sizeof(uint64_t));
                cudaMalloc(&s_cuda.d_raw_sorted, max_keys * sizeof(RawContact));
                cudaMalloc(&s_cuda.d_unique_flags, max_keys * sizeof(int));
                cudaMalloc(&s_cuda.d_unique_indices, max_keys * sizeof(int));
                s_cuda.cap_contact_keys = max_keys;
            } else {
                s_cuda.cap_contact_keys = 0;
            }
        }

        // Grid broadphase buffers: counts/offsets and cell entries.
        if (num_shapes > s_cuda.cap_shapes) {
            // handled above
        }
        // Allocate counts/offsets for all shapes.
        if (!s_cuda.d_cell_counts) cudaMalloc(&s_cuda.d_cell_counts, num_shapes * sizeof(int));
        if (!s_cuda.d_cell_offsets) cudaMalloc(&s_cuda.d_cell_offsets, num_shapes * sizeof(int));

        // Worst-case cell entries: allow moderate AABB overlaps (heuristic).
        // Over-allocate to avoid n_entries > cap_cell_entries fallback spikes.
        int max_entries = num_shapes * 64;
        if (max_entries < 2048) max_entries = 2048;
        if (max_entries > s_cuda.cap_cell_entries) {
            if (s_cuda.d_cell_keys) cudaFree(s_cuda.d_cell_keys);
            if (s_cuda.d_cell_shapes) cudaFree(s_cuda.d_cell_shapes);
            if (s_cuda.d_cell_keys_sorted) cudaFree(s_cuda.d_cell_keys_sorted);
            if (s_cuda.d_cell_shapes_sorted) cudaFree(s_cuda.d_cell_shapes_sorted);
            s_cuda.d_cell_keys = nullptr;
            s_cuda.d_cell_shapes = nullptr;
            s_cuda.d_cell_keys_sorted = nullptr;
            s_cuda.d_cell_shapes_sorted = nullptr;
            cudaMalloc(&s_cuda.d_cell_keys, max_entries * sizeof(uint64_t));
            cudaMalloc(&s_cuda.d_cell_shapes, max_entries * sizeof(int));
            cudaMalloc(&s_cuda.d_cell_keys_sorted, max_entries * sizeof(uint64_t));
            cudaMalloc(&s_cuda.d_cell_shapes_sorted, max_entries * sizeof(int));
            s_cuda.cap_cell_entries = max_entries;
        }

        // Temp storage sizes (scan + sort), allocate lazily for current capacity.
        size_t scan_bytes = 0;
        cub::DeviceScan::ExclusiveSum(nullptr, scan_bytes, s_cuda.d_cell_counts, s_cuda.d_cell_offsets, num_shapes);
        if (scan_bytes > s_cuda.scan_temp_bytes) {
            if (s_cuda.d_scan_temp) cudaFree(s_cuda.d_scan_temp);
            cudaMalloc(&s_cuda.d_scan_temp, scan_bytes);
            s_cuda.scan_temp_bytes = scan_bytes;
        }
        size_t sort_bytes = 0;
        cub::DeviceRadixSort::SortPairs(nullptr, sort_bytes,
                                       s_cuda.d_cell_keys, s_cuda.d_cell_keys_sorted,
                                       s_cuda.d_cell_shapes, s_cuda.d_cell_shapes_sorted,
                                       s_cuda.cap_cell_entries);
        if (sort_bytes > s_cuda.sort_temp_bytes) {
            if (s_cuda.d_sort_temp) cudaFree(s_cuda.d_sort_temp);
            cudaMalloc(&s_cuda.d_sort_temp, sort_bytes);
            s_cuda.sort_temp_bytes = sort_bytes;
        }
    }
}

// Helper: run the above kernels and CUB ops to compress d_raw_contacts in-place
// (result is stored back into s_cuda.d_raw_contacts, out_n is the new length).
static void compress_raw_contacts_on_gpu(int n_raw, int* out_n_compact) {
    *out_n_compact = 0;
    if (n_raw <= 0) return;
    if (!s_cuda.d_raw_contacts || !s_cuda.d_contact_keys || !s_cuda.d_contact_keys_sorted ||
        !s_cuda.d_raw_sorted || !s_cuda.d_unique_flags || !s_cuda.d_unique_indices) {
        *out_n_compact = n_raw;
        return;
    }
    const int threads = 256;
    const int blocks = (n_raw + threads - 1) / threads;
    build_contact_keys_kernel<<<blocks, threads>>>(s_cuda.d_raw_contacts, n_raw, s_cuda.d_contact_keys);
    size_t sort_bytes = 0;
    cub::DeviceRadixSort::SortPairs(
        nullptr, sort_bytes,
        s_cuda.d_contact_keys, s_cuda.d_contact_keys_sorted,
        s_cuda.d_raw_contacts, s_cuda.d_raw_sorted,
        n_raw);
    if (sort_bytes > s_cuda.sort_temp_bytes) {
        if (s_cuda.d_sort_temp) cudaFree(s_cuda.d_sort_temp);
        cudaMalloc(&s_cuda.d_sort_temp, sort_bytes);
        s_cuda.sort_temp_bytes = sort_bytes;
    }
    cub::DeviceRadixSort::SortPairs(
        s_cuda.d_sort_temp, s_cuda.sort_temp_bytes,
        s_cuda.d_contact_keys, s_cuda.d_contact_keys_sorted,
        s_cuda.d_raw_contacts, s_cuda.d_raw_sorted,
        n_raw);
    mark_unique_contact_flags_kernel<<<blocks, threads>>>(s_cuda.d_contact_keys_sorted, n_raw, s_cuda.d_unique_flags);
    size_t scan_bytes = 0;
    cub::DeviceScan::ExclusiveSum(
        nullptr, scan_bytes,
        s_cuda.d_unique_flags, s_cuda.d_unique_indices,
        n_raw);
    if (scan_bytes > s_cuda.scan_temp_bytes) {
        if (s_cuda.d_scan_temp) cudaFree(s_cuda.d_scan_temp);
        cudaMalloc(&s_cuda.d_scan_temp, scan_bytes);
        s_cuda.scan_temp_bytes = scan_bytes;
    }
    cub::DeviceScan::ExclusiveSum(
        s_cuda.d_scan_temp, s_cuda.scan_temp_bytes,
        s_cuda.d_unique_flags, s_cuda.d_unique_indices,
        n_raw);
    int last_flag = 0, last_index = 0;
    cudaMemcpy(&last_flag, s_cuda.d_unique_flags + (n_raw - 1), sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(&last_index, s_cuda.d_unique_indices + (n_raw - 1), sizeof(int), cudaMemcpyDeviceToHost);
    int n_compact = last_index + last_flag;
    if (n_compact <= 0 || n_compact > s_cuda.cap_raw_contacts) {
        *out_n_compact = n_raw;
        return;
    }
    compact_sorted_contacts_kernel<<<blocks, threads>>>(
        s_cuda.d_raw_sorted,
        s_cuda.d_unique_flags,
        s_cuda.d_unique_indices,
        n_raw,
        s_cuda.d_raw_contacts);
    *out_n_compact = n_compact;
}

// -----------------------------------------------------------------------------
// Vertex coloring: color bodies so that no two bodies sharing a contact have
// the same color. Same-color bodies can update in parallel (no data race);
// different colors are serialized. This gives the "collision detection →
// vertex coloring → per-color iteration" pipeline for maximum GPU parallelism.
// -----------------------------------------------------------------------------
void compute_body_colors(int nBodies,
                        const std::vector<AvbdContact>& contacts,
                        const std::vector<AvbdJoint>& joints,
                        const std::vector<AvbdSpring>& springs,
                        std::vector<int>* out_body_color,
                        int* out_num_colors) {
    out_body_color->resize(static_cast<size_t>(nBodies), -1);
    std::vector<std::vector<int>> adj(static_cast<size_t>(nBodies));
    for (const AvbdContact& c : contacts) {
        int ia = c.body_a;
        int ib = c.body_b;
        if (ia >= 0 && ia < nBodies && ib >= 0 && ib < nBodies) {
            adj[static_cast<size_t>(ia)].push_back(ib);
            adj[static_cast<size_t>(ib)].push_back(ia);
        }
    }
    for (const AvbdJoint& j : joints) {
        if (j.broken) continue;
        int ia = j.body_a;
        int ib = j.body_b;
        if (ia >= 0 && ia < nBodies && ib >= 0 && ib < nBodies) {
            adj[static_cast<size_t>(ia)].push_back(ib);
            adj[static_cast<size_t>(ib)].push_back(ia);
        }
    }
    for (const AvbdSpring& s : springs) {
        int ia = s.body_a;
        int ib = s.body_b;
        if (ia >= 0 && ia < nBodies && ib >= 0 && ib < nBodies) {
            adj[static_cast<size_t>(ia)].push_back(ib);
            adj[static_cast<size_t>(ib)].push_back(ia);
        }
    }
    int num_colors = 0;
    for (int bi = 0; bi < nBodies; ++bi) {
        std::set<int> used;
        for (int j : adj[static_cast<size_t>(bi)]) {
            int cj = (*out_body_color)[static_cast<size_t>(j)];
            if (cj >= 0) used.insert(cj);
        }
        int c = 0;
        while (used.count(c)) ++c;
        (*out_body_color)[static_cast<size_t>(bi)] = c;
        if (c + 1 > num_colors) num_colors = c + 1;
    }
    *out_num_colors = num_colors;
}

}  // namespace

void VbdSolver::step_cuda(const Model& model, SimState& state) {
    // AVBD on GPU: collision detection → vertex coloring → inertial init →
    // per-color iteration (build LHS/RHS → LDL solve → update position) →
    // update dual (lambda/penalty) → velocity update. Augmented Lagrangian
    // handles hard constraints; coloring allows same-color bodies to update
    // in parallel without constraint conflicts.
    // the existing CPU path. This allows us to:
    // - Verify that the build system correctly compiles and links CUDA code;
    // - Validate backend switching via VBDConfig::backend;
    // - Introduce and test device data structures and kernels gradually.
    //
    // NOTE: We intentionally do not call step() here to avoid recursion.
    // Instead we duplicate the high-level structure; later, shared parts
    // can be factored into helper functions when we start to move the
    // heavy AVBD iterations to CUDA kernels.

    const float dt = config_.dt;
    const Vec3f gravity = config_.gravity;
    const int n = model.num_bodies();

    if (state.transforms.size() != static_cast<size_t>(n)) return;

    inertial_positions_.resize(n);
    inertial_rotations_.resize(n);
    initial_positions_.resize(n);
    initial_rotations_.resize(n);
    if (prev_linear_velocities_.size() != static_cast<size_t>(n))
        prev_linear_velocities_ = state.linear_velocities;

    for (int i = 0; i < n; ++i) {
        initial_positions_[i] = state.transforms[i].position;
        initial_rotations_[i] = state.transforms[i].rotation;
    }

    // One-time CUDA warmup: first CUDA usage can pay ~100ms for context/JIT.
    // Do it before starting profiling timers so it doesn't pollute max frame time.
    static thread_local bool cuda_warmed_up = false;
    if (!cuda_warmed_up) {
        cudaFree(0);
        cudaDeviceSynchronize();
        cuda_warmed_up = true;
    }

    // 1) Collision: broadphase + narrowphase + warmstart.
    const bool profile = (std::getenv("NOVAPHY_VBD_PROFILE") != nullptr);
    const char* spike_env = std::getenv("NOVAPHY_VBD_PROFILE_SPIKE_MS");
    const double spike_ms = spike_env ? std::strtod(spike_env, nullptr) : 0.0;
    static thread_local int profile_step = 0;
    auto t_step0 = std::chrono::high_resolution_clock::now();
    auto t_prof0 = t_step0;
    double ms_broadphase = 0.0;
    double ms_upload_shapes = 0.0;
    float ms_narrowphase_kernel = 0.0f;
    double ms_d2h_raw = 0.0;
    double ms_build_contacts = 0.0;
    double ms_step_total = 0.0;
    int prof_pairs = 0;
    int prof_raw_contacts = 0;
    //    For large scenes, GPU all-pairs produces O(N^2) candidates; copying them
    //    back and running narrowphase on CPU for each causes severe stutter. So we
    //    use CPU SAP (fewer pairs) when shape count exceeds threshold.
    const int num_shapes = model.num_shapes();
    const int nJoints = static_cast<int>(joints_.size());
    const int nSprings = static_cast<int>(springs_.size());
    constexpr int GPU_COLLISION_SHAPE_THRESHOLD = 2000;  // above this, use CPU SAP (narrowphase is on GPU so no host stutter)
    const bool use_gpu_collision = (num_shapes > 0 && num_shapes <= GPU_COLLISION_SHAPE_THRESHOLD);

    ensure_cuda_buffers(n, 0, nJoints, nSprings, use_gpu_collision ? num_shapes : 0);

    if (use_gpu_collision && s_cuda.d_shapes && s_cuda.d_aabbs && s_cuda.d_pairs && s_cuda.d_pair_count) {
        // Upload current state for AABB computation.
        DeviceVec3* h_pos = s_cuda.h_pinned_positions;
        DeviceQuat* h_rot = s_cuda.h_pinned_rotations;
        if (h_pos && h_rot && n > 0) {
            for (int i = 0; i < n; ++i) {
                h_pos[i] = to_device(state.transforms[i].position);
                h_rot[i] = to_device(state.transforms[i].rotation);
            }
            cudaMemcpy(s_cuda.d_positions, h_pos, n * sizeof(DeviceVec3), cudaMemcpyHostToDevice);
            cudaMemcpy(s_cuda.d_rotations, h_rot, n * sizeof(DeviceQuat), cudaMemcpyHostToDevice);
        }

        DeviceShape* h_shapes = s_cuda.h_pinned_shapes;
        int* h_shape_static = s_cuda.h_pinned_shape_static;
        if (!h_shapes || !h_shape_static) {
            build_contact_constraints(model, state);
            goto after_collision;
        }
        auto t_upload0 = std::chrono::high_resolution_clock::now();
        for (int si = 0; si < num_shapes; ++si) {
            const auto& shape = model.shapes[si];
            DeviceShape& ds = h_shapes[si];
            ds.body_index = shape.body_index;
            ds.type = (shape.type == ShapeType::Box) ? 0 : (shape.type == ShapeType::Plane) ? 1 : (shape.type == ShapeType::Sphere) ? 2 : 0;
            ds.is_static = (shape.body_index >= 0 && shape.body_index < n && model.bodies[shape.body_index].is_static()) ? 1
                : (shape.body_index < 0) ? 1 : 0;
            ds.half[0] = shape.box.half_extents.x();
            ds.half[1] = shape.box.half_extents.y();
            ds.half[2] = shape.box.half_extents.z();
            ds.radius = shape.sphere.radius;
            ds.plane_n[0] = shape.plane.normal.x();
            ds.plane_n[1] = shape.plane.normal.y();
            ds.plane_n[2] = shape.plane.normal.z();
            ds.plane_d = shape.plane.offset;
            ds.local_pos[0] = shape.local_transform.position.x();
            ds.local_pos[1] = shape.local_transform.position.y();
            ds.local_pos[2] = shape.local_transform.position.z();
            const auto& q = shape.local_transform.rotation;
            ds.local_quat[0] = q.w();
            ds.local_quat[1] = q.x();
            ds.local_quat[2] = q.y();
            ds.local_quat[3] = q.z();
            ds.friction = shape.friction;
            h_shape_static[si] = ds.is_static;
        }
        cudaMemcpy(s_cuda.d_shapes, h_shapes, num_shapes * sizeof(DeviceShape), cudaMemcpyHostToDevice);
        cudaMemcpy(s_cuda.d_shape_static, h_shape_static, num_shapes * sizeof(int), cudaMemcpyHostToDevice);
        auto t_upload1 = std::chrono::high_resolution_clock::now();
        ms_upload_shapes = std::chrono::duration<double, std::milli>(t_upload1 - t_upload0).count();

        // Broadphase on GPU: spatial hash / uniform grid (no misses).
        // Step 1: compute AABBs on GPU for all shapes.
        const float cell_size = []() {
            const char* s = std::getenv("NOVAPHY_VBD_CELL_SIZE");
            if (!s) return 1.0f;
            float v = std::strtof(s, nullptr);
            return (v > 1e-6f) ? v : 1.0f;
        }();
        const float inv_cell = 1.0f / cell_size;

        const int threadsAabb = 256;
        const int blocksAabb = (num_shapes + threadsAabb - 1) / threadsAabb;
        compute_shape_aabbs_kernel<<<blocksAabb, threadsAabb>>>(
            s_cuda.d_shapes, s_cuda.d_positions, s_cuda.d_rotations,
            num_shapes, n, s_cuda.d_aabbs);

        // Step 2: count cell entries per shape, scan to offsets, fill entries.
        auto t_bp0 = std::chrono::high_resolution_clock::now();
        count_cell_entries_kernel<<<blocksAabb, threadsAabb>>>(
            s_cuda.d_aabbs, s_cuda.d_shapes, num_shapes, inv_cell, s_cuda.d_cell_counts);
        cub::DeviceScan::ExclusiveSum(s_cuda.d_scan_temp, s_cuda.scan_temp_bytes,
                                     s_cuda.d_cell_counts, s_cuda.d_cell_offsets, num_shapes);
        // Read back total entries: last offset + last count.
        int last_count = 0, last_off = 0;
        cudaMemcpy(&last_count, s_cuda.d_cell_counts + (num_shapes - 1), sizeof(int), cudaMemcpyDeviceToHost);
        cudaMemcpy(&last_off, s_cuda.d_cell_offsets + (num_shapes - 1), sizeof(int), cudaMemcpyDeviceToHost);
        int n_entries = last_off + last_count;
        if (n_entries <= 0 || n_entries > s_cuda.cap_cell_entries) {
            build_contact_constraints(model, state);
            goto after_collision;
        }
        fill_cell_entries_kernel<<<blocksAabb, threadsAabb>>>(
            s_cuda.d_aabbs, s_cuda.d_shapes, num_shapes, inv_cell,
            s_cuda.d_cell_offsets, s_cuda.d_cell_keys, s_cuda.d_cell_shapes);

        // Step 3: sort by cell key.
        cub::DeviceRadixSort::SortPairs(s_cuda.d_sort_temp, s_cuda.sort_temp_bytes,
                                       s_cuda.d_cell_keys, s_cuda.d_cell_keys_sorted,
                                       s_cuda.d_cell_shapes, s_cuda.d_cell_shapes_sorted,
                                       n_entries);

        // Step 4: emit pairs from each cell run.
        cudaMemset(s_cuda.d_pair_count, 0, sizeof(int));
        const int threadsEmit = 256;
        const int blocksEmit = (n_entries + threadsEmit - 1) / threadsEmit;
        emit_pairs_from_cells_kernel<<<blocksEmit, threadsEmit>>>(
            s_cuda.d_cell_keys_sorted, s_cuda.d_cell_shapes_sorted, n_entries,
            s_cuda.d_shape_static, s_cuda.d_pair_count, s_cuda.d_pairs, s_cuda.cap_pairs);

        int n_pairs = 0;
        cudaMemcpy(&n_pairs, s_cuda.d_pair_count, sizeof(int), cudaMemcpyDeviceToHost);
        auto t_bp1 = std::chrono::high_resolution_clock::now();
        ms_broadphase = std::chrono::duration<double, std::milli>(t_bp1 - t_bp0).count();
        prof_pairs = n_pairs;
        if (n_pairs <= 0 || n_pairs > s_cuda.cap_pairs) {
            build_contact_constraints(model, state);
            goto after_collision;
        }

        if (n_pairs > 0 && n_pairs <= s_cuda.cap_pairs && s_cuda.d_raw_contacts && s_cuda.d_narrowphase_contact_count) {
            cudaMemset(s_cuda.d_narrowphase_contact_count, 0, sizeof(int));
            cudaEvent_t ev0, ev1;
            cudaEventCreate(&ev0);
            cudaEventCreate(&ev1);
            cudaEventRecord(ev0);
            const int threadsNp = 256;
            const int blocksNp = (n_pairs + threadsNp - 1) / threadsNp;
            narrowphase_kernel<<<blocksNp, threadsNp>>>(
                s_cuda.d_pairs, n_pairs,
                s_cuda.d_shapes, s_cuda.d_positions, s_cuda.d_rotations, n,
                s_cuda.d_narrowphase_contact_count, s_cuda.d_raw_contacts, s_cuda.cap_raw_contacts);
            cudaEventRecord(ev1);
            cudaEventSynchronize(ev1);
            cudaEventElapsedTime(&ms_narrowphase_kernel, ev0, ev1);
            cudaEventDestroy(ev0);
            cudaEventDestroy(ev1);

            auto t_d2h0 = std::chrono::high_resolution_clock::now();
            int n_contacts_raw = 0;
            cudaMemcpy(&n_contacts_raw, s_cuda.d_narrowphase_contact_count, sizeof(int), cudaMemcpyDeviceToHost);
            int n_contacts = n_contacts_raw;
            // Optional GPU-side dedup / compaction of RawContact, to reduce host work.
            compress_raw_contacts_on_gpu(n_contacts_raw, &n_contacts);
            prof_raw_contacts = n_contacts;
            if (n_contacts > 0 && n_contacts <= s_cuda.cap_raw_contacts) {
                RawContactWarmstart* h_warmstart = s_cuda.h_pinned_raw_warmstart;
                if (!h_warmstart) {
                    build_contact_constraints(model, state);
                } else {
                    // GPU warmstart lookup (prev frame keys + binary search).
                    const int threadsWs = 256;
                    const int blocksWs = (n_contacts + threadsWs - 1) / threadsWs;
                    if (s_cuda.n_prev_contacts > 0 && s_cuda.d_prev_contact_keys_sorted && s_cuda.d_prev_contacts_sorted) {
                        warmstart_lookup_kernel<<<blocksWs, threadsWs>>>(
                            s_cuda.d_raw_contacts, n_contacts,
                            s_cuda.d_prev_contact_keys_sorted, s_cuda.d_prev_contacts_sorted,
                            s_cuda.n_prev_contacts, s_cuda.d_raw_contacts_warmstart);
                    } else {
                        warmstart_lookup_kernel<<<blocksWs, threadsWs>>>(
                            s_cuda.d_raw_contacts, n_contacts,
                            nullptr, nullptr, 0, s_cuda.d_raw_contacts_warmstart);
                    }

                    // Step 3: full GPU build (ignore filter + C0 + sort); else Step 2: D2H warmstart + CPU C0 + sort.
                    const bool use_full_gpu_build = (s_cuda.d_raw_warmstart_compact != nullptr && s_cuda.d_contacts_sorted != nullptr);
                    bool did_full_gpu_build = false;
                    if (use_full_gpu_build) {
                        std::vector<uint64_t> h_ignore_keys;
                        for (const auto& ic : ignore_collisions_) {
                            if (ic.body_a >= 0 && ic.body_b >= 0)
                                h_ignore_keys.push_back(make_pair_key_gpu(ic.body_a, ic.body_b));
                        }
                        int n_ignore = static_cast<int>(h_ignore_keys.size());
                        if (n_ignore > s_cuda.cap_ignore_pair_keys) {
                            if (s_cuda.d_ignore_pair_keys) cudaFree(s_cuda.d_ignore_pair_keys);
                            s_cuda.d_ignore_pair_keys = nullptr;
                            if (n_ignore > 0)
                                cudaMalloc(&s_cuda.d_ignore_pair_keys, n_ignore * sizeof(uint64_t));
                            s_cuda.cap_ignore_pair_keys = n_ignore;
                        }
                        s_cuda.n_ignore_pair_keys = n_ignore;
                        if (n_ignore > 0 && s_cuda.d_ignore_pair_keys)
                            cudaMemcpy(s_cuda.d_ignore_pair_keys, h_ignore_keys.data(), n_ignore * sizeof(uint64_t), cudaMemcpyHostToDevice);

                        filter_ignore_flags_kernel<<<blocksWs, threadsWs>>>(
                            s_cuda.d_raw_contacts_warmstart, n_contacts,
                            s_cuda.d_ignore_pair_keys ? s_cuda.d_ignore_pair_keys : nullptr,
                            n_ignore, s_cuda.d_unique_flags);
                        size_t scan_bytes_f = 0;
                        cub::DeviceScan::ExclusiveSum(nullptr, scan_bytes_f,
                                                     s_cuda.d_unique_flags, s_cuda.d_unique_indices, n_contacts);
                        if (scan_bytes_f > s_cuda.scan_temp_bytes) {
                            if (s_cuda.d_scan_temp) cudaFree(s_cuda.d_scan_temp);
                            cudaMalloc(&s_cuda.d_scan_temp, scan_bytes_f);
                            s_cuda.scan_temp_bytes = scan_bytes_f;
                        }
                        cub::DeviceScan::ExclusiveSum(s_cuda.d_scan_temp, s_cuda.scan_temp_bytes,
                                                     s_cuda.d_unique_flags, s_cuda.d_unique_indices, n_contacts);
                        int last_f = 0, last_idx = 0;
                        cudaMemcpy(&last_f, s_cuda.d_unique_flags + (n_contacts - 1), sizeof(int), cudaMemcpyDeviceToHost);
                        cudaMemcpy(&last_idx, s_cuda.d_unique_indices + (n_contacts - 1), sizeof(int), cudaMemcpyDeviceToHost);
                        int n_valid = last_idx + last_f;
                        if (n_valid > 0 && n_valid <= s_cuda.cap_contacts) {
                            compact_warmstart_kernel<<<blocksWs, threadsWs>>>(
                                s_cuda.d_raw_contacts_warmstart, s_cuda.d_unique_flags, s_cuda.d_unique_indices,
                                n_contacts, s_cuda.d_raw_warmstart_compact);
                            const int threadsC0 = 256;
                            const int blocksC0 = (n_valid + threadsC0 - 1) / threadsC0;
                            build_contact_c0_kernel<<<blocksC0, threadsC0>>>(
                                s_cuda.d_raw_warmstart_compact, n_valid,
                                s_cuda.d_positions, s_cuda.d_rotations, n,
                                config_.alpha, config_.gamma, COLLISION_MARGIN,
                                s_cuda.d_contacts);
                            build_contact_keys_from_device_kernel<<<blocksC0, threadsC0>>>(s_cuda.d_contacts, n_valid, s_cuda.d_contact_keys);
                            fill_indices_kernel<<<blocksC0, threadsC0>>>(s_cuda.d_unique_indices, n_valid);
                            size_t sort_bytes_c = 0;
                            cub::DeviceRadixSort::SortPairs(nullptr, sort_bytes_c,
                                                           s_cuda.d_contact_keys, s_cuda.d_contact_keys_sorted,
                                                           s_cuda.d_unique_indices, s_cuda.d_contact_indices,
                                                           n_valid);
                            if (sort_bytes_c > s_cuda.sort_temp_bytes) {
                                if (s_cuda.d_sort_temp) cudaFree(s_cuda.d_sort_temp);
                                cudaMalloc(&s_cuda.d_sort_temp, sort_bytes_c);
                                s_cuda.sort_temp_bytes = sort_bytes_c;
                            }
                            cub::DeviceRadixSort::SortPairs(s_cuda.d_sort_temp, s_cuda.sort_temp_bytes,
                                                           s_cuda.d_contact_keys, s_cuda.d_contact_keys_sorted,
                                                           s_cuda.d_unique_indices, s_cuda.d_contact_indices,
                                                           n_valid);
                            gather_contacts_kernel<<<blocksC0, threadsC0>>>(s_cuda.d_contacts, s_cuda.d_contact_indices, n_valid, s_cuda.d_contacts_sorted);
                            cudaMemcpy(s_cuda.d_contacts, s_cuda.d_contacts_sorted, n_valid * sizeof(DeviceContact), cudaMemcpyDeviceToDevice);
                            avbd_contacts_.resize(static_cast<size_t>(n_valid));
                            cudaMemcpy(s_cuda.h_pinned_contacts, s_cuda.d_contacts, n_valid * sizeof(DeviceContact), cudaMemcpyDeviceToHost);
                            for (int ci = 0; ci < n_valid; ++ci) {
                                const DeviceContact& dc = s_cuda.h_pinned_contacts[ci];
                                AvbdContact& c = avbd_contacts_[static_cast<size_t>(ci)];
                                c.body_a = dc.body_a;
                                c.body_b = dc.body_b;
                                c.rA = Vec3f(dc.rA.x, dc.rA.y, dc.rA.z);
                                c.rB = Vec3f(dc.rB.x, dc.rB.y, dc.rB.z);
                                for (int r = 0; r < 3; ++r)
                                    for (int col = 0; col < 3; ++col)
                                        c.basis(r, col) = dc.basis[3 * r + col];
                                c.C0 = Vec3f(dc.C0[0], dc.C0[1], dc.C0[2]);
                                c.penalty = Vec3f(dc.penalty[0], dc.penalty[1], dc.penalty[2]);
                                c.lambda = Vec3f(dc.lambda[0], dc.lambda[1], dc.lambda[2]);
                                c.friction = dc.friction;
                                c.feature_id = dc.feature_id;
                                c.stick = (dc.stick != 0);
                            }
                            auto t_d2h1 = std::chrono::high_resolution_clock::now();
                            ms_d2h_raw = std::chrono::duration<double, std::milli>(t_d2h1 - t_d2h0).count();
                            ms_build_contacts = 0.0;
                            did_full_gpu_build = true;
                        }
                    }
                    if (!did_full_gpu_build) {
                        // Step 2: D2H warmstart and CPU C0 + sort.
                        cudaMemcpy(h_warmstart, s_cuda.d_raw_contacts_warmstart,
                                  n_contacts * sizeof(RawContactWarmstart), cudaMemcpyDeviceToHost);
                        auto t_d2h1 = std::chrono::high_resolution_clock::now();
                        ms_d2h_raw = std::chrono::duration<double, std::milli>(t_d2h1 - t_d2h0).count();
                        auto t_build0 = std::chrono::high_resolution_clock::now();
                        build_contact_constraints_from_raw_contacts_warmstart(
                            model, state,
                            std::span<const RawContactHostWarmstart>(
                                reinterpret_cast<const RawContactHostWarmstart*>(h_warmstart),
                                static_cast<size_t>(n_contacts)));
                        auto t_build1 = std::chrono::high_resolution_clock::now();
                        ms_build_contacts = std::chrono::duration<double, std::milli>(t_build1 - t_build0).count();
                    }
                }
            } else {
                build_contact_constraints(model, state);
            }

            if (profile && (profile_step % 60 == 0)) {
                auto t_now = std::chrono::high_resolution_clock::now();
                double ms_collide_total = std::chrono::duration<double, std::milli>(t_now - t_prof0).count();
                std::printf("[VBD cuda] step=%d shapes=%d pairs=%d raw_contacts=%d collide_ms=%.3f (sap=%.3f uploadShapes=%.3f npKernel=%.3f d2h=%.3f build=%.3f)\n",
                            profile_step, num_shapes, prof_pairs, prof_raw_contacts, ms_collide_total,
                            ms_broadphase, ms_upload_shapes, ms_narrowphase_kernel, ms_d2h_raw, ms_build_contacts);
                std::fflush(stdout);
            }
        } else {
            build_contact_constraints(model, state);
        }
    } else {
        build_contact_constraints(model, state);
    }
after_collision:
    if (profile) ++profile_step;

    if (profile) {
        auto t_step1 = std::chrono::high_resolution_clock::now();
        ms_step_total = std::chrono::duration<double, std::milli>(t_step1 - t_step0).count();
        const bool print_periodic = (profile_step % 60 == 0);
        const bool print_spike = (spike_ms > 0.0 && ms_step_total >= spike_ms);
        if (print_periodic || print_spike) {
            std::printf("[VBD cuda] step=%d total_ms=%.3f (contacts=%d) spike=%d\n",
                        profile_step, ms_step_total, (int)avbd_contacts_.size(), print_spike ? 1 : 0);
            if (print_spike) {
                std::printf("[VBD cuda] spike_detail shapes=%d pairs=%d raw_contacts=%d collide_ms=%.3f (bp=%.3f uploadShapes=%.3f npKernel=%.3f d2h=%.3f build=%.3f)\n",
                            num_shapes, prof_pairs, prof_raw_contacts,
                            std::chrono::duration<double, std::milli>(t_step1 - t_prof0).count(),
                            ms_broadphase, ms_upload_shapes, ms_narrowphase_kernel, ms_d2h_raw, ms_build_contacts);
            }
            std::fflush(stdout);
        }
    }

    // 1.5) Initialize and warmstart joints/springs (same as CPU path).
    std::vector<Vec3f> body_size(static_cast<size_t>(n), Vec3f::Zero());
    for (const auto& shape : model.shapes) {
        if (shape.body_index < 0 || shape.body_index >= n) continue;
        if (shape.type != ShapeType::Box) continue;
        Vec3f full = shape.box.half_extents * 2.0f;
        body_size[static_cast<size_t>(shape.body_index)] =
            body_size[static_cast<size_t>(shape.body_index)].cwiseMax(full);
    }

    for (AvbdJoint& j : joints_) {
        if (j.broken) continue;
        Vec3f szA = (j.body_a >= 0 && j.body_a < n) ? body_size[static_cast<size_t>(j.body_a)] : Vec3f::Zero();
        Vec3f szB = (j.body_b >= 0 && j.body_b < n) ? body_size[static_cast<size_t>(j.body_b)] : Vec3f::Zero();
        j.torqueArm = (szA + szB).squaredNorm();
        if (!(j.torqueArm > 0.0f)) j.torqueArm = 1.0f;

        Vec3f xA = world_point_host(state, j.body_a, j.rA);
        Vec3f xB = world_point_host(state, j.body_b, j.rB);
        j.C0Lin = xA - xB;
        Quatf qA = (j.body_a >= 0 && j.body_a < n) ? state.transforms[j.body_a].rotation : Quatf::Identity();
        Quatf qB = (j.body_b >= 0 && j.body_b < n) ? state.transforms[j.body_b].rotation : Quatf::Identity();
        j.C0Ang = quat_diff_vec_demo3d_host(qA, qB) * j.torqueArm;

        j.lambdaLin = j.lambdaLin * config_.alpha * config_.gamma;
        j.lambdaAng = j.lambdaAng * config_.alpha * config_.gamma;
        j.penaltyLin = (j.penaltyLin * config_.gamma)
                           .cwiseMax(Vec3f(PENALTY_MIN, PENALTY_MIN, PENALTY_MIN))
                           .cwiseMin(Vec3f(PENALTY_MAX, PENALTY_MAX, PENALTY_MAX));
        j.penaltyAng = (j.penaltyAng * config_.gamma)
                           .cwiseMax(Vec3f(PENALTY_MIN, PENALTY_MIN, PENALTY_MIN))
                           .cwiseMin(Vec3f(PENALTY_MAX, PENALTY_MAX, PENALTY_MAX));

        float stiffLin = j.stiffnessLin;
        float stiffAng = j.stiffnessAng;
        if (std::isfinite(stiffLin)) {
            j.penaltyLin = j.penaltyLin.cwiseMin(Vec3f(stiffLin, stiffLin, stiffLin));
        }
        if (std::isfinite(stiffAng)) {
            j.penaltyAng = j.penaltyAng.cwiseMin(Vec3f(stiffAng, stiffAng, stiffAng));
        }
    }

    for (AvbdSpring& s : springs_) {
        if (s.rest < 0.0f && s.body_a >= 0 && s.body_b >= 0 && s.body_a < n && s.body_b < n) {
            Vec3f pA = state.transforms[s.body_a].transform_point(s.rA);
            Vec3f pB = state.transforms[s.body_b].transform_point(s.rB);
            s.rest = (pA - pB).norm();
        }
    }

    // 2) Initialize bodies (inertial state + initial), still on CPU.
    for (int i = 0; i < n; ++i) {
        const auto& body = model.bodies[i];
        if (body.is_static()) continue;

        Vec3f vel = state.linear_velocities[i];
        Vec3f omega = state.angular_velocities[i];

        inertial_positions_[i] = state.transforms[i].position + vel * dt + gravity * (dt * dt);
        inertial_rotations_[i] = quat_add_omega_dt_host(state.transforms[i].rotation, omega, dt);

        float g2 = gravity.squaredNorm();
        float accelWeight = 1.0f;
        if (g2 > 1e-12f && prev_linear_velocities_.size() == static_cast<size_t>(n)) {
            Vec3f accel = (vel - prev_linear_velocities_[i]) / dt;
            accelWeight = novaphy::clampf(accel.dot(gravity) / g2, 0.0f, 1.0f);
            if (!std::isfinite(accelWeight))
                accelWeight = 0.0f;
        }

        state.transforms[i].position =
            state.transforms[i].position + vel * dt + gravity * (accelWeight * dt * dt);
        state.transforms[i].rotation = quat_add_omega_dt_host(state.transforms[i].rotation, omega, dt);
    }

    // Prepare device buffers for bodies, positions/rotations and contacts.
    const int nBodies = n;
    const int nContacts = static_cast<int>(avbd_contacts_.size());
    // nJoints, nSprings already declared above for GPU collision path

    ensure_cuda_buffers(nBodies, nContacts, nJoints, nSprings);
    DeviceBody* d_bodies = s_cuda.d_bodies;
    int* d_body_color = s_cuda.d_body_color;
    DeviceJoint* d_joints = s_cuda.d_joints;
    DeviceSpring* d_springs = s_cuda.d_springs;
    DeviceVec3* d_initial_pos = s_cuda.d_initial_pos;
    DeviceQuat* d_initial_rot = s_cuda.d_initial_rot;
    DeviceVec3* d_inertial_pos = s_cuda.d_inertial_pos;
    DeviceQuat* d_inertial_rot = s_cuda.d_inertial_rot;
    DeviceVec3* d_positions = s_cuda.d_positions;
    DeviceQuat* d_rotations = s_cuda.d_rotations;
    DeviceContact* d_contacts = s_cuda.d_contacts;
    DeviceBody* h_bodies = s_cuda.h_pinned_bodies;
    DeviceVec3* h_initial_pos = s_cuda.h_pinned_initial_pos;
    DeviceQuat* h_initial_rot = s_cuda.h_pinned_initial_rot;
    DeviceVec3* h_inertial_pos = s_cuda.h_pinned_inertial_pos;
    DeviceQuat* h_inertial_rot = s_cuda.h_pinned_inertial_rot;
    DeviceVec3* h_positions = s_cuda.h_pinned_positions;
    DeviceQuat* h_rotations = s_cuda.h_pinned_rotations;
    DeviceContact* h_contacts = s_cuda.h_pinned_contacts;
    DeviceJoint* h_joints = s_cuda.h_pinned_joints;
    DeviceSpring* h_springs = s_cuda.h_pinned_springs;

    for (int i = 0; i < nBodies; ++i) {
        const auto& b = model.bodies[i];
        h_bodies[i].mass = b.mass;
        h_bodies[i].inertia_diag[0] = b.inertia(0, 0);
        h_bodies[i].inertia_diag[1] = b.inertia(1, 1);
        h_bodies[i].inertia_diag[2] = b.inertia(2, 2);
        h_bodies[i].is_static = b.is_static() ? 1 : 0;
        h_initial_pos[i] = to_device(initial_positions_[i]);
        h_initial_rot[i] = to_device(initial_rotations_[i]);
        h_inertial_pos[i] = to_device(inertial_positions_[i]);
        h_inertial_rot[i] = to_device(inertial_rotations_[i]);
        h_positions[i] = to_device(state.transforms[i].position);
        h_rotations[i] = to_device(state.transforms[i].rotation);
    }
    for (int ci = 0; ci < nContacts; ++ci) {
        const AvbdContact& c = avbd_contacts_[ci];
        DeviceContact& dc = h_contacts[ci];
        dc.body_a = c.body_a;
        dc.body_b = c.body_b;
        dc.rA = to_device(c.rA);
        dc.rB = to_device(c.rB);
        for (int r = 0; r < 3; ++r) {
            for (int col = 0; col < 3; ++col) {
                dc.basis[3 * r + col] = c.basis(r, col);
            }
        }
        dc.C0[0] = c.C0.x();
        dc.C0[1] = c.C0.y();
        dc.C0[2] = c.C0.z();
        dc.penalty[0] = c.penalty.x();
        dc.penalty[1] = c.penalty.y();
        dc.penalty[2] = c.penalty.z();
        dc.lambda[0] = c.lambda.x();
        dc.lambda[1] = c.lambda.y();
        dc.lambda[2] = c.lambda.z();
        dc.friction = c.friction;
        dc.feature_id = c.feature_id;
        dc.stick = c.stick ? 1 : 0;
    }
    for (int ji = 0; ji < nJoints; ++ji) {
        const AvbdJoint& j = joints_[ji];
        DeviceJoint& dj = h_joints[ji];
        dj.body_a = j.body_a;
        dj.body_b = j.body_b;
        dj.rA = to_device(j.rA);
        dj.rB = to_device(j.rB);
        dj.C0Lin[0] = j.C0Lin.x(); dj.C0Lin[1] = j.C0Lin.y(); dj.C0Lin[2] = j.C0Lin.z();
        dj.C0Ang[0] = j.C0Ang.x(); dj.C0Ang[1] = j.C0Ang.y(); dj.C0Ang[2] = j.C0Ang.z();
        dj.penaltyLin[0] = j.penaltyLin.x(); dj.penaltyLin[1] = j.penaltyLin.y(); dj.penaltyLin[2] = j.penaltyLin.z();
        dj.penaltyAng[0] = j.penaltyAng.x(); dj.penaltyAng[1] = j.penaltyAng.y(); dj.penaltyAng[2] = j.penaltyAng.z();
        dj.lambdaLin[0] = j.lambdaLin.x(); dj.lambdaLin[1] = j.lambdaLin.y(); dj.lambdaLin[2] = j.lambdaLin.z();
        dj.lambdaAng[0] = j.lambdaAng.x(); dj.lambdaAng[1] = j.lambdaAng.y(); dj.lambdaAng[2] = j.lambdaAng.z();
        dj.stiffnessLin = j.stiffnessLin;
        dj.stiffnessAng = j.stiffnessAng;
        dj.torqueArm = j.torqueArm;
        dj.fracture = j.fracture;
        dj.broken = j.broken ? 1 : 0;
    }
    for (int si = 0; si < nSprings; ++si) {
        const AvbdSpring& s = springs_[si];
        DeviceSpring& ds = h_springs[si];
        ds.body_a = s.body_a;
        ds.body_b = s.body_b;
        ds.rA = to_device(s.rA);
        ds.rB = to_device(s.rB);
        ds.rest = s.rest;
        ds.stiffness = s.stiffness;
    }

    if (nBodies > 0) {
        cudaMemcpy(d_bodies, h_bodies, nBodies * sizeof(DeviceBody), cudaMemcpyHostToDevice);
        cudaMemcpy(d_initial_pos, h_initial_pos, nBodies * sizeof(DeviceVec3), cudaMemcpyHostToDevice);
        cudaMemcpy(d_initial_rot, h_initial_rot, nBodies * sizeof(DeviceQuat), cudaMemcpyHostToDevice);
        cudaMemcpy(d_inertial_pos, h_inertial_pos, nBodies * sizeof(DeviceVec3), cudaMemcpyHostToDevice);
        cudaMemcpy(d_inertial_rot, h_inertial_rot, nBodies * sizeof(DeviceQuat), cudaMemcpyHostToDevice);
        cudaMemcpy(d_positions, h_positions, nBodies * sizeof(DeviceVec3), cudaMemcpyHostToDevice);
        cudaMemcpy(d_rotations, h_rotations, nBodies * sizeof(DeviceQuat), cudaMemcpyHostToDevice);
    }
    if (nContacts > 0)
        cudaMemcpy(d_contacts, h_contacts, nContacts * sizeof(DeviceContact), cudaMemcpyHostToDevice);
    if (nJoints > 0)
        cudaMemcpy(d_joints, h_joints, nJoints * sizeof(DeviceJoint), cudaMemcpyHostToDevice);
    if (nSprings > 0)
        cudaMemcpy(d_springs, h_springs, nSprings * sizeof(DeviceSpring), cudaMemcpyHostToDevice);

    // Vertex coloring: bodies that share a contact/joint/spring get different colors.
    std::vector<int> h_body_color(static_cast<size_t>(nBodies), 0);
    int num_colors = 1;
    if (nBodies > 0 && (nContacts > 0 || nJoints > 0 || nSprings > 0)) {
        compute_body_colors(nBodies, avbd_contacts_, joints_, springs_, &h_body_color, &num_colors);
        if (num_colors <= 0) num_colors = 1;
    }
    if (nBodies > 0 && d_body_color != nullptr)
        cudaMemcpy(d_body_color, h_body_color.data(), nBodies * sizeof(int), cudaMemcpyHostToDevice);
    else if (nBodies > 0)
        num_colors = 1;

    // 3) Main solver loop: per-color primal (contacts + joints + springs) then dual.
    //    Use cooperative-group "all colors in one launch" when supported to cut
    //    kernel launch overhead (critical for 100k+ bodies and many colors).
    const int threadsBody = 128;
    const int blocksBody = (nBodies > 0) ? (nBodies + threadsBody - 1) / threadsBody : 0;
    const int blocksBodyDual = (nBodies > 0 && nContacts > 0) ? (nBodies + threadsBody - 1) / threadsBody : 0;
    const int blocksJ = (nJoints > 0) ? (nJoints + 63) / 64 : 0;

    bool use_cooperative = false;
    if (nBodies > 0 && num_colors > 1 && d_body_color != nullptr && blocksBody > 0) {
        cudaDeviceProp prop;
        if (cudaGetDeviceProperties(&prop, 0) == cudaSuccess) {
            int coop_supported = 0;
            cudaDeviceGetAttribute(&coop_supported, cudaDevAttrCooperativeLaunch, 0);
            if (coop_supported && blocksBody <= prop.maxGridSize[0]) {
                use_cooperative = true;
            }
        }
    }

    int nBodies_arg = nBodies;
    int nContacts_arg = nContacts;
    int nJoints_arg = nJoints;
    int nSprings_arg = nSprings;
    float dt_arg = config_.dt;
    float alpha_arg = config_.alpha;
    float lhs_reg_arg = config_.lhs_regularization;
    float relax_arg = config_.primal_relaxation;

    for (int it = 0; it < config_.iterations; ++it) {
        if (nBodies > 0) {
            if (use_cooperative) {
                void* args[] = {&nBodies_arg, &d_bodies, &d_body_color, &num_colors,
                    &d_initial_pos, &d_initial_rot, &d_inertial_pos, &d_inertial_rot,
                    &d_positions, &d_rotations, &d_contacts, &nContacts_arg,
                    &d_joints, &nJoints_arg, &d_springs, &nSprings_arg,
                    &dt_arg, &alpha_arg, &lhs_reg_arg, &relax_arg};
                cudaError_t err = cudaLaunchCooperativeKernel(
                    (void*)avbd_primal_all_colors_kernel,
                    dim3(blocksBody), dim3(threadsBody), args, 0, 0);
                if (err != cudaSuccess) {
                    use_cooperative = false;
                    for (int c = 0; c < num_colors; ++c) {
                        avbd_primal_contacts_kernel<<<blocksBody, threadsBody>>>(
                            nBodies, d_bodies, d_body_color, c,
                            d_initial_pos, d_initial_rot, d_inertial_pos, d_inertial_rot,
                            d_positions, d_rotations, d_contacts, nContacts,
                            d_joints, nJoints, d_springs, nSprings,
                            config_.dt, config_.alpha, config_.lhs_regularization, config_.primal_relaxation);
                    }
                }
            } else {
                for (int c = 0; c < num_colors; ++c) {
                    avbd_primal_contacts_kernel<<<blocksBody, threadsBody>>>(
                        nBodies,
                        d_bodies,
                        d_body_color,
                        c,
                        d_initial_pos,
                        d_initial_rot,
                        d_inertial_pos,
                        d_inertial_rot,
                        d_positions,
                        d_rotations,
                        d_contacts,
                        nContacts,
                        d_joints,
                        nJoints,
                        d_springs,
                        nSprings,
                        config_.dt,
                        config_.alpha,
                        config_.lhs_regularization,
                        config_.primal_relaxation);
                }
            }
        }
        if (nBodies > 0 && nContacts > 0) {
            avbd_dual_contacts_per_body_kernel<<<blocksBodyDual, threadsBody>>>(
                nBodies,
                d_bodies,
                nContacts,
                d_contacts,
                d_initial_pos,
                d_initial_rot,
                d_positions,
                d_rotations,
                config_.alpha,
                config_.beta_linear);
        }
        if (nJoints > 0) {
            avbd_dual_joints_kernel<<<blocksJ, 64>>>(
                nJoints,
                d_joints,
                d_positions,
                d_rotations,
                d_bodies,
                nBodies,
                config_.alpha,
                config_.beta_linear,
                config_.beta_angular);
        }
    }
    cudaDeviceSynchronize();

    // 4) Copy final device state back to host once (D2H to pinned, then to state).
    if (nBodies > 0) {
        cudaMemcpy(h_positions, d_positions, nBodies * sizeof(DeviceVec3), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_rotations, d_rotations, nBodies * sizeof(DeviceQuat), cudaMemcpyDeviceToHost);
        for (int i = 0; i < nBodies; ++i) {
            state.transforms[i].position = to_host(h_positions[i]);
            state.transforms[i].rotation = Quatf(h_rotations[i].w, h_rotations[i].x,
                                                 h_rotations[i].y, h_rotations[i].z);
        }
    }
    if (nContacts > 0) {
        cudaMemcpy(h_contacts, d_contacts, nContacts * sizeof(DeviceContact), cudaMemcpyDeviceToHost);
        for (int ci = 0; ci < nContacts; ++ci) {
            const DeviceContact& dc = h_contacts[ci];
            AvbdContact& c = avbd_contacts_[ci];
            c.penalty = Vec3f(dc.penalty[0], dc.penalty[1], dc.penalty[2]);
            c.lambda = Vec3f(dc.lambda[0], dc.lambda[1], dc.lambda[2]);
            c.stick = (dc.stick != 0);
        }
        // Save current contacts as prev frame for GPU warmstart (key + sorted copy).
        if (s_cuda.d_prev_contacts && s_cuda.cap_prev_contacts >= nContacts) {
            cudaMemcpy(s_cuda.d_prev_contacts, d_contacts, nContacts * sizeof(DeviceContact), cudaMemcpyDeviceToDevice);
            const int threadsP = 256;
            const int blocksP = (nContacts + threadsP - 1) / threadsP;
            build_contact_keys_from_device_kernel<<<blocksP, threadsP>>>(s_cuda.d_prev_contacts, nContacts, s_cuda.d_prev_contact_keys);
            fill_indices_kernel<<<blocksP, threadsP>>>(s_cuda.d_contact_indices, nContacts);
            size_t sort_bytes_prev = 0;
            cub::DeviceRadixSort::SortPairs(nullptr, sort_bytes_prev,
                                           s_cuda.d_prev_contact_keys, s_cuda.d_prev_contact_keys_sorted,
                                           s_cuda.d_contact_indices, s_cuda.d_prev_contact_indices,
                                           nContacts);
            if (sort_bytes_prev > s_cuda.sort_temp_bytes) {
                if (s_cuda.d_sort_temp) cudaFree(s_cuda.d_sort_temp);
                cudaMalloc(&s_cuda.d_sort_temp, sort_bytes_prev);
                s_cuda.sort_temp_bytes = sort_bytes_prev;
            }
            cub::DeviceRadixSort::SortPairs(s_cuda.d_sort_temp, s_cuda.sort_temp_bytes,
                                           s_cuda.d_prev_contact_keys, s_cuda.d_prev_contact_keys_sorted,
                                           s_cuda.d_contact_indices, s_cuda.d_prev_contact_indices,
                                           nContacts);
            gather_contacts_kernel<<<blocksP, threadsP>>>(s_cuda.d_prev_contacts, s_cuda.d_prev_contact_indices, nContacts, s_cuda.d_prev_contacts_sorted);
            cudaMemcpy(s_cuda.d_prev_contacts, s_cuda.d_prev_contacts_sorted, nContacts * sizeof(DeviceContact), cudaMemcpyDeviceToDevice);
            s_cuda.n_prev_contacts = nContacts;
        }
    } else {
        s_cuda.n_prev_contacts = 0;
    }
    if (nJoints > 0) {
        cudaMemcpy(h_joints, d_joints, nJoints * sizeof(DeviceJoint), cudaMemcpyDeviceToHost);
        for (int ji = 0; ji < nJoints; ++ji) {
            const DeviceJoint& dj = h_joints[ji];
            AvbdJoint& j = joints_[static_cast<size_t>(ji)];
            j.penaltyLin = Vec3f(dj.penaltyLin[0], dj.penaltyLin[1], dj.penaltyLin[2]);
            j.penaltyAng = Vec3f(dj.penaltyAng[0], dj.penaltyAng[1], dj.penaltyAng[2]);
            j.lambdaLin = Vec3f(dj.lambdaLin[0], dj.lambdaLin[1], dj.lambdaLin[2]);
            j.lambdaAng = Vec3f(dj.lambdaAng[0], dj.lambdaAng[1], dj.lambdaAng[2]);
            j.broken = (dj.broken != 0);
        }
    }

    // 5) BDF1 velocities (same as CPU path).
    for (int i = 0; i < n; ++i) {
        if (model.bodies[i].is_static()) continue;
        prev_linear_velocities_[i] = state.linear_velocities[i];
        state.linear_velocities[i] = (state.transforms[i].position - initial_positions_[i]) / dt;
        state.angular_velocities[i] = angular_velocity_from_quat_diff_host(
            state.transforms[i].rotation, initial_rotations_[i], dt);
    }
}

void VbdSolver::release_cuda_buffers() {
    if (s_cuda.h_pinned_raw_warmstart) { cudaFreeHost(s_cuda.h_pinned_raw_warmstart); s_cuda.h_pinned_raw_warmstart = nullptr; }
    if (s_cuda.d_raw_contacts_warmstart) { cudaFree(s_cuda.d_raw_contacts_warmstart); s_cuda.d_raw_contacts_warmstart = nullptr; }
    if (s_cuda.d_prev_contacts_sorted) { cudaFree(s_cuda.d_prev_contacts_sorted); s_cuda.d_prev_contacts_sorted = nullptr; }
    if (s_cuda.d_prev_contact_indices) { cudaFree(s_cuda.d_prev_contact_indices); s_cuda.d_prev_contact_indices = nullptr; }
    if (s_cuda.d_prev_contact_keys_sorted) { cudaFree(s_cuda.d_prev_contact_keys_sorted); s_cuda.d_prev_contact_keys_sorted = nullptr; }
    if (s_cuda.d_prev_contact_keys) { cudaFree(s_cuda.d_prev_contact_keys); s_cuda.d_prev_contact_keys = nullptr; }
    if (s_cuda.d_prev_contacts) { cudaFree(s_cuda.d_prev_contacts); s_cuda.d_prev_contacts = nullptr; }
    if (s_cuda.d_contacts_sorted) { cudaFree(s_cuda.d_contacts_sorted); s_cuda.d_contacts_sorted = nullptr; }
    if (s_cuda.d_contact_indices) { cudaFree(s_cuda.d_contact_indices); s_cuda.d_contact_indices = nullptr; }
    if (s_cuda.d_raw_warmstart_compact) { cudaFree(s_cuda.d_raw_warmstart_compact); s_cuda.d_raw_warmstart_compact = nullptr; }
    if (s_cuda.d_ignore_pair_keys) { cudaFree(s_cuda.d_ignore_pair_keys); s_cuda.d_ignore_pair_keys = nullptr; }
    s_cuda.n_prev_contacts = 0;
    s_cuda.cap_prev_contacts = 0;
    s_cuda.n_ignore_pair_keys = 0;
    s_cuda.cap_ignore_pair_keys = 0;
    if (s_cuda.h_pinned_raw_contacts) { cudaFreeHost(s_cuda.h_pinned_raw_contacts); s_cuda.h_pinned_raw_contacts = nullptr; }
    if (s_cuda.h_pinned_shape_static) { cudaFreeHost(s_cuda.h_pinned_shape_static); s_cuda.h_pinned_shape_static = nullptr; }
    if (s_cuda.h_pinned_shapes) { cudaFreeHost(s_cuda.h_pinned_shapes); s_cuda.h_pinned_shapes = nullptr; }
    if (s_cuda.h_pinned_springs) { cudaFreeHost(s_cuda.h_pinned_springs); s_cuda.h_pinned_springs = nullptr; }
    if (s_cuda.h_pinned_joints) { cudaFreeHost(s_cuda.h_pinned_joints); s_cuda.h_pinned_joints = nullptr; }
    if (s_cuda.h_pinned_contacts) { cudaFreeHost(s_cuda.h_pinned_contacts); s_cuda.h_pinned_contacts = nullptr; }
    if (s_cuda.h_pinned_rotations) { cudaFreeHost(s_cuda.h_pinned_rotations); s_cuda.h_pinned_rotations = nullptr; }
    if (s_cuda.h_pinned_positions) { cudaFreeHost(s_cuda.h_pinned_positions); s_cuda.h_pinned_positions = nullptr; }
    if (s_cuda.h_pinned_inertial_rot) { cudaFreeHost(s_cuda.h_pinned_inertial_rot); s_cuda.h_pinned_inertial_rot = nullptr; }
    if (s_cuda.h_pinned_inertial_pos) { cudaFreeHost(s_cuda.h_pinned_inertial_pos); s_cuda.h_pinned_inertial_pos = nullptr; }
    if (s_cuda.h_pinned_initial_rot) { cudaFreeHost(s_cuda.h_pinned_initial_rot); s_cuda.h_pinned_initial_rot = nullptr; }
    if (s_cuda.h_pinned_initial_pos) { cudaFreeHost(s_cuda.h_pinned_initial_pos); s_cuda.h_pinned_initial_pos = nullptr; }
    if (s_cuda.h_pinned_bodies) { cudaFreeHost(s_cuda.h_pinned_bodies); s_cuda.h_pinned_bodies = nullptr; }
    if (s_cuda.d_springs) { cudaFree(s_cuda.d_springs); s_cuda.d_springs = nullptr; }
    if (s_cuda.d_joints) { cudaFree(s_cuda.d_joints); s_cuda.d_joints = nullptr; }
    if (s_cuda.d_body_color) { cudaFree(s_cuda.d_body_color); s_cuda.d_body_color = nullptr; }
    if (s_cuda.d_contacts) { cudaFree(s_cuda.d_contacts); s_cuda.d_contacts = nullptr; }
    if (s_cuda.d_rotations) { cudaFree(s_cuda.d_rotations); s_cuda.d_rotations = nullptr; }
    if (s_cuda.d_positions) { cudaFree(s_cuda.d_positions); s_cuda.d_positions = nullptr; }
    if (s_cuda.d_inertial_rot) { cudaFree(s_cuda.d_inertial_rot); s_cuda.d_inertial_rot = nullptr; }
    if (s_cuda.d_inertial_pos) { cudaFree(s_cuda.d_inertial_pos); s_cuda.d_inertial_pos = nullptr; }
    if (s_cuda.d_initial_rot) { cudaFree(s_cuda.d_initial_rot); s_cuda.d_initial_rot = nullptr; }
    if (s_cuda.d_initial_pos) { cudaFree(s_cuda.d_initial_pos); s_cuda.d_initial_pos = nullptr; }
    if (s_cuda.d_bodies) { cudaFree(s_cuda.d_bodies); s_cuda.d_bodies = nullptr; }
    if (s_cuda.d_narrowphase_contact_count) { cudaFree(s_cuda.d_narrowphase_contact_count); s_cuda.d_narrowphase_contact_count = nullptr; }
    if (s_cuda.d_raw_contacts) { cudaFree(s_cuda.d_raw_contacts); s_cuda.d_raw_contacts = nullptr; }
    if (s_cuda.d_unique_indices) { cudaFree(s_cuda.d_unique_indices); s_cuda.d_unique_indices = nullptr; }
    if (s_cuda.d_unique_flags) { cudaFree(s_cuda.d_unique_flags); s_cuda.d_unique_flags = nullptr; }
    if (s_cuda.d_raw_sorted) { cudaFree(s_cuda.d_raw_sorted); s_cuda.d_raw_sorted = nullptr; }
    if (s_cuda.d_contact_keys_sorted) { cudaFree(s_cuda.d_contact_keys_sorted); s_cuda.d_contact_keys_sorted = nullptr; }
    if (s_cuda.d_contact_keys) { cudaFree(s_cuda.d_contact_keys); s_cuda.d_contact_keys = nullptr; }
    if (s_cuda.d_pair_count) { cudaFree(s_cuda.d_pair_count); s_cuda.d_pair_count = nullptr; }
    if (s_cuda.d_pairs) { cudaFree(s_cuda.d_pairs); s_cuda.d_pairs = nullptr; }
    if (s_cuda.d_sort_temp) { cudaFree(s_cuda.d_sort_temp); s_cuda.d_sort_temp = nullptr; }
    if (s_cuda.d_scan_temp) { cudaFree(s_cuda.d_scan_temp); s_cuda.d_scan_temp = nullptr; }
    if (s_cuda.d_cell_shapes_sorted) { cudaFree(s_cuda.d_cell_shapes_sorted); s_cuda.d_cell_shapes_sorted = nullptr; }
    if (s_cuda.d_cell_keys_sorted) { cudaFree(s_cuda.d_cell_keys_sorted); s_cuda.d_cell_keys_sorted = nullptr; }
    if (s_cuda.d_cell_shapes) { cudaFree(s_cuda.d_cell_shapes); s_cuda.d_cell_shapes = nullptr; }
    if (s_cuda.d_cell_keys) { cudaFree(s_cuda.d_cell_keys); s_cuda.d_cell_keys = nullptr; }
    if (s_cuda.d_cell_offsets) { cudaFree(s_cuda.d_cell_offsets); s_cuda.d_cell_offsets = nullptr; }
    if (s_cuda.d_cell_counts) { cudaFree(s_cuda.d_cell_counts); s_cuda.d_cell_counts = nullptr; }
    if (s_cuda.d_shape_static) { cudaFree(s_cuda.d_shape_static); s_cuda.d_shape_static = nullptr; }
    if (s_cuda.d_aabbs) { cudaFree(s_cuda.d_aabbs); s_cuda.d_aabbs = nullptr; }
    if (s_cuda.d_shapes) { cudaFree(s_cuda.d_shapes); s_cuda.d_shapes = nullptr; }
    s_cuda.cap_bodies = 0;
    s_cuda.cap_contacts = 0;
    s_cuda.cap_joints = 0;
    s_cuda.cap_springs = 0;
    s_cuda.cap_shapes = 0;
    s_cuda.cap_pairs = 0;
    s_cuda.cap_raw_contacts = 0;
    s_cuda.cap_contact_keys = 0;
    s_cuda.cap_cell_entries = 0;
    s_cuda.scan_temp_bytes = 0;
    s_cuda.sort_temp_bytes = 0;
}

}  // namespace novaphy

