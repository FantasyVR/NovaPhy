/**
 * @file vbd_solver.cpp
 * @brief 3D AVBD solver, 完全仿照 avbd-demo3d 的 Solver 流程与公式。
 */
 #include "novaphy/vbd/vbd_solver.h"

 #include "novaphy/math/math_utils.h"
 
 #include <algorithm>
 #include <cstdint>
 #include <cmath>
#include <tuple>
 #include <unordered_map>
 #include <vector>
 
 namespace novaphy {
 
 namespace {
 
 constexpr float PENALTY_MIN = 1.0f;
 constexpr float PENALTY_MAX = 10000000000.0f;
 constexpr float COLLISION_MARGIN = 0.01f;
 constexpr float STICK_THRESH = 0.00001f;
 constexpr float PRIMAL_RELAX = 1.0f;
 
 struct WarmstartContactData {
     Vec3f rA = Vec3f::Zero();
     Vec3f rB = Vec3f::Zero();
     Vec3f penalty = Vec3f::Zero();
     Vec3f lambda = Vec3f::Zero();
     bool stick = false;
 };
 
 inline uint64_t fnv1a_u64(uint64_t h, uint64_t v) {
     // 64-bit FNV-1a
     constexpr uint64_t kPrime = 1099511628211ull;
     h ^= v;
     h *= kPrime;
     return h;
 }
 
 inline int quantize_float(float x, float q) {
     // symmetric rounding to nearest integer
     float s = x / q;
     return static_cast<int>(std::floor(s + (s >= 0.0f ? 0.5f : -0.5f)));
 }
 
 inline int argmax_abs3(const Vec3f& v) {
     const float ax = std::abs(v.x());
     const float ay = std::abs(v.y());
     const float az = std::abs(v.z());
     return (ax >= ay && ax >= az) ? 0 : (ay >= az ? 1 : 2);
 }
 
 inline void face_uv_from_local_point(const Vec3f& r, int axis, float& u, float& v) {
     // Choose the two coordinates spanning the face orthogonal to `axis`.
     if (axis == 0) { u = r.y(); v = r.z(); }
     else if (axis == 1) { u = r.x(); v = r.z(); }
     else { u = r.x(); v = r.y(); }
 }
 
 inline uint64_t contact_key(const AvbdContact& c) {
     // Contact persistence key.
     //
     // demo3d uses a feature key (face/edge ids). We first try to use a similar feature_id
     // forwarded from narrowphase; if unavailable we approximate a stable feature signature
     // from local anchors (rA/rB) using quantized face-local coordinates.
     uint64_t h = 1469598103934665603ull;
     h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(c.body_a + 2)));
     h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(c.body_b + 2)));
 
     if (c.feature_id >= 0) {
         h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(c.feature_id)));
         return h;
     }
 
     constexpr float Q_UV = 0.02f;  // 2cm quantization for face-local coordinates
 
     auto add_face_uv = [&](const Vec3f& r_local) {
         const int axis = argmax_abs3(r_local);
         const int sign = (r_local[axis] >= 0.0f) ? 1 : -1;
         float u = 0.0f, v = 0.0f;
         face_uv_from_local_point(r_local, axis, u, v);
 
         h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(axis)));
         h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(sign > 0 ? 1u : 0u)));
         h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(quantize_float(u, Q_UV))));
         h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(quantize_float(v, Q_UV))));
     };
 
     if (c.body_a == -1 && c.body_b >= 0) {
         // plane/world vs dynamic body
         add_face_uv(c.rB);
     } else if (c.body_b == -1 && c.body_a >= 0) {
         add_face_uv(c.rA);
     } else {
         add_face_uv(c.rA);
         add_face_uv(c.rB);
     }
     return h;
 }
 
/** 与 demo3d maths.h orthonormal(normal) 完全一致：行0=normal，行1/2=两切向。 */
inline Mat3f orthonormal_basis(const Vec3f& normal) {
    Vec3f n = normal.normalized();
    Vec3f t1 = (std::abs(n.x()) > std::abs(n.z()))
                   ? Vec3f(-n.y(), n.x(), 0.0f)
                   : Vec3f(0.0f, -n.z(), n.y());
    t1.normalize();
    Vec3f t2 = n.cross(t1);
    t2.normalize();
    Mat3f basis;
    basis.row(0) = n;
    basis.row(1) = t1;
    basis.row(2) = t2;
    return basis;
}
 
 /** 用世界系角速度积分四元数：q_{t+dt} = exp([ω_world] dt) ⊗ q_t 。 */
 inline Quatf quat_add_omega_dt(const Quatf& q, const Vec3f& omega, float dt) {
     // Match demo3d: first-order quaternion integration using world-space angular velocity.
     // q_dot = 0.5 * [0, ω] ⊗ q
     if (dt <= 0.0f) return q;
     const Quatf omega_q(0.0f, omega.x(), omega.y(), omega.z());
     Quatf q_new = q;
     q_new.coeffs() += (0.5f * dt) * (omega_q * q).coeffs();
     q_new.normalize();
     return q_new;
 }
 
 /** 小角近似：dq = q * q0^{-1}，返回 2*vec(dq)，并做最短弧（w>=0）处理。 */
 inline Vec3f quat_small_angle_diff_vec(const Quatf& q, const Quatf& q0) {
     Quatf dq = q * q0.inverse();
     dq.normalize();
     if (dq.w() < 0.0f) dq.coeffs() *= -1.0f;  // shortest arc
     return 2.0f * Vec3f(dq.x(), dq.y(), dq.z());
 }
 
 /** 角速度由四元数差分得到（BDF1）：omega = (q_now * q_prev^{-1}) 的轴角 / dt。 */
 inline Vec3f angular_velocity_from_quat_diff(const Quatf& q_now, const Quatf& q_prev, float dt) {
     // Match demo3d: use small-angle delta divided by dt.
     if (dt <= 0.0f) return Vec3f::Zero();
     return quat_small_angle_diff_vec(q_now, q_prev) / dt;
 }
 
 }  // namespace
 
 VbdSolver::VbdSolver(const VBDConfig& cfg)
     : config_(cfg) {}
 
 void VbdSolver::set_config(const VBDConfig& cfg) {
     config_ = cfg;
 }
 
 void VbdSolver::set_model(const Model&) {
     // 仅占位，接触每步从碰撞检测重建。
 }
 
 void VbdSolver::build_contact_constraints(const Model& model, const SimState& state) {
     const int n = model.num_bodies();
     const int num_shapes = model.num_shapes();
     std::vector<AABB> shape_aabbs(num_shapes);
     std::vector<bool> shape_static(num_shapes);
 
     // --- Contact persistence (demo3d Manifold::initialize merges by feature.key) ---
     // Build cache from previous frame's contacts so warmstarting actually reuses λ/penalty.
     std::unordered_map<uint64_t, WarmstartContactData> old_cache;
     old_cache.reserve(avbd_contacts_.size() * 2 + 8);
     for (const AvbdContact& oldc : avbd_contacts_) {
         WarmstartContactData d;
         d.rA = oldc.rA;
         d.rB = oldc.rB;
         d.penalty = oldc.penalty;
         d.lambda = oldc.lambda;
         d.stick = oldc.stick;
         old_cache[contact_key(oldc)] = d;
     }
     avbd_contacts_.clear();
 
     for (int si = 0; si < num_shapes; ++si) {
         const auto& shape = model.shapes[si];
         if (shape.body_index >= 0) {
             shape_aabbs[si] = shape.compute_aabb(state.transforms[shape.body_index]);
             shape_static[si] = model.bodies[shape.body_index].is_static();
         } else {
             shape_aabbs[si] = shape.compute_aabb(Transform::identity());
             shape_static[si] = true;
         }
     }
 
     broadphase_.update(shape_aabbs, shape_static);
     contacts_.clear();
     const auto& pairs = broadphase_.get_pairs();
 
     for (const auto& pair : pairs) {
         const auto& sa = model.shapes[pair.body_a];
         const auto& sb = model.shapes[pair.body_b];
         Transform ta = (sa.body_index >= 0) ? state.transforms[sa.body_index] : Transform::identity();
         Transform tb = (sb.body_index >= 0) ? state.transforms[sb.body_index] : Transform::identity();
 
         std::vector<ContactPoint> new_contacts;
         if (!collide_shapes(sa, ta, sb, tb, new_contacts)) continue;
 
         // 接触点合并：每对体最多 max_contacts_per_pair 个，减少过约束（box-plane 8→4）
         const int max_cp = config_.max_contacts_per_pair;
         if (static_cast<int>(new_contacts.size()) > max_cp) {
             std::partial_sort(new_contacts.begin(), new_contacts.begin() + max_cp,
                              new_contacts.end(),
                              [](const ContactPoint& a, const ContactPoint& b) {
                                  return a.penetration > b.penetration;
                              });
             new_contacts.resize(max_cp);
         }
 
         for (const auto& cp : new_contacts) {
             int ia = cp.body_a;
             int ib = cp.body_b;
             bool valid_a = (ia >= 0 && ia < n);
             bool valid_b = (ib >= 0 && ib < n);
             if (!valid_a && !valid_b) continue;
             if (valid_a && valid_b && model.bodies[ia].is_static() && model.bodies[ib].is_static()) continue;
 
             AvbdContact ac;
             ac.body_a = ia;
             ac.body_b = ib;
             ac.friction = combine_friction(sa.friction, sb.friction);
             ac.feature_id = cp.feature_id;
 
             // 法向 NovaPhy 为 A→B；demo 用 B→A 为 basis 行0，故用 -n。
             Vec3f n = cp.normal.normalized();
             ac.basis = orthonormal_basis(-n);
 
             Vec3f pA = cp.position;
             Vec3f pB = cp.position - n * cp.penetration;
             if (!valid_a)
                 ac.rA = pA;
             else if (model.bodies[ia].is_static())
                 ac.rA = pA;
             else
                 ac.rA = state.transforms[ia].rotation.inverse() * (pA - state.transforms[ia].position);
             if (!valid_b)
                 ac.rB = pB;
             else if (model.bodies[ib].is_static())
                 ac.rB = pB;
             else
                 ac.rB = state.transforms[ib].rotation.inverse() * (pB - state.transforms[ib].position);
 
             // Merge warmstart data if this contact matches a persisted one.
             // If previous frame had static friction (stick), keep old local anchors like demo3d.
             {
                 const uint64_t k_new = contact_key(ac);
                 auto it = old_cache.find(k_new);
                 if (it != old_cache.end()) {
                     ac.penalty = it->second.penalty;
                     ac.lambda = it->second.lambda;
                     ac.stick = it->second.stick;
                     if (ac.stick) {
                         ac.rA = it->second.rA;
                         ac.rB = it->second.rB;
                     }
                 }
             }
 
             // C0 = basis*(xA - xB) + margin（与 demo 一致）；法向仅允许支撑力 F[0]<=0，步末做法向速度归零防漂移
             Vec3f xA = !valid_a ? ac.rA : (model.bodies[ia].is_static() ? ac.rA : state.transforms[ia].transform_point(ac.rA));
             Vec3f xB = !valid_b ? ac.rB : (model.bodies[ib].is_static() ? ac.rB : state.transforms[ib].transform_point(ac.rB));
             ac.C0 = ac.basis * (xA - xB) + Vec3f(COLLISION_MARGIN, 0, 0);
 
             // Warmstart（Eq.19）：lambda *= alpha*gamma, penalty = clamp(penalty*gamma, MIN, MAX)
             ac.lambda = ac.lambda * config_.alpha * config_.gamma;
             ac.penalty.x() = clampf(ac.penalty.x() * config_.gamma, PENALTY_MIN, PENALTY_MAX);
             ac.penalty.y() = clampf(ac.penalty.y() * config_.gamma, PENALTY_MIN, PENALTY_MAX);
             ac.penalty.z() = clampf(ac.penalty.z() * config_.gamma, PENALTY_MIN, PENALTY_MAX);
 
             avbd_contacts_.push_back(ac);
         }
     }

    // 稳定排序：减少浮点累加顺序导致的系统性偏差（如金字塔“向后倾”）
    // 以 body pair + feature_id + 局部锚点量化为键排序（feature_id=-1 时主要依赖锚点）。
    constexpr float Q_ANCHOR = 0.01f;  // 1cm
    auto qv = [&](const Vec3f& v) {
        return std::tuple<int, int, int>(
            quantize_float(v.x(), Q_ANCHOR),
            quantize_float(v.y(), Q_ANCHOR),
            quantize_float(v.z(), Q_ANCHOR));
    };
    std::sort(avbd_contacts_.begin(), avbd_contacts_.end(),
              [&](const AvbdContact& a, const AvbdContact& b) {
                  const int a0 = std::min(a.body_a, a.body_b);
                  const int a1 = std::max(a.body_a, a.body_b);
                  const int b0 = std::min(b.body_a, b.body_b);
                  const int b1 = std::max(b.body_a, b.body_b);
                  if (a0 != b0) return a0 < b0;
                  if (a1 != b1) return a1 < b1;
                  if (a.feature_id != b.feature_id) return a.feature_id < b.feature_id;
                  if (a.stick != b.stick) return a.stick < b.stick;
                  auto arA = qv(a.rA), arB = qv(a.rB);
                  auto brA = qv(b.rA), brB = qv(b.rB);
                  if (arA != brA) return arA < brA;
                  return arB < brB;
              });
 }
 
 void VbdSolver::avbd_primal(const Model& model, SimState& state) {
     const int n = model.num_bodies();
     const float dt = config_.dt;
     const float dt2 = dt * dt;
     const float alpha = config_.alpha;
 
     if (dt2 < 1e-12f) return;
 
     for (int bi = 0; bi < n; ++bi) {
         const auto& body = model.bodies[bi];
         if (body.is_static()) continue;
 
         Vec3f pos = state.transforms[bi].position;
         Quatf rot = state.transforms[bi].rotation;
         Vec3f dqLin = pos - initial_positions_[bi];
         // Match demo3d: small-angle quaternion difference as angular delta vector.
         Vec3f dqAng = quat_small_angle_diff_vec(rot, initial_rotations_[bi]);
 
         // LHS = M/dt², RHS = M/dt²*(position - inertial)，与 demo3d 完全一致
         Mat3f MLin = body.mass * Mat3f::Identity();
         // demo3d: MAng = body.moment（对角惯性），角部未知数为世界系，无 R 变换
         Mat3f MAng = body.inertia;
 
         Mat3f lhsLin = MLin / dt2;
         Mat3f lhsAng = MAng / dt2;
         Mat3f lhsCross = Mat3f::Zero();
         Vec3f rhsLin = (MLin / dt2) * (pos - inertial_positions_[bi]);
         // 与 demo3d 一致：rhsAng = MAng/(dt²)*(positionAng - inertialAng) + jAng^T*F，再传 -rhsAng 给 solve
         Vec3f rot_err = quat_small_angle_diff_vec(rot, inertial_rotations_[bi]);  // current - inertial
         Vec3f rhsAng = (MAng / dt2) * rot_err;
 
         int num_contacts_on_body = 0;
         for (const AvbdContact& ac : avbd_contacts_) {
             bool onA = (ac.body_a == bi);
             bool onB = (ac.body_b == bi);
             if (!onA && !onB) continue;
             ++num_contacts_on_body;
 
             Vec3f rA_w = (onA && ac.body_a >= 0) ? state.transforms[ac.body_a].rotation * ac.rA : (ac.body_a == -1 ? ac.rA : Vec3f::Zero());
             Vec3f rB_w = (onB && ac.body_b >= 0) ? state.transforms[ac.body_b].rotation * ac.rB : (ac.body_b == -1 ? ac.rB : Vec3f::Zero());
 
             // jALin = basis, jBLin = -basis；仅允许支撑力 F[0]<=0
             Mat3f jALin = ac.basis;
             Mat3f jBLin = -ac.basis;
             // d(xB-xA)/d(omega_A) => angular jacobian for A: +rA x (basis.row)
             Mat3f jAAng;
             for (int i = 0; i < 3; ++i)
                 jAAng.row(i) = rA_w.cross(ac.basis.row(i));
             Mat3f jBAng;
             for (int i = 0; i < 3; ++i)
                 jBAng.row(i) = -rB_w.cross(ac.basis.row(i));
 
             Vec3f dqALin = (ac.body_a == bi) ? dqLin : Vec3f::Zero();
             Vec3f dqBLin = (ac.body_b == bi) ? dqLin : Vec3f::Zero();
             Vec3f dqAAng = (ac.body_a == bi) ? dqAng : Vec3f::Zero();
             Vec3f dqBAng = (ac.body_b == bi) ? dqAng : Vec3f::Zero();
             if (ac.body_a != bi && ac.body_a >= 0 && ac.body_a < n) {
                 int oa = ac.body_a;
                 dqALin = Vec3f(state.transforms[oa].position - initial_positions_[oa]);
                 dqAAng = quat_small_angle_diff_vec(state.transforms[oa].rotation, initial_rotations_[oa]);
             }
             if (ac.body_b != bi && ac.body_b >= 0 && ac.body_b < n) {
                 int ob = ac.body_b;
                 dqBLin = Vec3f(state.transforms[ob].position - initial_positions_[ob]);
                 dqBAng = quat_small_angle_diff_vec(state.transforms[ob].rotation, initial_rotations_[ob]);
             }
 
             Vec3f C = ac.C0 * (1.0f - alpha) + jALin * dqALin + jBLin * dqBLin;
             for (int i = 0; i < 3; ++i)
                 C(i) += jAAng.row(i).dot(dqAAng) + jBAng.row(i).dot(dqBAng);
 
             Mat3f K = Mat3f::Zero();
             K.diagonal() = ac.penalty;
             Vec3f F = K * C + ac.lambda;
             if (F(0) > 0.0f) F(0) = 0.0f;
             float bounds = std::abs(F(0)) * ac.friction;
             float ft_len = std::sqrt(F(1) * F(1) + F(2) * F(2));
             if (ft_len > bounds && ft_len > 1e-12f) {
                 F(1) *= bounds / ft_len;
                 F(2) *= bounds / ft_len;
             }
 
             Mat3f jLin = (bi == ac.body_a) ? jALin : jBLin;
             Mat3f jAng = (bi == ac.body_a) ? jAAng : jBAng;
             Mat3f jLinT = jLin.transpose();
             Mat3f jAngT = jAng.transpose();
             Mat3f jAngTk = jAngT * K;
 
             lhsLin += jLinT * K * jLin;
             lhsAng += jAngTk * jAng;
             lhsCross += jAngTk * jLin;
             rhsLin += jLinT * F;
             rhsAng += jAngT * F;
         }
 
         // 与 demo3d 完全一致：solve(LHS, -rhsLin, -rhsAng) => RHS = [-rhsLin; -rhsAng]
         Mat6f LHS = Mat6f::Zero();
         LHS.block<3, 3>(0, 0) = lhsLin;
         LHS.block<3, 3>(0, 3) = lhsCross.transpose();
         LHS.block<3, 3>(3, 0) = lhsCross;
         LHS.block<3, 3>(3, 3) = lhsAng;
         SpatialVector RHS = SpatialVector::Zero();
         RHS.head<3>() = -rhsLin;
         RHS.tail<3>() = -rhsAng;
 
         Eigen::LDLT<Mat6f> ldlt(LHS);
         if (ldlt.info() != Eigen::Success) {
             const float reg = 1e-6f * (MLin(0, 0) / dt2);
             for (int i = 0; i < 6; ++i) LHS(i, i) += reg;
             ldlt.compute(LHS);
         }
         SpatialVector dq;
         if (ldlt.info() == Eigen::Success)
             dq = ldlt.solve(RHS);
         else
             dq.setZero();
         Vec3f dxLin = dq.head<3>() * PRIMAL_RELAX;
         Vec3f dxAng = dq.tail<3>() * PRIMAL_RELAX;
 
         state.transforms[bi].position += dxLin;
         float angle = dxAng.norm();
         if (angle > 1e-8f) {
             // 与 demo3d 一致：角部世界系，左乘更新 positionAng + dxAng
             state.transforms[bi].rotation = (Quatf(Eigen::AngleAxisf(angle, dxAng / angle)) * state.transforms[bi].rotation).normalized();
         }
     }
 }
 
 void VbdSolver::avbd_dual(const Model& model, const SimState& state) {
     const int n = model.num_bodies();
     const float alpha = config_.alpha;
 
     for (AvbdContact& ac : avbd_contacts_) {
         int ia = ac.body_a;
         int ib = ac.body_b;
         bool dynA = (ia >= 0 && ia < n) && !model.bodies[ia].is_static();
         bool dynB = (ib >= 0 && ib < n) && !model.bodies[ib].is_static();
         if (!dynA && !dynB) continue;
 
         Vec3f dqALin = dynA ? Vec3f(state.transforms[ia].position - initial_positions_[ia]) : Vec3f::Zero();
         Vec3f dqBLin = dynB ? Vec3f(state.transforms[ib].position - initial_positions_[ib]) : Vec3f::Zero();
         Vec3f dqAAng = Vec3f::Zero(), dqBAng = Vec3f::Zero();
         if (dynA) {
             dqAAng = quat_small_angle_diff_vec(state.transforms[ia].rotation, initial_rotations_[ia]);
         }
         if (dynB) {
             dqBAng = quat_small_angle_diff_vec(state.transforms[ib].rotation, initial_rotations_[ib]);
         }
 
         Vec3f rA_w = dynA ? state.transforms[ia].rotation * ac.rA : Vec3f::Zero();
         Vec3f rB_w = dynB ? state.transforms[ib].rotation * ac.rB : Vec3f::Zero();
         Mat3f jALin = ac.basis;
         Mat3f jBLin = -ac.basis;
         Mat3f jAAng;
         for (int i = 0; i < 3; ++i) jAAng.row(i) = rA_w.cross(ac.basis.row(i));
         Mat3f jBAng;
         for (int i = 0; i < 3; ++i) jBAng.row(i) = -rB_w.cross(ac.basis.row(i));
 
         Vec3f C = ac.C0 * (1.0f - alpha) + jALin * dqALin + jBLin * dqBLin;
         for (int i = 0; i < 3; ++i)
             C(i) += jAAng.row(i).dot(dqAAng) + jBAng.row(i).dot(dqBAng);
 
         Mat3f K = Mat3f::Zero();
         K.diagonal() = ac.penalty;
         Vec3f F = K * C + ac.lambda;
         if (F(0) > 0.0f) F(0) = 0.0f;
         float bounds = std::abs(F(0)) * ac.friction;
         float ft_len = std::sqrt(F(1) * F(1) + F(2) * F(2));
         if (ft_len > bounds && ft_len > 1e-12f) {
             F(1) *= bounds / ft_len;
             F(2) *= bounds / ft_len;
         }
 
         ac.lambda = F;
         if (F(0) < 0.0f)
             ac.penalty.x() = std::min(ac.penalty.x() + config_.beta_linear * std::abs(C(0)), PENALTY_MAX);
         if (ft_len <= bounds) {
             ac.penalty.y() = std::min(ac.penalty.y() + config_.beta_linear * std::abs(C(1)), PENALTY_MAX);
             ac.penalty.z() = std::min(ac.penalty.z() + config_.beta_linear * std::abs(C(2)), PENALTY_MAX);
             ac.stick = (Vec2f(C(1), C(2)).norm() < STICK_THRESH);
         }
     }
 }
 
 void VbdSolver::step(const Model& model, SimState& state) {
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
 
     // 1) Broadphase + 建接触 + C0、warmstart（与 demo Initialize and warmstart forces 一致）
     build_contact_constraints(model, state);
 
     // 2) Initialize bodies：惯性位姿、initial（与 demo 一致）
     for (int i = 0; i < n; ++i) {
         const auto& body = model.bodies[i];
         if (body.is_static()) continue;
 
         Vec3f vel = state.linear_velocities[i];
         Vec3f omega = state.angular_velocities[i];
        // Constant-acceleration term should be 0.5 * g * dt^2.
        // Using g * dt^2 injects extra downward drift, causing excessive penetration and oscillatory correction.
        inertial_positions_[i] = state.transforms[i].position + vel * dt + 0.5f * gravity * (dt * dt);
         inertial_rotations_[i] = quat_add_omega_dt(state.transforms[i].rotation, omega, dt);
 
         float g2 = gravity.squaredNorm();
         float accelWeight = 1.0f;
         if (g2 > 1e-12f && prev_linear_velocities_.size() == static_cast<size_t>(n)) {
             Vec3f accel = (vel - prev_linear_velocities_[i]) / std::max(dt, 1e-6f);
             accelWeight = clampf(accel.dot(gravity) / g2, 0.0f, 1.0f);
         }
        state.transforms[i].position =
            state.transforms[i].position + vel * dt + 0.5f * gravity * (accelWeight * dt * dt);
         state.transforms[i].rotation = quat_add_omega_dt(state.transforms[i].rotation, omega, dt);
     }
 
     // 3) Main solver loop
     for (int it = 0; it < config_.iterations; ++it) {
         avbd_primal(model, state);
         avbd_dual(model, state);
     }
 
     // 4) BDF1 velocities
     for (int i = 0; i < n; ++i) {
         if (model.bodies[i].is_static()) continue;
         prev_linear_velocities_[i] = state.linear_velocities[i];
         state.linear_velocities[i] = (state.transforms[i].position - initial_positions_[i]) / dt;
         state.angular_velocities[i] = angular_velocity_from_quat_diff(
             state.transforms[i].rotation, initial_rotations_[i], dt);
     }
 }
 
 }  // namespace novaphy
 