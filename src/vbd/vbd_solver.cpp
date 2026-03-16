/**
 * @file vbd_solver.cpp
 * @brief 3D AVBD solver, following avbd-demo3d's step flow and equations.
 */
#include "novaphy/vbd/vbd_solver.h"
#include "novaphy/vbd/vbd_collide.h"
#include "novaphy/math/math_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <unordered_map>
#include <limits>
#include <vector>
 
 namespace novaphy {
 
 namespace {
 
constexpr float PENALTY_MIN = 1.0f;
constexpr float PENALTY_MAX = 10000000000.0f;
constexpr float COLLISION_MARGIN = 0.01f;
constexpr float STICK_THRESH = 0.00001f;
 
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
     float s = x / q;
     return static_cast<int>(std::floor(s + (s >= 0.0f ? 0.5f : -0.5f)));
 }

 inline uint64_t contact_key(const AvbdContact& c) {
     // demo3d: merge by (body_a, body_b, feature.key). VBD contacts always have feature_id from vbd_collide.
     uint64_t h = 1469598103934665603ull;
     h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(c.body_a + 2)));
     h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(c.body_b + 2)));
     h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(c.feature_id)));
     return h;
 }
 
 inline uint64_t pair_key(int a, int b) {
     int lo = std::min(a, b);
     int hi = std::max(a, b);
     uint64_t h = 1469598103934665603ull;
     h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(lo + 2)));
     h = fnv1a_u64(h, static_cast<uint64_t>(static_cast<uint32_t>(hi + 2)));
     return h;
 }

/** Integrate quaternion using world-space angular velocity (first-order). */
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
 
/** Small-angle rotation vector: returns 2*vec(q*q0^{-1}), with shortest-arc handling. */
 inline Vec3f quat_small_angle_diff_vec(const Quatf& q, const Quatf& q0) {
     Quatf dq = q * q0.inverse();
     dq.normalize();
     if (dq.w() < 0.0f) dq.coeffs() *= -1.0f;  // shortest arc
     return 2.0f * Vec3f(dq.x(), dq.y(), dq.z());
 }

 // demo3d quat operator-(a,b): (a*inverse(b)).vec() * 2, no shortest-arc flip.
 inline Vec3f quat_diff_vec_demo3d(const Quatf& a, const Quatf& b) {
     Quatf dq = a * b.inverse();
     dq.normalize();
     return 2.0f * Vec3f(dq.x(), dq.y(), dq.z());
 }
 
/** Angular velocity from quaternion difference (BDF1 small-angle). */
 inline Vec3f angular_velocity_from_quat_diff(const Quatf& q_now, const Quatf& q_prev, float dt) {
     // Match demo3d: use small-angle delta divided by dt.
     if (dt <= 0.0f) return Vec3f::Zero();
     return quat_small_angle_diff_vec(q_now, q_prev) / dt;
 }

 /** Apply angular correction exactly like avbd-demo3d: q_new = normalize(q + quat(dxAng,0)*q*0.5). */
 inline Quatf quat_add_angular_vec(const Quatf& q, const Vec3f& dxAng) {
     const Quatf dq(0.0f, dxAng.x(), dxAng.y(), dxAng.z());
     Quatf out = q;
     out.coeffs() += 0.5f * (dq * q).coeffs();
     out.normalize();
     return out;
 }

 }  // namespace
 
VbdSolver::VbdSolver(const VBDConfig& cfg)
    : config_(cfg) {}

VbdSolver::~VbdSolver() {
    release_cuda_buffers();
}

#ifndef NOVAPHY_VBD_CUDA
// Stub when CUDA backend is not compiled (avoids undefined reference).
void VbdSolver::release_cuda_buffers() {}
#endif

void VbdSolver::set_config(const VBDConfig& cfg) {
     config_ = cfg;
 }
 
void VbdSolver::set_model(const Model&) {
   // No-op: contacts are rebuilt from collision detection every step.
}

 void VbdSolver::clear_forces() {
     ignore_collisions_.clear();
     joints_.clear();
     springs_.clear();
 }

 void VbdSolver::add_ignore_collision(int body_a, int body_b) {
     AvbdIgnoreCollision ic;
     ic.body_a = body_a;
     ic.body_b = body_b;
     ignore_collisions_.push_back(ic);
 }

 int VbdSolver::add_joint(int body_a, int body_b, const Vec3f& rA, const Vec3f& rB,
                          float stiffnessLin, float stiffnessAng, float fracture) {
     AvbdJoint j;
     j.body_a = body_a;
     j.body_b = body_b;
     j.rA = rA;
     j.rB = rB;
     j.stiffnessLin = stiffnessLin;
     j.stiffnessAng = stiffnessAng;
     j.fracture = fracture;
     joints_.push_back(j);
     return static_cast<int>(joints_.size()) - 1;
 }

 int VbdSolver::add_spring(int body_a, int body_b, const Vec3f& rA, const Vec3f& rB,
                           float stiffness, float rest) {
     AvbdSpring s;
     s.body_a = body_a;
     s.body_b = body_b;
     s.rA = rA;
     s.rB = rB;
     s.stiffness = stiffness;
     s.rest = rest;
     springs_.push_back(s);
     return static_cast<int>(springs_.size()) - 1;
 }
 
namespace {

// Shared inner loop: given shape-index pairs, run narrowphase and fill avbd_contacts_
// with warmstart and C0. Caller must have built ignore_pair and old_cache.
void build_contacts_for_shape_pairs(
    const Model& model, const SimState& state,
    const std::vector<BroadPhasePair>& pairs,
    const std::unordered_map<uint64_t, bool>& ignore_pair,
    const std::unordered_map<uint64_t, WarmstartContactData>& old_cache,
    std::vector<AvbdContact>& avbd_contacts_out,
    const VBDConfig& config_) {
    const int n = model.num_bodies();
    std::vector<VbdContactPoint> vbd_points;
     vbd_points.reserve(static_cast<size_t>(config_.max_contacts_per_pair));

    for (const auto& pair : pairs) {
        const auto& sa = model.shapes[pair.body_a];
        const auto& sb = model.shapes[pair.body_b];
        int ia = sa.body_index;
        int ib = sb.body_index;
        bool valid_a = (ia >= 0 && ia < n);
        bool valid_b = (ib >= 0 && ib < n);
        if (valid_a && valid_b && model.bodies[ia].is_static() && model.bodies[ib].is_static())
            continue;
        if (valid_a && valid_b) {
            if (ignore_pair.find(pair_key(ia, ib)) != ignore_pair.end())
                continue;
        }

        Transform wa = (ia >= 0) ? state.transforms[ia] * sa.local_transform : Transform::identity();
        Transform wb = (ib >= 0) ? state.transforms[ib] * sb.local_transform : Transform::identity();

        Mat3f basis;
        int num_contacts = 0;
        if (sa.type == ShapeType::Box && sb.type == ShapeType::Box) {
            num_contacts = vbd_collide_box_box(
                wa.position, wa.rotation, sa.box.half_extents,
                wb.position, wb.rotation, sb.box.half_extents,
                &vbd_points, &basis);
        } else if (sa.type == ShapeType::Plane && sb.type == ShapeType::Box) {
            num_contacts = vbd_collide_box_plane(
                sa.plane.normal, sa.plane.offset,
                wb.position, wb.rotation, sb.box.half_extents,
                &vbd_points, &basis);
        } else if (sa.type == ShapeType::Box && sb.type == ShapeType::Plane) {
            num_contacts = vbd_collide_box_plane(
                -sb.plane.normal, -sb.plane.offset,
                wa.position, wa.rotation, sa.box.half_extents,
                &vbd_points, &basis);
            for (auto& p : vbd_points) {
                std::swap(p.rA, p.rB);
            }
        }
        if (num_contacts <= 0) continue;

        const float friction = combine_friction(sa.friction, sb.friction);
        const int max_cp = config_.max_contacts_per_pair;
        int to_add = std::min(num_contacts, max_cp);

        for (int k = 0; k < to_add; ++k) {
            const auto& pt = vbd_points[k];
            AvbdContact ac;
            ac.body_a = ia;
            ac.body_b = ib;
            ac.rA = pt.rA;
            ac.rB = pt.rB;
            ac.basis = basis;
            ac.friction = friction;
            ac.feature_id = pt.feature_key;

            uint64_t k_new = contact_key(ac);
            auto it = old_cache.find(k_new);
            if (it != old_cache.end()) {
                ac.penalty = it->second.penalty;
                ac.lambda = it->second.lambda;
                ac.stick = it->second.stick;
                if (ac.stick) {
                    ac.rA = it->second.rA;
                    ac.rB = it->second.rB;
                }
            } else {
                ac.penalty = Vec3f(PENALTY_MIN, PENALTY_MIN, PENALTY_MIN);
            }

            Vec3f xA = !valid_a ? ac.rA : state.transforms[ia].transform_point(ac.rA);
            Vec3f xB = !valid_b ? ac.rB : state.transforms[ib].transform_point(ac.rB);
            ac.C0 = ac.basis * (xA - xB) + Vec3f(COLLISION_MARGIN, 0, 0);

            ac.lambda = ac.lambda * config_.alpha * config_.gamma;
            ac.penalty.x() = clampf(ac.penalty.x() * config_.gamma, PENALTY_MIN, PENALTY_MAX);
            ac.penalty.y() = clampf(ac.penalty.y() * config_.gamma, PENALTY_MIN, PENALTY_MAX);
            ac.penalty.z() = clampf(ac.penalty.z() * config_.gamma, PENALTY_MIN, PENALTY_MAX);

            avbd_contacts_out.push_back(ac);
        }
    }

    constexpr float Q_ANCHOR = 0.01f;
    auto qv = [](float x, float y, float z) {
        return std::tuple<int, int, int>(
            quantize_float(x, Q_ANCHOR),
            quantize_float(y, Q_ANCHOR),
            quantize_float(z, Q_ANCHOR));
    };
    std::sort(avbd_contacts_out.begin(), avbd_contacts_out.end(),
              [&qv](const AvbdContact& a, const AvbdContact& b) {
                  const int a0 = std::min(a.body_a, a.body_b);
                  const int a1 = std::max(a.body_a, a.body_b);
                  const int b0 = std::min(b.body_a, b.body_b);
                  const int b1 = std::max(b.body_a, b.body_b);
                  if (a0 != b0) return a0 < b0;
                  if (a1 != b1) return a1 < b1;
                  if (a.feature_id != b.feature_id) return a.feature_id < b.feature_id;
                  if (a.stick != b.stick) return a.stick < b.stick;
                  auto arA = qv(a.rA.x(), a.rA.y(), a.rA.z()), arB = qv(a.rB.x(), a.rB.y(), a.rB.z());
                  auto brA = qv(b.rA.x(), b.rA.y(), b.rA.z()), brB = qv(b.rB.x(), b.rB.y(), b.rB.z());
                  if (arA != brA) return arA < brA;
                  return arB < brB;
              });
}

}  // namespace

void VbdSolver::build_contact_constraints(const Model& model, const SimState& state) {
    const int num_shapes = model.num_shapes();
    std::vector<AABB> shape_aabbs(num_shapes);
    std::vector<bool> shape_static(num_shapes);

    // Reuse hash tables to avoid per-step allocations/rehash hitches.
    static thread_local std::unordered_map<uint64_t, bool> ignore_pair;
    ignore_pair.clear();
    ignore_pair.reserve(ignore_collisions_.size() * 2 + 8);
    for (const auto& ic : ignore_collisions_) {
        if (ic.body_a >= 0 && ic.body_b >= 0)
            ignore_pair[pair_key(ic.body_a, ic.body_b)] = true;
    }

    static thread_local std::unordered_map<uint64_t, WarmstartContactData> old_cache;
    old_cache.clear();
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
    build_contacts_for_shape_pairs(model, state, broadphase_.get_pairs(),
                                  ignore_pair, old_cache, avbd_contacts_, config_);
}

void VbdSolver::build_contact_constraints_from_pairs(const Model& model, const SimState& state,
                                                     const std::vector<std::pair<int, int>>& shape_pairs) {
    static thread_local std::unordered_map<uint64_t, bool> ignore_pair;
    ignore_pair.clear();
    ignore_pair.reserve(ignore_collisions_.size() * 2 + 8);
    for (const auto& ic : ignore_collisions_) {
        if (ic.body_a >= 0 && ic.body_b >= 0)
            ignore_pair[pair_key(ic.body_a, ic.body_b)] = true;
    }

    static thread_local std::unordered_map<uint64_t, WarmstartContactData> old_cache;
    old_cache.clear();
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

    std::vector<BroadPhasePair> pairs;
    pairs.reserve(shape_pairs.size());
    for (const auto& p : shape_pairs) {
        int a = std::min(p.first, p.second);
        int b = std::max(p.first, p.second);
        if (a != b)
            pairs.push_back({a, b});
    }
    std::sort(pairs.begin(), pairs.end(), [](const BroadPhasePair& x, const BroadPhasePair& y) {
        return x.body_a < y.body_a || (x.body_a == y.body_a && x.body_b < y.body_b);
    });
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());

    build_contacts_for_shape_pairs(model, state, pairs, ignore_pair, old_cache, avbd_contacts_, config_);
}

void VbdSolver::build_contact_constraints_from_raw_contacts(const Model& model, const SimState& state,
                                                          const std::vector<RawContactHost>& raw_contacts) {
    build_contact_constraints_from_raw_contacts(model, state, std::span<const RawContactHost>(raw_contacts.data(), raw_contacts.size()));
}

void VbdSolver::build_contact_constraints_from_raw_contacts(const Model& model, const SimState& state,
                                                          std::span<const RawContactHost> raw_contacts) {
    const int n = model.num_bodies();
    static thread_local std::unordered_map<uint64_t, bool> ignore_pair;
    ignore_pair.clear();
    ignore_pair.reserve(ignore_collisions_.size() * 2 + 8);
    for (const auto& ic : ignore_collisions_) {
        if (ic.body_a >= 0 && ic.body_b >= 0)
            ignore_pair[pair_key(ic.body_a, ic.body_b)] = true;
    }

    static thread_local std::unordered_map<uint64_t, WarmstartContactData> old_cache;
    old_cache.clear();
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

    for (const auto& rc : raw_contacts) {
        int ia = rc.body_a;
        int ib = rc.body_b;
        bool valid_a = (ia >= 0 && ia < n);
        bool valid_b = (ib >= 0 && ib < n);
        if (valid_a && valid_b && ignore_pair.find(pair_key(ia, ib)) != ignore_pair.end())
            continue;

        AvbdContact ac;
        ac.body_a = ia;
        ac.body_b = ib;
        ac.rA = Vec3f(rc.rA[0], rc.rA[1], rc.rA[2]);
        ac.rB = Vec3f(rc.rB[0], rc.rB[1], rc.rB[2]);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                ac.basis(r, c) = rc.basis[r * 3 + c];
        ac.friction = rc.friction;
        ac.feature_id = rc.feature_id;

        uint64_t k_new = contact_key(ac);
        auto it = old_cache.find(k_new);
        if (it != old_cache.end()) {
            ac.penalty = it->second.penalty;
            ac.lambda = it->second.lambda;
            ac.stick = it->second.stick;
            if (ac.stick) {
                ac.rA = it->second.rA;
                ac.rB = it->second.rB;
            }
        } else {
            ac.penalty = Vec3f(PENALTY_MIN, PENALTY_MIN, PENALTY_MIN);
        }

        Vec3f xA = !valid_a ? ac.rA : state.transforms[ia].transform_point(ac.rA);
        Vec3f xB = !valid_b ? ac.rB : state.transforms[ib].transform_point(ac.rB);
        ac.C0 = ac.basis * (xA - xB) + Vec3f(COLLISION_MARGIN, 0, 0);

        ac.lambda = ac.lambda * config_.alpha * config_.gamma;
        ac.penalty.x() = clampf(ac.penalty.x() * config_.gamma, PENALTY_MIN, PENALTY_MAX);
        ac.penalty.y() = clampf(ac.penalty.y() * config_.gamma, PENALTY_MIN, PENALTY_MAX);
        ac.penalty.z() = clampf(ac.penalty.z() * config_.gamma, PENALTY_MIN, PENALTY_MAX);

        avbd_contacts_.push_back(ac);
    }

    constexpr float Q_ANCHOR = 0.01f;
    auto qv = [](float x, float y, float z) {
        return std::tuple<int, int, int>(
            quantize_float(x, Q_ANCHOR),
            quantize_float(y, Q_ANCHOR),
            quantize_float(z, Q_ANCHOR));
    };
    std::sort(avbd_contacts_.begin(), avbd_contacts_.end(),
              [&qv](const AvbdContact& a, const AvbdContact& b) {
                  const int a0 = std::min(a.body_a, a.body_b);
                  const int a1 = std::max(a.body_a, a.body_b);
                  const int b0 = std::min(b.body_a, b.body_b);
                  const int b1 = std::max(b.body_a, b.body_b);
                  if (a0 != b0) return a0 < b0;
                  if (a1 != b1) return a1 < b1;
                  if (a.feature_id != b.feature_id) return a.feature_id < b.feature_id;
                  if (a.stick != b.stick) return a.stick < b.stick;
                  auto arA = qv(a.rA.x(), a.rA.y(), a.rA.z()), arB = qv(a.rB.x(), a.rB.y(), a.rB.z());
                  auto brA = qv(b.rA.x(), b.rA.y(), b.rA.z()), brB = qv(b.rB.x(), b.rB.y(), b.rB.z());
                  if (arA != brA) return arA < brA;
                  return arB < brB;
              });
}

void VbdSolver::build_contact_constraints_from_raw_contacts_warmstart(const Model& model, const SimState& state,
                                                                      std::span<const RawContactHostWarmstart> raw_warmstart) {
    const int n = model.num_bodies();
    static thread_local std::unordered_map<uint64_t, bool> ignore_pair;
    ignore_pair.clear();
    ignore_pair.reserve(ignore_collisions_.size() * 2 + 8);
    for (const auto& ic : ignore_collisions_) {
        if (ic.body_a >= 0 && ic.body_b >= 0)
            ignore_pair[pair_key(ic.body_a, ic.body_b)] = true;
    }
    avbd_contacts_.clear();

    for (const auto& rw : raw_warmstart) {
        const RawContactHost& rc = rw.base;
        int ia = rc.body_a;
        int ib = rc.body_b;
        bool valid_a = (ia >= 0 && ia < n);
        bool valid_b = (ib >= 0 && ib < n);
        if (valid_a && valid_b && ignore_pair.find(pair_key(ia, ib)) != ignore_pair.end())
            continue;

        AvbdContact ac;
        ac.body_a = ia;
        ac.body_b = ib;
        ac.rA = Vec3f(rc.rA[0], rc.rA[1], rc.rA[2]);
        ac.rB = Vec3f(rc.rB[0], rc.rB[1], rc.rB[2]);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                ac.basis(r, c) = rc.basis[r * 3 + c];
        ac.friction = rc.friction;
        ac.feature_id = rc.feature_id;
        ac.lambda = Vec3f(rw.lambda[0], rw.lambda[1], rw.lambda[2]);
        ac.penalty = Vec3f(rw.penalty[0], rw.penalty[1], rw.penalty[2]);
        ac.stick = (rw.stick != 0);

        Vec3f xA = !valid_a ? ac.rA : state.transforms[ia].transform_point(ac.rA);
        Vec3f xB = !valid_b ? ac.rB : state.transforms[ib].transform_point(ac.rB);
        ac.C0 = ac.basis * (xA - xB) + Vec3f(COLLISION_MARGIN, 0, 0);

        ac.lambda = ac.lambda * config_.alpha * config_.gamma;
        ac.penalty.x() = clampf(ac.penalty.x() * config_.gamma, PENALTY_MIN, PENALTY_MAX);
        ac.penalty.y() = clampf(ac.penalty.y() * config_.gamma, PENALTY_MIN, PENALTY_MAX);
        ac.penalty.z() = clampf(ac.penalty.z() * config_.gamma, PENALTY_MIN, PENALTY_MAX);

        avbd_contacts_.push_back(ac);
    }

    constexpr float Q_ANCHOR = 0.01f;
    auto qv = [](float x, float y, float z) {
        return std::tuple<int, int, int>(
            quantize_float(x, Q_ANCHOR),
            quantize_float(y, Q_ANCHOR),
            quantize_float(z, Q_ANCHOR));
    };
    std::sort(avbd_contacts_.begin(), avbd_contacts_.end(),
              [&qv](const AvbdContact& a, const AvbdContact& b) {
                  const int a0 = std::min(a.body_a, a.body_b);
                  const int a1 = std::max(a.body_a, a.body_b);
                  const int b0 = std::min(b.body_a, b.body_b);
                  const int b1 = std::max(b.body_a, b.body_b);
                  if (a0 != b0) return a0 < b0;
                  if (a1 != b1) return a1 < b1;
                  if (a.feature_id != b.feature_id) return a.feature_id < b.feature_id;
                  if (a.stick != b.stick) return a.stick < b.stick;
                  auto arA = qv(a.rA.x(), a.rA.y(), a.rA.z()), arB = qv(a.rB.x(), a.rB.y(), a.rB.z());
                  auto brA = qv(b.rA.x(), b.rA.y(), b.rA.z()), brB = qv(b.rB.x(), b.rB.y(), b.rB.z());
                  if (arA != brA) return arA < brA;
                  return arB < brB;
              });
}

 namespace {
 inline Mat3f diagonalize(const Mat3f& m) {
     // demo3d: diagonal(length(col0), length(col1), length(col2))
     Vec3f d(m.col(0).norm(), m.col(1).norm(), m.col(2).norm());
     return d.asDiagonal();
 }

 inline Mat3f geometricStiffnessBallSocket(int k, const Vec3f& v) {
     Mat3f m = (-v[k]) * Mat3f::Identity();
     m(0, k) += v.x();
     m(1, k) += v.y();
     m(2, k) += v.z();
     return m;
 }

 inline Vec3f world_point(const SimState& state, int body, const Vec3f& r_local_or_world) {
     if (body < 0) return r_local_or_world;
     return state.transforms[body].transform_point(r_local_or_world);
 }

 inline Vec3f world_dir(const SimState& state, int body, const Vec3f& r_local) {
     if (body < 0) return r_local;
     return state.transforms[body].rotation * r_local;
 }
 }  // namespace
 
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
 
        // LHS = M/dt^2, RHS = M/dt^2*(position - inertial) (demo3d).
         Mat3f MLin = body.mass * Mat3f::Identity();
        // Match demo3d: use diagonal inertia (moment) only for angular block.
         Mat3f MAng = Mat3f::Zero();
         MAng.diagonal() = Vec3f(body.inertia(0, 0), body.inertia(1, 1), body.inertia(2, 2));
 
         Mat3f lhsLin = MLin / dt2;
         Mat3f lhsAng = MAng / dt2;
         Mat3f lhsCross = Mat3f::Zero();
         Vec3f rhsLin = (MLin / dt2) * (pos - inertial_positions_[bi]);
        // demo3d: rhsAng = MAng/dt^2*(rot - inertialRot) + jAng^T*F, then solve with -rhsAng.
         Vec3f rot_err = quat_small_angle_diff_vec(rot, inertial_rotations_[bi]);  // current - inertial
         Vec3f rhsAng = (MAng / dt2) * rot_err;
 
         // --- Contact constraints (manifolds) ---
         for (const AvbdContact& ac : avbd_contacts_) {
             bool onA = (ac.body_a == bi);
             bool onB = (ac.body_b == bi);
             if (!onA && !onB) continue;
 
             // World-space contact arms for both bodies (needed for C and for jAAng/jBAng).
             Vec3f rA_w = (ac.body_a >= 0) ? state.transforms[ac.body_a].rotation * ac.rA : (ac.body_a == -1 ? ac.rA : Vec3f::Zero());
             Vec3f rB_w = (ac.body_b >= 0) ? state.transforms[ac.body_b].rotation * ac.rB : (ac.body_b == -1 ? ac.rB : Vec3f::Zero());
 
            // jALin = basis, jBLin = -basis; normal support uses F[0] <= 0.
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

         // --- Joint constraints (demo3d Joint) ---
         for (const AvbdJoint& j : joints_) {
             if (j.broken) continue;
             bool onA = (j.body_a == bi);
             bool onB = (j.body_b == bi);
             if (!onA && !onB) continue;

             // Linear part
             if (j.penaltyLin.squaredNorm() > 0.0f) {
                 Mat3f K = j.penaltyLin.asDiagonal();
                 Vec3f xA = world_point(state, j.body_a, j.rA);
                 Vec3f xB = world_point(state, j.body_b, j.rB);
                 Vec3f C = xA - xB;
                 if (std::isinf(j.stiffnessLin)) C -= j.C0Lin * alpha;
                 Vec3f F = K * C + j.lambdaLin;

                 Mat3f jLin;
                 if (onA) jLin.setIdentity();
                 else jLin = -Mat3f::Identity();
                 Vec3f rA_w = world_dir(state, j.body_a, j.rA);
                 Vec3f rB_w = world_dir(state, j.body_b, j.rB);
                 Mat3f jAng;
                 if (onA) jAng = novaphy::skew(-rA_w);
                 else jAng = novaphy::skew(rB_w);

                 Mat3f jLinT = jLin.transpose();
                 Mat3f jAngT = jAng.transpose();
                 Mat3f jAngTk = jAngT * K;

                 lhsLin += jLinT * K * jLin;
                 lhsAng += jAngTk * jAng;
                 lhsCross += jAngTk * jLin;

                 Vec3f r = onA ? rA_w : -rB_w;
                 Mat3f H = geometricStiffnessBallSocket(0, r) * F.x()
                         + geometricStiffnessBallSocket(1, r) * F.y()
                         + geometricStiffnessBallSocket(2, r) * F.z();
                 lhsAng += diagonalize(H);

                 rhsLin += jLinT * F;
                 rhsAng += jAngT * F;
             }

             // Angular part
             if (j.penaltyAng.squaredNorm() > 0.0f) {
                 Mat3f K = j.penaltyAng.asDiagonal();
                 Quatf qA = (j.body_a >= 0) ? state.transforms[j.body_a].rotation : Quatf::Identity();
                 Quatf qB = (j.body_b >= 0) ? state.transforms[j.body_b].rotation : Quatf::Identity();
                 Vec3f C = quat_diff_vec_demo3d(qA, qB) * j.torqueArm;
                 if (std::isinf(j.stiffnessAng)) C -= j.C0Ang * alpha;
                 Vec3f F = K * C + j.lambdaAng;
                 Mat3f jAng;
                 if (onA) jAng.setIdentity();
                 else jAng = -Mat3f::Identity();
                 jAng *= j.torqueArm;
                 lhsAng += jAng.transpose() * K * jAng;
                 rhsAng += jAng.transpose() * F;
             }
         }

         // --- Spring forces (demo3d Spring) ---
         for (const AvbdSpring& s : springs_) {
             bool onA = (s.body_a == bi);
             bool onB = (s.body_b == bi);
             if (!onA && !onB) continue;
             if (s.body_a < 0 || s.body_b < 0) continue;  // springs are body-body in demo3d

             Vec3f pA = state.transforms[s.body_a].transform_point(s.rA);
             Vec3f pB = state.transforms[s.body_b].transform_point(s.rB);
             Vec3f d = pA - pB;
             float dLen = d.norm();
             if (dLen <= 1.0e-6f) continue;
             Vec3f nrm = d / dLen;
             float rest = s.rest;
             if (rest < 0.0f) rest = dLen;
             float C = dLen - rest;
             float f = s.stiffness * C;

             Vec3f rWorld, jLin_v, jAng_v;
             if (onA) {
                 rWorld = state.transforms[s.body_a].rotation * s.rA;
                 jLin_v = nrm;
                 jAng_v = rWorld.cross(nrm);
             } else {
                 rWorld = state.transforms[s.body_b].rotation * s.rB;
                 jLin_v = -nrm;
                 jAng_v = -rWorld.cross(nrm);
             }
             Vec3f F = jLin_v * f;
             Vec3f Tau = jAng_v * f;
             Mat3f Kll = (jLin_v * jLin_v.transpose()) * s.stiffness;
             Mat3f Kla = (jAng_v * jLin_v.transpose()) * s.stiffness;
             Mat3f Kaa = (jAng_v * jAng_v.transpose()) * s.stiffness;
             lhsLin += Kll;
             lhsAng += Kaa;
             lhsCross += Kla;
             rhsLin += F;
             rhsAng += Tau;
         }
 
        // Solve with RHS = [-rhsLin; -rhsAng] (demo3d).
         Mat6f LHS = Mat6f::Zero();
         LHS.block<3, 3>(0, 0) = lhsLin;
         LHS.block<3, 3>(0, 3) = lhsCross.transpose();
         LHS.block<3, 3>(3, 0) = lhsCross;
         LHS.block<3, 3>(3, 3) = lhsAng;
         SpatialVector RHS = SpatialVector::Zero();
         RHS.head<3>() = -rhsLin;
         RHS.tail<3>() = -rhsAng;

         // Optional LHS regularization (stack/many contacts benefit; pyramid often fine without).
         const float reg = std::max(0.0f, config_.lhs_regularization);
         if (reg > 0.0f) {
             LHS.block<3, 3>(0, 0).diagonal().array() += reg;
             LHS.block<3, 3>(3, 3).diagonal().array() += reg;
         }
         Eigen::LDLT<Mat6f> ldlt(LHS);
         SpatialVector dq;
         if (ldlt.info() == Eigen::Success) {
             dq = ldlt.solve(RHS);
         } else {
             constexpr float reg_fallback = 1e-5f;
             LHS.block<3, 3>(0, 0).diagonal().array() += reg_fallback;
             LHS.block<3, 3>(3, 3).diagonal().array() += reg_fallback;
             Eigen::LDLT<Mat6f> ldlt2(LHS);
             if (ldlt2.info() == Eigen::Success)
                 dq = ldlt2.solve(RHS);
             else
                 dq.setZero();
         }
         const float relax = std::max(0.01f, std::min(1.0f, config_.primal_relaxation));
         Vec3f dxLin = dq.head<3>() * relax;
         Vec3f dxAng = dq.tail<3>() * relax;
 
         state.transforms[bi].position += dxLin;
         // Apply angular update exactly like demo3d: positionAng = positionAng + dxAng (quat + float3).
         state.transforms[bi].rotation = quat_add_angular_vec(state.transforms[bi].rotation, dxAng);
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
 
         // World-space contact arms (use current rotation; static bodies still have valid transform).
         Vec3f rA_w = (ia >= 0) ? state.transforms[ia].rotation * ac.rA : Vec3f::Zero();
         Vec3f rB_w = (ib >= 0) ? state.transforms[ib].rotation * ac.rB : Vec3f::Zero();
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

     // Joint dual update (demo3d Joint::updateDual)
     for (AvbdJoint& j : joints_) {
         if (j.broken) continue;
         int ia = j.body_a;
         int ib = j.body_b;
         bool dynA = (ia >= 0 && ia < n) && !model.bodies[ia].is_static();
         bool dynB = (ib >= 0 && ib < n) && !model.bodies[ib].is_static();
         if (!dynA && !dynB) continue;

         // Linear
         if (j.penaltyLin.squaredNorm() > 0.0f) {
             Mat3f K = j.penaltyLin.asDiagonal();
             Vec3f xA = world_point(state, ia, j.rA);
             Vec3f xB = world_point(state, ib, j.rB);
             Vec3f C = xA - xB;
             if (std::isinf(j.stiffnessLin)) {
                 C -= j.C0Lin * alpha;
                 Vec3f F = K * C + j.lambdaLin;
                 j.lambdaLin = F;
             }
             Vec3f absC = C.cwiseAbs();
             j.penaltyLin = (j.penaltyLin + absC * config_.beta_linear).cwiseMin(
                 Vec3f(std::min(j.stiffnessLin, PENALTY_MAX),
                       std::min(j.stiffnessLin, PENALTY_MAX),
                       std::min(j.stiffnessLin, PENALTY_MAX)));
         }

         // Angular
         if (j.penaltyAng.squaredNorm() > 0.0f) {
             Mat3f K = j.penaltyAng.asDiagonal();
             Quatf qA = (ia >= 0) ? state.transforms[ia].rotation : Quatf::Identity();
             Quatf qB = (ib >= 0) ? state.transforms[ib].rotation : Quatf::Identity();
             Vec3f C = quat_diff_vec_demo3d(qA, qB) * j.torqueArm;
             if (std::isinf(j.stiffnessAng)) {
                 C -= j.C0Ang * alpha;
                 Vec3f F = K * C + j.lambdaAng;
                 j.lambdaAng = F;
             }
             Vec3f absC = C.cwiseAbs();
             j.penaltyAng = (j.penaltyAng + absC * config_.beta_angular).cwiseMin(
                 Vec3f(std::min(j.stiffnessAng, PENALTY_MAX),
                       std::min(j.stiffnessAng, PENALTY_MAX),
                       std::min(j.stiffnessAng, PENALTY_MAX)));
         }

         if (j.lambdaAng.squaredNorm() > j.fracture * j.fracture) {
             j.penaltyLin.setZero();
             j.penaltyAng.setZero();
             j.lambdaLin.setZero();
             j.lambdaAng.setZero();
             j.broken = true;
         }
     }
 }
 
void VbdSolver::step(const Model& model, SimState& state) {
    // Dispatch to the selected backend. CUDA backend is implemented in
    // vbd_solver_cuda.cu (only when NOVAPHY_VBD_CUDA is defined).
#ifdef NOVAPHY_VBD_CUDA
    if (config_.backend == VbdBackend::CUDA) {
        step_cuda(model, state);
        return;
    }
#else
    if (config_.backend == VbdBackend::CUDA) {
        // Built without CUDA; fall back to CPU step so CI / non-CUDA builds run.
    }
#endif

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
 
    // 1) Broadphase + contacts + C0 + warmstart
     build_contact_constraints(model, state);

     // 1.5) Initialize and warmstart joints/springs (demo3d Force::initialize)
     // Approximate torqueArm using body box sizes (demo3d uses lengthSq(sizeA + sizeB)).
     std::vector<Vec3f> body_size(static_cast<size_t>(n), Vec3f::Zero());
     for (const auto& shape : model.shapes) {
         if (shape.body_index < 0 || shape.body_index >= n) continue;
         if (shape.type != ShapeType::Box) continue;
         Vec3f full = shape.box.half_extents * 2.0f;
         body_size[static_cast<size_t>(shape.body_index)] = body_size[static_cast<size_t>(shape.body_index)].cwiseMax(full);
     }

     for (AvbdJoint& j : joints_) {
         if (j.broken) continue;
         Vec3f szA = (j.body_a >= 0 && j.body_a < n) ? body_size[static_cast<size_t>(j.body_a)] : Vec3f::Zero();
         Vec3f szB = (j.body_b >= 0 && j.body_b < n) ? body_size[static_cast<size_t>(j.body_b)] : Vec3f::Zero();
         j.torqueArm = (szA + szB).squaredNorm();
         if (!(j.torqueArm > 0.0f)) j.torqueArm = 1.0f;

         Vec3f xA = world_point(state, j.body_a, j.rA);
         Vec3f xB = world_point(state, j.body_b, j.rB);
         j.C0Lin = xA - xB;
         Quatf qA = (j.body_a >= 0 && j.body_a < n) ? state.transforms[j.body_a].rotation : Quatf::Identity();
         Quatf qB = (j.body_b >= 0 && j.body_b < n) ? state.transforms[j.body_b].rotation : Quatf::Identity();
         j.C0Ang = quat_diff_vec_demo3d(qA, qB) * j.torqueArm;

         j.lambdaLin = j.lambdaLin * config_.alpha * config_.gamma;
         j.lambdaAng = j.lambdaAng * config_.alpha * config_.gamma;
         // demo3d: warmstart clamps penalty to [PENALTY_MIN, PENALTY_MAX] so constraints activate on first step.
         j.penaltyLin = (j.penaltyLin * config_.gamma).cwiseMax(Vec3f(PENALTY_MIN, PENALTY_MIN, PENALTY_MIN))
                                              .cwiseMin(Vec3f(PENALTY_MAX, PENALTY_MAX, PENALTY_MAX));
         j.penaltyAng = (j.penaltyAng * config_.gamma).cwiseMax(Vec3f(PENALTY_MIN, PENALTY_MIN, PENALTY_MIN))
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
 
    // 2) Initialize bodies (inertial state + initial)
     for (int i = 0; i < n; ++i) {
         const auto& body = model.bodies[i];
         if (body.is_static()) continue;
 
        Vec3f vel = state.linear_velocities[i];
        Vec3f omega = state.angular_velocities[i];
        // Match avbd-demo3d: inertialLin = x + v*dt + g*dt^2 (no 0.5 factor).
        inertial_positions_[i] = state.transforms[i].position + vel * dt + gravity * (dt * dt);
         inertial_rotations_[i] = quat_add_omega_dt(state.transforms[i].rotation, omega, dt);
 
        float g2 = gravity.squaredNorm();
        float accelWeight = 1.0f;
        if (g2 > 1e-12f && prev_linear_velocities_.size() == static_cast<size_t>(n)) {
            Vec3f accel = (vel - prev_linear_velocities_[i]) / dt;
            accelWeight = clampf(accel.dot(gravity) / g2, 0.0f, 1.0f);
            if (!std::isfinite(accelWeight))
                accelWeight = 0.0f;
        }
        // Match avbd-demo3d warmstart position: x = x + v*dt + g*(accelWeight*dt^2).
        state.transforms[i].position =
            state.transforms[i].position + vel * dt + gravity * (accelWeight * dt * dt);
        state.transforms[i].rotation = quat_add_omega_dt(state.transforms[i].rotation, omega, dt);
     }
 
     // 3) Main solver loop
     for (int it = 0; it < config_.iterations; ++it) {
         avbd_primal(model, state);
         avbd_dual(model, state);
     }
 
     // 4) BDF1 velocities (demo3d)
     for (int i = 0; i < n; ++i) {
         if (model.bodies[i].is_static()) continue;
         prev_linear_velocities_[i] = state.linear_velocities[i];
         state.linear_velocities[i] = (state.transforms[i].position - initial_positions_[i]) / dt;
         state.angular_velocities[i] = angular_velocity_from_quat_diff(
             state.transforms[i].rotation, initial_rotations_[i], dt);
     }
 }
 
 }  // namespace novaphy
 