#pragma once

#include "novaphy/core/model.h"
#include "novaphy/sim/state.h"
#include "novaphy/vbd/vbd_config.h"

#include "novaphy/collision/broadphase.h"
#include "novaphy/core/contact.h"
#include "novaphy/math/math_types.h"
#include "novaphy/math/spatial.h"
#include "novaphy/vbd/vbd_collide.h"
#include "novaphy/vbd/vbd_forces.h"

#include <limits>
#include <span>
#include <utility>
#include <vector>

namespace novaphy {

/**
 * @brief Single-point contact constraint (aligned with avbd-demo3d Manifold::Contact).
 *
 * Uses a 3D constraint: normal + two tangents.
 * basis row0 = normal (B→A), row1/2 = tangents.
 * C0 = basis * (xA - xB) + {COLLISION_MARGIN, 0, 0}, and F = K*C + lambda.
 */
struct AvbdContact {
    int body_a = -1;
    int body_b = -1;
    Vec3f rA = Vec3f::Zero();  // Contact point in A local coordinates.
    Vec3f rB = Vec3f::Zero();  // Contact point in B local coordinates.
    Mat3f basis = Mat3f::Identity();  // row0=normal (B→A), row1/2=tangents.

    Vec3f C0 = Vec3f::Zero();   // Constraint value at step start.
    Vec3f penalty = Vec3f::Zero();
    Vec3f lambda = Vec3f::Zero();
    float friction = 0.5f;
    bool stick = false;

    // Optional: feature id forwarded from narrowphase for contact persistence (demo3d FeaturePair::key).
    int feature_id = -1;
};

/**
 * @brief 3D AVBD solver, following avbd-demo3d's step flow and equations.
 *
 * Flow: broadphase → build contacts & initialize (C0, warmstart) → body initialize (inertial, initial) →
 * main loop (primal per-body 6x6 + dual update) → BDF1 velocities.
 */
class VbdSolver {
public:
    explicit VbdSolver(const VBDConfig& cfg);
    ~VbdSolver();

    void set_config(const VBDConfig& cfg);
    const VBDConfig& config() const { return config_; }

    void set_model(const Model& model);

    // demo3d-style constraints/forces
    void clear_forces();
    void add_ignore_collision(int body_a, int body_b);
    int add_joint(int body_a, int body_b,
                  const Vec3f& rA, const Vec3f& rB,
                  float stiffnessLin = std::numeric_limits<float>::infinity(),
                  float stiffnessAng = 0.0f,
                  float fracture = std::numeric_limits<float>::infinity());
    int add_spring(int body_a, int body_b,
                   const Vec3f& rA, const Vec3f& rB,
                   float stiffness, float rest = -1.0f);

    /**
     * @brief One AVBD step (matches demo3d Solver::step()).
     */
    void step(const Model& model, SimState& state);

private:
    /// CPU path: build contacts, run AVBD on host.
    /** Build contacts at step start and initialize C0 + warmstart lambda/penalty. */
    void build_contact_constraints(const Model& model, const SimState& state);

    /** Build contacts from a given list of shape-index pairs (for GPU broadphase path). */
    void build_contact_constraints_from_pairs(const Model& model, const SimState& state,
                                             const std::vector<std::pair<int, int>>& shape_pairs);

    /** Raw contact from GPU narrowphase (body_a, body_b, rA, rB, basis, friction, feature_id). */
    struct RawContactHost {
        int body_a = -1;
        int body_b = -1;
        float rA[3] = {};
        float rB[3] = {};
        float basis[9] = {};
        float friction = 0.5f;
        int feature_id = 0;
    };
    /** Raw contact with warmstart data (lambda, penalty, stick) filled on GPU. Same layout as device RawContactWarmstart. */
    struct RawContactHostWarmstart {
        RawContactHost base;
        float lambda[3] = {};
        float penalty[3] = {};
        int stick = 0;
    };
    /** Build contacts from GPU narrowphase output (warmstart + C0 + sort). */
    void build_contact_constraints_from_raw_contacts(const Model& model, const SimState& state,
                                                    const std::vector<RawContactHost>& raw_contacts);
    /** Span-based overload to avoid per-step allocations in CUDA path. */
    void build_contact_constraints_from_raw_contacts(const Model& model, const SimState& state,
                                                    std::span<const RawContactHost> raw_contacts);
    /** Build contacts from GPU narrowphase + GPU warmstart; no old_cache, only C0 + scale + sort. */
    void build_contact_constraints_from_raw_contacts_warmstart(const Model& model, const SimState& state,
                                                               std::span<const RawContactHostWarmstart> raw_warmstart);

    /** Main loop primal: assemble per-body 6x6 LHS/RHS, solve and apply dq. */
    void avbd_primal(const Model& model, SimState& state);
    /** Main loop dual: update lambda and penalty. */
    void avbd_dual(const Model& model, const SimState& state);

#if defined(NOVAPHY_VBD_CUDA)
    /// CUDA path entry point (implemented in vbd_solver_cuda.cu).
    void step_cuda(const Model& model, SimState& state);
#endif
    /// Frees persistent CUDA device buffers (called from destructor); no-op if not using CUDA.
    void release_cuda_buffers();

    VBDConfig config_;
    SweepAndPrune broadphase_;
    std::vector<AvbdContact> avbd_contacts_;
    std::vector<AvbdIgnoreCollision> ignore_collisions_;
    std::vector<AvbdJoint> joints_;
    std::vector<AvbdSpring> springs_;
    std::vector<Vec3f> inertial_positions_;
    std::vector<Quatf> inertial_rotations_;
    std::vector<Vec3f> initial_positions_;
    std::vector<Quatf> initial_rotations_;
    std::vector<Vec3f> prev_linear_velocities_;
};

}  // namespace novaphy
