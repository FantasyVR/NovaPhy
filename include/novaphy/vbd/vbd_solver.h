#pragma once

#include "novaphy/core/model.h"
#include "novaphy/sim/state.h"
#include "novaphy/vbd/vbd_config.h"

#include "novaphy/collision/broadphase.h"
#include "novaphy/collision/narrowphase.h"
#include "novaphy/core/contact.h"
#include "novaphy/math/math_types.h"
#include "novaphy/math/spatial.h"

#include <vector>

namespace novaphy {

/**
 * @brief 单点接触约束（与 avbd-demo3d Manifold::Contact 一致）。
 *
 * 使用 3 维约束：法向 + 两个切向。basis 行0 = 法向（B→A），行1/2 = 切向。
 * C0 = basis * (xA - xB) + {COLLISION_MARGIN, 0, 0}，F = K*C + lambda。
 */
struct AvbdContact {
    int body_a = -1;
    int body_b = -1;
    Vec3f rA = Vec3f::Zero();  // 接触点在 A 局部系
    Vec3f rB = Vec3f::Zero();  // 接触点在 B 局部系
    Mat3f basis = Mat3f::Identity();  // 行0=法向(B→A)，行1/2=切向

    Vec3f C0 = Vec3f::Zero();   // 步初约束值
    Vec3f penalty = Vec3f::Zero();
    Vec3f lambda = Vec3f::Zero();
    float friction = 0.5f;
    bool stick = false;

    // 可选：从 narrowphase 传递过来的特征 id，用于接触持久化（类似 demo3d FeaturePair::key）。
    int feature_id = -1;
};

/**
 * @brief 3D AVBD 求解器，完全仿照 avbd-demo3d 的 Solver 流程与公式。
 *
 * 流程：broadphase → 建接触并 initialize（C0、warmstart）→ 体初始化（惯性位姿、initial）→
 * 主循环（primal 每体 6x6 → dual 更新）→ BDF1 速度。
 */
class VbdSolver {
public:
    explicit VbdSolver(const VBDConfig& cfg);

    void set_config(const VBDConfig& cfg);
    const VBDConfig& config() const { return config_; }

    void set_model(const Model& model);

    /**
     * @brief 单步 AVBD：与 demo3d Solver::step() 一致。
     */
    void step(const Model& model, SimState& state);

private:
    /** 每步开始时建接触列表并填 C0、warmstart lambda/penalty。 */
    void build_contact_constraints(const Model& model, const SimState& state);

    /** 主循环：每体组装 6x6 LHS/RHS，求解并施加 dq。 */
    void avbd_primal(const Model& model, SimState& state);
    /** 主循环：对偶更新 lambda、penalty。 */
    void avbd_dual(const Model& model, const SimState& state);

    VBDConfig config_;
    SweepAndPrune broadphase_;
    std::vector<ContactPoint> contacts_;
    std::vector<AvbdContact> avbd_contacts_;
    std::vector<Vec3f> inertial_positions_;
    std::vector<Quatf> inertial_rotations_;
    std::vector<Vec3f> initial_positions_;
    std::vector<Quatf> initial_rotations_;
    std::vector<Vec3f> prev_linear_velocities_;
};

}  // namespace novaphy
