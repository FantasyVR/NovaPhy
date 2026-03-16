#pragma once

#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief VBD/AVBD configuration, aligned with avbd-demo3d solver parameters.
 */
enum class VbdBackend : int {
    CPU = 0,   ///< Run AVBD on CPU (current default implementation).
    CUDA = 1,  ///< Run AVBD on a CUDA backend (experimental).
};

struct VBDConfig {
    float dt = 1.0f / 60.0f;
    Vec3f gravity = Vec3f(0.0f, -9.81f, 0.0f);
    int iterations = 10;
    /// Max contact points per shape pair (demo3d uses 8 for more symmetric support).
    int max_contacts_per_pair = 8;

    /// Stabilization: fraction of step-initial error kept in C_eff (demo: alpha=0.99).
    float alpha = 0.99f;
    /// Warmstart decay for penalty/lambda per step (demo: gamma=0.999).
    float gamma = 0.999f;

    /// Penalty growth factor for contact constraints (demo: betaLin=10000).
    float beta_linear = 10000.0f;
    /// Penalty growth factor for angular constraints (demo: betaAng=100).
    float beta_angular = 100.0f;

    /// Initial penalty for new contacts (reduces first-frame overshoot/jitter when > PENALTY_MIN).
    float initial_penalty = 1000.0f;
    /// Velocity smoothing factor in (0,1]: 1 = no smoothing, lower = smoother (e.g. 0.7 reduces jitter).
    float velocity_smoothing = 1.0f;

    /// Primal update relaxation (demo3d uses 1.0). Values < 1 (e.g. 0.9) reduce overshoot and can reduce jitter.
    float primal_relaxation = 1.0f;
    /// Small diagonal regularization added to 6x6 LHS before solve (improves conditioning, reduces numerical jitter).
    float lhs_regularization = 1e-6f;

    /// Backend selection for the AVBD solver. CPU is the default; CUDA is experimental.
    VbdBackend backend = VbdBackend::CPU;
};

}  // namespace novaphy
