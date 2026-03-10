#pragma once

#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief VBD/AVBD 配置，与 avbd-demo3d 的 Solver 参数一致。
 */
struct VBDConfig {
    float dt = 1.0f / 60.0f;
    Vec3f gravity = Vec3f(0.0f, -9.81f, 0.0f);
    int iterations = 10;
    int max_contacts_per_pair = 8;  ///< 与 demo3d 一致：每对体最多 8 个接触，支撑更对称、静止更稳

    /// 稳定化：C_eff 中保留的步初误差比例（demo: alpha=0.99）
    float alpha = 0.99f;
    /// 每步 penalty/lambda 暖启动衰减（demo: gamma=0.999）
    float gamma = 0.999f;

    /// 接触约束 penalty 增长系数（demo: betaLin=10000）
    float beta_linear = 10000.0f;
    /// 关节约束 penalty 增长系数（demo: betaAng=100）
    float beta_angular = 100.0f;
};

}  // namespace novaphy
