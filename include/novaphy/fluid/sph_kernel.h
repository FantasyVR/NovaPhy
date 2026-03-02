#pragma once

#include <cmath>

#include "novaphy/math/math_types.h"
#include "novaphy/math/math_utils.h"

namespace novaphy {

/**
 * @brief SPH smoothing kernels for fluid simulation.
 *
 * @details All kernels operate in 3D with support radius h.
 * Uses standard Muller et al. 2003 formulations.
 */
struct SPHKernels {
    /**
     * @brief Poly6 kernel for density estimation.
     *
     * @param[in] r_sq Squared distance between particles.
     * @param[in] h Support radius.
     * @return Kernel value W(r, h).
     */
    static float poly6(float r_sq, float h) {
        if (r_sq >= h * h) return 0.0f;
        float h2 = h * h;
        float diff = h2 - r_sq;
        float h9 = h2 * h2 * h2 * h2 * h;  // h^9
        return (315.0f / (64.0f * PI * h9)) * diff * diff * diff;
    }

    /**
     * @brief Gradient of the Spiky kernel for pressure forces.
     *
     * @param[in] r Vector from particle j to particle i (r_i - r_j).
     * @param[in] h Support radius.
     * @return Gradient vector of spiky kernel.
     */
    static Vec3f spiky_grad(const Vec3f& r, float h) {
        float r_len = r.norm();
        if (r_len <= 1e-7f || r_len >= h) return Vec3f::Zero();
        float diff = h - r_len;
        float h6 = h * h * h * h * h * h;  // h^6
        float coeff = -(45.0f / (PI * h6)) * diff * diff / r_len;
        return coeff * r;
    }

    /**
     * @brief Spiky kernel value (used in PBF gradient computation).
     *
     * @param[in] r_len Distance between particles.
     * @param[in] h Support radius.
     * @return Kernel value W(r, h).
     */
    static float spiky(float r_len, float h) {
        if (r_len >= h || r_len < 0.0f) return 0.0f;
        float diff = h - r_len;
        float h6 = h * h * h * h * h * h;  // h^6
        return (15.0f / (PI * h6)) * diff * diff * diff;
    }

    /**
     * @brief Cubic spline kernel (alternative, more stable for PBF).
     *
     * @param[in] r_len Distance between particles.
     * @param[in] h Support radius.
     * @return Kernel value W(r, h).
     */
    static float cubic_spline(float r_len, float h) {
        float q = r_len / h;
        if (q >= 1.0f) return 0.0f;
        float h3 = h * h * h;
        float sigma = 8.0f / (PI * h3);
        if (q <= 0.5f) {
            return sigma * (6.0f * (q * q * q - q * q) + 1.0f);
        } else {
            float t = 1.0f - q;
            return sigma * 2.0f * t * t * t;
        }
    }
};

}  // namespace novaphy
