#pragma once

#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief Configuration for IPCWorld simulation.
 *
 * Controls libuipc backend parameters including timestep, contact tolerances,
 * friction, and Newton solver settings.
 */
struct IPCConfig {
    float dt = 0.01f;                       /**< Timestep in seconds. */
    Vec3f gravity = Vec3f(0.0f, -9.81f, 0.0f); /**< Gravity acceleration (m/s^2). */

    // IPC contact parameters
    float d_hat = 0.01f;      /**< Contact distance threshold (m), mapped to scene `contact/d_hat`. */
    float kappa = 1e8f;       /**< Barrier/contact stiffness (Pa), used as default contact resistance. */
    float friction = 0.5f;    /**< Default friction coefficient. */
    float contact_resistance = 1e9f;  /**< Optional contact-resistance override (Pa), if customized. */

    // Affine body material
    float body_kappa = 1e8f;      /**< Affine body stiffness (Pa). */
    float mass_density = 1e3f;    /**< Default mass density (kg/m^3). */

    // Newton solver
    int newton_max_iter = 100;    /**< Max Newton iterations per step (`newton/max_iter`). */
    float newton_tol = 1e-2f;     /**< Newton tolerance (`newton/velocity_tol` and `newton/transrate_tol`). */

    // Backend
    const char* backend = "cuda"; /**< libuipc backend name ("cuda" or "none"). */
    const char* workspace = "./ipc_workspace"; /**< Output workspace directory. */
};

}  // namespace novaphy
