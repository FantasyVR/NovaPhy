#pragma once

#include <vector>

#include "novaphy/collision/broadphase.h"
#include "novaphy/collision/narrowphase.h"
#include "novaphy/core/contact.h"
#include "novaphy/core/model.h"
#include "novaphy/dynamics/free_body_solver.h"
#include "novaphy/dynamics/integrator.h"
#include "novaphy/sim/state.h"

namespace novaphy {

/**
 * @brief High-level free-rigid-body simulation container.
 *
 * @details Owns model topology, mutable simulation state, broadphase collision
 * detector, and contact solver. `step()` advances one fixed-size time step in
 * SI units using gravity, collision detection, and impulse-based resolution.
 */
class World {
public:
    /**
     * @brief Construct a simulation world from an immutable model.
     *
     * @param [in] model Immutable body/shape model definition.
     * @param [in] solver_settings Contact-solver tuning parameters.
     */
    explicit World(const Model& model, SolverSettings solver_settings = {});

    /**
     * @brief Advance the simulation by one time step.
     *
     * @param [in] dt Time step in seconds.
     * @return void
     */
    void step(float dt);

    /**
     * @brief Set the world gravity vector.
     *
     * @param [in] g Gravity in world frame (m/s^2).
     * @return void
     */
    void set_gravity(const Vec3f& g) { gravity_ = g; }

    /**
     * @brief Get the current world gravity vector.
     *
     * @return Gravity in world frame (m/s^2).
     */
    const Vec3f& gravity() const { return gravity_; }

    /**
     * @brief Mutable access to simulation state buffers.
     *
     * @return Reference to SimState.
     */
    SimState& state() { return state_; }

    /**
     * @brief Read-only access to simulation state buffers.
     *
     * @return Const reference to SimState.
     */
    const SimState& state() const { return state_; }

    /**
     * @brief Access the immutable model definition.
     *
     * @return Const reference to the model.
     */
    const Model& model() const { return model_; }

    /**
     * @brief Mutable access to contact-solver settings.
     *
     * @return Reference to solver settings.
     */
    SolverSettings& solver_settings() { return solver_.settings(); }

    /**
     * @brief Contact points generated in the most recent `step()`.
     *
     * @return Const reference to world-space contact list.
     */
    const std::vector<ContactPoint>& contacts() const { return contacts_; }

    /**
     * @brief Accumulate an external force for the next integration step.
     *
     * @param [in] body_index Body index.
     * @param [in] force Force in world frame (N).
     * @return void
     */
    void apply_force(int body_index, const Vec3f& force);

    /**
     * @brief Accumulate an external torque for the next integration step.
     *
     * @param [in] body_index Body index.
     * @param [in] torque Torque in world frame (N*m).
     * @return void
     */
    void apply_torque(int body_index, const Vec3f& torque);

private:
    Model model_;
    SimState state_;
    SweepAndPrune broadphase_;
    FreeBodySolver solver_;
    Vec3f gravity_ = Vec3f(0.0f, -9.81f, 0.0f);
    std::vector<ContactPoint> contacts_;
};

}  // namespace novaphy
