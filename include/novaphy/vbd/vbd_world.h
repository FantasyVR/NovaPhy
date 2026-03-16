#pragma once

#include "novaphy/core/model.h"
#include "novaphy/sim/state.h"
#include "novaphy/vbd/vbd_config.h"

#include <limits>
#include <memory>

namespace novaphy {

/**
 * @brief VBD/AVBD-based simulation world.
 *
 * The public interface mirrors `World` / `IPCWorld` as closely as possible:
 *
 * - Uses `Model` as an immutable scene description;
 * - Maintains a mutable `SimState` buffer internally;
 * - Exposes a simple `step()` method that advances time and updates `state()`.
 *
 * All VBD/AVBD solver details are hidden behind a pimpl so you can iterate
 * on algorithms and GPU backends without breaking the public API.
 */
class VBDWorld {
public:
    /**
     * @brief Construct a VBD world from a NovaPhy model.
     *
     * @param model  Immutable model containing bodies, shapes and initial transforms.
     * @param config VBD/AVBD configuration parameters.
     */
    explicit VBDWorld(const Model& model, const VBDConfig& config = VBDConfig{});
    ~VBDWorld();

    // Move-only
    VBDWorld(VBDWorld&&) noexcept;
    VBDWorld& operator=(VBDWorld&&) noexcept;
    VBDWorld(const VBDWorld&) = delete;
    VBDWorld& operator=(const VBDWorld&) = delete;

    /**
     * @brief Advance the VBD simulation by one timestep.
     *
     * Time integration, constraint solving, and (in the future) GPU
     * acceleration are all implemented inside the hidden implementation
     * object. This public method remains a simple `step()` call.
     */
    void step();

    // demo3d-style AVBD constraints/forces
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

    /// Access current simulation state (positions, velocities).
    SimState& state();
    const SimState& state() const;

    /// Access the model used to build this world.
    const Model& model() const;

    /// Access the VBD configuration.
    const VBDConfig& config() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace novaphy

