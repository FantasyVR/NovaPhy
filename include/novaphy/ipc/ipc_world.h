#pragma once

#include "novaphy/core/model.h"
#include "novaphy/ipc/ipc_config.h"
#include "novaphy/sim/state.h"

#include <memory>
#include <vector>

namespace novaphy {

/**
 * @brief IPC-based simulation world using libuipc as backend.
 *
 * Provides the same high-level interface as World, but delegates contact
 * resolution to libuipc's GPU-accelerated Incremental Potential Contact
 * solver, which mathematically guarantees no interpenetration.
 *
 * Implementation is hidden behind a pimpl to keep libuipc headers out of the
 * public NovaPhy interface.
 */
class IPCWorld {
public:
    /**
     * @brief Construct an IPC world from a NovaPhy model.
     *
     * Converts all bodies and shapes to libuipc geometry, configures the
     * IPC engine, and initializes the simulation scene.
     *
     * @param model NovaPhy model containing bodies, shapes, and transforms.
     * @param config IPC configuration parameters.
     */
    explicit IPCWorld(const Model& model, const IPCConfig& config = IPCConfig{});
    ~IPCWorld();

    // Move-only
    IPCWorld(IPCWorld&&) noexcept;
    IPCWorld& operator=(IPCWorld&&) noexcept;
    IPCWorld(const IPCWorld&) = delete;
    IPCWorld& operator=(const IPCWorld&) = delete;

    /**
     * @brief Advance the IPC simulation by one timestep.
     *
     * Calls libuipc advance() + sync() + retrieve(), then extracts the
     * resulting rigid transforms back into the NovaPhy SimState.
     */
    void step();

    /**
     * @brief Access current simulation state (positions, velocities).
     */
    SimState& state();
    const SimState& state() const;

    /**
     * @brief Access the model used to build this world.
     */
    const Model& model() const;

    /**
     * @brief Get the IPC configuration.
     */
    const IPCConfig& config() const;

    /**
     * @brief Get the current simulation frame number.
     */
    int frame() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace novaphy
