/**
 * @file state.cpp
 * @brief Mutable simulation state storage for body poses and velocities.
 */
#include "novaphy/sim/state.h"
#include "novaphy/core/contact.h"
#include <map>

namespace novaphy {

/**
 * @brief Initializes state arrays from the model's initial transforms.
 * @param[in] initial_transforms Initial world transforms for each body.
 */
void SimState::init(std::span<const Transform> initial_transforms) {
    const int n = static_cast<int>(initial_transforms.size());
    transforms.assign(initial_transforms.begin(), initial_transforms.end());
    linear_velocities.assign(n, Vec3f::Zero());
    angular_velocities.assign(n, Vec3f::Zero());
    forces.assign(n, Vec3f::Zero());
    torques.assign(n, Vec3f::Zero());
    sleeping.assign(n, 0);
    sleep_timer.assign(n, 0.0f);
    smoothed_energy.assign(n, 0.0f);
    island_id.assign(n, -1);
}

/**
 * @brief Clears all accumulated external forces and torques.
 */
void SimState::clear_forces() {
    for (auto& f : forces) f = Vec3f::Zero();
    for (auto& t : torques) t = Vec3f::Zero();
}

/**
 * @brief Sets linear velocity for one body index.
 * @param[in] body_index Body index in model order.
 * @param[in] vel Target linear velocity in world frame (m/s).
 */
void SimState::set_linear_velocity(int body_index, const Vec3f& vel) {
    if (body_index >= 0 && body_index < static_cast<int>(linear_velocities.size())) {
        linear_velocities[body_index] = vel;
    }
}

/**
 * @brief Sets angular velocity for one body index.
 * @param[in] body_index Body index in model order.
 * @param[in] vel Target angular velocity in world frame (rad/s).
 */
void SimState::set_angular_velocity(int body_index, const Vec3f& vel) {
    if (body_index >= 0 && body_index < static_cast<int>(angular_velocities.size())) {
        angular_velocities[body_index] = vel;
    }
}

/**
 * @brief Accumulates an external force on one body.
 * @param[in] body_index Body index in model order.
 * @param[in] force Force vector in world frame (N).
 */
void SimState::apply_force(int body_index, const Vec3f& force) {
    if (body_index >= 0 && body_index < static_cast<int>(forces.size())) {
        forces[body_index] += force;
    }
}

/**
 * @brief Accumulates an external torque on one body.
 * @param[in] body_index Body index in model order.
 * @param[in] torque Torque vector in world frame (N*m).
 */
void SimState::apply_torque(int body_index, const Vec3f& torque) {
    if (body_index >= 0 && body_index < static_cast<int>(torques.size())) {
        torques[body_index] += torque;
    }
}

/**
 * @brief Wakes a sleeping body, making it dynamic again.
 * @param[in] body_index Body index in model order.
 */
void SimState::wake_body(int body_index) {
    if (body_index >= 0 && body_index < static_cast<int>(sleeping.size())) {
        sleeping[body_index] = 0;
        sleep_timer[body_index] = 0.0f;
        smoothed_energy[body_index] = 0.02f;  // Prevent immediate re-sleep
        // Zero velocities to ensure clean start from rest
        linear_velocities[body_index] = Vec3f::Zero();
        angular_velocities[body_index] = Vec3f::Zero();
        // Propagate wake to all bodies in the same island
        propagate_wake_through_island(body_index);
    }
}

/**
 * @brief Checks if a body is currently sleeping.
 * @param[in] body_index Body index in model order.
 * @return true if body is sleeping, false otherwise.
 */
bool SimState::is_sleeping(int body_index) const {
    if (body_index >= 0 && body_index < static_cast<int>(sleeping.size())) {
        return sleeping[body_index] != 0;
    }
    return false;
}

/**
 * @brief Updates smoothed kinetic energy using EMA.
 * @param[in] body_index Body index.
 * @param[in] energy Current kinetic energy (unsmoothed).
 * @param[in] alpha EMA smoothing factor (0-1).
 */
void SimState::update_energy(int body_index, float energy, float alpha) {
    if (body_index >= 0 && body_index < static_cast<int>(smoothed_energy.size())) {
        smoothed_energy[body_index] = alpha * smoothed_energy[body_index] +
                                      (1.0f - alpha) * energy;
    }
}

/**
 * @brief Gets smoothed kinetic energy for a body.
 * @param[in] body_index Body index.
 * @return Smoothed energy value.
 */
float SimState::get_smoothed_energy(int body_index) const {
    if (body_index >= 0 && body_index < static_cast<int>(smoothed_energy.size())) {
        return smoothed_energy[body_index];
    }
    return 0.0f;
}

/**
 * @brief Builds islands from contact points using Union-Find algorithm.
 * @param[in] contacts Contact points for the current step.
 */
void SimState::build_islands(const std::vector<ContactPoint>& contacts) {
    const int n = static_cast<int>(sleeping.size());

    // Reset island IDs
    island_id.assign(n, -1);

    // Union-Find data structures
    std::vector<int> parent(n, -1);
    std::vector<int> rank(n, 0);

    // Union-Find helper functions
    auto find = [&parent](int x) {
        int root = x;
        while (parent[root] != root) {
            root = parent[root];
        }
        // Path compression
        while (parent[x] != root) {
            int next = parent[x];
            parent[x] = root;
            x = next;
        }
        return root;
    };

    auto union_sets = [&find, &parent, &rank](int x, int y) {
        int px = find(x);
        int py = find(y);
        if (px == py) return;

        // Union by rank
        if (rank[px] < rank[py]) {
            parent[px] = py;
        } else if (rank[px] > rank[py]) {
            parent[py] = px;
        } else {
            parent[py] = px;
            rank[px]++;
        }
    };

    // Initialize: only dynamic bodies participate in islands
    // Note: We need to check if body is static, but we don't have model_ access here
    // For now, we'll initialize all as potential island members
    for (int i = 0; i < n; ++i) {
        parent[i] = i;
    }

    // Union bodies that are in contact
    for (const auto& contact : contacts) {
        int a = contact.body_a;
        int b = contact.body_b;
        if (a >= 0 && b >= 0 && a < n && b < n) {
            union_sets(a, b);
        }
    }

    // Assign island IDs (compressed)
    int current_island = 0;
    for (int i = 0; i < n; ++i) {
        if (parent[i] >= 0) {
            int root = find(i);
            if (island_id[root] < 0) {
                island_id[root] = current_island++;
            }
            island_id[i] = island_id[root];
        }
    }
}

/**
 * @brief Evaluates sleep state based on energy thresholds (island-level).
 * @param[in] dt Time step in seconds.
 * @param[in] energy_threshold Energy threshold for sleep consideration.
 * @param[in] time_threshold Time below threshold before sleeping (seconds).
 */
void SimState::evaluate_sleep(float dt, float energy_threshold, float time_threshold) {
    const int n = static_cast<int>(sleeping.size());

    // Track island sleep eligibility
    std::map<int, bool> island_can_sleep;
    std::map<int, int> island_body_count;

    // First pass: accumulate time for bodies below threshold
    for (int i = 0; i < n; ++i) {
        int island = island_id[i];
        if (island < 0) continue;  // Not part of any island (static or isolated)

        island_body_count[island]++;

        if (smoothed_energy[i] < energy_threshold) {
            sleep_timer[i] += dt;
        } else {
            sleep_timer[i] = 0.0f;
            island_can_sleep[island] = false;  // Island has an awake body
        }
    }

    // Second pass: check which islands can sleep
    for (int i = 0; i < n; ++i) {
        int island = island_id[i];
        if (island < 0) continue;

        // Island can only sleep if ALL bodies have been below threshold for long enough
        if (island_can_sleep.find(island) == island_can_sleep.end()) {
            // Check if all bodies in this island meet time threshold
            bool all_meet_threshold = true;
            for (int j = 0; j < n; ++j) {
                if (island_id[j] == island && sleep_timer[j] < time_threshold) {
                    all_meet_threshold = false;
                    break;
                }
            }
            if (all_meet_threshold) {
                island_can_sleep[island] = true;
            }
        }
    }

    // Third pass: apply sleep state
    for (int i = 0; i < n; ++i) {
        int island = island_id[i];
        if (island < 0) {
            sleeping[i] = 0;  // Not in an island, keep awake
            continue;
        }

        if (island_can_sleep[island]) {
            sleeping[i] = 1;
            linear_velocities[i] = Vec3f::Zero();
            angular_velocities[i] = Vec3f::Zero();
        } else {
            sleeping[i] = 0;
        }
    }
}

/**
 * @brief Propagates wake state to all bodies in the same island.
 * @param[in] body_index Body that was woken up.
 */
void SimState::propagate_wake_through_island(int body_index) {
    if (body_index < 0 || body_index >= static_cast<int>(island_id.size())) return;

    int target_island = island_id[body_index];
    if (target_island < 0) return;  // Not in an island

    for (int i = 0; i < static_cast<int>(island_id.size()); ++i) {
        if (island_id[i] == target_island && sleeping[i] != 0) {
            sleeping[i] = 0;
            sleep_timer[i] = 0.0f;
            smoothed_energy[i] = 0.02f;  // Prevent immediate re-sleep (2x default threshold)
            // Note: we don't zero velocities here, let them respond to the wake stimulus
        }
    }
}

}  // namespace novaphy
