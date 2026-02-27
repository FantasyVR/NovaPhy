/**
 * @file broadphase.cpp
 * @brief Sweep-and-prune broadphase collision detection.
 */
#include "novaphy/collision/broadphase.h"

#include <algorithm>

namespace novaphy {

/**
 * @brief Updates broadphase acceleration structures for current AABBs.
 * @param[in] body_aabbs Per-body world-space AABBs.
 * @param[in] static_mask Flags indicating static (non-moving) bodies.
 */
void SweepAndPrune::update(const std::vector<AABB>& body_aabbs,
                            const std::vector<bool>& static_mask) {
    rebuild(body_aabbs, static_mask);
}

/**
 * @brief Rebuilds endpoint arrays and overlapping broadphase pairs.
 * @details Uses axis sweep on X and interval overlap filtering on Y/Z.
 * @param[in] aabbs Per-body world-space AABBs.
 * @param[in] static_mask Flags indicating static (non-moving) bodies.
 */
void SweepAndPrune::rebuild(const std::vector<AABB>& aabbs,
                             const std::vector<bool>& static_mask) {
    int n = static_cast<int>(aabbs.size());

    // Build endpoint arrays (2 endpoints per body per axis)
    endpoints_x_.resize(2 * n);
    endpoints_y_.resize(2 * n);
    endpoints_z_.resize(2 * n);

    for (int i = 0; i < n; ++i) {
        endpoints_x_[2 * i] = {aabbs[i].min.x(), i, true};
        endpoints_x_[2 * i + 1] = {aabbs[i].max.x(), i, false};
        endpoints_y_[2 * i] = {aabbs[i].min.y(), i, true};
        endpoints_y_[2 * i + 1] = {aabbs[i].max.y(), i, false};
        endpoints_z_[2 * i] = {aabbs[i].min.z(), i, true};
        endpoints_z_[2 * i + 1] = {aabbs[i].max.z(), i, false};
    }

    // Sort endpoints (insertion sort for temporal coherence in subsequent frames)
    if (initialized_) {
        insertion_sort(endpoints_x_);
        insertion_sort(endpoints_y_);
        insertion_sort(endpoints_z_);
    } else {
        auto cmp = [](const Endpoint& a, const Endpoint& b) {
            return a.value < b.value;
        };
        std::sort(endpoints_x_.begin(), endpoints_x_.end(), cmp);
        std::sort(endpoints_y_.begin(), endpoints_y_.end(), cmp);
        std::sort(endpoints_z_.begin(), endpoints_z_.end(), cmp);
        initialized_ = true;
    }

    // Find overlapping pairs: sweep along x-axis, then filter by y and z
    pairs_.clear();

    for (int i = 0; i < static_cast<int>(endpoints_x_.size()); ++i) {
        if (!endpoints_x_[i].is_min) continue;  // only process min endpoints

        int body_a = endpoints_x_[i].body_index;
        float max_a = aabbs[body_a].max.x();

        // Sweep forward through subsequent endpoints
        for (int j = i + 1; j < static_cast<int>(endpoints_x_.size()); ++j) {
            if (endpoints_x_[j].value > max_a) break;  // no more overlaps on x
            if (!endpoints_x_[j].is_min) continue;      // skip max endpoints

            int body_b = endpoints_x_[j].body_index;

            // Skip same body
            if (body_a == body_b) continue;

            // Skip static-static pairs
            if (static_mask[body_a] && static_mask[body_b]) continue;

            // Check y and z overlap
            if (aabbs[body_a].min.y() <= aabbs[body_b].max.y() &&
                aabbs[body_a].max.y() >= aabbs[body_b].min.y() &&
                aabbs[body_a].min.z() <= aabbs[body_b].max.z() &&
                aabbs[body_a].max.z() >= aabbs[body_b].min.z()) {
                // Canonical order: smaller index first
                int a = std::min(body_a, body_b);
                int b = std::max(body_a, body_b);
                pairs_.push_back({a, b});
            }
        }
    }

    // Remove duplicate pairs
    std::sort(pairs_.begin(), pairs_.end(), [](const BroadPhasePair& a, const BroadPhasePair& b) {
        return a.body_a < b.body_a || (a.body_a == b.body_a && a.body_b < b.body_b);
    });
    pairs_.erase(std::unique(pairs_.begin(), pairs_.end()), pairs_.end());
}

/**
 * @brief In-place insertion sort for temporally coherent endpoint lists.
 * @param[in,out] eps Endpoint array to sort by scalar axis coordinate.
 */
void SweepAndPrune::insertion_sort(std::vector<Endpoint>& eps) {
    for (int i = 1; i < static_cast<int>(eps.size()); ++i) {
        Endpoint key = eps[i];
        int j = i - 1;
        while (j >= 0 && eps[j].value > key.value) {
            eps[j + 1] = eps[j];
            --j;
        }
        eps[j + 1] = key;
    }
}

}  // namespace novaphy
