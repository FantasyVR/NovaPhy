#pragma once

#include <algorithm>
#include <unordered_set>
#include <vector>

#include "novaphy/core/aabb.h"

namespace novaphy {

/**
 * @brief Candidate overlap pair produced by broadphase.
 *
 * @details Each entry stores indices into the shape/body arrays used by
 * narrowphase contact generation.
 */
struct BroadPhasePair {
    int body_a;  /**< First index in canonical ascending order. */
    int body_b;  /**< Second index in canonical ascending order. */

    /**
     * @brief Equality comparison for pair deduplication.
     *
     * @param [in] other Another broadphase pair.
     * @return True if both indices match exactly.
     */
    bool operator==(const BroadPhasePair& other) const {
        return body_a == other.body_a && body_b == other.body_b;
    }
};

/**
 * @brief Sweep-and-Prune broadphase collider for axis-aligned bounds.
 *
 * @details Maintains sorted min/max endpoints and computes candidate overlap
 * pairs before expensive narrowphase tests. This stage filters impossible
 * collisions and reduces overall contact-detection cost.
 */
class SweepAndPrune {
public:
    /**
     * @brief Construct an empty broadphase accelerator.
     */
    SweepAndPrune() = default;

    /**
     * @brief Update candidate overlap pairs from current world-space AABBs.
     *
     * @param [in] body_aabbs Axis-aligned bounding boxes in world coordinates.
     * @param [in] static_mask Static-body mask used to skip static-static pairs.
     * @return void
     */
    void update(const std::vector<AABB>& body_aabbs,
                const std::vector<bool>& static_mask);

    /**
     * @brief Get broadphase overlap candidates from the latest update.
     *
     * @return Read-only list of potentially overlapping pairs.
     */
    const std::vector<BroadPhasePair>& get_pairs() const { return pairs_; }

private:
    /**
     * @brief One scalar endpoint in a sweep axis list.
     */
    struct Endpoint {
        float value;      /**< Endpoint coordinate value along one axis (m). */
        int body_index;   /**< Index of owning body/shape AABB. */
        bool is_min;      /**< True for minimum bound, false for maximum bound. */
    };

    std::vector<Endpoint> endpoints_x_;
    std::vector<Endpoint> endpoints_y_;
    std::vector<Endpoint> endpoints_z_;
    std::vector<BroadPhasePair> pairs_;

    bool initialized_ = false;

    /**
     * @brief Rebuild axis endpoint arrays and refresh overlap pairs.
     *
     * @param [in] aabbs World-space AABBs.
     * @param [in] static_mask Static-body mask.
     * @return void
     */
    void rebuild(const std::vector<AABB>& aabbs, const std::vector<bool>& static_mask);

    /**
     * @brief Insertion-sort endpoint array for temporal-coherence-friendly updates.
     *
     * @param [in,out] eps Endpoint list to sort in ascending scalar order.
     * @return void
     */
    static void insertion_sort(std::vector<Endpoint>& eps);
};

}  // namespace novaphy
