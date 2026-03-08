#pragma once

#include <cstdint>
#include <span>
#include <utility>
#include <unordered_map>
#include <vector>

#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief Uniform spatial hash grid for SPH neighbor queries.
 *
 * @details Provides O(1) amortized neighbor lookup within a given radius.
 * Cell size should match the SPH kernel support radius.
 */
class SpatialHashGrid {
public:
    /**
     * @brief Construct a spatial hash grid with given cell size.
     *
     * @param[in] cell_size Size of each grid cell (should equal kernel radius h).
     */
    explicit SpatialHashGrid(float cell_size = 0.1f);

    /**
     * @brief Build the grid from particle positions.
     *
     * @param[in] positions Particle positions to insert.
     */
    void build(std::span<const Vec3f> positions);

    /**
     * @brief Clear all grid data.
     */
    void clear();

    /**
     * @brief Query all neighbor particle indices within radius of a point.
     *
     * @param[in] point Query point.
     * @param[in] radius Search radius.
     * @return Candidate neighbor particle indices from nearby occupied cells.
     */
    std::vector<int> query_neighbors(const Vec3f& point, float radius) const;

    /**
     * @brief Query all neighbor pairs. Each pair (i, j) with i < j and distance < radius.
     *
     * @param[in] positions Particle positions.
     * @param[in] radius Search radius.
     * @return All pairs of particle indices whose distance is smaller than `radius`.
     */
    std::vector<std::pair<int, int>> query_all_pairs(std::span<const Vec3f> positions,
                                                     float radius) const;

    /**
     * @brief Get cell size.
     *
     * @return Cell size in meters.
     */
    float cell_size() const { return cell_size_; }

    /**
     * @brief Set cell size. Grid must be rebuilt after changing this.
     *
     * @param[in] size New cell size in meters.
     */
    void set_cell_size(float size) { cell_size_ = size; }

private:
    /**
     * @brief Compute spatial hash for a cell coordinate.
     */
    static uint64_t hash_cell(int cx, int cy, int cz);

    /**
     * @brief Convert world position to cell coordinates.
     */
    void world_to_cell(const Vec3f& pos, int& cx, int& cy, int& cz) const;

    float cell_size_;
    std::unordered_map<uint64_t, std::vector<int>> cells_;
};

}  // namespace novaphy
