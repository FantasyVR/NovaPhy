/**
 * @file neighbor_search.cpp
 * @brief Uniform spatial hash grid for SPH neighbor queries.
 */
#include "novaphy/fluid/neighbor_search.h"

#include <cmath>

namespace novaphy {

SpatialHashGrid::SpatialHashGrid(float cell_size)
    : cell_size_(cell_size) {}

void SpatialHashGrid::build(std::span<const Vec3f> positions) {
    cells_.clear();
    for (size_t i = 0; i < positions.size(); ++i) {
        int cx, cy, cz;
        world_to_cell(positions[i], cx, cy, cz);
        const uint64_t key = hash_cell(cx, cy, cz);
        cells_[key].push_back(static_cast<int>(i));
    }
}

void SpatialHashGrid::clear() {
    cells_.clear();
}

std::vector<int> SpatialHashGrid::query_neighbors(const Vec3f& point, float radius) const {
    (void)radius;
    std::vector<int> neighbors;

    int cx, cy, cz;
    world_to_cell(point, cx, cy, cz);

    // Search 3x3x3 neighborhood of cells
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                uint64_t key = hash_cell(cx + dx, cy + dy, cz + dz);
                auto it = cells_.find(key);
                if (it == cells_.end()) continue;
                for (int idx : it->second) {
                    neighbors.push_back(idx);
                }
            }
        }
    }

    return neighbors;
}

std::vector<std::pair<int, int>> SpatialHashGrid::query_all_pairs(
    std::span<const Vec3f> positions,
    float radius) const {
    std::vector<std::pair<int, int>> pairs;
    const float radius_sq = radius * radius;

    for (size_t i = 0; i < positions.size(); ++i) {
        int cx, cy, cz;
        world_to_cell(positions[i], cx, cy, cz);

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    uint64_t key = hash_cell(cx + dx, cy + dy, cz + dz);
                    auto it = cells_.find(key);
                    if (it == cells_.end()) continue;
                    for (int j : it->second) {
                        if (j <= static_cast<int>(i)) continue;  // only i < j pairs
                        const float dist_sq =
                            (positions[i] - positions[static_cast<size_t>(j)]).squaredNorm();
                        if (dist_sq < radius_sq) {
                            pairs.emplace_back(static_cast<int>(i), j);
                        }
                    }
                }
            }
        }
    }

    return pairs;
}

uint64_t SpatialHashGrid::hash_cell(int cx, int cy, int cz) {
    // Large primes for spatial hashing (Teschner et al. 2003)
    const uint64_t p1 = 73856093ULL;
    const uint64_t p2 = 19349663ULL;
    const uint64_t p3 = 83492791ULL;
    return (static_cast<uint64_t>(cx) * p1) ^
           (static_cast<uint64_t>(cy) * p2) ^
           (static_cast<uint64_t>(cz) * p3);
}

void SpatialHashGrid::world_to_cell(const Vec3f& pos,
                                     int& cx, int& cy, int& cz) const {
    float inv = 1.0f / cell_size_;
    cx = static_cast<int>(std::floor(pos.x() * inv));
    cy = static_cast<int>(std::floor(pos.y() * inv));
    cz = static_cast<int>(std::floor(pos.z() * inv));
}

}  // namespace novaphy
