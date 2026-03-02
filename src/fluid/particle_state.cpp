/**
 * @file particle_state.cpp
 * @brief Fluid particle state storage and block generation.
 */
#include "novaphy/fluid/particle_state.h"

#include <cmath>

namespace novaphy {

void ParticleState::init(const std::vector<Vec3f>& initial_positions,
                         const Vec3f& initial_velocity) {
    int n = static_cast<int>(initial_positions.size());
    positions = initial_positions;
    velocities.assign(n, initial_velocity);
    densities.assign(n, 0.0f);
    lambdas.assign(n, 0.0f);
    predicted_positions = initial_positions;
    delta_positions.assign(n, Vec3f::Zero());
}

void ParticleState::clear() {
    positions.clear();
    velocities.clear();
    densities.clear();
    lambdas.clear();
    predicted_positions.clear();
    delta_positions.clear();
}

std::vector<Vec3f> generate_fluid_block(const FluidBlockDef& def) {
    std::vector<Vec3f> positions;

    float spacing = def.particle_spacing;
    if (spacing <= 0.0f) return positions;

    // Count particles per axis
    int nx = std::max(1, static_cast<int>(std::floor((def.upper.x() - def.lower.x()) / spacing)) + 1);
    int ny = std::max(1, static_cast<int>(std::floor((def.upper.y() - def.lower.y()) / spacing)) + 1);
    int nz = std::max(1, static_cast<int>(std::floor((def.upper.z() - def.lower.z()) / spacing)) + 1);

    positions.reserve(nx * ny * nz);

    for (int iz = 0; iz < nz; ++iz) {
        for (int iy = 0; iy < ny; ++iy) {
            for (int ix = 0; ix < nx; ++ix) {
                Vec3f pos(
                    def.lower.x() + ix * spacing,
                    def.lower.y() + iy * spacing,
                    def.lower.z() + iz * spacing
                );
                positions.push_back(pos);
            }
        }
    }

    return positions;
}

}  // namespace novaphy
