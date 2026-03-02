#pragma once

#include <vector>

#include "novaphy/core/model.h"
#include "novaphy/core/shape.h"
#include "novaphy/fluid/neighbor_search.h"
#include "novaphy/fluid/particle_state.h"
#include "novaphy/math/math_types.h"

namespace novaphy {

/**
 * @brief A boundary particle sampled from a rigid body surface.
 *
 * @details Used in Akinci et al. (2012) versatile rigid-fluid coupling.
 * Each boundary particle has a local position (in body frame) and a
 * precomputed volume psi for density contribution.
 */
struct BoundaryParticle {
    Vec3f local_position = Vec3f::Zero();  /**< Position in body-local frame (m). */
    int body_index = -1;                    /**< Owning rigid body index (-1 = static/world). */
    float volume = 0.0f;                    /**< Akinci volume psi for density contribution. */
};

/**
 * @brief Sample boundary particles on the surface of a box shape.
 *
 * @param[in] shape Box collision shape.
 * @param[in] spacing Particle spacing (m).
 * @return Vector of boundary particles in body-local frame.
 */
std::vector<BoundaryParticle> sample_box_boundary(const CollisionShape& shape, float spacing);

/**
 * @brief Sample boundary particles on the surface of a sphere shape.
 *
 * @param[in] shape Sphere collision shape.
 * @param[in] spacing Particle spacing (m).
 * @return Vector of boundary particles in body-local frame.
 */
std::vector<BoundaryParticle> sample_sphere_boundary(const CollisionShape& shape, float spacing);

/**
 * @brief Sample boundary particles on a plane (finite region).
 *
 * @param[in] shape Plane collision shape.
 * @param[in] spacing Particle spacing (m).
 * @param[in] extent Half-extent of the sampled region (m).
 * @return Vector of boundary particles.
 */
std::vector<BoundaryParticle> sample_plane_boundary(const CollisionShape& shape,
                                                     float spacing, float extent = 1.0f);

/**
 * @brief Sample boundary particles from all shapes in a model.
 *
 * @param[in] model Scene model with shapes.
 * @param[in] spacing Particle spacing for boundary sampling (m).
 * @param[in] plane_extent Half-extent for plane boundary regions (m).
 * @return All boundary particles for the model.
 */
std::vector<BoundaryParticle> sample_model_boundaries(const Model& model, float spacing,
                                                       float plane_extent = 1.0f);

/**
 * @brief Compute Akinci boundary particle volumes.
 *
 * @details Volume psi_b = 1 / sum_b' W(x_b - x_b', h) over all boundary
 * neighbors. Called once after generating boundary particles.
 *
 * @param[in,out] particles Boundary particles (volume field is updated).
 * @param[in] world_positions World-frame positions of boundary particles.
 * @param[in] kernel_radius SPH kernel support radius (m).
 */
void compute_boundary_volumes(std::vector<BoundaryParticle>& particles,
                              const std::vector<Vec3f>& world_positions,
                              float kernel_radius);

/**
 * @brief Compute world-frame positions of boundary particles.
 *
 * @param[in] particles Boundary particles with local positions.
 * @param[in] transforms Body transforms.
 * @return World-frame positions.
 */
std::vector<Vec3f> boundary_world_positions(
    const std::vector<BoundaryParticle>& particles,
    const std::vector<Transform>& transforms);

}  // namespace novaphy
