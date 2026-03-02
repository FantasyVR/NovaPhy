/**
 * @file boundary.cpp
 * @brief Boundary particle sampling and Akinci volume computation.
 */
#include "novaphy/fluid/boundary.h"

#include <cmath>

#include "novaphy/fluid/sph_kernel.h"
#include "novaphy/math/math_utils.h"

namespace novaphy {

std::vector<BoundaryParticle> sample_box_boundary(const CollisionShape& shape,
                                                   float spacing) {
    std::vector<BoundaryParticle> particles;
    if (shape.type != ShapeType::Box) return particles;

    const Vec3f& he = shape.box.half_extents;
    int body_idx = shape.body_index;

    // Sample 6 faces of the box
    // For each face, sample a 2D grid within the face bounds
    for (int axis = 0; axis < 3; ++axis) {
        int a1 = (axis + 1) % 3;
        int a2 = (axis + 2) % 3;

        int n1 = std::max(1, static_cast<int>(std::floor(2.0f * he[a1] / spacing)) + 1);
        int n2 = std::max(1, static_cast<int>(std::floor(2.0f * he[a2] / spacing)) + 1);

        for (int sign = -1; sign <= 1; sign += 2) {
            for (int i1 = 0; i1 < n1; ++i1) {
                for (int i2 = 0; i2 < n2; ++i2) {
                    Vec3f local_pos = Vec3f::Zero();
                    local_pos[axis] = sign * he[axis];
                    local_pos[a1] = -he[a1] + i1 * spacing;
                    local_pos[a2] = -he[a2] + i2 * spacing;

                    // Apply shape's local transform
                    Vec3f body_pos = shape.local_transform.transform_point(local_pos);

                    BoundaryParticle bp;
                    bp.local_position = body_pos;
                    bp.body_index = body_idx;
                    particles.push_back(bp);
                }
            }
        }
    }

    return particles;
}

std::vector<BoundaryParticle> sample_sphere_boundary(const CollisionShape& shape,
                                                      float spacing) {
    std::vector<BoundaryParticle> particles;
    if (shape.type != ShapeType::Sphere) return particles;

    float r = shape.sphere.radius;
    int body_idx = shape.body_index;

    // Use Fibonacci sphere sampling for uniform distribution
    // Number of points based on surface area / spacing^2
    float area = 4.0f * PI * r * r;
    int n = std::max(12, static_cast<int>(area / (spacing * spacing)));

    float golden_ratio = (1.0f + std::sqrt(5.0f)) / 2.0f;
    float golden_angle = 2.0f * PI / (golden_ratio * golden_ratio);

    for (int i = 0; i < n; ++i) {
        float theta = std::acos(1.0f - 2.0f * (i + 0.5f) / n);
        float phi = golden_angle * i;

        Vec3f local_pos(
            r * std::sin(theta) * std::cos(phi),
            r * std::sin(theta) * std::sin(phi),
            r * std::cos(theta)
        );

        Vec3f body_pos = shape.local_transform.transform_point(local_pos);

        BoundaryParticle bp;
        bp.local_position = body_pos;
        bp.body_index = body_idx;
        particles.push_back(bp);
    }

    return particles;
}

std::vector<BoundaryParticle> sample_plane_boundary(const CollisionShape& shape,
                                                     float spacing, float extent) {
    std::vector<BoundaryParticle> particles;
    if (shape.type != ShapeType::Plane) return particles;

    const Vec3f& normal = shape.plane.normal;
    float offset = shape.plane.offset;

    // Build tangent basis
    Vec3f t1, t2;
    if (std::abs(normal.y()) < 0.9f) {
        t1 = Vec3f(0, 1, 0).cross(normal).normalized();
    } else {
        t1 = Vec3f(1, 0, 0).cross(normal).normalized();
    }
    t2 = normal.cross(t1).normalized();

    int n = static_cast<int>(std::floor(2.0f * extent / spacing)) + 1;

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            float s = -extent + i * spacing;
            float t = -extent + j * spacing;
            Vec3f pos = normal * offset + t1 * s + t2 * t;

            BoundaryParticle bp;
            bp.local_position = pos;  // planes are world-owned, so local = world
            bp.body_index = shape.body_index;
            particles.push_back(bp);
        }
    }

    return particles;
}

std::vector<BoundaryParticle> sample_model_boundaries(const Model& model,
                                                       float spacing,
                                                       float plane_extent) {
    std::vector<BoundaryParticle> all;

    for (const auto& shape : model.shapes) {
        std::vector<BoundaryParticle> shape_particles;
        switch (shape.type) {
            case ShapeType::Box:
                shape_particles = sample_box_boundary(shape, spacing);
                break;
            case ShapeType::Sphere:
                shape_particles = sample_sphere_boundary(shape, spacing);
                break;
            case ShapeType::Plane:
                shape_particles = sample_plane_boundary(shape, spacing, plane_extent);
                break;
        }
        all.insert(all.end(), shape_particles.begin(), shape_particles.end());
    }

    return all;
}

void compute_boundary_volumes(std::vector<BoundaryParticle>& particles,
                              const std::vector<Vec3f>& world_positions,
                              float kernel_radius) {
    int n = static_cast<int>(particles.size());
    if (n == 0) return;

    SpatialHashGrid grid(kernel_radius);
    grid.build(world_positions);

    std::vector<int> neighbors;
    for (int i = 0; i < n; ++i) {
        float sum_w = 0.0f;
        grid.query_neighbors(world_positions[i], kernel_radius, neighbors);
        for (int j : neighbors) {
            Vec3f r = world_positions[i] - world_positions[j];
            float r_sq = r.squaredNorm();
            sum_w += SPHKernels::poly6(r_sq, kernel_radius);
        }
        particles[i].volume = (sum_w > 1e-10f) ? 1.0f / sum_w : 0.0f;
    }
}

std::vector<Vec3f> boundary_world_positions(
    const std::vector<BoundaryParticle>& particles,
    const std::vector<Transform>& transforms) {
    std::vector<Vec3f> positions(particles.size());
    for (int i = 0; i < static_cast<int>(particles.size()); ++i) {
        int bi = particles[i].body_index;
        if (bi >= 0 && bi < static_cast<int>(transforms.size())) {
            positions[i] = transforms[bi].transform_point(particles[i].local_position);
        } else {
            positions[i] = particles[i].local_position;  // world-owned
        }
    }
    return positions;
}

}  // namespace novaphy
