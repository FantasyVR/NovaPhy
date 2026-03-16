#pragma once

#include "novaphy/math/math_types.h"
#include <vector>

namespace novaphy {

/**
 * Single contact from demo3d-style box-box collision (rA/rB in body-local, feature_key for persistence).
 */
struct VbdContactPoint {
    Vec3f rA;
    Vec3f rB;
    int feature_key = 0;
};

/**
 * Box-box collision ported from avbd-demo3d (SAT, face/edge manifold, FeaturePair key).
 * Uses step-start positions and rotations only; no dependency on narrowphase.
 * @return Number of contacts (0 if separated). basis_out = orthonormal(-normalAB), row0 = B→A.
 */
int vbd_collide_box_box(const Vec3f& pa, const Quatf& qa, const Vec3f& half_a,
                        const Vec3f& pb, const Quatf& qb, const Vec3f& half_b,
                        std::vector<VbdContactPoint>* out, Mat3f* basis_out);

/**
 * Box vs plane (ground): one contact per penetrating box corner, demo3d-style feature key.
 * Plane: n·x = d (n unit, d offset). Box in world at (pb, qb) with half_b.
 * basis_out row0 = plane normal (from plane toward box).
 */
int vbd_collide_box_plane(const Vec3f& n, float d,
                          const Vec3f& pb, const Quatf& qb, const Vec3f& half_b,
                          std::vector<VbdContactPoint>* out, Mat3f* basis_out);

}  // namespace novaphy
