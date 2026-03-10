/**
 * @file narrowphase.cpp
 * @brief Narrowphase contact generation between primitive collision shapes.
 */
#include "novaphy/collision/narrowphase.h"

#include <algorithm>
#include <cmath>

#include "novaphy/math/math_utils.h"

namespace novaphy {
namespace narrowphase {

// ---- Sphere vs Sphere ----
/**
 * @brief Computes contact(s) between two spheres.
 * @param[in] a Sphere shape A.
 * @param[in] ta World transform of A's parent body.
 * @param[in] b Sphere shape B.
 * @param[in] tb World transform of B's parent body.
 * @param[out] contacts Appended world-space contact manifold entries.
 * @return `true` if the spheres overlap; otherwise `false`.
 */
bool collide_sphere_sphere(const CollisionShape& a, const Transform& ta,
                           const CollisionShape& b, const Transform& tb,
                           std::vector<ContactPoint>& contacts) {
    Vec3f ca = ta.transform_point(a.local_transform.position);
    Vec3f cb = tb.transform_point(b.local_transform.position);
    float ra = a.sphere.radius;
    float rb = b.sphere.radius;

    Vec3f diff = cb - ca;
    float dist_sq = diff.squaredNorm();
    float radius_sum = ra + rb;

    if (dist_sq > radius_sum * radius_sum) return false;

    float dist = std::sqrt(dist_sq);
    Vec3f normal;
    if (dist < 1e-6f) {
        normal = Vec3f(0, 1, 0);  // degenerate: pick arbitrary normal
    } else {
        normal = diff / dist;
    }

    ContactPoint cp;
    cp.normal = normal;
    cp.penetration = radius_sum - dist;
    cp.position = ca + normal * (ra - cp.penetration * 0.5f);
    cp.body_a = a.body_index;
    cp.body_b = b.body_index;
    cp.friction = combine_friction(a.friction, b.friction);
    cp.restitution = combine_restitution(a.restitution, b.restitution);
    contacts.push_back(cp);
    return true;
}

// ---- Sphere vs Plane ----
/**
 * @brief Computes contact(s) between a sphere and an infinite plane.
 * @param[in] sphere_shape Sphere collision shape.
 * @param[in] ts World transform of the sphere body.
 * @param[in] plane_shape Plane collision shape.
 * @param[out] contacts Appended world-space contact manifold entries.
 * @return `true` if the sphere intersects or penetrates the plane.
 */
bool collide_sphere_plane(const CollisionShape& sphere_shape, const Transform& ts,
                          const CollisionShape& plane_shape,
                          std::vector<ContactPoint>& contacts) {
    Vec3f center = ts.transform_point(sphere_shape.local_transform.position);
    float radius = sphere_shape.sphere.radius;

    Vec3f n = plane_shape.plane.normal;
    float d = plane_shape.plane.offset;

    float distance = n.dot(center) - d;

    if (distance > radius) return false;

    ContactPoint cp;
    cp.normal = n;  // points from plane toward sphere (away from plane surface)
    cp.penetration = radius - distance;
    cp.position = center - n * distance;
    cp.body_a = plane_shape.body_index;   // plane is body_a
    cp.body_b = sphere_shape.body_index;  // sphere is body_b
    cp.friction = combine_friction(sphere_shape.friction, plane_shape.friction);
    cp.restitution = combine_restitution(sphere_shape.restitution, plane_shape.restitution);
    contacts.push_back(cp);
    return true;
}

// ---- Box vs Sphere ----
/**
 * @brief Computes contact(s) between an oriented box and a sphere.
 * @param[in] box_shape Box collision shape.
 * @param[in] tb World transform of the box body.
 * @param[in] sphere_shape Sphere collision shape.
 * @param[in] ts World transform of the sphere body.
 * @param[out] contacts Appended world-space contact manifold entries.
 * @return `true` if the shapes overlap; otherwise `false`.
 */
bool collide_box_sphere(const CollisionShape& box_shape, const Transform& tb,
                        const CollisionShape& sphere_shape, const Transform& ts,
                        std::vector<ContactPoint>& contacts) {
    // Transform sphere center into box's local space
    Transform box_world = tb * box_shape.local_transform;
    Transform box_inv = box_world.inverse();
    Vec3f sphere_center = ts.transform_point(sphere_shape.local_transform.position);
    Vec3f local_center = box_inv.transform_point(sphere_center);

    Vec3f half = box_shape.box.half_extents;
    float radius = sphere_shape.sphere.radius;

    // Clamp to box bounds to find closest point on box
    Vec3f closest;
    closest.x() = clampf(local_center.x(), -half.x(), half.x());
    closest.y() = clampf(local_center.y(), -half.y(), half.y());
    closest.z() = clampf(local_center.z(), -half.z(), half.z());

    Vec3f diff = local_center - closest;
    float dist_sq = diff.squaredNorm();

    if (dist_sq > radius * radius) return false;

    // Transform back to world space
    Vec3f world_closest = box_world.transform_point(closest);
    Vec3f world_diff = sphere_center - world_closest;
    float dist = std::sqrt(dist_sq);

    Vec3f normal;
    if (dist < 1e-6f) {
        // Sphere center is inside box - find nearest face
        Vec3f face_dist(half.x() - std::abs(local_center.x()),
                        half.y() - std::abs(local_center.y()),
                        half.z() - std::abs(local_center.z()));
        int min_axis = 0;
        if (face_dist.y() < face_dist(min_axis)) min_axis = 1;
        if (face_dist.z() < face_dist(min_axis)) min_axis = 2;
        Vec3f local_normal = Vec3f::Zero();
        local_normal(min_axis) = local_center(min_axis) > 0 ? 1.0f : -1.0f;
        normal = box_world.transform_vector(local_normal);
    } else {
        normal = world_diff / dist;
    }

    ContactPoint cp;
    cp.normal = normal;
    cp.penetration = radius - dist;
    cp.position = world_closest;
    cp.body_a = box_shape.body_index;
    cp.body_b = sphere_shape.body_index;
    cp.friction = combine_friction(box_shape.friction, sphere_shape.friction);
    cp.restitution = combine_restitution(box_shape.restitution, sphere_shape.restitution);
    contacts.push_back(cp);
    return true;
}

// ---- Box vs Plane ----
/**
 * @brief Computes contact(s) between an oriented box and an infinite plane.
 * @param[in] box_shape Box collision shape.
 * @param[in] tb World transform of the box body.
 * @param[in] plane_shape Plane collision shape.
 * @param[out] contacts Appended world-space contact manifold entries.
 * @return `true` when at least one box corner lies behind the plane.
 */
bool collide_box_plane(const CollisionShape& box_shape, const Transform& tb,
                       const CollisionShape& plane_shape,
                       std::vector<ContactPoint>& contacts) {
    Transform box_world = tb * box_shape.local_transform;
    Vec3f half = box_shape.box.half_extents;
    Vec3f n = plane_shape.plane.normal;
    float d = plane_shape.plane.offset;

    bool has_contact = false;

    // Test all 8 corners of the box against the plane
    for (int i = 0; i < 8; ++i) {
        Vec3f corner(
            (i & 1) ? half.x() : -half.x(),
            (i & 2) ? half.y() : -half.y(),
            (i & 4) ? half.z() : -half.z());

        Vec3f world_corner = box_world.transform_point(corner);
        float distance = n.dot(world_corner) - d;

        if (distance < 0.0f) {
            ContactPoint cp;
            cp.normal = n;  // points from plane toward box (away from plane surface)
            cp.penetration = -distance;
            cp.position = world_corner - n * distance;
            cp.body_a = plane_shape.body_index;   // plane is body_a
            cp.body_b = box_shape.body_index;     // box is body_b
            cp.friction = combine_friction(box_shape.friction, plane_shape.friction);
            cp.restitution = combine_restitution(box_shape.restitution, plane_shape.restitution);
            contacts.push_back(cp);
            has_contact = true;
        }
    }

    return has_contact;
}

// ---- Box vs Box (SAT) ----
// Separating Axis Theorem with contact point generation
/**
 * @brief Computes contact(s) between two oriented boxes using SAT.
 * @details Tests 15 separating axes (face normals and edge cross products),
 *          then generates a compact face or edge contact set.
 * @param[in] a Box shape A.
 * @param[in] ta World transform of body A.
 * @param[in] b Box shape B.
 * @param[in] tb World transform of body B.
 * @param[out] contacts Appended world-space contact manifold entries.
 * @return `true` if overlap is detected; otherwise `false`.
 */
bool collide_box_box(const CollisionShape& a, const Transform& ta,
                     const CollisionShape& b, const Transform& tb,
                     std::vector<ContactPoint>& contacts) {
    Transform wa = ta * a.local_transform;
    Transform wb = tb * b.local_transform;

    Vec3f ha = a.box.half_extents;
    Vec3f hb = b.box.half_extents;

    Mat3f Ra = wa.rotation_matrix();
    Mat3f Rb = wb.rotation_matrix();
    Vec3f pa = wa.position;
    Vec3f pb = wb.position;

    Vec3f d = pb - pa;
    Vec3f axes_a[3] = {Ra.col(0), Ra.col(1), Ra.col(2)};
    Vec3f axes_b[3] = {Rb.col(0), Rb.col(1), Rb.col(2)};

    // R: rotation from B to A's frame
    Mat3f R = Ra.transpose() * Rb;
    Mat3f abs_R = R.cwiseAbs().array() + 1e-6f;  // add epsilon for parallel edges

    Vec3f t = Ra.transpose() * d;  // translation in A's frame

    float min_pen = std::numeric_limits<float>::max();
    Vec3f best_axis;
    int best_axis_idx = -1;

    // Test 15 separating axes
    // 3 face axes of A
    for (int i = 0; i < 3; ++i) {
        float ra_proj = ha(i);
        float rb_proj = hb.x() * abs_R(i, 0) + hb.y() * abs_R(i, 1) + hb.z() * abs_R(i, 2);
        float pen = (ra_proj + rb_proj) - std::abs(t(i));
        if (pen < 0) return false;
        if (pen < min_pen) {
            min_pen = pen;
            best_axis = axes_a[i];
            if (t(i) < 0) best_axis = -best_axis;
            best_axis_idx = i;
        }
    }

    // 3 face axes of B
    for (int i = 0; i < 3; ++i) {
        float ra_proj = ha.x() * abs_R(0, i) + ha.y() * abs_R(1, i) + ha.z() * abs_R(2, i);
        float rb_proj = hb(i);
        float s = R.col(i).dot(t);
        float pen = (ra_proj + rb_proj) - std::abs(s);
        if (pen < 0) return false;
        if (pen < min_pen) {
            min_pen = pen;
            best_axis = axes_b[i];
            if (s < 0) best_axis = -best_axis;
            best_axis_idx = 3 + i;
        }
    }

    // 9 edge-edge axes (cross products)
    // Prefer face axes when penetration is effectively tied, since stacked
    // boxes are more stable with a face manifold than an edge-edge fallback.
    constexpr float edge_axis_margin = 1e-4f;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3f axis = axes_a[i].cross(axes_b[j]);
            float len = axis.norm();
            if (len < 1e-6f) continue;  // parallel axes
            axis /= len;

            float ra_proj = 0, rb_proj = 0;
            for (int k = 0; k < 3; ++k) {
                ra_proj += ha(k) * std::abs(axis.dot(axes_a[k]));
                rb_proj += hb(k) * std::abs(axis.dot(axes_b[k]));
            }
            float s = axis.dot(d);
            float pen = (ra_proj + rb_proj) - std::abs(s);
            if (pen < 0) return false;
            if (pen < min_pen - edge_axis_margin) {
                min_pen = pen;
                best_axis = axis;
                if (s < 0) best_axis = -best_axis;
                best_axis_idx = 6 + i * 3 + j;
            }
        }
    }

    // Generate contact points
    // Use a simplified reference/incident face approach.
    // For face contacts we keep incident corners inside the reference box bounds,
    // which yields up to 4 stable contacts for stacked boxes.
    if (best_axis_idx < 6) {
        // Face-something contact
        bool a_is_reference = (best_axis_idx < 3);

        const Vec3f& ref_half = a_is_reference ? ha : hb;
        const Vec3f& inc_half = a_is_reference ? hb : ha;
        const Transform& ref_xform = a_is_reference ? wa : wb;
        const Transform& inc_xform = a_is_reference ? wb : wa;
        constexpr float inside_epsilon = 0.01f;
        constexpr int max_face_contacts = 8;  // 与 demo3d MAX_CONTACTS=8 一致，支撑更对称
        float contact_pen = std::max(0.0f, min_pen);
        int num_face_contacts = 0;

        // Keep incident corners that project inside the reference box (up to 8).
        for (int i = 0; i < 8; ++i) {
            Vec3f corner(
                (i & 1) ? inc_half.x() : -inc_half.x(),
                (i & 2) ? inc_half.y() : -inc_half.y(),
                (i & 4) ? inc_half.z() : -inc_half.z());

            Vec3f world_corner = inc_xform.transform_point(corner);
            Vec3f local_corner = ref_xform.inverse().transform_point(world_corner);

            if (std::abs(local_corner.x()) <= ref_half.x() + inside_epsilon &&
                std::abs(local_corner.y()) <= ref_half.y() + inside_epsilon &&
                std::abs(local_corner.z()) <= ref_half.z() + inside_epsilon) {
                ContactPoint cp;
                cp.normal = best_axis;
                cp.penetration = contact_pen;
                float offset = 0.5f * contact_pen;
                if (a_is_reference) {
                    cp.position = world_corner + best_axis * offset;
                } else {
                    cp.position = world_corner - best_axis * offset;
                }
                cp.body_a = a.body_index;
                cp.body_b = b.body_index;
                cp.friction = combine_friction(a.friction, b.friction);
                cp.restitution = combine_restitution(a.restitution, b.restitution);
                // 对齐 demo3d 的“特征稳定性”诉求：此处用 -1 触发上层 contact_key 的锚点 UV 量化路径，
                // 避免 corner/axis 抖动导致 warmstart 断裂与长期偏置。
                cp.feature_id = -1;
                contacts.push_back(cp);
                ++num_face_contacts;
                if (num_face_contacts >= max_face_contacts) break;
            }
        }

        // If no corner contacts found, use the midpoint
        if (contacts.empty()) {
            ContactPoint cp;
            cp.normal = best_axis;
            cp.penetration = min_pen;
            cp.position = pa + d * 0.5f;
            cp.body_a = a.body_index;
            cp.body_b = b.body_index;
            cp.friction = combine_friction(a.friction, b.friction);
            cp.restitution = combine_restitution(a.restitution, b.restitution);
            cp.feature_id = -1;
            contacts.push_back(cp);
        }
    } else {
        // Edge-edge contact: single contact point
        ContactPoint cp;
        cp.normal = best_axis;
        cp.penetration = min_pen;
        cp.position = pa + d * 0.5f;  // approximate: midpoint
        cp.body_a = a.body_index;
        cp.body_b = b.body_index;
        cp.friction = combine_friction(a.friction, b.friction);
        cp.restitution = combine_restitution(a.restitution, b.restitution);
        cp.feature_id = -1;
        contacts.push_back(cp);
    }

    return !contacts.empty();
}

}  // namespace narrowphase

// ---- Collision Dispatcher ----
/**
 * @brief Dispatches narrowphase collision routines by shape-type pair.
 * @param[in] a Shape A.
 * @param[in] ta World transform of A's parent body.
 * @param[in] b Shape B.
 * @param[in] tb World transform of B's parent body.
 * @param[out] contacts Appended world-space contact manifold entries.
 * @return `true` if at least one contact is generated.
 */
bool collide_shapes(const CollisionShape& a, const Transform& ta,
                    const CollisionShape& b, const Transform& tb,
                    std::vector<ContactPoint>& contacts) {
    ShapeType typeA = a.type;
    ShapeType typeB = b.type;

    // Sphere-Sphere
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Sphere) {
        return narrowphase::collide_sphere_sphere(a, ta, b, tb, contacts);
    }
    // Sphere-Plane
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Plane) {
        return narrowphase::collide_sphere_plane(a, ta, b, contacts);
    }
    if (typeA == ShapeType::Plane && typeB == ShapeType::Sphere) {
        return narrowphase::collide_sphere_plane(b, tb, a, contacts);
    }
    // Box-Sphere
    if (typeA == ShapeType::Box && typeB == ShapeType::Sphere) {
        return narrowphase::collide_box_sphere(a, ta, b, tb, contacts);
    }
    if (typeA == ShapeType::Sphere && typeB == ShapeType::Box) {
        bool r = narrowphase::collide_box_sphere(b, tb, a, ta, contacts);
        for (auto& cp : contacts) {
            cp.normal = -cp.normal;
            std::swap(cp.body_a, cp.body_b);
        }
        return r;
    }
    // Box-Plane
    if (typeA == ShapeType::Box && typeB == ShapeType::Plane) {
        return narrowphase::collide_box_plane(a, ta, b, contacts);
    }
    if (typeA == ShapeType::Plane && typeB == ShapeType::Box) {
        return narrowphase::collide_box_plane(b, tb, a, contacts);
    }
    // Box-Box
    if (typeA == ShapeType::Box && typeB == ShapeType::Box) {
        return narrowphase::collide_box_box(a, ta, b, tb, contacts);
    }
    // Plane-Plane: no collision
    return false;
}

}  // namespace novaphy
