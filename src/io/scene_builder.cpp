#include "novaphy/io/scene_builder.h"

#include <algorithm>
#include <queue>
#include <unordered_map>

#include "novaphy/core/model_builder.h"

namespace novaphy {
namespace {

JointType urdf_joint_type(const std::string& t) {
    if (t == "revolute" || t == "continuous") return JointType::Revolute;
    if (t == "prismatic") return JointType::Slide;
    if (t == "floating") return JointType::Free;
    if (t == "spherical") return JointType::Ball;
    return JointType::Fixed;
}

RigidBody urdf_link_to_body(const UrdfLink& link) {
    RigidBody body;
    body.mass = link.inertial.mass;
    body.inertia = link.inertial.inertia;
    body.com = link.inertial.origin.position;
    if (body.mass <= 0.0f) {
        return RigidBody::make_static();
    }
    return body;
}

void add_collision_shape_from_urdf(
    const UrdfCollision& collision,
    int body_idx,
    ModelBuilder& builder,
    std::vector<std::string>& warnings) {
    if (collision.geometry.type == UrdfGeometryType::Box) {
        const Vec3f half = collision.geometry.size * 0.5f;
        builder.add_shape(CollisionShape::make_box(half, body_idx, collision.origin, collision.friction, collision.restitution));
    } else if (collision.geometry.type == UrdfGeometryType::Sphere) {
        builder.add_shape(CollisionShape::make_sphere(collision.geometry.radius, body_idx, collision.origin, collision.friction, collision.restitution));
    } else if (collision.geometry.type == UrdfGeometryType::Cylinder) {
        const float r = collision.geometry.radius;
        const float half_l = collision.geometry.length * 0.5f;
        builder.add_shape(CollisionShape::make_box(Vec3f(r, half_l, r), body_idx, collision.origin, collision.friction, collision.restitution));
        warnings.push_back("Cylinder collision approximated as box.");
    } else if (collision.geometry.type == UrdfGeometryType::Mesh) {
        const Vec3f extent = collision.geometry.mesh_scale.cwiseAbs() * 0.5f;
        builder.add_shape(CollisionShape::make_box(extent, body_idx, collision.origin, collision.friction, collision.restitution));
        warnings.push_back("Mesh collision approximated as box.");
    }
}

}  // namespace

std::vector<const UrdfLink*> topologically_order_links(
    const UrdfModelData& urdf_model,
    std::vector<std::string>& warnings) {
    std::unordered_map<std::string, const UrdfLink*> link_by_name;
    std::unordered_map<std::string, int> indegree;
    std::unordered_map<std::string, std::vector<std::string>> children;
    std::vector<std::string> declaration_order;
    declaration_order.reserve(urdf_model.links.size());

    for (const UrdfLink& link : urdf_model.links) {
        if (link_by_name.find(link.name) != link_by_name.end()) {
            warnings.push_back("Duplicate URDF link name encountered; using first declaration.");
            continue;
        }
        link_by_name[link.name] = &link;
        indegree[link.name] = 0;
        declaration_order.push_back(link.name);
    }

    for (const UrdfJoint& joint : urdf_model.joints) {
        auto child_it = link_by_name.find(joint.child_link);
        if (child_it == link_by_name.end()) {
            warnings.push_back("URDF joint child link not found in links; joint ignored for ordering.");
            continue;
        }
        if (joint.parent_link.empty()) {
            continue;
        }
        auto parent_it = link_by_name.find(joint.parent_link);
        if (parent_it == link_by_name.end()) {
            warnings.push_back("URDF joint parent link not found in links; joint treated as root attachment.");
            continue;
        }
        children[joint.parent_link].push_back(joint.child_link);
        ++indegree[joint.child_link];
    }

    std::queue<std::string> ready;
    for (const std::string& name : declaration_order) {
        if (indegree[name] == 0) ready.push(name);
    }

    std::vector<const UrdfLink*> ordered_links;
    ordered_links.reserve(declaration_order.size());
    while (!ready.empty()) {
        std::string parent = ready.front();
        ready.pop();
        ordered_links.push_back(link_by_name[parent]);
        auto child_it = children.find(parent);
        if (child_it == children.end()) continue;
        for (const std::string& child : child_it->second) {
            int& degree = indegree[child];
            --degree;
            if (degree == 0) ready.push(child);
        }
    }

    if (ordered_links.size() != declaration_order.size()) {
        warnings.push_back("URDF link graph is cyclic or disconnected; appending unresolved links in declaration order.");
        std::unordered_map<std::string, bool> emitted;
        emitted.reserve(ordered_links.size());
        for (const UrdfLink* link : ordered_links) emitted[link->name] = true;
        for (const std::string& name : declaration_order) {
            if (emitted.find(name) == emitted.end()) {
                ordered_links.push_back(link_by_name[name]);
            }
        }
    }

    return ordered_links;
}

SceneBuildResult SceneBuilderEngine::build_from_urdf(const UrdfModelData& urdf_model) const {
    SceneBuildResult result;
    ModelBuilder builder;
    std::unordered_map<std::string, int> link_to_index;
    const std::vector<const UrdfLink*> ordered_links =
        topologically_order_links(urdf_model, result.warnings);

    std::unordered_map<std::string, std::string> child_to_parent;
    std::unordered_map<std::string, Transform> child_joint_transform;
    std::unordered_map<std::string, const UrdfJoint*> child_joint;
    for (const UrdfJoint& j : urdf_model.joints) {
        child_to_parent[j.child_link] = j.parent_link;
        child_joint_transform[j.child_link] = j.origin;
        child_joint[j.child_link] = &j;
    }

    auto world_transform_of = [&](const std::string& link_name) {
        Transform tf = Transform::identity();
        std::string cursor = link_name;
        std::unordered_map<std::string, bool> visited;
        while (child_to_parent.find(cursor) != child_to_parent.end()) {
            if (visited[cursor]) break;
            visited[cursor] = true;
            tf = child_joint_transform[cursor] * tf;
            cursor = child_to_parent[cursor];
        }
        return tf;
    };

    for (const UrdfLink* link : ordered_links) {
        Transform initial = world_transform_of(link->name);
        const int body_idx = builder.add_body(urdf_link_to_body(*link), initial);
        link_to_index[link->name] = body_idx;
    }

    for (const UrdfLink* link : ordered_links) {
        const int body_idx = link_to_index[link->name];
        for (const UrdfCollision& collision : link->collisions) {
            add_collision_shape_from_urdf(collision, body_idx, builder, result.warnings);
        }
        if (link->collisions.empty() && !link->visuals.empty()) {
            const UrdfVisual& visual = link->visuals.front();
            UrdfCollision proxy;
            proxy.origin = visual.origin;
            proxy.geometry = visual.geometry;
            add_collision_shape_from_urdf(proxy, body_idx, builder, result.warnings);
            result.warnings.push_back("Visual geometry used as fallback collision.");
        }
    }

    result.model = builder.build();

    result.articulation.joints.reserve(ordered_links.size());
    result.articulation.bodies.reserve(ordered_links.size());
    for (const UrdfLink* link : ordered_links) {
        Joint j;
        auto it = child_joint.find(link->name);
        if (it != child_joint.end()) {
            const UrdfJoint& urdf_joint = *it->second;
            j.type = urdf_joint_type(urdf_joint.type);
            j.axis = urdf_joint.axis.normalized();
            j.parent_to_joint = urdf_joint.origin;
            if (!urdf_joint.parent_link.empty() &&
                link_to_index.find(urdf_joint.parent_link) != link_to_index.end()) {
                j.parent = link_to_index[urdf_joint.parent_link];
            }
        } else {
            j.type = JointType::Free;
            j.parent = -1;
        }
        result.articulation.joints.push_back(j);
        result.articulation.bodies.push_back(urdf_link_to_body(*link));
    }
    result.articulation.build_spatial_inertias();

    return result;
}

SceneBuildResult SceneBuilderEngine::build_from_openusd(const UsdStageData& stage) const {
    SceneBuildResult result;
    ModelBuilder builder;
    std::unordered_map<std::string, int> prim_to_body;

    for (const UsdPrim& prim : stage.prims) {
        if (prim.mass <= 0.0f && prim.box_half_extents.isZero(0.0f) && prim.sphere_radius <= 0.0f) {
            continue;
        }
        RigidBody body;
        if (prim.mass > 0.0f) {
            body.mass = prim.mass;
            const Vec3f half = prim.box_half_extents.isZero(0.0f) ? Vec3f(0.25f, 0.25f, 0.25f) : prim.box_half_extents;
            body = RigidBody::from_box(prim.mass, half);
        } else {
            body = RigidBody::make_static();
        }
        const int body_idx = builder.add_body(body, prim.local_transform);
        prim_to_body[prim.path] = body_idx;

        if (!prim.box_half_extents.isZero(0.0f)) {
            builder.add_shape(CollisionShape::make_box(prim.box_half_extents, body_idx));
        } else if (prim.sphere_radius > 0.0f) {
            builder.add_shape(CollisionShape::make_sphere(prim.sphere_radius, body_idx));
        } else {
            builder.add_shape(CollisionShape::make_box(Vec3f(0.25f, 0.25f, 0.25f), body_idx));
            result.warnings.push_back("USD prim missing collider size, default box inserted.");
        }
    }

    result.model = builder.build();

    for (const UsdPrim& prim : stage.prims) {
        if (prim.type_name.find("Joint") == std::string::npos) continue;
        Joint j;
        if (prim.type_name == "PhysicsRevoluteJoint") j.type = JointType::Revolute;
        else if (prim.type_name == "PhysicsPrismaticJoint") j.type = JointType::Slide;
        else if (prim.type_name == "PhysicsSphericalJoint") j.type = JointType::Ball;
        else j.type = JointType::Fixed;
        j.parent = -1;
        j.parent_to_joint = prim.local_transform;
        result.articulation.joints.push_back(j);
        result.articulation.bodies.push_back(RigidBody::make_static());
    }

    if (result.articulation.joints.empty()) {
        result.articulation.joints.resize(result.model.num_bodies(), Joint{JointType::Free});
        result.articulation.bodies = result.model.bodies;
    }
    if (result.articulation.bodies.size() != result.articulation.joints.size()) {
        result.articulation.bodies.resize(result.articulation.joints.size(), RigidBody::make_static());
    }
    result.articulation.build_spatial_inertias();
    return result;
}

}  // namespace novaphy
