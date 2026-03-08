#include "novaphy/io/urdf_parser.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <tinyxml2.h>

namespace novaphy {
namespace {

Vec3f parse_vec3(const char* text, const Vec3f& fallback) {
    if (text == nullptr) return fallback;
    std::stringstream ss(text);
    float x = fallback.x();
    float y = fallback.y();
    float z = fallback.z();
    ss >> x >> y >> z;
    return Vec3f(x, y, z);
}

Transform parse_origin(const tinyxml2::XMLElement* elem) {
    if (elem == nullptr) return Transform::identity();
    Vec3f xyz = parse_vec3(elem->Attribute("xyz"), Vec3f::Zero());
    Vec3f rpy = parse_vec3(elem->Attribute("rpy"), Vec3f::Zero());
    Quatf q =
        Eigen::AngleAxisf(rpy.z(), Vec3f::UnitZ()) *
        Eigen::AngleAxisf(rpy.y(), Vec3f::UnitY()) *
        Eigen::AngleAxisf(rpy.x(), Vec3f::UnitX());
    return Transform(xyz, q);
}

Mat3f parse_inertia_matrix(const tinyxml2::XMLElement* inertia_elem) {
    Mat3f I = Mat3f::Zero();
    if (inertia_elem == nullptr) return I;
    float ixx = inertia_elem->FloatAttribute("ixx", 0.0f);
    float ixy = inertia_elem->FloatAttribute("ixy", 0.0f);
    float ixz = inertia_elem->FloatAttribute("ixz", 0.0f);
    float iyy = inertia_elem->FloatAttribute("iyy", 0.0f);
    float iyz = inertia_elem->FloatAttribute("iyz", 0.0f);
    float izz = inertia_elem->FloatAttribute("izz", 0.0f);
    I(0, 0) = ixx;
    I(0, 1) = ixy;
    I(0, 2) = ixz;
    I(1, 0) = ixy;
    I(1, 1) = iyy;
    I(1, 2) = iyz;
    I(2, 0) = ixz;
    I(2, 1) = iyz;
    I(2, 2) = izz;
    return I;
}

UrdfGeometry parse_geometry(const tinyxml2::XMLElement* geometry_elem) {
    UrdfGeometry g;
    if (geometry_elem == nullptr) return g;
    if (const tinyxml2::XMLElement* box_elem = geometry_elem->FirstChildElement("box")) {
        g.type = UrdfGeometryType::Box;
        g.size = parse_vec3(box_elem->Attribute("size"), Vec3f::Ones());
        return g;
    }
    if (const tinyxml2::XMLElement* sphere_elem = geometry_elem->FirstChildElement("sphere")) {
        g.type = UrdfGeometryType::Sphere;
        g.radius = sphere_elem->FloatAttribute("radius", 0.0f);
        return g;
    }
    if (const tinyxml2::XMLElement* cyl_elem = geometry_elem->FirstChildElement("cylinder")) {
        g.type = UrdfGeometryType::Cylinder;
        g.radius = cyl_elem->FloatAttribute("radius", 0.0f);
        g.length = cyl_elem->FloatAttribute("length", 0.0f);
        return g;
    }
    if (const tinyxml2::XMLElement* mesh_elem = geometry_elem->FirstChildElement("mesh")) {
        g.type = UrdfGeometryType::Mesh;
        g.mesh_filename = mesh_elem->Attribute("filename") ? mesh_elem->Attribute("filename") : "";
        g.mesh_scale = parse_vec3(mesh_elem->Attribute("scale"), Vec3f::Ones());
        return g;
    }
    return g;
}

std::string to_geometry_xml(const UrdfGeometry& geometry) {
    std::ostringstream out;
    out << "    <geometry>\n";
    if (geometry.type == UrdfGeometryType::Box) {
        out << "      <box size=\"" << geometry.size.x() << " " << geometry.size.y() << " " << geometry.size.z() << "\"/>\n";
    } else if (geometry.type == UrdfGeometryType::Sphere) {
        out << "      <sphere radius=\"" << geometry.radius << "\"/>\n";
    } else if (geometry.type == UrdfGeometryType::Cylinder) {
        out << "      <cylinder radius=\"" << geometry.radius << "\" length=\"" << geometry.length << "\"/>\n";
    } else if (geometry.type == UrdfGeometryType::Mesh) {
        out << "      <mesh filename=\"" << geometry.mesh_filename << "\" scale=\""
            << geometry.mesh_scale.x() << " " << geometry.mesh_scale.y() << " " << geometry.mesh_scale.z()
            << "\"/>\n";
    }
    out << "    </geometry>\n";
    return out.str();
}

std::string transform_to_origin_xml(const Transform& t) {
    Vec3f zyx = t.rotation.toRotationMatrix().eulerAngles(2, 1, 0);
    Vec3f rpy(zyx.z(), zyx.y(), zyx.x());
    std::ostringstream out;
    out << "<origin xyz=\""
        << t.position.x() << " " << t.position.y() << " " << t.position.z()
        << "\" rpy=\""
        << rpy.x() << " " << rpy.y() << " " << rpy.z()
        << "\"/>";
    return out.str();
}

}  // namespace

UrdfModelData UrdfParser::parse_file(const std::filesystem::path& urdf_path) const {
    tinyxml2::XMLDocument doc;
    const std::string urdf_path_string = urdf_path.string();
    if (doc.LoadFile(urdf_path_string.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Failed to load URDF file: " + urdf_path_string);
    }
    const tinyxml2::XMLElement* robot_elem = doc.FirstChildElement("robot");
    if (robot_elem == nullptr) {
        throw std::runtime_error("Invalid URDF, missing <robot>: " + urdf_path_string);
    }

    UrdfModelData model;
    model.name = robot_elem->Attribute("name") ? robot_elem->Attribute("name") : "";

    for (const tinyxml2::XMLElement* link_elem = robot_elem->FirstChildElement("link");
         link_elem != nullptr;
         link_elem = link_elem->NextSiblingElement("link")) {
        UrdfLink link;
        link.name = link_elem->Attribute("name") ? link_elem->Attribute("name") : "";

        if (const tinyxml2::XMLElement* inertial_elem = link_elem->FirstChildElement("inertial")) {
            link.inertial.origin = parse_origin(inertial_elem->FirstChildElement("origin"));
            if (const tinyxml2::XMLElement* mass_elem = inertial_elem->FirstChildElement("mass")) {
                link.inertial.mass = mass_elem->FloatAttribute("value", 0.0f);
            }
            link.inertial.inertia = parse_inertia_matrix(inertial_elem->FirstChildElement("inertia"));
        }

        for (const tinyxml2::XMLElement* visual_elem = link_elem->FirstChildElement("visual");
             visual_elem != nullptr;
             visual_elem = visual_elem->NextSiblingElement("visual")) {
            UrdfVisual visual;
            visual.origin = parse_origin(visual_elem->FirstChildElement("origin"));
            visual.geometry = parse_geometry(visual_elem->FirstChildElement("geometry"));
            if (const tinyxml2::XMLElement* mat_elem = visual_elem->FirstChildElement("material")) {
                visual.material_name = mat_elem->Attribute("name") ? mat_elem->Attribute("name") : "";
            }
            link.visuals.push_back(visual);
        }

        for (const tinyxml2::XMLElement* collision_elem = link_elem->FirstChildElement("collision");
             collision_elem != nullptr;
             collision_elem = collision_elem->NextSiblingElement("collision")) {
            UrdfCollision collision;
            collision.origin = parse_origin(collision_elem->FirstChildElement("origin"));
            collision.geometry = parse_geometry(collision_elem->FirstChildElement("geometry"));
            link.collisions.push_back(collision);
        }

        model.links.push_back(link);
    }

    for (const tinyxml2::XMLElement* joint_elem = robot_elem->FirstChildElement("joint");
         joint_elem != nullptr;
         joint_elem = joint_elem->NextSiblingElement("joint")) {
        UrdfJoint joint;
        joint.name = joint_elem->Attribute("name") ? joint_elem->Attribute("name") : "";
        joint.type = joint_elem->Attribute("type") ? joint_elem->Attribute("type") : "fixed";
        joint.origin = parse_origin(joint_elem->FirstChildElement("origin"));
        if (const tinyxml2::XMLElement* parent_elem = joint_elem->FirstChildElement("parent")) {
            joint.parent_link = parent_elem->Attribute("link") ? parent_elem->Attribute("link") : "";
        }
        if (const tinyxml2::XMLElement* child_elem = joint_elem->FirstChildElement("child")) {
            joint.child_link = child_elem->Attribute("link") ? child_elem->Attribute("link") : "";
        }
        if (const tinyxml2::XMLElement* axis_elem = joint_elem->FirstChildElement("axis")) {
            joint.axis = parse_vec3(axis_elem->Attribute("xyz"), Vec3f(0.0f, 0.0f, 1.0f));
        }
        if (const tinyxml2::XMLElement* limit_elem = joint_elem->FirstChildElement("limit")) {
            joint.lower_limit = limit_elem->FloatAttribute("lower", 0.0f);
            joint.upper_limit = limit_elem->FloatAttribute("upper", 0.0f);
            joint.effort_limit = limit_elem->FloatAttribute("effort", 0.0f);
            joint.velocity_limit = limit_elem->FloatAttribute("velocity", 0.0f);
        }
        model.joints.push_back(joint);
    }

    return model;
}

std::string UrdfParser::write_string(const UrdfModelData& model) const {
    std::ostringstream out;
    out << "<robot name=\"" << model.name << "\">\n";
    for (const UrdfLink& link : model.links) {
        out << "  <link name=\"" << link.name << "\">\n";
        out << "    <inertial>\n";
        out << "      " << transform_to_origin_xml(link.inertial.origin) << "\n";
        out << "      <mass value=\"" << link.inertial.mass << "\"/>\n";
        out << "      <inertia ixx=\"" << link.inertial.inertia(0, 0)
            << "\" ixy=\"" << link.inertial.inertia(0, 1)
            << "\" ixz=\"" << link.inertial.inertia(0, 2)
            << "\" iyy=\"" << link.inertial.inertia(1, 1)
            << "\" iyz=\"" << link.inertial.inertia(1, 2)
            << "\" izz=\"" << link.inertial.inertia(2, 2)
            << "\"/>\n";
        out << "    </inertial>\n";

        for (const UrdfVisual& visual : link.visuals) {
            out << "    <visual>\n";
            out << "      " << transform_to_origin_xml(visual.origin) << "\n";
            out << to_geometry_xml(visual.geometry);
            if (!visual.material_name.empty()) {
                out << "      <material name=\"" << visual.material_name << "\"/>\n";
            }
            out << "    </visual>\n";
        }

        for (const UrdfCollision& collision : link.collisions) {
            out << "    <collision>\n";
            out << "      " << transform_to_origin_xml(collision.origin) << "\n";
            out << to_geometry_xml(collision.geometry);
            out << "    </collision>\n";
        }
        out << "  </link>\n";
    }

    for (const UrdfJoint& joint : model.joints) {
        out << "  <joint name=\"" << joint.name << "\" type=\"" << joint.type << "\">\n";
        out << "    <parent link=\"" << joint.parent_link << "\"/>\n";
        out << "    <child link=\"" << joint.child_link << "\"/>\n";
        out << "    " << transform_to_origin_xml(joint.origin) << "\n";
        out << "    <axis xyz=\"" << joint.axis.x() << " " << joint.axis.y() << " " << joint.axis.z() << "\"/>\n";
        out << "    <limit lower=\"" << joint.lower_limit << "\" upper=\"" << joint.upper_limit
            << "\" effort=\"" << joint.effort_limit << "\" velocity=\"" << joint.velocity_limit << "\"/>\n";
        out << "  </joint>\n";
    }

    out << "</robot>\n";
    return out.str();
}

void UrdfParser::write_file(const UrdfModelData& model, const std::filesystem::path& urdf_path) const {
    if (urdf_path.has_parent_path()) {
        std::filesystem::create_directories(urdf_path.parent_path());
    }
    std::ofstream out(urdf_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open output URDF file: " + urdf_path.string());
    }
    out << write_string(model);
}

}  // namespace novaphy
