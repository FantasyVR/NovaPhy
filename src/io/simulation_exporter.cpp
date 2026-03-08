#include "novaphy/io/simulation_exporter.h"

#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>

namespace novaphy {
namespace {

void ensure_parent_directory(const std::filesystem::path& output_path) {
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
}

}  // namespace

void SimulationExporter::capture_frame(const World& world, float time_seconds) {
    const SimState& state = world.state();
    for (int i = 0; i < static_cast<int>(state.transforms.size()); ++i) {
        const Transform& t = state.transforms[i];
        RecordedKeyframe frame;
        frame.time = time_seconds;
        frame.body_index = i;
        frame.position = t.position;
        frame.rotation = Vec4f(t.rotation.x(), t.rotation.y(), t.rotation.z(), t.rotation.w());
        frame.linear_velocity = state.linear_velocities[i];
        frame.angular_velocity = state.angular_velocities[i];
        keyframes_.push_back(frame);
    }
    for (const ContactPoint& c : world.contacts()) {
        RecordedCollisionEvent e;
        e.time = time_seconds;
        e.body_a = c.body_a;
        e.body_b = c.body_b;
        e.position = c.position;
        e.normal = c.normal;
        e.penetration = c.penetration;
        collision_events_.push_back(e);
    }
}

void SimulationExporter::add_constraint_reaction(const RecordedConstraintReaction& reaction) {
    constraint_reactions_.push_back(reaction);
}

const std::vector<RecordedKeyframe>& SimulationExporter::keyframes() const {
    return keyframes_;
}

const std::vector<RecordedCollisionEvent>& SimulationExporter::collision_events() const {
    return collision_events_;
}

const std::vector<RecordedConstraintReaction>& SimulationExporter::constraint_reactions() const {
    return constraint_reactions_;
}

void SimulationExporter::write_keyframes_csv(const std::filesystem::path& output_path) const {
    ensure_parent_directory(output_path);
    std::ofstream out(output_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) throw std::runtime_error("Failed to open output file: " + output_path.string());
    out << "time,body_index,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n";
    for (const RecordedKeyframe& f : keyframes_) {
        out << f.time << "," << f.body_index << ","
            << f.position.x() << "," << f.position.y() << "," << f.position.z() << ","
            << f.rotation.x() << "," << f.rotation.y() << "," << f.rotation.z() << "," << f.rotation.w() << ","
            << f.linear_velocity.x() << "," << f.linear_velocity.y() << "," << f.linear_velocity.z() << ","
            << f.angular_velocity.x() << "," << f.angular_velocity.y() << "," << f.angular_velocity.z() << "\n";
    }
}

void SimulationExporter::write_collision_log_csv(const std::filesystem::path& output_path) const {
    ensure_parent_directory(output_path);
    std::ofstream out(output_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) throw std::runtime_error("Failed to open output file: " + output_path.string());
    out << "time,body_a,body_b,px,py,pz,nx,ny,nz,penetration\n";
    for (const RecordedCollisionEvent& e : collision_events_) {
        out << e.time << "," << e.body_a << "," << e.body_b << ","
            << e.position.x() << "," << e.position.y() << "," << e.position.z() << ","
            << e.normal.x() << "," << e.normal.y() << "," << e.normal.z() << ","
            << e.penetration << "\n";
    }
}

void SimulationExporter::write_constraint_reactions_csv(const std::filesystem::path& output_path) const {
    ensure_parent_directory(output_path);
    std::ofstream out(output_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) throw std::runtime_error("Failed to open output file: " + output_path.string());
    out << "time,joint_name,mx,my,mz,fx,fy,fz\n";
    for (const RecordedConstraintReaction& r : constraint_reactions_) {
        VecXf w = r.wrench;
        if (w.size() < 6) {
            VecXf padded = VecXf::Zero(6);
            padded.head(w.size()) = w;
            w = padded;
        }
        out << r.time << "," << r.joint_name << ","
            << w(0) << "," << w(1) << "," << w(2) << ","
            << w(3) << "," << w(4) << "," << w(5) << "\n";
    }
}

void SimulationExporter::write_urdf(const UrdfModelData& model,
                                    const std::filesystem::path& output_path) const {
    UrdfParser parser;
    parser.write_file(model, output_path);
}

void SimulationExporter::write_openusd_animation_layer(const std::filesystem::path& output_path) const {
    ensure_parent_directory(output_path);
    std::ofstream out(output_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) throw std::runtime_error("Failed to open output file: " + output_path.string());
    out << "#usda 1.0\n(\n    upAxis = \"Y\"\n)\n\n";
    std::map<int, std::vector<RecordedKeyframe>> grouped;
    for (const RecordedKeyframe& frame : keyframes_) {
        grouped[frame.body_index].push_back(frame);
    }
    for (const auto& kv : grouped) {
        const int body_idx = kv.first;
        const auto& frames = kv.second;
        out << "def Xform \"body_" << body_idx << "\"\n{\n";
        out << "    double3 xformOp:translate.timeSamples = {\n";
        for (size_t i = 0; i < frames.size(); ++i) {
            const RecordedKeyframe& f = frames[i];
            out << "        " << f.time << ": (" << f.position.x() << ", " << f.position.y() << ", " << f.position.z() << ")";
            out << (i + 1 == frames.size() ? "\n" : ",\n");
        }
        out << "    }\n";
        out << "    quatf xformOp:orient.timeSamples = {\n";
        for (size_t i = 0; i < frames.size(); ++i) {
            const RecordedKeyframe& f = frames[i];
            out << "        " << f.time << ": (" << f.rotation.w() << ", " << f.rotation.x() << ", " << f.rotation.y() << ", " << f.rotation.z() << ")";
            out << (i + 1 == frames.size() ? "\n" : ",\n");
        }
        out << "    }\n";
        out << "    uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]\n";
        out << "}\n\n";
    }
}

}  // namespace novaphy
