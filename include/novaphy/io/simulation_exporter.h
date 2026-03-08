#pragma once

#include <filesystem>
#include <vector>

#include "novaphy/io/scene_types.h"
#include "novaphy/io/urdf_parser.h"
#include "novaphy/sim/world.h"

namespace novaphy {

class SimulationExporter {
public:
    void capture_frame(const World& world, float time_seconds);
    void add_constraint_reaction(const RecordedConstraintReaction& reaction);

    const std::vector<RecordedKeyframe>& keyframes() const;
    const std::vector<RecordedCollisionEvent>& collision_events() const;
    const std::vector<RecordedConstraintReaction>& constraint_reactions() const;

    void write_keyframes_csv(const std::filesystem::path& output_path) const;
    void write_collision_log_csv(const std::filesystem::path& output_path) const;
    void write_constraint_reactions_csv(const std::filesystem::path& output_path) const;
    void write_urdf(const UrdfModelData& model, const std::filesystem::path& output_path) const;
    void write_openusd_animation_layer(const std::filesystem::path& output_path) const;

private:
    std::vector<RecordedKeyframe> keyframes_;
    std::vector<RecordedCollisionEvent> collision_events_;
    std::vector<RecordedConstraintReaction> constraint_reactions_;
};

}  // namespace novaphy
