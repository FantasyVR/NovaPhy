#include <gtest/gtest.h>

#include <filesystem>

#include "novaphy/core/body.h"
#include "novaphy/io/simulation_exporter.h"
#include "novaphy/io/urdf_parser.h"
#include "novaphy/sim/performance_monitor.h"
#include "novaphy/sim/world.h"

namespace novaphy {
namespace {

TEST(IoPathApisTest, UrdfParserRoundTripsFilesystemPaths) {
    const std::filesystem::path output_dir =
        std::filesystem::temp_directory_path() / "novaphy_cpp_tests" / "urdf_roundtrip";
    std::filesystem::remove_all(output_dir);

    UrdfModelData model;
    model.name = "path_robot";
    UrdfLink link;
    link.name = "base";
    model.links.push_back(link);

    const std::filesystem::path urdf_path = output_dir / "robot.urdf";
    UrdfParser parser;
    parser.write_file(model, urdf_path);

    ASSERT_TRUE(std::filesystem::exists(urdf_path));
    const UrdfModelData parsed = parser.parse_file(urdf_path);
    EXPECT_EQ(parsed.name, "path_robot");
    ASSERT_EQ(parsed.links.size(), 1u);
    EXPECT_EQ(parsed.links.front().name, "base");
}

TEST(IoPathApisTest, ExportersAndTraceWriterAcceptFilesystemPaths) {
    const std::filesystem::path output_dir =
        std::filesystem::temp_directory_path() / "novaphy_cpp_tests" / "exporters";
    std::filesystem::remove_all(output_dir);

    Model model;
    model.bodies.push_back(RigidBody::from_box(1.0f, Vec3f(0.5f, 0.5f, 0.5f)));
    model.initial_transforms.push_back(Transform::identity());

    World world(model);
    SimulationExporter exporter;
    exporter.capture_frame(world, 0.0f);

    const std::filesystem::path keyframe_path = output_dir / "nested" / "keyframes.csv";
    exporter.write_keyframes_csv(keyframe_path);
    ASSERT_TRUE(std::filesystem::exists(keyframe_path));

    PerformanceMonitor monitor;
    const std::filesystem::path trace_path = output_dir / "nested" / "trace.json";
    monitor.write_trace_json(trace_path);
    ASSERT_TRUE(std::filesystem::exists(trace_path));
}

}  // namespace
}  // namespace novaphy
