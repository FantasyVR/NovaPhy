#include <gtest/gtest.h>

#include <array>

#include "novaphy/core/articulation.h"
#include "novaphy/core/body.h"
#include "novaphy/core/joint.h"
#include "novaphy/dynamics/featherstone.h"
#include "novaphy/fluid/neighbor_search.h"
#include "novaphy/fluid/particle_state.h"
#include "novaphy/sim/state.h"

namespace novaphy {
namespace {

TEST(CoreSpanApisTest, StateAndParticleStateInitializeFromSpanBackedStorage) {
    const std::array<Transform, 2> initial_transforms = {
        Transform::identity(),
        Transform::from_translation(Vec3f(1.0f, 2.0f, 3.0f)),
    };

    SimState state;
    state.init(initial_transforms);
    ASSERT_EQ(state.transforms.size(), 2u);
    EXPECT_EQ(state.linear_velocities.size(), 2u);
    EXPECT_FLOAT_EQ(state.transforms[1].position.x(), 1.0f);
    EXPECT_FLOAT_EQ(state.transforms[1].position.y(), 2.0f);
    EXPECT_FLOAT_EQ(state.transforms[1].position.z(), 3.0f);

    const std::array<Vec3f, 3> initial_positions = {
        Vec3f(0.0f, 0.0f, 0.0f),
        Vec3f(0.1f, 0.0f, 0.0f),
        Vec3f(1.0f, 0.0f, 0.0f),
    };

    ParticleState particles;
    particles.init(initial_positions, Vec3f(0.0f, 1.0f, 0.0f));
    ASSERT_EQ(particles.positions.size(), 3u);
    EXPECT_EQ(particles.velocities.size(), 3u);
    EXPECT_FLOAT_EQ(particles.predicted_positions[1].x(), 0.1f);

    SpatialHashGrid grid(0.25f);
    grid.build(initial_positions);
    const std::vector<int> neighbors = grid.query_neighbors(initial_positions[0], 0.25f);
    EXPECT_FALSE(neighbors.empty());

    const std::vector<std::pair<int, int>> pairs = grid.query_all_pairs(initial_positions, 0.2f);
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs.front().first, 0);
    EXPECT_EQ(pairs.front().second, 1);
}

TEST(CoreSpanApisTest, FeatherstoneReturnsForwardKinematicsResult) {
    Articulation articulation;

    Joint root;
    root.type = JointType::Slide;
    root.axis = Vec3f(1.0f, 0.0f, 0.0f);
    root.parent = -1;
    root.parent_to_joint = Transform::from_translation(Vec3f(0.0f, 1.0f, 0.0f));
    articulation.joints.push_back(root);
    articulation.bodies.push_back(RigidBody::from_box(1.0f, Vec3f(0.1f, 0.1f, 0.1f)));

    Joint child;
    child.type = JointType::Fixed;
    child.parent = 0;
    child.parent_to_joint = Transform::from_translation(Vec3f(0.0f, 0.5f, 0.0f));
    articulation.joints.push_back(child);
    articulation.bodies.push_back(RigidBody::from_box(1.0f, Vec3f(0.1f, 0.1f, 0.1f)));

    articulation.build_spatial_inertias();

    VecXf q = VecXf::Zero(articulation.total_q());
    q(0) = 0.25f;

    const featherstone::ForwardKinematicsResult fk =
        featherstone::forward_kinematics(articulation, q);
    ASSERT_EQ(fk.joint_transforms.size(), 2u);
    ASSERT_EQ(fk.parent_transforms.size(), 2u);
    ASSERT_EQ(fk.world_transforms.size(), 2u);
    EXPECT_FLOAT_EQ(fk.world_transforms[0].position.x(), 0.25f);
    EXPECT_FLOAT_EQ(fk.world_transforms[0].position.y(), 1.0f);
    EXPECT_FLOAT_EQ(fk.world_transforms[1].position.x(), 0.25f);
    EXPECT_FLOAT_EQ(fk.world_transforms[1].position.y(), 1.5f);

    const VecXf qd = VecXf::Zero(articulation.total_qd());
    const std::array<SpatialVector, 2> external_forces = {
        SpatialVector::Zero(),
        SpatialVector::Zero(),
    };
    const VecXf tau = featherstone::inverse_dynamics(
        articulation,
        q,
        qd,
        qd,
        Vec3f(0.0f, -9.81f, 0.0f),
        external_forces);
    EXPECT_EQ(tau.size(), articulation.total_qd());
}

}  // namespace
}  // namespace novaphy
