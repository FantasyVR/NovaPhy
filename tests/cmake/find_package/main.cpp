#include <cmath>
#include <iomanip>
#include <iostream>

#include "novaphy/core/body.h"
#include "novaphy/core/model_builder.h"
#include "novaphy/core/shape.h"
#include "novaphy/novaphy.h"
#include "novaphy/sim/world.h"

int main() {
    using namespace novaphy;

    ModelBuilder builder;

    const int sphere_body = builder.add_body(RigidBody::from_sphere(1.0f, 0.25f),
                                             Transform::from_translation(Vec3f(0.0f, 1.0f, 0.0f)));

    builder.add_shape(CollisionShape::make_sphere(0.25f, sphere_body));
    builder.add_ground_plane(0.0f, 0.6f, 0.1f);

    Model model = builder.build();
    World world(model);

    const float y0 = world.state().transforms[sphere_body].position.y();

    constexpr float dt = 1.0f / 240.0f;
    constexpr int steps = 120;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "step,time,y,vy,contacts" << '\n';
    std::cout << 0 << "," << 0.0f << "," << world.state().transforms[sphere_body].position.y()
              << "," << world.state().linear_velocities[sphere_body].y() << ","
              << world.contacts().size() << '\n';

    for (int i = 0; i < steps; ++i) {
        world.step(dt);
        const int step_id = i + 1;
        if (step_id <= 10 || step_id % 10 == 0) {
            const float time = dt * static_cast<float>(step_id);
            const float y = world.state().transforms[sphere_body].position.y();
            const float vy = world.state().linear_velocities[sphere_body].y();
            const std::size_t n_contacts = world.contacts().size();
            std::cout << step_id << "," << time << "," << y << "," << vy << "," << n_contacts
                      << '\n';
        }
    }

    const float y1 = world.state().transforms[sphere_body].position.y();
    const bool moved_down = (y1 < y0 - 1e-4f);

    std::cout << "NovaPhy version: " << novaphy::version() << '\n';
    std::cout << "libuipc bind type: " << novaphy::libuipc_bind_type() << '\n';
    std::cout << "sphere y before: " << y0 << " after: " << y1 << '\n';

    if (!moved_down) {
        std::cerr << "Simulation sanity check failed: sphere did not move down." << '\n';
        return 1;
    }

    std::cout << "Simulation sanity check passed." << '\n';
    return 0;
}
