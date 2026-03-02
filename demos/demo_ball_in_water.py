"""Ball dropping into water - rigid-fluid coupling demo.

A sphere falls into a pool of fluid particles, demonstrating
Akinci boundary particle coupling between PBF fluid and rigid bodies.
"""

import sys
import numpy as np

try:
    import polyscope as ps
    HAS_POLYSCOPE = True
except ImportError:
    HAS_POLYSCOPE = False

import novaphy


def build_scene():
    """Build a ball-in-water scene."""
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0)

    # Add a sphere above the fluid
    body = novaphy.RigidBody.from_sphere(0.5, 0.15)
    t = novaphy.Transform.from_translation(
        np.array([0.25, 0.8, 0.25], dtype=np.float32))
    idx = builder.add_body(body, t)
    shape = novaphy.CollisionShape.make_sphere(0.15, idx)
    builder.add_shape(shape)

    model = builder.build()

    # Fluid pool
    spacing = 0.025
    block = novaphy.FluidBlockDef()
    block.lower = np.array([0.0, 0.05, 0.0], dtype=np.float32)
    block.upper = np.array([0.5, 0.4, 0.5], dtype=np.float32)
    block.particle_spacing = spacing
    block.rest_density = 1000.0

    # PBF settings
    pbf = novaphy.PBFSettings()
    pbf.rest_density = 1000.0
    pbf.kernel_radius = spacing * 4.0
    pbf.solver_iterations = 4
    pbf.xsph_viscosity = 0.01
    pbf.epsilon = 100.0

    world = novaphy.FluidWorld(model, [block], novaphy.SolverSettings(), pbf,
                                boundary_extent=0.6)
    print(f"Created ball-in-water: {world.num_particles} fluid, "
          f"{world.num_boundary_particles} boundary particles")
    return world, spacing


def run_headless(world, n_steps=300, dt=1.0/120.0):
    """Run simulation without visualization."""
    print(f"Running {n_steps} steps headless...")
    for i in range(n_steps):
        world.step(dt)
        if i % 50 == 0:
            ball_y = world.state.transforms[0].position[1]
            fluid_ys = [p[1] for p in world.fluid_state.positions]
            print(f"  step {i}: ball_y={ball_y:.3f}, "
                  f"fluid_mean_y={np.mean(fluid_ys):.3f}")
    print("Done.")


def run_polyscope(world, spacing, dt=1.0/120.0):
    """Run with Polyscope visualization."""
    ps.init()
    ps.set_program_name("NovaPhy Ball in Water")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")

    # Register fluid particles
    positions = np.array([np.array(p) for p in world.fluid_state.positions])
    cloud = ps.register_point_cloud("fluid", positions,
                                     radius=spacing * 0.5,
                                     color=(0.2, 0.5, 0.9))

    # Register ball as a point (simple viz)
    ball_pos = world.state.transforms[0].position
    ball_cloud = ps.register_point_cloud("ball",
                                          np.array([[ball_pos[0], ball_pos[1], ball_pos[2]]]),
                                          radius=0.15,
                                          color=(0.9, 0.3, 0.2))

    def callback():
        world.step(dt)

        # Update fluid
        positions = np.array([np.array(p) for p in world.fluid_state.positions])
        cloud.update_point_positions(positions)

        # Update ball
        ball_pos = world.state.transforms[0].position
        ball_cloud.update_point_positions(
            np.array([[ball_pos[0], ball_pos[1], ball_pos[2]]]))

    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    world, spacing = build_scene()

    if "--headless" in sys.argv or not HAS_POLYSCOPE:
        run_headless(world)
    else:
        run_polyscope(world, spacing)
