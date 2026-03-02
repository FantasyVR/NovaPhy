"""Demo: Fluid particles in a moving box.

Demonstrates PBF fluid simulation in isolation (no rigid bodies):
- A block of fluid settles under gravity inside a domain-bounded box
- The box oscillates horizontally, causing the fluid to slosh
- Polyscope visualization with speed-based particle coloring

Scene: ~8000 fluid particles in a 0.5 x 0.5 x 0.5 m box that
moves sinusoidally along the X axis.
"""

import sys
import os
import math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

try:
    import polyscope as ps
    HAS_POLYSCOPE = True
except ImportError:
    HAS_POLYSCOPE = False

import novaphy


def build_fluid():
    """Build a standalone PBF fluid simulation (no rigid bodies).

    Returns:
        tuple: (PBFSolver, ParticleState, spacing, particle_mass)
    """
    spacing = 0.025
    rest_density = 1000.0

    # Generate fluid block
    block = novaphy.FluidBlockDef()
    block.lower = np.array([0.05, 0.05, 0.05], dtype=np.float32)
    block.upper = np.array([0.45, 0.30, 0.45], dtype=np.float32)
    block.particle_spacing = spacing
    block.rest_density = rest_density

    positions = novaphy.generate_fluid_block(block)

    # Initialize particle state
    state = novaphy.ParticleState()
    state.init(positions)

    # PBF solver settings
    pbf = novaphy.PBFSettings()
    pbf.rest_density = rest_density
    pbf.kernel_radius = spacing * 4.0
    pbf.solver_iterations = 4
    pbf.xsph_viscosity = 0.01
    pbf.epsilon = 100.0
    pbf.particle_radius = spacing * 0.5

    # Domain bounds (the "box" container)
    pbf.use_domain_bounds = True
    pbf.domain_lower = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    pbf.domain_upper = np.array([0.5, 0.8, 0.5], dtype=np.float32)

    solver = novaphy.PBFSolver(pbf)
    particle_mass = pbf.particle_mass(spacing)

    print(f"Scene created:")
    print(f"  Fluid particles: {state.num_particles}")
    print(f"  Particle mass: {particle_mass:.6f} kg")
    print(f"  Kernel radius: {pbf.kernel_radius:.4f} m")
    print(f"  Domain: [{pbf.domain_lower[0]:.2f}, {pbf.domain_lower[1]:.2f}, {pbf.domain_lower[2]:.2f}]"
          f" to [{pbf.domain_upper[0]:.2f}, {pbf.domain_upper[1]:.2f}, {pbf.domain_upper[2]:.2f}]")

    return solver, state, spacing, particle_mass


def run_headless(solver, state, particle_mass, n_steps=500, dt=1.0/120.0):
    """Run simulation without visualization."""
    gravity = np.array([0.0, -9.81, 0.0], dtype=np.float32)

    # Box oscillation parameters
    amplitude = 0.15     # m
    frequency = 0.5      # Hz
    box_center_x = 0.25  # center of the box

    print(f"Running {n_steps} steps headless (dt={dt:.4f}s)...")
    for i in range(n_steps):
        t = i * dt

        # Move the box: oscillate domain bounds along X
        offset_x = amplitude * math.sin(2 * math.pi * frequency * t)
        solver.settings.domain_lower = np.array(
            [box_center_x - 0.25 + offset_x, 0.0, 0.0], dtype=np.float32)
        solver.settings.domain_upper = np.array(
            [box_center_x + 0.25 + offset_x, 0.8, 0.5], dtype=np.float32)

        solver.step(state, dt, gravity, particle_mass)

        if i % 100 == 0:
            ys = [p[1] for p in state.positions]
            xs = [p[0] for p in state.positions]
            print(f"  step {i:4d}: "
                  f"x=[{np.min(xs):.3f}, {np.mean(xs):.3f}, {np.max(xs):.3f}]  "
                  f"y=[{np.min(ys):.3f}, {np.mean(ys):.3f}, {np.max(ys):.3f}]  "
                  f"box_offset={offset_x:.3f}")
    print("Done.")


def run_polyscope(solver, state, spacing, particle_mass, dt=1.0/120.0):
    """Run with full Polyscope visualization."""
    ps.init()
    ps.set_program_name("NovaPhy - Fluid in Moving Box")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")

    # Camera
    ps.look_at((1.0, 0.6, 1.0), (0.25, 0.2, 0.25))

    # Register fluid particles
    fluid_pos = np.array([np.array(p) for p in state.positions], dtype=np.float32)
    fluid_cloud = ps.register_point_cloud("fluid", fluid_pos)
    fluid_cloud.set_radius(spacing * 0.45, relative=False)
    fluid_cloud.set_color((0.15, 0.45, 0.85))

    # Register box wireframe as line segments
    def make_box_lines(lo, hi):
        """Create 12 edges of an axis-aligned box."""
        v = np.array([
            [lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]],
            [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
            [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]],
            [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]],
        ], dtype=np.float32)
        edges = np.array([
            [0,1],[1,2],[2,3],[3,0],  # front face
            [4,5],[5,6],[6,7],[7,4],  # back face
            [0,4],[1,5],[2,6],[3,7],  # connecting edges
        ], dtype=np.int32)
        return v, edges

    lo = solver.settings.domain_lower
    hi = solver.settings.domain_upper
    box_v, box_e = make_box_lines(lo, hi)
    box_net = ps.register_curve_network("box", box_v, box_e)
    box_net.set_color((0.8, 0.8, 0.8))
    box_net.set_radius(0.003, relative=False)

    gravity = np.array([0.0, -9.81, 0.0], dtype=np.float32)

    # Box oscillation parameters
    amplitude = 0.15
    frequency = 0.5
    box_center_x = 0.25

    step_count = [0]

    def callback():
        t = step_count[0] * dt

        # Move the box
        offset_x = amplitude * math.sin(2 * math.pi * frequency * t)
        new_lo = np.array(
            [box_center_x - 0.25 + offset_x, 0.0, 0.0], dtype=np.float32)
        new_hi = np.array(
            [box_center_x + 0.25 + offset_x, 0.8, 0.5], dtype=np.float32)
        solver.settings.domain_lower = new_lo
        solver.settings.domain_upper = new_hi

        # Step physics
        solver.step(state, dt, gravity, particle_mass)
        step_count[0] += 1

        # Update fluid particle positions
        fluid_pos = np.array([np.array(p) for p in state.positions],
                             dtype=np.float32)
        fluid_cloud.update_point_positions(fluid_pos)

        # Update box wireframe
        box_v, box_e = make_box_lines(new_lo, new_hi)
        box_net.update_node_positions(box_v)

        # Color by speed every few frames
        if step_count[0] % 3 == 0:
            vels = np.array([np.array(v) for v in state.velocities],
                            dtype=np.float32)
            speeds = np.linalg.norm(vels, axis=1)
            max_speed = max(speeds.max(), 0.01)
            t_val = np.clip(speeds / max_speed, 0.0, 1.0)
            colors = np.column_stack([
                0.15 + 0.85 * t_val,
                0.45 + 0.55 * t_val,
                0.85 + 0.15 * t_val,
            ])
            fluid_cloud.add_color_quantity("speed", colors, enabled=True)

    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    solver, state, spacing, particle_mass = build_fluid()

    if "--headless" in sys.argv or not HAS_POLYSCOPE:
        if not HAS_POLYSCOPE:
            print("Polyscope not available, running headless.")
        run_headless(solver, state, particle_mass)
    else:
        run_polyscope(solver, state, spacing, particle_mass)
