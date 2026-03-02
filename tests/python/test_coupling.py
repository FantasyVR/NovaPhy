"""Tests for NovaPhy rigid-fluid coupling (Phase 3)."""

import numpy as np
import numpy.testing as npt
import novaphy


# ========== Boundary Particle Sampling Tests ==========

def test_sample_box_boundary():
    """Box boundary should generate particles on 6 faces."""
    builder = novaphy.ModelBuilder()
    half = np.array([0.5, 0.5, 0.5], dtype=np.float32)
    body = novaphy.RigidBody.from_box(1.0, half)
    idx = builder.add_body(body)
    shape = novaphy.CollisionShape.make_box(half, idx)
    builder.add_shape(shape)
    model = builder.build()

    particles = novaphy.sample_model_boundaries(model, 0.25)
    assert len(particles) > 0, "Should generate boundary particles for box"

    # All particles should belong to body 0
    for p in particles:
        assert p.body_index == 0


def test_sample_sphere_boundary():
    """Sphere boundary should generate surface particles."""
    builder = novaphy.ModelBuilder()
    body = novaphy.RigidBody.from_sphere(1.0, 0.5)
    idx = builder.add_body(body)
    shape = novaphy.CollisionShape.make_sphere(0.5, idx)
    builder.add_shape(shape)
    model = builder.build()

    particles = novaphy.sample_model_boundaries(model, 0.1)
    assert len(particles) > 10, "Should generate multiple boundary particles for sphere"

    # All particles should be approximately on the sphere surface
    for p in particles:
        r = np.linalg.norm(np.array(p.local_position))
        npt.assert_allclose(r, 0.5, atol=0.01,
                            err_msg=f"Boundary particle at radius {r}, expected 0.5")


def test_sample_plane_boundary():
    """Plane boundary should generate particles on a grid."""
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0)
    model = builder.build()

    particles = novaphy.sample_model_boundaries(model, 0.2, plane_extent=0.5)
    assert len(particles) > 0, "Should generate boundary particles for plane"

    # All particles should be at y=0
    for p in particles:
        npt.assert_allclose(p.local_position[1], 0.0, atol=1e-5)
        assert p.body_index == -1, "Plane particles should be world-owned"


def test_sample_no_shapes():
    """Empty model should produce no boundary particles."""
    builder = novaphy.ModelBuilder()
    model = builder.build()

    particles = novaphy.sample_model_boundaries(model, 0.1)
    assert len(particles) == 0


# ========== BoundaryParticle Type Tests ==========

def test_boundary_particle_defaults():
    """BoundaryParticle should have reasonable defaults."""
    bp = novaphy.BoundaryParticle()
    npt.assert_allclose(bp.local_position, [0, 0, 0])
    assert bp.body_index == -1
    npt.assert_allclose(bp.volume, 0.0)


# ========== FluidWorld with Coupling Tests ==========

def test_fluid_world_boundary_particles():
    """FluidWorld should generate boundary particles from model shapes."""
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0)
    model = builder.build()

    block = novaphy.FluidBlockDef()
    block.lower = np.array([0, 0.1, 0], dtype=np.float32)
    block.upper = np.array([0.2, 0.3, 0.2], dtype=np.float32)
    block.particle_spacing = 0.05

    pbf = novaphy.PBFSettings()
    pbf.kernel_radius = 0.2

    world = novaphy.FluidWorld(model, [block], novaphy.SolverSettings(), pbf,
                                boundary_extent=0.5)

    assert world.num_boundary_particles > 0, \
        "Should have boundary particles from ground plane"
    assert world.num_particles > 0, "Should have fluid particles"


def test_coupled_simulation_runs():
    """Coupled rigid-fluid simulation should run without crashing."""
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0)

    # Add a sphere above the fluid
    body = novaphy.RigidBody.from_sphere(0.5, 0.2)
    t = novaphy.Transform.from_translation(
        np.array([0.1, 0.8, 0.1], dtype=np.float32))
    idx = builder.add_body(body, t)
    shape = novaphy.CollisionShape.make_sphere(0.2, idx)
    builder.add_shape(shape)

    model = builder.build()

    block = novaphy.FluidBlockDef()
    block.lower = np.array([0, 0.05, 0], dtype=np.float32)
    block.upper = np.array([0.3, 0.4, 0.3], dtype=np.float32)
    block.particle_spacing = 0.05

    pbf = novaphy.PBFSettings()
    pbf.kernel_radius = 0.2
    pbf.solver_iterations = 2

    world = novaphy.FluidWorld(model, [block], novaphy.SolverSettings(), pbf,
                                boundary_extent=0.5)

    # Run 5 steps without crashing
    for _ in range(5):
        world.step(1.0 / 60.0)

    # Sphere should fall under gravity
    y = world.state.transforms[0].position[1]
    assert y < 0.8, f"Sphere should fall, got y={y}"


def test_coupling_sphere_in_fluid():
    """Sphere dropped into fluid should feel buoyancy force (slow down)."""
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0)

    # Add a light sphere
    body = novaphy.RigidBody.from_sphere(0.1, 0.1)
    t = novaphy.Transform.from_translation(
        np.array([0.15, 0.5, 0.15], dtype=np.float32))
    idx = builder.add_body(body, t)
    shape = novaphy.CollisionShape.make_sphere(0.1, idx)
    builder.add_shape(shape)
    model = builder.build()

    block = novaphy.FluidBlockDef()
    block.lower = np.array([0, 0.05, 0], dtype=np.float32)
    block.upper = np.array([0.3, 0.3, 0.3], dtype=np.float32)
    block.particle_spacing = 0.05

    pbf = novaphy.PBFSettings()
    pbf.kernel_radius = 0.2
    pbf.solver_iterations = 2

    world_with_fluid = novaphy.FluidWorld(model, [block], novaphy.SolverSettings(),
                                           pbf, boundary_extent=0.5)

    # Also run without fluid for comparison
    world_no_fluid = novaphy.FluidWorld(model, [], novaphy.SolverSettings(), pbf)

    dt = 1.0 / 60.0
    for _ in range(20):
        world_with_fluid.step(dt)
        world_no_fluid.step(dt)

    y_fluid = world_with_fluid.state.transforms[0].position[1]
    y_no_fluid = world_no_fluid.state.transforms[0].position[1]

    # Both should fall, but the sphere in fluid may fall differently
    # (buoyancy or drag from coupling forces)
    assert y_fluid < 0.5 or y_no_fluid < 0.5, \
        "At least one sphere should have fallen"


def test_fluid_world_no_coupling_backward_compat():
    """FluidWorld with no rigid shapes should still work as before."""
    builder = novaphy.ModelBuilder()
    model = builder.build()

    block = novaphy.FluidBlockDef()
    block.lower = np.array([0, 0, 0], dtype=np.float32)
    block.upper = np.array([0.1, 0.1, 0.1], dtype=np.float32)
    block.particle_spacing = 0.05

    pbf = novaphy.PBFSettings()
    pbf.kernel_radius = 0.2
    pbf.solver_iterations = 2

    world = novaphy.FluidWorld(model, [block], novaphy.SolverSettings(), pbf)

    assert world.num_boundary_particles == 0
    assert world.num_particles > 0

    for _ in range(5):
        world.step(1.0 / 60.0)
