"""Tests for NovaPhy fluid simulation infrastructure (Phase 1)."""

import math
import numpy as np
import numpy.testing as npt
import novaphy


# ========== SPH Kernel Tests ==========

def test_poly6_at_zero():
    """Poly6 kernel should have maximum value at r=0."""
    h = 0.1
    val = novaphy.SPHKernels.poly6(0.0, h)
    assert val > 0, f"Poly6(0, h) should be positive, got {val}"


def test_poly6_at_boundary():
    """Poly6 kernel should be zero (or negligible) at r=h."""
    h = 0.1
    val = novaphy.SPHKernels.poly6(h * h, h)
    assert abs(val) < 1e-10, f"Poly6(h^2, h) should be ~zero, got {val}"


def test_poly6_outside():
    """Poly6 kernel should be zero for r > h."""
    h = 0.1
    val = novaphy.SPHKernels.poly6(h * h * 1.1, h)
    assert val == 0.0, f"Poly6 outside support should be zero, got {val}"


def test_poly6_normalization():
    """Numerical integration of Poly6 in 3D should approximate 1."""
    h = 0.1
    # Integrate W(r) * 4*pi*r^2 dr from 0 to h
    n_steps = 1000
    dr = h / n_steps
    integral = 0.0
    for i in range(n_steps):
        r = (i + 0.5) * dr
        r_sq = r * r
        w = novaphy.SPHKernels.poly6(r_sq, h)
        integral += w * 4.0 * math.pi * r_sq * dr
    npt.assert_allclose(integral, 1.0, atol=0.02,
                        err_msg=f"Poly6 3D integral = {integral}, expected ~1.0")


def test_spiky_at_zero():
    """Spiky kernel value at r=0 should be positive."""
    h = 0.1
    val = novaphy.SPHKernels.spiky(0.0, h)
    # spiky(0) = 15/(pi*h^6) * h^3, should be positive
    assert val > 0, f"Spiky(0, h) should be positive, got {val}"


def test_spiky_at_boundary():
    """Spiky kernel value should be zero at r=h."""
    h = 0.1
    val = novaphy.SPHKernels.spiky(h, h)
    assert val == 0.0, f"Spiky(h, h) should be zero, got {val}"


def test_spiky_grad_zero_at_boundary():
    """Spiky gradient should be zero at r >= h."""
    h = 0.1
    r = np.array([h, 0, 0], dtype=np.float32)
    grad = novaphy.SPHKernels.spiky_grad(r, h)
    npt.assert_allclose(grad, [0, 0, 0], atol=1e-7)


def test_spiky_grad_direction():
    """Spiky gradient should point from j to i (opposite of r direction)."""
    h = 0.1
    r = np.array([0.05, 0, 0], dtype=np.float32)
    grad = novaphy.SPHKernels.spiky_grad(r, h)
    # Gradient should be negative in x (points from j toward i means -x if r is in +x)
    assert grad[0] < 0, f"Spiky grad x should be negative, got {grad[0]}"


def test_cubic_spline_at_zero():
    """Cubic spline should have maximum at r=0."""
    h = 0.1
    val = novaphy.SPHKernels.cubic_spline(0.0, h)
    assert val > 0, f"Cubic spline(0, h) should be positive, got {val}"


def test_cubic_spline_at_boundary():
    """Cubic spline should be zero at r=h."""
    h = 0.1
    val = novaphy.SPHKernels.cubic_spline(h, h)
    assert val == 0.0, f"Cubic spline(h, h) should be zero, got {val}"


# ========== FluidBlockDef and generate_fluid_block Tests ==========

def test_fluid_block_def_defaults():
    """FluidBlockDef should have sensible defaults."""
    block = novaphy.FluidBlockDef()
    npt.assert_allclose(block.particle_spacing, 0.02, atol=1e-6)
    npt.assert_allclose(block.rest_density, 1000.0, atol=1e-3)
    npt.assert_allclose(block.lower, [0, 0, 0])
    npt.assert_allclose(block.upper, [0, 0, 0])


def test_generate_fluid_block_count():
    """Generated block should have expected number of particles."""
    block = novaphy.FluidBlockDef()
    block.lower = np.array([0, 0, 0], dtype=np.float32)
    block.upper = np.array([0.1, 0.1, 0.1], dtype=np.float32)
    block.particle_spacing = 0.05

    positions = novaphy.generate_fluid_block(block)
    # 0.0, 0.05, 0.10 => 3 per axis => 3^3 = 27
    assert len(positions) == 27, f"Expected 27 particles, got {len(positions)}"


def test_generate_fluid_block_bounds():
    """All particles should be within [lower, upper] bounds."""
    block = novaphy.FluidBlockDef()
    block.lower = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    block.upper = np.array([1.5, 2.5, 3.5], dtype=np.float32)
    block.particle_spacing = 0.1

    positions = novaphy.generate_fluid_block(block)
    for pos in positions:
        p = np.array(pos)
        assert np.all(p >= block.lower - 1e-6), f"Particle below lower bound: {p}"
        assert np.all(p <= block.upper + 1e-6), f"Particle above upper bound: {p}"


def test_generate_fluid_block_spacing():
    """Adjacent particles should be separated by particle_spacing."""
    block = novaphy.FluidBlockDef()
    block.lower = np.array([0, 0, 0], dtype=np.float32)
    block.upper = np.array([0.2, 0.0, 0.0], dtype=np.float32)
    block.particle_spacing = 0.1

    positions = novaphy.generate_fluid_block(block)
    # Should get 3 particles along x: 0.0, 0.1, 0.2
    assert len(positions) == 3, f"Expected 3, got {len(positions)}"
    dists = [np.linalg.norm(np.array(positions[i+1]) - np.array(positions[i]))
             for i in range(len(positions) - 1)]
    for d in dists:
        npt.assert_allclose(d, 0.1, atol=1e-5)


# ========== ParticleState Tests ==========

def test_particle_state_init():
    """ParticleState should initialize with correct sizes."""
    positions = [
        np.array([0, 0, 0], dtype=np.float32),
        np.array([1, 0, 0], dtype=np.float32),
        np.array([0, 1, 0], dtype=np.float32),
    ]
    state = novaphy.ParticleState()
    state.init(positions)

    assert state.num_particles == 3
    assert len(state.positions) == 3
    assert len(state.velocities) == 3
    assert len(state.densities) == 3


def test_particle_state_init_with_velocity():
    """ParticleState init should set initial velocity."""
    positions = [np.array([0, 0, 0], dtype=np.float32)]
    vel = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    state = novaphy.ParticleState()
    state.init(positions, vel)

    npt.assert_allclose(state.velocities[0], [1, 2, 3], atol=1e-6)


def test_particle_state_clear():
    """ParticleState clear should empty all arrays."""
    positions = [np.array([0, 0, 0], dtype=np.float32)]
    state = novaphy.ParticleState()
    state.init(positions)
    assert state.num_particles == 1

    state.clear()
    assert state.num_particles == 0


# ========== SpatialHashGrid Tests ==========

def test_spatial_hash_grid_creation():
    """Grid should store cell_size correctly."""
    grid = novaphy.SpatialHashGrid(0.1)
    npt.assert_allclose(grid.cell_size, 0.1)


def test_spatial_hash_grid_find_self():
    """Grid query should find the particle itself."""
    positions = [np.array([0.5, 0.5, 0.5], dtype=np.float32)]
    grid = novaphy.SpatialHashGrid(0.2)
    grid.build(positions)

    neighbors = grid.query_neighbors(
        np.array([0.5, 0.5, 0.5], dtype=np.float32), 0.2)
    assert 0 in neighbors, "Should find particle at query point"


def test_spatial_hash_grid_find_close_neighbor():
    """Grid should find a particle within radius."""
    positions = [
        np.array([0.0, 0.0, 0.0], dtype=np.float32),
        np.array([0.05, 0.0, 0.0], dtype=np.float32),
    ]
    grid = novaphy.SpatialHashGrid(0.1)
    grid.build(positions)

    neighbors = grid.query_neighbors(
        np.array([0.0, 0.0, 0.0], dtype=np.float32), 0.1)
    assert 0 in neighbors
    assert 1 in neighbors


def test_spatial_hash_grid_no_far_neighbor():
    """Grid should not include particles outside 3x3x3 cell neighborhood."""
    positions = [
        np.array([0.0, 0.0, 0.0], dtype=np.float32),
        np.array([5.0, 5.0, 5.0], dtype=np.float32),
    ]
    grid = novaphy.SpatialHashGrid(0.1)
    grid.build(positions)

    neighbors = grid.query_neighbors(
        np.array([0.0, 0.0, 0.0], dtype=np.float32), 0.1)
    assert 0 in neighbors
    assert 1 not in neighbors, "Far particle should not be a neighbor"


def test_spatial_hash_grid_many_particles():
    """Grid should handle moderate particle counts correctly."""
    np.random.seed(42)
    n = 500
    positions = [np.array(np.random.uniform(0, 1, 3), dtype=np.float32)
                 for _ in range(n)]

    grid = novaphy.SpatialHashGrid(0.1)
    grid.build(positions)

    # Check that a brute-force search matches the grid query for one particle
    query = np.array([0.5, 0.5, 0.5], dtype=np.float32)
    radius = 0.1

    grid_neighbors = set(grid.query_neighbors(query, radius))

    # Brute force: find all particles within radius
    brute_neighbors = set()
    for i, p in enumerate(positions):
        if np.linalg.norm(np.array(p) - query) < radius:
            brute_neighbors.add(i)

    # Grid returns candidates from cells, which may include some just outside radius.
    # All brute-force neighbors must be in the grid result (no false negatives).
    assert brute_neighbors.issubset(grid_neighbors), \
        f"Missing neighbors: {brute_neighbors - grid_neighbors}"
