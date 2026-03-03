# NovaPhy

A C++17/Python 3D physics engine for embodied intelligence (robotics, RL, sim-to-real).

## Features

- **Rigid Body Dynamics** — collision detection + Sequential Impulse solver (PGS) with warm starting, Coulomb friction, and Baumgarte stabilization
- **Articulated Body Dynamics** — Featherstone algorithms in generalized coordinates (FK, RNEA inverse dynamics, CRBA mass matrix, Cholesky forward dynamics)
- **Collision Detection** — Sweep-and-Prune broadphase + 5 narrowphase pairs (sphere-sphere, sphere-plane, box-sphere, box-plane, box-box SAT)
- **Joint Types** — Revolute, Fixed, Free (6-DOF), Slide (prismatic), Ball (spherical)
- **Position Based Fluids** — PBF solver (Macklin & Müller 2013) with SPH kernels, spatial hash grid neighbor search, XSPH viscosity, tensile instability correction, and vorticity confinement
- **Rigid-Fluid Coupling** — Akinci et al. 2012 boundary particle method for two-way fluid-solid interaction
- **IPC Contact** *(optional)* — GPU-accelerated Incremental Potential Contact via [libuipc](https://github.com/spiriMirror/libuipc) with mathematically guaranteed penetration-free contact (requires CUDA ≥ 12.4)
- **Python API** via pybind11 with Polyscope visualization
- **pip-installable** C++17 core via scikit-build-core

## Quick Start

### Prerequisites

- [Conda](https://docs.conda.io/) (Miniconda or Anaconda)
- [vcpkg](https://vcpkg.io/) installed (default: `F:/vcpkg`)
- C++17 compiler (MSVC 2019+, GCC 9+, Clang 10+)
- *(Optional for IPC)* CUDA ≥ 12.4

### Setup

```bash
# Create conda environment
conda env create -f environment.yml
conda activate novaphy

# Install NovaPhy
pip install -e .

# (Optional) Install with IPC support
CMAKE_ARGS="-DNOVAPHY_WITH_IPC=ON -DCMAKE_TOOLCHAIN_FILE=F:/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_CUDA_COMPILER=/path/to/nvcc" pip install -e .
```

### Verify

```python
import novaphy
print(novaphy.version())   # 0.1.0
print(novaphy.has_ipc())   # True if built with IPC
```

### Run a Demo

```bash
# Rigid body demos
python demos/demo_stack.py
python demos/demo_double_pendulum.py

# Fluid demos
python demos/demo_dam_break.py
python demos/demo_ball_in_water.py

# IPC demo (requires IPC build)
python demos/demo_ipc_stack.py
```

## Demos

### Rigid Body

| Demo | Description | Physics |
|------|-------------|---------|
| `demo_stack.py` | 3 boxes falling and stacking | Free body + contact solver |
| `demo_pyramid.py` | 4-3-2-1 box pyramid | Multi-body stacking |
| `demo_friction_ramp.py` | Boxes on 30° ramp, different friction | Coulomb friction |
| `demo_wall_break.py` | 5×5 wall hit by sphere | Dynamic collision |
| `demo_newtons_cradle.py` | 5 elastic spheres | Restitution + momentum |
| `demo_dominoes.py` | 20 dominoes chain reaction | Angular impulse |

### Articulated Body

| Demo | Description | Physics |
|------|-------------|---------|
| `demo_double_pendulum.py` | Chaotic 2-link pendulum | Featherstone CRBA |
| `demo_hinge.py` | Door swinging on hinge | Revolute joint |
| `demo_rope_bridge.py` | 10-segment rope | Multi-joint chain |
| `demo_joint_chain.py` | 6-link hanging chain | 3D articulation |

### Fluid

| Demo | Description | Physics |
|------|-------------|---------|
| `demo_dam_break.py` | Rectangular fluid block collapses | PBF solver |
| `demo_fluid_box.py` | ~8000 particles sloshing in a moving box | PBF + domain bounds |
| `demo_ball_in_water.py` | Ball dropping into water | Rigid-fluid coupling (Akinci) |
| `demo_fluid_coupling.py` | Boxes and spheres splashing into water | PBF + Akinci + rigid collision |

### IPC

| Demo | Description | Physics |
|------|-------------|---------|
| `demo_ipc_stack.py` | Box stacking with guaranteed no-penetration | IPC via libuipc (CUDA) |

## Architecture

```
User (Python) -> ModelBuilder -> Model (immutable) -> World -> step(dt)
                                                       |
                                                       |-> Free bodies:   SAP -> Narrowphase -> Sequential Impulse (PGS)
                                                       |-> Articulated:   FK -> RNEA(bias) -> CRBA(H) -> Cholesky -> Integrate
                                                       |
                                              FluidWorld extends World:
                                                       |-> PBF Solver:    predict -> neighbor search -> density constraint (iter) -> update
                                                       |-> Akinci:        boundary sampling -> density contribution -> coupling forces
                                                       |
                                              IPCWorld (standalone):
                                                       |-> libuipc:       tet mesh conversion -> GPU Newton solver -> barrier contact

Visualization: Polyscope (Python-side per-frame mesh/particle transform updates)
```

## Python API Overview

```python
import numpy as np
import novaphy

# === Free body simulation ===
builder = novaphy.ModelBuilder()
builder.add_ground_plane(y=0.0)
body = novaphy.RigidBody.from_box(1.0, np.array([0.5, 0.5, 0.5]))
idx = builder.add_body(body, novaphy.Transform.from_translation(np.array([0, 5, 0])))
builder.add_shape(novaphy.CollisionShape.make_box(np.array([0.5, 0.5, 0.5]), idx))
world = novaphy.World(builder.build())
for _ in range(1000):
    world.step(1/120)

# === Articulated body ===
art = novaphy.Articulation()
joint = novaphy.Joint()
joint.type = novaphy.JointType.Revolute
joint.axis = np.array([0, 0, 1])
art.joints = [joint]
art.bodies = [novaphy.RigidBody.from_box(1.0, np.array([0.1, 0.5, 0.1]))]
art.build_spatial_inertias()
q = np.array([0.5])
qd = np.zeros(1)
H = novaphy.mass_matrix_crba(art, q)
qdd = novaphy.forward_dynamics(art, q, qd, np.zeros(1), np.array([0, -9.81, 0]))

# === PBF fluid simulation ===
fluid_block = novaphy.FluidBlockDef()
fluid_block.lower = np.array([0.0, 0.0, 0.0])
fluid_block.upper = np.array([1.0, 1.0, 1.0])
fluid_block.spacing = 0.05
particles = novaphy.generate_fluid_block(fluid_block)

settings = novaphy.PBFSettings()
settings.h = 4.0 * 0.05  # kernel radius = 4 * spacing
settings.solver_iterations = 4

fluid_world = novaphy.FluidWorld(builder.build(), settings, [fluid_block])
for _ in range(500):
    fluid_world.step(1/120)

# === IPC simulation (requires NOVAPHY_WITH_IPC=ON) ===
if novaphy.has_ipc():
    config = novaphy.IPCConfig()
    config.dt = 1/60
    ipc_world = novaphy.IPCWorld(builder.build(), config)
    for _ in range(100):
        ipc_world.step()
```

## File Organization

| Directory | Purpose |
|-----------|---------|
| `include/novaphy/math/` | Math types (Vec3f, Mat3f, Quatf), spatial algebra, utilities |
| `include/novaphy/core/` | Body, Shape, Joint, Articulation, Model, ModelBuilder, AABB, Contact |
| `include/novaphy/collision/` | Broadphase (SAP), Narrowphase (5 collision pairs) |
| `include/novaphy/dynamics/` | Integrator, FreeBodySolver, Featherstone, ArticulatedSolver |
| `include/novaphy/fluid/` | SPH kernels, ParticleState, SpatialHashGrid, PBFSolver, FluidWorld, Akinci boundary |
| `include/novaphy/ipc/` | IPCConfig, IPCWorld, shape-to-tetmesh converter |
| `src/` | C++ implementations (mirrors include/) |
| `python/novaphy/` | Python package (`__init__.py`, `viz.py`) |
| `python/bindings/` | pybind11 bindings (math, core, collision, sim, dynamics, fluid, ipc) |
| `tests/python/` | pytest test files (8 files, 97 tests) |
| `demos/` | 14 demo scripts + shared `demo_utils.py` |

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Core | C++17 (float32 only) |
| Math | Eigen3 |
| Bindings | pybind11 |
| Build | CMake + scikit-build-core |
| C++ Deps | vcpkg (eigen3, gtest) |
| Visualization | Polyscope |
| IPC (optional) | libuipc + CUDA ≥ 12.4 |
| CI | GitHub Actions (Ubuntu + Windows) |

## Testing

```bash
pytest tests/python/ -v     # 97 tests
```

Tests cover math utilities, collision detection, free body simulation, articulated dynamics, PBF fluid, rigid-fluid coupling, and IPC contact.

## License

MIT
