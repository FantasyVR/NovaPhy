# NovaPhy - Claude Code Rules

## Project Overview

NovaPhy is a C++20/Python 3D physics engine for embodied intelligence (robotics, RL, sim-to-real).
Working directory: `E:\NovaPhy`

## Architecture

- **ModelBuilder** builds scenes -> **Model** (immutable) -> **World** runs simulation
- Two solver pipelines:
  - **Free bodies**: Broadphase(SAP) -> Narrowphase -> Sequential Impulse (PGS)
  - **Articulated bodies**: FK -> RNEA -> CRBA -> Cholesky -> Semi-implicit Euler
- Python visualization via Polyscope

## Code Conventions

### C++
- **float32 only** - never use `double`. Use Eigen `*f` types: `Vec3f`, `Mat3f`, `Quatf`, `VecXf`, `MatXf`
- `using Scalar = float;` defined in `novaphy_types.h`
- Modern C++20: RAII, `std::unique_ptr` for ownership, `std::vector` for collections
- Header files in `include/novaphy/`, source files in `src/`
- Header-only for simple utilities (AABB, math typedefs)
- `.h` for headers, `.cpp` for sources
- `#pragma once` for include guards
- Namespace: `novaphy` for all code
- Spatial algebra convention: **[angular; linear]** (Featherstone convention)
- Contact normal convention: **from body_a toward body_b** (positive impulse separates)

### Python
- Package: `novaphy` (imports from `novaphy._core` C++ extension)
- Visualization: `novaphy.viz` module using Polyscope
- Demos: `demos/` directory, each script is standalone
- Use `set_linear_velocity()` / `set_angular_velocity()` to modify state (direct array assignment doesn't work through bindings)

### Naming
- C++ classes/structs: `PascalCase` (e.g., `RigidBody`, `SweepAndPrune`)
- C++ functions/methods: `snake_case` (e.g., `forward_kinematics`, `compute_aabb`)
- C++ member variables: `snake_case` with trailing underscore for private (e.g., `pairs_`)
- Python: standard PEP 8

## Build System

- **CMake** root at `CMakeLists.txt`
- **vcpkg** for C++ deps (eigen3, gtest) - manifest in `vcpkg.json`, path: `F:/vcpkg`
- **pybind11** via pip (NOT vcpkg, to avoid Python version conflicts)
- **scikit-build-core** for pip install - config in `pyproject.toml`
- **Conda** for Python env - config in `environment.yml`

### Build Commands
```bash
# Development install (set CMAKE_ARGS to point to your vcpkg)
conda activate novaphy
$env:CMAKE_TOOLCHAIN_FILE="C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake"
pip install -e .

# Python tests
pytest tests/python/ -v

# Run a demo
python demos/demo_stack.py
```

## File Organization

| Directory | Purpose |
|-----------|---------|
| `include/novaphy/math/` | Math types (Vec3f, Mat3f, Quatf), spatial algebra, utilities |
| `include/novaphy/core/` | Body, Shape, Joint, Articulation, Model, ModelBuilder, AABB, Contact |
| `include/novaphy/collision/` | Broadphase (SAP), Narrowphase (5 collision pairs) |
| `include/novaphy/dynamics/` | Integrator, FreeBodySolver, Featherstone, ArticulatedSolver |
| `include/novaphy/sim/` | World, SimState |
| `src/` | C++ implementations (mirrors include/) |
| `python/novaphy/` | Python package (`__init__.py`, `viz.py`) |
| `python/bindings/` | pybind11 binding files (bind_math/core/collision/sim/dynamics) |
| `tests/python/` | pytest test files (4 files, 38 tests) |
| `demos/` | 10 demo scripts + shared `demo_utils.py` |

## Testing Rules

- Every new Python-exposed feature needs a pytest in `tests/python/`
- Test names: `test_<module>.py`
- Use analytical comparisons for physics (free fall, pendulum period, energy conservation)
- All 38 tests must pass before committing

## Key Design Patterns

- `ModelBuilder` (mutable) -> `Model` (immutable) -> `World` (simulation)
- Collision: broadphase filters, narrowphase generates contacts, solver resolves
- Featherstone: FK -> bias forces (RNEA) -> mass matrix (CRBA) -> Cholesky solve -> integrate
- Contact solver: accumulated impulse clamping, warm starting, Baumgarte stabilization
- Narrowphase sets body_a/body_b and normal direction; World.step() does NOT override body indices

## Known Quirks

- vcpkg pybind11 conflicts with conda Python — use pip pybind11 instead
- Python bindings return copies of std::vector — use setter methods to modify C++ state
- Plane shapes use body_index=-1 (world-owned, always static)
- Free joint q = [px, py, pz, qx, qy, qz, qw] (7 DOF), qd = [wx, wy, wz, vx, vy, vz] (6 DOF)
