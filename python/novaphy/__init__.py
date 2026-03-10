"""Python package entrypoint for NovaPhy.

Exports pybind11-backed physics engine APIs, including rigid-body simulation,
collision detection, articulated-body dynamics, and math/spatial utilities.
All physical quantities follow SI units (MKS) unless otherwise noted.
"""

import os as _os
import sys as _sys
from pathlib import Path as _Path

def _add_dll_directories():
    """Add shared library search paths for libuipc and its dependencies."""
    _pkg_dir = _Path(__file__).resolve().parent
    _build_root = _pkg_dir.parent.parent / "build"

    if _sys.platform == "win32":
        # Search scikit-build-core dirs AND standalone build dirs (e.g. local-ipc-cxx20)
        _candidates = list(sorted(_build_root.glob("cp*-win_amd64"), reverse=True))
        if _build_root.is_dir():
            for _d in _build_root.iterdir():
                if _d.is_dir() and not _d.name.startswith("cp") and (_d / "Release" / "bin").is_dir():
                    _candidates.append(_d)
        for _d in _candidates:
            _bin_dir = _d / "Release" / "bin"
            if _bin_dir.is_dir():
                _os.add_dll_directory(str(_bin_dir))
            _vcpkg_bin = _d / "vcpkg_installed" / "x64-windows" / "bin"
            if _vcpkg_bin.is_dir():
                _os.add_dll_directory(str(_vcpkg_bin))
        _cuda_path = _os.environ.get("CUDA_PATH", "")
        # CUDA runtime
        if _cuda_path:
            _cuda_bin = _Path(_cuda_path) / "bin"
            if _cuda_bin.is_dir():
                _os.add_dll_directory(str(_cuda_bin))
    elif _sys.platform.startswith("linux"):
        import ctypes
        import ctypes.util
        # Find all Linux build dirs (scikit-build-core naming pattern)
        for _d in sorted(_build_root.glob("cp*-linux_*"), reverse=True):
            _bin_dir = _d / "Release" / "bin"
            if not _bin_dir.is_dir():
                continue
            # Preload libuipc .so files so the dynamic linker can find them
            # when _core.so imports them at load time.
            for _so in sorted(_bin_dir.glob("libuipc*.so*")):
                try:
                    ctypes.CDLL(str(_so), mode=ctypes.RTLD_GLOBAL)
                except OSError:
                    pass

_add_dll_directories()

from novaphy._core import (
    version,
    has_ipc,
    # Math types
    Transform,
    SpatialTransform,
    # Math functions
    skew,
    spatial_cross_motion,
    spatial_cross_force,
    spatial_inertia_matrix,
    deg2rad,
    rad2deg,
    # Core types
    ShapeType,
    RigidBody,
    CollisionShape,
    AABB,
    ContactPoint,
    # Collision
    BroadPhasePair,
    SweepAndPrune,
    collide_shapes,
    PerformanceMonitor,
    PerformanceMetric,
    PerformancePhaseStat,
    # Simulation
    ModelBuilder,
    Model,
    SolverSettings,
    SimState,
    World,
    # Articulated bodies
    JointType,
    Joint,
    Articulation,
    ArticulatedSolver,
    # Featherstone algorithms
    forward_kinematics,
    inverse_dynamics,
    mass_matrix_crba,
    forward_dynamics,
    # Fluid simulation
    SPHKernels,
    FluidBlockDef,
    ParticleState,
    SpatialHashGrid,
    generate_fluid_block,
    PBFSettings,
    PBFSolver,
    FluidWorld,
    BoundaryParticle,
    sample_model_boundaries,
    UrdfGeometryType,
    UrdfGeometry,
    UrdfVisual,
    UrdfCollision,
    UrdfInertial,
    UrdfLink,
    UrdfJoint,
    UrdfModelData,
    UsdAnimationTrack,
    UsdPrim,
    UsdStageData,
    SceneBuildResult,
    FeatureCheckItem,
    FeatureCheckReport,
    RecordedKeyframe,
    RecordedCollisionEvent,
    RecordedConstraintReaction,
    UrdfParser,
    OpenUsdImporter,
    SceneBuilderEngine,
    SimulationExporter,
    FeatureCompletenessChecker,
    # VBD/AVBD
    VBDConfig,
    VBDWorld,
)

# Optional IPC support (requires CUDA + libuipc)
try:
    from novaphy._core import IPCConfig, IPCWorld
except ImportError:
    pass

__version__ = version()

__all__ = [
    "version",
    "has_ipc",
    "Transform",
    "SpatialTransform",
    "skew",
    "spatial_cross_motion",
    "spatial_cross_force",
    "spatial_inertia_matrix",
    "deg2rad",
    "rad2deg",
    "ShapeType",
    "RigidBody",
    "CollisionShape",
    "AABB",
    "ContactPoint",
    "BroadPhasePair",
    "SweepAndPrune",
    "collide_shapes",
    "PerformanceMonitor",
    "PerformanceMetric",
    "PerformancePhaseStat",
    "ModelBuilder",
    "Model",
    "SolverSettings",
    "SimState",
    "World",
    "JointType",
    "Joint",
    "Articulation",
    "ArticulatedSolver",
    "forward_kinematics",
    "inverse_dynamics",
    "mass_matrix_crba",
    "forward_dynamics",
    "SPHKernels",
    "FluidBlockDef",
    "ParticleState",
    "SpatialHashGrid",
    "generate_fluid_block",
    "PBFSettings",
    "PBFSolver",
    "FluidWorld",
    "BoundaryParticle",
    "sample_model_boundaries",
    "UrdfGeometryType",
    "UrdfGeometry",
    "UrdfVisual",
    "UrdfCollision",
    "UrdfInertial",
    "UrdfLink",
    "UrdfJoint",
    "UrdfModelData",
    "UsdAnimationTrack",
    "UsdPrim",
    "UsdStageData",
    "SceneBuildResult",
    "FeatureCheckItem",
    "FeatureCheckReport",
    "RecordedKeyframe",
    "RecordedCollisionEvent",
    "RecordedConstraintReaction",
    "UrdfParser",
    "OpenUsdImporter",
    "SceneBuilderEngine",
    "SimulationExporter",
    "FeatureCompletenessChecker",
    # VBD/AVBD
    "VBDConfig",
    "VBDWorld",
]

# Conditionally export IPC symbols
if has_ipc():
    __all__ += ["IPCConfig", "IPCWorld"]
