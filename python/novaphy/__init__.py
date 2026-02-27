"""Python package entrypoint for NovaPhy.

Exports pybind11-backed physics engine APIs, including rigid-body simulation,
collision detection, articulated-body dynamics, and math/spatial utilities.
All physical quantities follow SI units (MKS) unless otherwise noted.
"""

from novaphy._core import (
    version,
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
)

__version__ = version()

__all__ = [
    "version",
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
]
