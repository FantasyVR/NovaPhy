"""

        NovaPhy core Python extension module.

        This module exposes math types, collision detection, free-body simulation,
        and articulated-body dynamics routines implemented in C++.
    
"""
from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import typing
__all__: list[str] = ['AABB', 'ArticulatedSolver', 'Articulation', 'BroadPhasePair', 'CollisionShape', 'ContactPoint', 'Joint', 'JointType', 'Model', 'ModelBuilder', 'RigidBody', 'ShapeType', 'SimState', 'SolverSettings', 'SpatialTransform', 'SweepAndPrune', 'Transform', 'World', 'collide_shapes', 'deg2rad', 'forward_dynamics', 'forward_kinematics', 'inverse_dynamics', 'mass_matrix_crba', 'rad2deg', 'skew', 'spatial_cross_force', 'spatial_cross_motion', 'spatial_inertia_matrix', 'version']
class AABB:
    """
    
            Axis-aligned bounding box in world coordinates.
        
    """
    @staticmethod
    def from_sphere(center: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], radius: typing.SupportsFloat) -> AABB:
        """
                                 Creates an AABB that bounds a sphere.
        
                                 Args:
                                     center (Vector3): Sphere center in world coordinates.
                                     radius (float): Sphere radius in meters.
        
                                 Returns:
                                     AABB: Bounding box of the sphere.
        """
    @typing.overload
    def __init__(self) -> None:
        """
                    Creates an empty/zero AABB.
        """
    @typing.overload
    def __init__(self, min: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], max: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
                         Creates an AABB from min and max corners.
        
                         Args:
                             min (Vector3): Minimum corner in world coordinates.
                             max (Vector3): Maximum corner in world coordinates.
        """
    def center(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Returns box center.
        
                    Returns:
                        Vector3: Center point.
        """
    def half_extents(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Returns box half extents.
        
                    Returns:
                        Vector3: Half extents along x/y/z.
        """
    def is_valid(self) -> bool:
        """
                    Returns whether min corner is component-wise <= max corner.
        
                    Returns:
                        bool: True if AABB is valid.
        """
    def overlaps(self, other: AABB) -> bool:
        """
                    Tests overlap against another AABB.
        
                    Args:
                        other (AABB): Other axis-aligned box.
        
                    Returns:
                        bool: True if boxes overlap.
        """
    def surface_area(self) -> float:
        """
                    Returns AABB surface area.
        
                    Returns:
                        float: Surface area in square meters.
        """
    @property
    def max(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Vector3: Maximum corner.
        """
    @max.setter
    def max(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def min(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Vector3: Minimum corner.
        """
    @min.setter
    def min(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
class ArticulatedSolver:
    """
    
            Stateful articulated-body integrator built on Featherstone dynamics.
        
    """
    def __init__(self) -> None:
        """
                    Creates a solver with default integration settings.
        """
    def step(self, model: Articulation, q: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"], qd: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"], tau: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"], gravity: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], dt: typing.SupportsFloat) -> tuple[typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, 1]"], typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, 1]"]]:
        """
                    Advances the articulated system by one time step.
        
                    Args:
                        model (Articulation): Articulation model.
                        q (ndarray): Current generalized positions.
                        qd (ndarray): Current generalized velocities.
                        tau (ndarray): Applied joint efforts.
                        gravity (Vector3): Gravity vector in world coordinates (m/s^2).
                        dt (float): Integration time step in seconds.
        
                    Returns:
                        tuple[ndarray, ndarray]: Updated `(q, qd)` state.
        """
class Articulation:
    """
    
            Tree-structured multibody model used by Featherstone dynamics routines.
        
    """
    def __init__(self) -> None:
        """
                    Creates an empty articulation model.
        """
    def build_spatial_inertias(self) -> None:
        """
                    Builds per-link spatial inertia matrices from rigid-body properties.
        
                    Returns:
                        None
        """
    def num_links(self) -> int:
        """
                    Returns the number of links in the articulation.
        
                    Returns:
                        int: Number of links.
        """
    def q_start(self, link: typing.SupportsInt) -> int:
        """
                    Returns the starting index of a link's position block in `q`.
        
                    Args:
                        link (int): Link index in the articulation.
        
                    Returns:
                        int: Start index into the generalized-position vector.
        """
    def qd_start(self, link: typing.SupportsInt) -> int:
        """
                    Returns the starting index of a link's velocity block in `qd`.
        
                    Args:
                        link (int): Link index in the articulation.
        
                    Returns:
                        int: Start index into the generalized-velocity vector.
        """
    def total_q(self) -> int:
        """
                    Returns the total generalized-position dimension.
        
                    Returns:
                        int: Size of the full `q` vector.
        """
    def total_qd(self) -> int:
        """
                    Returns the total generalized-velocity dimension.
        
                    Returns:
                        int: Size of the full `qd` vector.
        """
    @property
    def bodies(self) -> list[RigidBody]:
        """
                    Per-link rigid-body inertial properties.
        """
    @bodies.setter
    def bodies(self, arg0: collections.abc.Sequence[RigidBody]) -> None:
        ...
    @property
    def joints(self) -> list[Joint]:
        """
                    Ordered joint list (one joint per link).
        """
    @joints.setter
    def joints(self, arg0: collections.abc.Sequence[Joint]) -> None:
        ...
class BroadPhasePair:
    """
    
            Candidate overlap pair produced by broadphase culling.
        
    """
    def __init__(self) -> None:
        """
                    Creates an empty broadphase pair.
        """
    @property
    def body_a(self) -> int:
        """
                    int: First body/shape index of the overlap pair.
        """
    @body_a.setter
    def body_a(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def body_b(self) -> int:
        """
                    int: Second body/shape index of the overlap pair.
        """
    @body_b.setter
    def body_b(self, arg0: typing.SupportsInt) -> None:
        ...
class CollisionShape:
    """
    
            Collision shape descriptor with material and local pose information.
        
    """
    @staticmethod
    def make_box(half_extents: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], body_idx: typing.SupportsInt, local: Transform = ..., friction: typing.SupportsFloat = 0.5, restitution: typing.SupportsFloat = 0.30000001192092896) -> CollisionShape:
        """
                                 Creates a box collision shape.
        
                                 Args:
                                     half_extents (Vector3): Box half extents (m).
                                     body_idx (int): Owning body index.
                                     local (Transform): Local shape pose in body frame.
                                     friction (float): Friction coefficient.
                                     restitution (float): Restitution coefficient.
        
                                 Returns:
                                     CollisionShape: Box shape descriptor.
        """
    @staticmethod
    def make_plane(normal: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], offset: typing.SupportsFloat, friction: typing.SupportsFloat = 0.5, restitution: typing.SupportsFloat = 0.0) -> CollisionShape:
        """
                                 Creates an infinite plane collision shape.
        
                                 Args:
                                     normal (Vector3): Plane normal vector.
                                     offset (float): Plane offset along normal (m).
                                     friction (float): Friction coefficient.
                                     restitution (float): Restitution coefficient.
        
                                 Returns:
                                     CollisionShape: Plane shape descriptor.
        """
    @staticmethod
    def make_sphere(radius: typing.SupportsFloat, body_idx: typing.SupportsInt, local: Transform = ..., friction: typing.SupportsFloat = 0.5, restitution: typing.SupportsFloat = 0.30000001192092896) -> CollisionShape:
        """
                                 Creates a sphere collision shape.
        
                                 Args:
                                     radius (float): Sphere radius in meters.
                                     body_idx (int): Owning body index.
                                     local (Transform): Local shape pose in body frame.
                                     friction (float): Friction coefficient.
                                     restitution (float): Restitution coefficient.
        
                                 Returns:
                                     CollisionShape: Sphere shape descriptor.
        """
    def __init__(self) -> None:
        """
                    Creates a default box collision shape.
        """
    def compute_aabb(self, body_transform: Transform) -> AABB:
        """
                    Computes world-space AABB for the shape.
        
                    Args:
                        body_transform (Transform): Owning body transform in world frame.
        
                    Returns:
                        AABB: World-space bounding box.
        """
    @property
    def body_index(self) -> int:
        """
                    int: Owning body index, or -1 for world-owned shapes.
        """
    @body_index.setter
    def body_index(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def box_half_extents(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                        Vector3: Box half extents in local coordinates (m).
        """
    @box_half_extents.setter
    def box_half_extents(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def friction(self) -> float:
        """
                    float: Friction coefficient used by contact solver.
        """
    @friction.setter
    def friction(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def local_transform(self) -> Transform:
        """
                    Transform: Local shape pose relative to owning body frame.
        """
    @local_transform.setter
    def local_transform(self, arg0: Transform) -> None:
        ...
    @property
    def plane_normal(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                        Vector3: Plane normal vector.
        """
    @plane_normal.setter
    def plane_normal(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def plane_offset(self) -> float:
        """
                        float: Plane offset in meters along normal direction.
        """
    @plane_offset.setter
    def plane_offset(self, arg1: typing.SupportsFloat) -> None:
        ...
    @property
    def restitution(self) -> float:
        """
                    float: Restitution coefficient in [0, 1].
        """
    @restitution.setter
    def restitution(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def sphere_radius(self) -> float:
        """
                        float: Sphere radius in meters.
        """
    @sphere_radius.setter
    def sphere_radius(self, arg1: typing.SupportsFloat) -> None:
        ...
    @property
    def type(self) -> ShapeType:
        """
                    ShapeType: Active primitive type.
        """
    @type.setter
    def type(self, arg0: ShapeType) -> None:
        ...
class ContactPoint:
    """
    
            One world-space contact sample between two bodies/shapes.
        
    """
    def __init__(self) -> None:
        """
                    Creates an empty contact point.
        """
    @property
    def body_a(self) -> int:
        """
                    int: First body index (`-1` may denote world).
        """
    @body_a.setter
    def body_a(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def body_b(self) -> int:
        """
                    int: Second body index (`-1` may denote world).
        """
    @body_b.setter
    def body_b(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def friction(self) -> float:
        """
                    float: Combined friction coefficient at the contact.
        """
    @friction.setter
    def friction(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def normal(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Vector3: Contact normal from body A toward body B.
        """
    @normal.setter
    def normal(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def penetration(self) -> float:
        """
                    float: Penetration depth in meters.
        """
    @penetration.setter
    def penetration(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def position(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Vector3: Contact position in world coordinates (m).
        """
    @position.setter
    def position(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def restitution(self) -> float:
        """
                    float: Combined restitution coefficient at the contact.
        """
    @restitution.setter
    def restitution(self, arg0: typing.SupportsFloat) -> None:
        ...
class Joint:
    """
    
            Joint metadata and kinematic parameters for one articulation link.
        
    """
    def __init__(self) -> None:
        """
                    Creates a default joint descriptor.
        """
    def num_q(self) -> int:
        """
                    Returns the number of generalized position coordinates for this joint.
        
                    Returns:
                        int: Number of position coordinates.
        """
    def num_qd(self) -> int:
        """
                    Returns the number of generalized velocity coordinates for this joint.
        
                    Returns:
                        int: Number of velocity coordinates.
        """
    @property
    def axis(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Joint axis vector used by revolute/slide joints in local coordinates.
        """
    @axis.setter
    def axis(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def parent(self) -> int:
        """
                    Parent link index (`-1` denotes root/world parent).
        """
    @parent.setter
    def parent(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def parent_to_joint(self) -> Transform:
        """
                    Transform from parent-link frame to joint frame.
        """
    @parent_to_joint.setter
    def parent_to_joint(self, arg0: Transform) -> None:
        ...
    @property
    def type(self) -> JointType:
        """
                    Joint type enumeration controlling motion subspace.
        """
    @type.setter
    def type(self, arg0: JointType) -> None:
        ...
class JointType:
    """
    
            Joint model type used by articulated-body dynamics.
        
    
    Members:
    
      Revolute : 
                One rotational degree of freedom around `axis` (radians).
            
    
      Fixed : 
                Zero degree-of-freedom rigid attachment.
            
    
      Free : 
                Six degree-of-freedom floating base joint.
            
    
      Slide : 
                One translational degree of freedom along `axis` (meters).
            
    
      Ball : 
                Three rotational degrees of freedom for orientation.
            
    """
    Ball: typing.ClassVar[JointType]  # value = <JointType.Ball: 4>
    Fixed: typing.ClassVar[JointType]  # value = <JointType.Fixed: 1>
    Free: typing.ClassVar[JointType]  # value = <JointType.Free: 2>
    Revolute: typing.ClassVar[JointType]  # value = <JointType.Revolute: 0>
    Slide: typing.ClassVar[JointType]  # value = <JointType.Slide: 3>
    __members__: typing.ClassVar[dict[str, JointType]]  # value = {'Revolute': <JointType.Revolute: 0>, 'Fixed': <JointType.Fixed: 1>, 'Free': <JointType.Free: 2>, 'Slide': <JointType.Slide: 3>, 'Ball': <JointType.Ball: 4>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Model:
    """
    
            Immutable collection of rigid bodies and collision shapes.
        
    """
    @property
    def bodies(self) -> list[RigidBody]:
        """
                    list[RigidBody]: Body inertial properties.
        """
    @property
    def num_bodies(self) -> int:
        """
                    int: Number of bodies in the model.
        """
    @property
    def num_shapes(self) -> int:
        """
                    int: Number of shapes in the model.
        """
    @property
    def shapes(self) -> list[CollisionShape]:
        """
                    list[CollisionShape]: Collision shape definitions.
        """
class ModelBuilder:
    """
    
            Builder for constructing an immutable simulation model.
        
    """
    def __init__(self) -> None:
        """
                    Creates an empty model builder.
        """
    def add_body(self, body: RigidBody, transform: Transform = ...) -> int:
        """
                         Adds a rigid body and returns its index.
        
                         Args:
                             body (RigidBody): Body mass and inertia properties.
                             transform (Transform): Initial world transform.
        
                         Returns:
                             int: New body index.
        """
    def add_ground_plane(self, y: typing.SupportsFloat = 0.0, friction: typing.SupportsFloat = 0.5, restitution: typing.SupportsFloat = 0.0) -> int:
        """
                         Adds an infinite ground plane shape.
        
                         Args:
                             y (float): Plane offset along +Y world axis in meters.
                             friction (float): Friction coefficient used by contact solver.
                             restitution (float): Restitution coefficient in [0, 1].
        
                         Returns:
                             int: New shape index.
        """
    def add_shape(self, shape: CollisionShape) -> int:
        """
                         Adds a collision shape and returns its index.
        
                         Args:
                             shape (CollisionShape): Shape attached to a body or world.
        
                         Returns:
                             int: New shape index.
        """
    def build(self) -> ...:
        """
                    Builds an immutable `Model` from accumulated bodies and shapes.
        
                    Returns:
                        Model: Immutable model object.
        """
    @property
    def num_bodies(self) -> int:
        """
                    int: Number of currently added bodies.
        """
    @property
    def num_shapes(self) -> int:
        """
                    int: Number of currently added shapes.
        """
class RigidBody:
    """
    
            Rigid-body inertial parameters in body-local coordinates.
        
    """
    @staticmethod
    def from_box(mass: typing.SupportsFloat, half_extents: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> RigidBody:
        """
                                 Constructs a rigid body with solid-box inertia.
        
                                 Args:
                                     mass (float): Body mass in kilograms.
                                     half_extents (Vector3): Box half extents in meters.
        
                                 Returns:
                                     RigidBody: Body with diagonal box inertia tensor.
        """
    @staticmethod
    def from_sphere(mass: typing.SupportsFloat, radius: typing.SupportsFloat) -> RigidBody:
        """
                                 Constructs a rigid body with solid-sphere inertia.
        
                                 Args:
                                     mass (float): Body mass in kilograms.
                                     radius (float): Sphere radius in meters.
        
                                 Returns:
                                     RigidBody: Body with isotropic inertia tensor.
        """
    @staticmethod
    def make_static() -> RigidBody:
        """
                    Creates an immovable rigid body.
        
                    Returns:
                        RigidBody: Static body with zero mass and inertia.
        """
    def __init__(self) -> None:
        """
                    Creates a rigid body with default unit mass and identity inertia.
        """
    def inv_inertia(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 3]"]:
        """
                    Returns inverse inertia tensor in body-local coordinates.
        
                    Returns:
                        Matrix3: Inverse inertia tensor, or zero matrix for static bodies.
        """
    def inv_mass(self) -> float:
        """
                    Returns inverse mass.
        
                    Returns:
                        float: Inverse mass (kg^-1), or 0 for static bodies.
        """
    def is_static(self) -> bool:
        """
                    Returns whether the body is static.
        
                    Returns:
                        bool: True when body mass is non-positive.
        """
    @property
    def com(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Vector3: Center of mass in body-local coordinates (m).
        """
    @com.setter
    def com(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def inertia(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 3]"]:
        """
                    Matrix3: Inertia tensor in body-local frame about center of mass (kg*m^2).
        """
    @inertia.setter
    def inertia(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"]) -> None:
        ...
    @property
    def mass(self) -> float:
        """
                    float: Body mass in kilograms.
        """
    @mass.setter
    def mass(self, arg0: typing.SupportsFloat) -> None:
        ...
class ShapeType:
    """
    
            Collision shape primitive type.
        
    
    Members:
    
      Box : 
                Oriented box primitive.
            
    
      Sphere : 
                Sphere primitive.
            
    
      Plane : 
                Infinite plane primitive.
            
    """
    Box: typing.ClassVar[ShapeType]  # value = <ShapeType.Box: 0>
    Plane: typing.ClassVar[ShapeType]  # value = <ShapeType.Plane: 2>
    Sphere: typing.ClassVar[ShapeType]  # value = <ShapeType.Sphere: 1>
    __members__: typing.ClassVar[dict[str, ShapeType]]  # value = {'Box': <ShapeType.Box: 0>, 'Sphere': <ShapeType.Sphere: 1>, 'Plane': <ShapeType.Plane: 2>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class SimState:
    """
    
            Mutable simulation state for all bodies in the world.
        
    """
    def __init__(self) -> None:
        """
                    Creates an empty simulation state.
        """
    def apply_force(self, body_index: typing.SupportsInt, force: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
                         Accumulates an external force at body center of mass.
        
                         Args:
                             body_index (int): Body index.
                             force (Vector3): Force in world coordinates (N).
        
                         Returns:
                             None
        """
    def apply_torque(self, body_index: typing.SupportsInt, torque: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
                         Accumulates an external torque on a body.
        
                         Args:
                             body_index (int): Body index.
                             torque (Vector3): Torque in world coordinates (N*m).
        
                         Returns:
                             None
        """
    def set_angular_velocity(self, body_index: typing.SupportsInt, velocity: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
                         Sets one body's angular velocity.
        
                         Args:
                             body_index (int): Body index.
                             velocity (Vector3): Angular velocity in world coordinates (rad/s).
        
                         Returns:
                             None
        """
    def set_linear_velocity(self, body_index: typing.SupportsInt, velocity: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
                         Sets one body's linear velocity.
        
                         Args:
                             body_index (int): Body index.
                             velocity (Vector3): Velocity in world coordinates (m/s).
        
                         Returns:
                             None
        """
    @property
    def angular_velocities(self) -> list[typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]]:
        """
                    list[Vector3]: Angular velocities in world coordinates (rad/s).
        """
    @property
    def linear_velocities(self) -> list[typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]]:
        """
                    list[Vector3]: Linear velocities in world coordinates (m/s).
        """
    @property
    def transforms(self) -> list[Transform]:
        """
                    list[Transform]: World transforms per body.
        """
class SolverSettings:
    """
    
            Configuration for the contact solver iteration and stabilization.
        
    """
    def __init__(self) -> None:
        """
                    Creates solver settings with default stable values.
        """
    @property
    def baumgarte(self) -> float:
        """
                    float: Position-error correction factor (dimensionless).
        """
    @baumgarte.setter
    def baumgarte(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def slop(self) -> float:
        """
                    float: Penetration allowance before correction (meters).
        """
    @slop.setter
    def slop(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def velocity_iterations(self) -> int:
        """
                    int: Number of PGS velocity iterations per time step.
        """
    @velocity_iterations.setter
    def velocity_iterations(self, arg0: typing.SupportsInt) -> None:
        ...
    @property
    def warm_starting(self) -> bool:
        """
                    bool: Reuse previous-frame impulses for faster convergence.
        """
    @warm_starting.setter
    def warm_starting(self, arg0: bool) -> None:
        ...
class SpatialTransform:
    """
    
            Spatial transform for Featherstone-style spatial vectors.
        
    """
    @staticmethod
    def from_transform(t: Transform) -> SpatialTransform:
        """
                    Converts rigid `Transform` to `SpatialTransform`.
        
                    Args:
                        t (Transform): Rigid transform.
        
                    Returns:
                        SpatialTransform: Equivalent spatial transform.
        """
    @staticmethod
    def identity() -> SpatialTransform:
        """
                    Creates identity spatial transform.
        
                    Returns:
                        SpatialTransform: Identity transform.
        """
    @typing.overload
    def __init__(self) -> None:
        """
                    Creates identity spatial transform.
        """
    @typing.overload
    def __init__(self, E: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"], r: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
                    Creates spatial transform X_{A<-B}.
        
                    Args:
                        E (Matrix3): Rotation from frame B to frame A.
                        r (Vector3): Origin of B expressed in A (m).
        """
    def __mul__(self, other: SpatialTransform) -> SpatialTransform:
        ...
    def apply_force(self, f: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[6, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[6, 1]"]:
        """
                    Applies dual transform to a spatial force vector.
        
                    Args:
                        f (ndarray): Spatial force vector [moment; force].
        
                    Returns:
                        ndarray: Transformed spatial force vector.
        """
    def apply_motion(self, v: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[6, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[6, 1]"]:
        """
                    Applies this transform to a spatial motion vector.
        
                    Args:
                        v (ndarray): Spatial motion vector [angular; linear].
        
                    Returns:
                        ndarray: Transformed spatial motion vector.
        """
    def inverse(self) -> SpatialTransform:
        """
                    Computes inverse spatial transform.
        
                    Returns:
                        SpatialTransform: Inverse transform.
        """
    def to_matrix(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[6, 6]"]:
        """
                    Converts transform to 6x6 matrix form.
        
                    Returns:
                        ndarray: Spatial transform matrix.
        """
    @property
    def E(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 3]"]:
        """
                    Matrix3: Rotation from frame B to frame A.
        """
    @E.setter
    def E(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"]) -> None:
        ...
    @property
    def r(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Vector3: Origin of frame B expressed in frame A (m).
        """
    @r.setter
    def r(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
class SweepAndPrune:
    """
    
            Sweep-and-Prune broadphase collision detector.
        
    """
    def __init__(self) -> None:
        """
                    Creates an empty broadphase accelerator.
        """
    def get_pairs(self) -> list[BroadPhasePair]:
        """
                    Returns candidate overlap pairs from the latest update.
        
                    Returns:
                        list[BroadPhasePair]: Potentially overlapping body/shape pairs.
        """
    def update(self, body_aabbs: collections.abc.Sequence[AABB], static_mask: collections.abc.Sequence[bool]) -> None:
        """
                         Updates overlap candidates from current world-space AABBs.
        
                         Args:
                             body_aabbs (list[AABB]): Per-body world-space AABBs.
                             static_mask (list[bool]): Static-body mask for pruning static-static pairs.
        
                         Returns:
                             None
        """
class Transform:
    """
    
            Rigid transform represented by translation and quaternion orientation.
        
    """
    @staticmethod
    def from_axis_angle(axis: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], angle: typing.SupportsFloat) -> Transform:
        """
                                 Creates rotation-only transform from axis-angle representation.
        
                                 Args:
                                     axis (Vector3): Rotation axis.
                                     angle (float): Rotation angle in radians.
        
                                 Returns:
                                     Transform: Rotation-only transform.
        """
    @staticmethod
    def from_rotation(q: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> Transform:
        """
                    Creates rotation-only transform.
        
                    Args:
                        q (Vector4): Quaternion as [x, y, z, w].
        
                    Returns:
                        Transform: Transform with zero translation.
        """
    @staticmethod
    def from_translation(t: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> Transform:
        """
                    Creates translation-only transform.
        
                    Args:
                        t (Vector3): Translation vector (m).
        
                    Returns:
                        Transform: Transform with identity rotation.
        """
    @staticmethod
    def identity() -> Transform:
        """
                    Creates identity transform.
        
                    Returns:
                        Transform: Identity transform.
        """
    @typing.overload
    def __init__(self) -> None:
        """
                    Creates an identity transform.
        """
    @typing.overload
    def __init__(self, position: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], rotation: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> None:
        """
                        Creates a transform from position and quaternion.
        
                        Args:
                            position (Vector3): Translation in world coordinates (m).
                            rotation (Vector4): Quaternion as [x, y, z, w].
        """
    def __mul__(self, other: Transform) -> Transform:
        ...
    def __repr__(self) -> str:
        ...
    def inverse(self) -> Transform:
        """
                    Computes inverse rigid transform.
        
                    Returns:
                        Transform: Inverse transform.
        """
    def rotation_matrix(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 3]"]:
        """
                    Returns 3x3 rotation matrix corresponding to quaternion orientation.
        
                    Returns:
                        Matrix3: Rotation matrix.
        """
    def transform_point(self, point: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Transforms a point from local frame to world frame.
        
                    Args:
                        point (Vector3): Local-space point.
        
                    Returns:
                        Vector3: World-space point (m).
        """
    def transform_vector(self, vector: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Rotates a direction vector without translation.
        
                    Args:
                        vector (Vector3): Local-space direction vector.
        
                    Returns:
                        Vector3: Rotated world-space direction.
        """
    @property
    def position(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Vector3: Translation component in meters.
        """
    @position.setter
    def position(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        ...
    @property
    def rotation(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[4, 1]"]:
        """
                        Quaternion orientation as [x, y, z, w].
        """
    @rotation.setter
    def rotation(self, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[4, 1]"]) -> None:
        ...
class World:
    """
    
            Top-level container that advances free-rigid-body simulation.
        
    """
    def __init__(self, model: Model, solver_settings: SolverSettings = ...) -> None:
        """
                         Creates a simulation world from an immutable model.
        
                         Args:
                             model (Model): Immutable model definition.
                             solver_settings (SolverSettings): Contact solver parameters.
        """
    def apply_force(self, body_index: typing.SupportsInt, force: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
                         Applies an external force for the next simulation step.
        
                         Args:
                             body_index (int): Body index.
                             force (Vector3): Force in world coordinates (N).
        
                         Returns:
                             None
        """
    def apply_torque(self, body_index: typing.SupportsInt, torque: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
                         Applies an external torque for the next simulation step.
        
                         Args:
                             body_index (int): Body index.
                             torque (Vector3): Torque in world coordinates (N*m).
        
                         Returns:
                             None
        """
    def set_gravity(self, gravity: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> None:
        """
                         Sets the world gravity vector.
        
                         Args:
                             gravity (Vector3): Gravity in world coordinates (m/s^2).
        
                         Returns:
                             None
        """
    def step(self, dt: typing.SupportsFloat) -> None:
        """
                         Advances simulation by one fixed time step.
        
                         Args:
                             dt (float): Time step in seconds.
        
                         Returns:
                             None
        """
    @property
    def contacts(self) -> list[ContactPoint]:
        """
                         list[ContactPoint]: Contact points generated during last step.
        """
    @property
    def gravity(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 1]"]:
        """
                    Vector3: Current gravity vector in world coordinates (m/s^2).
        """
    @property
    def model(self) -> Model:
        """
                         Model: Immutable world model (reference).
        """
    @property
    def state(self) -> SimState:
        """
                         SimState: Mutable world state (reference).
        """
def collide_shapes(shape_a: CollisionShape, transform_a: Transform, shape_b: CollisionShape, transform_b: Transform) -> tuple[bool, list[ContactPoint]]:
    """
                  Tests collision between two shapes and returns contact data.
    
                  Args:
                      shape_a (CollisionShape): First shape.
                      transform_a (Transform): World transform for shape A parent body.
                      shape_b (CollisionShape): Second shape.
                      transform_b (Transform): World transform for shape B parent body.
    
                  Returns:
                      tuple[bool, list[ContactPoint]]: `(hit, contacts)` where `hit`
                      indicates overlap and `contacts` stores world-space manifold points.
    """
def deg2rad(deg: typing.SupportsFloat) -> float:
    """
            Converts degrees to radians.
    
            Args:
                deg (float): Angle in degrees.
    
            Returns:
                float: Angle in radians.
    """
def forward_dynamics(model: Articulation, q: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"], qd: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"], tau: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"], gravity: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, 1]"]:
    """
            Computes generalized accelerations from applied joint efforts.
    
            Args:
                model (Articulation): Articulation model.
                q (ndarray): Generalized positions.
                qd (ndarray): Generalized velocities.
                tau (ndarray): Applied joint efforts.
                gravity (Vector3): Gravity vector in world coordinates (m/s^2).
    
            Returns:
                ndarray: Generalized accelerations.
    """
def forward_kinematics(model: Articulation, q: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"]) -> list[Transform]:
    """
            Computes world-frame transforms for all links from generalized positions.
    
            Args:
                model (Articulation): Articulation model with joints and bodies.
                q (ndarray): Generalized positions.
    
            Returns:
                list[Transform]: World transform per link.
    """
def inverse_dynamics(model: Articulation, q: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"], qd: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"], qdd: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"], gravity: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, 1]"]:
    """
            Runs Recursive Newton-Euler inverse dynamics.
    
            Args:
                model (Articulation): Articulation model.
                q (ndarray): Generalized positions.
                qd (ndarray): Generalized velocities.
                qdd (ndarray): Generalized accelerations.
                gravity (Vector3): Gravity vector in world coordinates (m/s^2).
    
            Returns:
                ndarray: Required joint efforts (torques/forces).
    """
def mass_matrix_crba(model: Articulation, q: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[m, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[m, n]"]:
    """
            Computes the joint-space mass matrix using CRBA.
    
            Args:
                model (Articulation): Articulation model.
                q (ndarray): Generalized positions.
    
            Returns:
                ndarray: Symmetric positive-definite mass matrix H(q).
    """
def rad2deg(rad: typing.SupportsFloat) -> float:
    """
            Converts radians to degrees.
    
            Args:
                rad (float): Angle in radians.
    
            Returns:
                float: Angle in degrees.
    """
def skew(v: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[3, 3]"]:
    """
            Builds skew-symmetric matrix [v]_x for cross-product operations.
    
            Args:
                v (Vector3): Input vector.
    
            Returns:
                Matrix3: Skew-symmetric matrix.
    """
def spatial_cross_force(v: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[6, 1]"], f: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[6, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[6, 1]"]:
    """
                  Computes spatial cross product for force vectors.
    
                  Args:
                      v (ndarray): Spatial motion vector [angular; linear].
                      f (ndarray): Spatial force vector [moment; force].
    
                  Returns:
                      ndarray: Force cross product.
    """
def spatial_cross_motion(v: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[6, 1]"], u: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[6, 1]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[6, 1]"]:
    """
                  Computes spatial cross product for motion vectors.
    
                  Args:
                      v (ndarray): Left spatial motion vector [angular; linear].
                      u (ndarray): Right spatial motion vector [angular; linear].
    
                  Returns:
                      ndarray: Motion cross product.
    """
def spatial_inertia_matrix(mass: typing.SupportsFloat, com: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 1]"], I_rot: typing.Annotated[numpy.typing.ArrayLike, numpy.float32, "[3, 3]"]) -> typing.Annotated[numpy.typing.NDArray[numpy.float32], "[6, 6]"]:
    """
                  Builds 6x6 spatial inertia matrix from rigid-body parameters.
    
                  Args:
                      mass (float): Body mass in kilograms.
                      com (Vector3): Center of mass in body coordinates (m).
                      I_rot (Matrix3): Rotational inertia about CoM (kg*m^2).
    
                  Returns:
                      ndarray: Spatial inertia matrix.
    """
def version() -> str:
    """
            Returns the NovaPhy library version string.
    
            Returns:
                str: Semantic version string.
    """
