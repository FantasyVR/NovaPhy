"""NovaPhy Polyscope visualization helpers.

Provides mesh generation for primitive shapes and per-frame transform updates.
"""

import numpy as np

try:
    import polyscope as ps
except ImportError:
    ps = None


def _require_polyscope():
    """Ensures Polyscope is importable before visualization setup.

    Raises:
        ImportError: If `polyscope` is not installed in the active environment.
    """
    if ps is None:
        raise ImportError("polyscope is required for visualization. "
                          "Install with: pip install polyscope")


def make_box_mesh(half_extents):
    """Generate a box mesh (vertices + face indices).

    Args:
        half_extents (array-like): Half-extents `[hx, hy, hz]` in meters.

    Returns:
        tuple[np.ndarray, np.ndarray]: Vertices `(8, 3)` and triangulated faces
        `(12, 3)`.
    """
    hx, hy, hz = half_extents
    verts = np.array([
        [-hx, -hy, -hz], [+hx, -hy, -hz], [+hx, +hy, -hz], [-hx, +hy, -hz],
        [-hx, -hy, +hz], [+hx, -hy, +hz], [+hx, +hy, +hz], [-hx, +hy, +hz],
    ], dtype=np.float32)

    faces = np.array([
        [0, 1, 2], [0, 2, 3],  # -Z
        [4, 6, 5], [4, 7, 6],  # +Z
        [0, 4, 5], [0, 5, 1],  # -Y
        [2, 6, 7], [2, 7, 3],  # +Y
        [0, 7, 4], [0, 3, 7],  # -X
        [1, 5, 6], [1, 6, 2],  # +X
    ], dtype=np.int32)

    return verts, faces


def make_sphere_mesh(radius, n_lat=16, n_lon=32):
    """Generate a UV sphere mesh.

    Args:
        radius (float): Sphere radius in meters.
        n_lat (int): Number of latitude divisions.
        n_lon (int): Number of longitude divisions.

    Returns:
        tuple[np.ndarray, np.ndarray]: Vertices `(N, 3)` and triangular faces
        `(M, 3)`.
    """
    verts = []
    faces = []

    # Top pole
    verts.append([0, radius, 0])

    for i in range(1, n_lat):
        theta = np.pi * i / n_lat
        for j in range(n_lon):
            phi = 2 * np.pi * j / n_lon
            x = radius * np.sin(theta) * np.cos(phi)
            y = radius * np.cos(theta)
            z = radius * np.sin(theta) * np.sin(phi)
            verts.append([x, y, z])

    # Bottom pole
    verts.append([0, -radius, 0])

    verts = np.array(verts, dtype=np.float32)

    # Top cap
    for j in range(n_lon):
        j_next = (j + 1) % n_lon
        faces.append([0, 1 + j, 1 + j_next])

    # Middle rows
    for i in range(n_lat - 2):
        for j in range(n_lon):
            j_next = (j + 1) % n_lon
            curr = 1 + i * n_lon + j
            next_row = 1 + (i + 1) * n_lon + j
            curr_next = 1 + i * n_lon + j_next
            next_row_next = 1 + (i + 1) * n_lon + j_next
            faces.append([curr, next_row, curr_next])
            faces.append([curr_next, next_row, next_row_next])

    # Bottom cap
    bottom = len(verts) - 1
    base = 1 + (n_lat - 2) * n_lon
    for j in range(n_lon):
        j_next = (j + 1) % n_lon
        faces.append([bottom, base + j_next, base + j])

    faces = np.array(faces, dtype=np.int32)
    return verts, faces


def make_ground_plane_mesh(size=20.0, y=0.0):
    """Generate a flat ground plane mesh.

    Args:
        size (float): Half-size of the rendered square plane in meters.
        y (float): Plane height in world Y (meters).

    Returns:
        tuple[np.ndarray, np.ndarray]: Vertices `(4, 3)` and triangular faces
        `(2, 3)`.
    """
    verts = np.array([
        [-size, y, -size],
        [+size, y, -size],
        [+size, y, +size],
        [-size, y, +size],
    ], dtype=np.float32)
    faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
    return verts, faces


def quat_to_rotation_matrix(quat_xyzw):
    """Converts an `xyzw` quaternion to a 3x3 rotation matrix.

    Args:
        quat_xyzw (array-like): Quaternion `[x, y, z, w]`.

    Returns:
        np.ndarray: Rotation matrix with shape `(3, 3)`.
    """
    x, y, z, w = quat_xyzw
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float32)


def transform_vertices(verts, transform):
    """Apply a NovaPhy Transform to a set of vertices.

    Args:
        verts (np.ndarray): Local-space vertices with shape `(N, 3)`.
        transform (novaphy.Transform): Body transform in world coordinates.

    Returns:
        np.ndarray: World-space vertices with shape `(N, 3)`.
    """
    pos = np.array(transform.position, dtype=np.float32)
    quat = np.array(transform.rotation, dtype=np.float32)  # [x, y, z, w]
    R = quat_to_rotation_matrix(quat)
    return (verts @ R.T) + pos


class SceneVisualizer:
    """Manages Polyscope meshes for a NovaPhy world state."""

    def __init__(self, world, ground_size=20.0):
        """Initialize visualizer.

        Args:
            world (novaphy.World): Physics world to visualize.
            ground_size (float): Half-size of generated ground mesh in meters.

        Raises:
            ImportError: If Polyscope is unavailable.
        """
        _require_polyscope()
        self.world = world
        self.meshes = []  # list of (name, local_verts, faces, body_index)
        self._setup_scene(ground_size)

    def _setup_scene(self, ground_size):
        """Creates Polyscope meshes for all model shapes.

        Args:
            ground_size (float): Half-size of generated plane mesh in meters.
        """
        model = self.world.model

        for i, shape in enumerate(model.shapes):
            name = f"shape_{i}"

            if shape.type.name == "Box":
                he = np.array(shape.box_half_extents, dtype=np.float32)
                verts, faces = make_box_mesh(he)
                self.meshes.append((name, verts, faces, shape.body_index))

            elif shape.type.name == "Sphere":
                verts, faces = make_sphere_mesh(shape.sphere_radius)
                self.meshes.append((name, verts, faces, shape.body_index))

            elif shape.type.name == "Plane":
                verts, faces = make_ground_plane_mesh(ground_size, shape.plane_offset)
                ps_mesh = ps.register_surface_mesh(name, verts, faces)
                ps_mesh.set_color((0.6, 0.6, 0.6))
                ps_mesh.set_edge_width(1.0)
                # Ground plane doesn't need transform updates
                continue

        # Register dynamic meshes at initial positions
        self.update()

    def update(self):
        """Updates mesh vertex positions from current world transforms."""
        state = self.world.state

        for name, local_verts, faces, body_idx in self.meshes:
            if body_idx < 0:
                continue
            transform = state.transforms[body_idx]
            world_verts = transform_vertices(local_verts, transform)

            if ps.has_surface_mesh(name):
                ps.get_surface_mesh(name).update_vertex_positions(world_verts)
            else:
                ps_mesh = ps.register_surface_mesh(name, world_verts, faces)
                ps_mesh.set_smooth_shade(True)
