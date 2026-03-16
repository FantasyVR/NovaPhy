"""NovaPhy Polyscope visualization helpers.

Provides mesh generation for primitive shapes and per-frame transform updates.
"""

import numpy as np
from dataclasses import dataclass
from novaphy import batch_transform_vertices


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


def _batch_quat_to_rotation_matrices(quats, out=None):
    """Vectorized conversion of N quaternions (xyzw) to (N, 3, 3) rotation matrices.

    Stores R^T directly so callers can use ``np.matmul(local_verts, R)`` without
    a ``.transpose(0,2,1)`` view, keeping the second operand C-contiguous.

    Args:
        quats (np.ndarray): Shape ``(N, 4)`` quaternions in ``[x, y, z, w]`` order.
        out (np.ndarray | None): Optional pre-allocated ``(N, 3, 3)`` output array.
            If provided, writes directly into it (zero allocation).

    Returns:
        np.ndarray: Transposed rotation matrices ``R^T`` with shape ``(N, 3, 3)``
        such that ``v_world = v_local @ R^T`` (i.e. standard row-vector convention).
    """
    R = np.empty((quats.shape[0], 3, 3), dtype=np.float32) if out is None else out
    x, y, z, w = quats[:, 0], quats[:, 1], quats[:, 2], quats[:, 3]
    # Store columns of R as rows (= rows of R^T) so matmul(v, R) is correct
    # and R is C-contiguous — no .transpose() view needed downstream
    R[:, 0, 0] = 1 - 2*(y*y + z*z);  R[:, 1, 0] = 2*(x*y - z*w);  R[:, 2, 0] = 2*(x*z + y*w)
    R[:, 0, 1] = 2*(x*y + z*w);      R[:, 1, 1] = 1 - 2*(x*x + z*z); R[:, 2, 1] = 2*(y*z - x*w)
    R[:, 0, 2] = 2*(x*z - y*w);      R[:, 1, 2] = 2*(y*z + x*w);  R[:, 2, 2] = 1 - 2*(x*x + y*y)
    return R


def _apply_transforms(local_verts, body_indices_arr, positions_all, quats_all, out_buf,
                      rot_buf=None, pos_gather=None, quat_gather=None):
    """Core transform: given full-world arrays, write world verts into out_buf.

    Args:
        local_verts (np.ndarray): Shape ``(N, V, 3)`` float32, C-contiguous.
        body_indices_arr (np.ndarray): Shape ``(N,)`` int32 body indices.
        positions_all (np.ndarray): ``(N_bodies, 3)`` float32 world positions.
        quats_all (np.ndarray): ``(N_bodies, 4)`` float32 quaternions [x,y,z,w].
        out_buf (np.ndarray): Pre-allocated ``(N*V, 3)`` float32 output buffer.
        rot_buf (np.ndarray | None): Optional pre-allocated ``(N, 3, 3)`` rotation buffer.
        pos_gather (np.ndarray | None): Optional pre-allocated ``(N, 3)`` gather buffer.
        quat_gather (np.ndarray | None): Optional pre-allocated ``(N, 4)`` gather buffer.

    Returns:
        np.ndarray: ``out_buf`` filled with world-space vertices ``(N*V, 3)``.
    """
    N, V = local_verts.shape[0], local_verts.shape[1]
    if pos_gather is not None:
        np.take(positions_all, body_indices_arr, axis=0, out=pos_gather)
        np.take(quats_all,     body_indices_arr, axis=0, out=quat_gather)
    else:
        pos_gather  = positions_all[body_indices_arr]
        quat_gather = quats_all[body_indices_arr]
    R = _batch_quat_to_rotation_matrices(quat_gather, out=rot_buf)  # stores R^T
    np.matmul(local_verts, R, out=out_buf.reshape(N, V, 3))  # R is already R^T, C-contiguous
    out_buf.reshape(N, V, 3)[:] += pos_gather[:, np.newaxis, :]
    return out_buf


def _batch_transform(local_verts, body_indices_arr, state, out_buf):
    """Vectorised transform: (N, V, 3) local → (N*V, 3) world, zero malloc.

    Args:
        local_verts (np.ndarray): Shape ``(N, V, 3)`` float32, C-contiguous.
        body_indices_arr (np.ndarray): Shape ``(N,)`` int32 body indices.
        state: NovaPhy SimState.
        out_buf (np.ndarray): Pre-allocated ``(N*V, 3)`` float32 output buffer.

    Returns:
        np.ndarray: ``out_buf`` filled with world-space vertices ``(N*V, 3)``.
    """
    if hasattr(state, "get_transforms_numpy"):
        positions, quats = state.get_transforms_numpy()
        return _apply_transforms(local_verts, body_indices_arr, positions, quats, out_buf)
    else:
        _tf = state.transforms
        positions = np.array([_tf[i].position for i in body_indices_arr], dtype=np.float32)
        quats     = np.array([_tf[i].rotation for i in body_indices_arr], dtype=np.float32)
        R = _batch_quat_to_rotation_matrices(quats)  # stores R^T
        N, V = local_verts.shape[0], local_verts.shape[1]
        np.matmul(local_verts, R, out=out_buf.reshape(N, V, 3))
        out_buf.reshape(N, V, 3)[:] += positions[:, np.newaxis, :]
        return out_buf


@dataclass
class _ShapeBatch:
    """Internal per-shape-type batch: pre-allocated buffers + polyscope mesh."""
    body_indices: np.ndarray  # (N,) int32
    local_verts:  np.ndarray  # (N, V, 3) float32, C-contiguous
    world_buf:    np.ndarray  # (N*V, 3) float32 — pre-allocated, zero malloc/frame
    ps_mesh:      object      # polyscope SurfaceMesh
    rot_buf:      np.ndarray  # (N, 3, 3) float32 — rotation matrix scratch, zero malloc
    pos_gather:   np.ndarray  # (N, 3) float32    — gathered positions, zero malloc
    quat_gather:  np.ndarray  # (N, 4) float32    — gathered quats, zero malloc


class GeneralBatchedVisualizer:
    """High-performance visualizer supporting arbitrary shape types.

    Groups bodies by shape type into merged Polyscope meshes.  Per-frame cost
    is O(num_types) Polyscope calls with zero per-frame memory allocation.
    Supports Box, Sphere out of the box; easily extensible.

    Args:
        world (novaphy.World): Physics world to visualize.
        ground_size (float): Half-size of the rendered ground plane in metres.
        sphere_lat (int): Latitude divisions per sphere hemisphere.
        sphere_lon (int): Longitude divisions around sphere circumference.
        colors (dict | None): Optional ``{type_name: (r, g, b)}`` overrides.
    """

    _DEFAULT_COLORS = {
        "Box":     (0.75, 0.55, 0.30),
        "Sphere":  (0.20, 0.50, 0.90),
    }

    def __init__(self, world, ground_size=50.0,
                 sphere_lat=8, sphere_lon=16,
                 colors=None):
        _require_polyscope()
        self.world = world
        self._sphere_lat   = sphere_lat
        self._sphere_lon   = sphere_lon
        self._colors = dict(self._DEFAULT_COLORS)
        if colors:
            self._colors.update(colors)
        self._batches = {}  # stype -> _ShapeBatch
        self._setup(ground_size)

    def _mesh_for_shape(self, shape):
        """Return ``(verts, faces)`` for *shape*, or ``None`` to skip."""
        stype = shape.type.name
        if stype == "Box":
            return make_box_mesh(np.array(shape.box_half_extents, dtype=np.float32))
        if stype == "Sphere":
            return make_sphere_mesh(shape.sphere_radius,
                                    n_lat=self._sphere_lat, n_lon=self._sphere_lon)
        return None

    def _setup(self, ground_size):
        model = self.world.model
        # groups: stype -> {'verts': [...], 'body_indices': [...], 'base_faces': ndarray}
        groups = {}

        for shape in model.shapes:
            stype = shape.type.name
            if stype == "Plane":
                gv, gf = make_ground_plane_mesh(ground_size, shape.plane_offset)
                gm = ps.register_surface_mesh("ground_plane", gv, gf)
                gm.set_color((0.55, 0.55, 0.55))
                gm.set_edge_width(0.5)
                continue

            result = self._mesh_for_shape(shape)
            if result is None:
                continue
            v, f = result
            if stype not in groups:
                groups[stype] = {"verts": [], "body_indices": [], "base_faces": f}
            groups[stype]["verts"].append(v)
            groups[stype]["body_indices"].append(shape.body_index)

        for stype, data in groups.items():
            verts_list   = data["verts"]
            body_indices = np.array(data["body_indices"], dtype=np.int32)
            base_faces   = data["base_faces"]   # (F, 3)
            N = len(verts_list)
            V = verts_list[0].shape[0]

            offsets = np.arange(N, dtype=np.int32)[:, np.newaxis, np.newaxis] * V
            merged_faces = (base_faces[np.newaxis] + offsets).reshape(-1, 3)

            local_verts = np.ascontiguousarray(
                np.stack(verts_list, axis=0).astype(np.float32))  # (N, V, 3)
            world_buf   = np.empty((N * V, 3), dtype=np.float32)
            rot_buf     = np.empty((N, 3, 3),  dtype=np.float32)
            pos_gather  = np.empty((N, 3),     dtype=np.float32)
            quat_gather = np.empty((N, 4),     dtype=np.float32)

            ps_mesh = ps.register_surface_mesh(
                f"batched_{stype.lower()}s",
                np.zeros((N * V, 3), dtype=np.float32),
                merged_faces)
            ps_mesh.set_color(self._colors.get(stype, (0.6, 0.6, 0.6)))
            if stype == "Sphere":
                ps_mesh.set_smooth_shade(True)

            self._batches[stype] = _ShapeBatch(
                body_indices=body_indices,
                local_verts=local_verts,
                world_buf=world_buf,
                ps_mesh=ps_mesh,
                rot_buf=rot_buf,
                pos_gather=pos_gather,
                quat_gather=quat_gather,
            )

        self.update()

    def update(self):
        """Vectorised update: O(num_types) Polyscope calls, zero malloc."""
        state = self.world.state
        for batch in self._batches.values():
            _batch_transform(batch.local_verts, batch.body_indices, state, batch.world_buf)
            batch.ps_mesh.update_vertex_positions(batch.world_buf)

    def update_from_arrays(self, positions, quats):
        """Update mesh positions from pre-fetched transform arrays (async path).

        Uses the C++ fused kernel when available (zero intermediate allocs, GIL released),
        falling back to the NumPy path otherwise.

        Args:
            positions (np.ndarray): (N_bodies, 3) float32 world positions.
            quats (np.ndarray): (N_bodies, 4) float32 quaternions [x,y,z,w].
        """
        for batch in self._batches.values():
            batch_transform_vertices(positions, quats,
                     batch.body_indices, batch.local_verts, batch.world_buf)
            batch.ps_mesh.update_vertex_positions(batch.world_buf)


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
