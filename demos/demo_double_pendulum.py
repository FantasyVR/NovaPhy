"""Demo: Double pendulum exhibiting chaotic motion.

Demonstrates Featherstone articulated body dynamics with two revolute joints.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import novaphy

try:
    import polyscope as ps
    HAS_POLYSCOPE = True
except ImportError:
    HAS_POLYSCOPE = False

from novaphy.viz import make_box_mesh, quat_to_rotation_matrix


def build_double_pendulum(l1=1.0, l2=1.0, m1=1.0, m2=1.0):
    """Builds a two-link revolute articulated pendulum model.

    Args:
        l1 (float): Length of link 1 in meters.
        l2 (float): Length of link 2 in meters.
        m1 (float): Mass of link 1 in kilograms.
        m2 (float): Mass of link 2 in kilograms.

    Returns:
        novaphy.Articulation: Configured articulated model.
    """
    art = novaphy.Articulation()

    # Joint 0: attached to world origin
    j0 = novaphy.Joint()
    j0.type = novaphy.JointType.Revolute
    j0.axis = np.array([0, 0, 1], dtype=np.float32)
    j0.parent = -1
    j0.parent_to_joint = novaphy.Transform.identity()

    # Joint 1: attached to end of link 0
    j1 = novaphy.Joint()
    j1.type = novaphy.JointType.Revolute
    j1.axis = np.array([0, 0, 1], dtype=np.float32)
    j1.parent = 0
    j1.parent_to_joint = novaphy.Transform.from_translation(
        np.array([0, -l1, 0], dtype=np.float32))

    art.joints = [j0, j1]

    # Link bodies
    b0 = novaphy.RigidBody()
    b0.mass = m1
    b0.com = np.array([0, -l1 / 2, 0], dtype=np.float32)
    b0.inertia = np.eye(3, dtype=np.float32) * m1 * l1 * l1 / 3.0

    b1 = novaphy.RigidBody()
    b1.mass = m2
    b1.com = np.array([0, -l2 / 2, 0], dtype=np.float32)
    b1.inertia = np.eye(3, dtype=np.float32) * m2 * l2 * l2 / 3.0

    art.bodies = [b0, b1]
    art.build_spatial_inertias()
    return art


def get_link_endpoints(art, q, link_lengths):
    """Computes world-space line segments for articulated links.

    Args:
        art (novaphy.Articulation): Articulation model.
        q (np.ndarray): Generalized position vector.
        link_lengths (list[float]): Link lengths in meters.

    Returns:
        list[tuple[np.ndarray, np.ndarray]]: `(start, end)` endpoints per link.
    """
    transforms = novaphy.forward_kinematics(art, q)
    endpoints = []
    for i, t in enumerate(transforms):
        pos = np.array(t.position, dtype=np.float32)
        rot = quat_to_rotation_matrix(np.array(t.rotation, dtype=np.float32))
        end = pos + rot @ np.array([0, -link_lengths[i], 0], dtype=np.float32)
        endpoints.append((pos, end))
    return endpoints


def run_headless(steps=600):
    """Runs the demo without visualization and prints sampled state.

    Args:
        steps (int): Number of simulation steps.

    Returns:
        None
    """
    art = build_double_pendulum()
    q = np.array([np.pi / 2, np.pi / 4], dtype=np.float32)
    qd = np.zeros(2, dtype=np.float32)
    tau = np.zeros(2, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)

    solver = novaphy.ArticulatedSolver()
    print("=== Double Pendulum ===")
    for i in range(steps):
        q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 120.0)
        if i % 100 == 0:
            eps = get_link_endpoints(art, q, [1.0, 1.0])
            tip = eps[-1][1]
            print(f"  step {i}: q=({q[0]:.3f}, {q[1]:.3f}), "
                  f"tip=({tip[0]:.3f}, {tip[1]:.3f})")


def run_visual():
    """Runs the interactive Polyscope visualization loop.

    Returns:
        None
    """
    if not HAS_POLYSCOPE:
        run_headless()
        return

    ps.init()
    ps.set_program_name("NovaPhy - Double Pendulum")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")

    art = build_double_pendulum()
    l1, l2 = 1.0, 1.0
    q = np.array([np.pi / 2, np.pi / 4], dtype=np.float32)
    qd = np.zeros(2, dtype=np.float32)
    tau = np.zeros(2, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)
    solver = novaphy.ArticulatedSolver()

    # Create link meshes (thin boxes)
    link_half = [
        np.array([0.05, l1 / 2, 0.05], dtype=np.float32),
        np.array([0.05, l2 / 2, 0.05], dtype=np.float32),
    ]

    link_verts = []
    link_faces = []
    for i in range(2):
        v, f = make_box_mesh(link_half[i])
        # Offset so top of box is at origin (joint) and bottom extends down
        v[:, 1] -= link_half[i][1]
        link_verts.append(v)
        link_faces.append(f)

    def callback():
        nonlocal q, qd
        for _ in range(2):  # substep
            q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 240.0)

        transforms = novaphy.forward_kinematics(art, q)
        for i in range(2):
            pos = np.array(transforms[i].position, dtype=np.float32)
            rot = quat_to_rotation_matrix(np.array(transforms[i].rotation))
            world_v = (link_verts[i] @ rot.T) + pos
            name = f"link_{i}"
            if ps.has_surface_mesh(name):
                ps.get_surface_mesh(name).update_vertex_positions(world_v)
            else:
                ps.register_surface_mesh(name, world_v, link_faces[i])

    # Register pivot point
    ps.register_point_cloud("pivot", np.array([[0, 0, 0]], dtype=np.float32))

    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    run_visual()
