"""Demo: Chain of 5-8 links hanging from a fixed point in the world.

Demonstrates multi-link articulated body swinging under gravity.
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


def build_chain(num_links=6, link_length=0.6, link_mass=0.5):
    """Builds a multi-link articulated chain with alternating hinge axes.

    Args:
        num_links (int): Number of links.
        link_length (float): Link length in meters.
        link_mass (float): Mass per link in kilograms.

    Returns:
        novaphy.Articulation: Configured articulated model.
    """
    art = novaphy.Articulation()
    joints = []
    bodies = []

    for i in range(num_links):
        j = novaphy.Joint()
        j.type = novaphy.JointType.Revolute
        # Alternate rotation axes for 3D motion
        if i % 2 == 0:
            j.axis = np.array([0, 0, 1], dtype=np.float32)  # Z axis
        else:
            j.axis = np.array([1, 0, 0], dtype=np.float32)  # X axis

        if i == 0:
            j.parent = -1
            j.parent_to_joint = novaphy.Transform.identity()
        else:
            j.parent = i - 1
            j.parent_to_joint = novaphy.Transform.from_translation(
                np.array([0, -link_length, 0], dtype=np.float32))

        joints.append(j)

        body = novaphy.RigidBody()
        body.mass = link_mass
        body.com = np.array([0, -link_length / 2, 0], dtype=np.float32)
        c = link_mass / 12.0
        body.inertia = np.diag([c * link_length * link_length,
                                c * 0.01,
                                c * link_length * link_length]).astype(np.float32)
        bodies.append(body)

    art.joints = joints
    art.bodies = bodies
    art.build_spatial_inertias()
    return art


def run_headless(steps=600):
    """Runs the chain demo without visualization and logs tip position.

    Args:
        steps (int): Number of simulation steps.

    Returns:
        None
    """
    num_links = 6
    link_len = 0.6
    art = build_chain(num_links, link_len)
    nq = art.total_q()
    q = np.zeros(nq, dtype=np.float32)
    # Start with some initial displacement
    q[0] = np.pi / 3  # first link at 60 degrees
    qd = np.zeros(nq, dtype=np.float32)
    tau = np.zeros(nq, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)

    solver = novaphy.ArticulatedSolver()
    print("=== Joint Chain ===")
    for i in range(steps):
        q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 120.0)
        if i % 100 == 0:
            transforms = novaphy.forward_kinematics(art, q)
            tip = transforms[-1]
            pos = tip.position
            print(f"  step {i}: tip pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")


def run_visual():
    """Runs the interactive Polyscope articulated-chain visualization.

    Returns:
        None
    """
    if not HAS_POLYSCOPE:
        run_headless()
        return

    ps.init()
    ps.set_program_name("NovaPhy - Joint Chain")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")

    num_links = 6
    link_len = 0.6
    art = build_chain(num_links, link_len)
    nq = art.total_q()
    q = np.zeros(nq, dtype=np.float32)
    q[0] = np.pi / 3
    qd = np.zeros(nq, dtype=np.float32)
    tau = np.zeros(nq, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)
    solver = novaphy.ArticulatedSolver()

    # Link mesh
    link_half = np.array([0.04, link_len / 2, 0.04], dtype=np.float32)
    link_verts_local, link_faces = make_box_mesh(link_half)
    link_verts_local[:, 1] -= link_len / 2  # top at joint, extends downward

    def callback():
        nonlocal q, qd
        for _ in range(2):
            q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 240.0)

        transforms = novaphy.forward_kinematics(art, q)
        for i in range(num_links):
            pos = np.array(transforms[i].position, dtype=np.float32)
            rot = quat_to_rotation_matrix(np.array(transforms[i].rotation))
            world_v = (link_verts_local @ rot.T) + pos
            name = f"link_{i}"
            if ps.has_surface_mesh(name):
                ps.get_surface_mesh(name).update_vertex_positions(world_v)
            else:
                # Color gradient from red to blue
                t = i / max(1, num_links - 1)
                color = (1 - t, 0.2, t)
                m = ps.register_surface_mesh(name, world_v, link_faces)
                m.set_color(color)

    ps.register_point_cloud("anchor", np.array([[0, 0, 0]], dtype=np.float32))
    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    run_visual()
