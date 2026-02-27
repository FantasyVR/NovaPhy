"""Demo: Rope bridge made of revolute-linked segments with fixed endpoints.

10 segments connected by revolute joints, hanging under gravity.
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


def build_rope_bridge(num_segments=10, seg_length=0.5, seg_mass=0.2):
    """Builds a serial revolute chain approximating a rope bridge.

    Args:
        num_segments (int): Number of bridge segments.
        seg_length (float): Segment length in meters.
        seg_mass (float): Segment mass in kilograms.

    Returns:
        novaphy.Articulation: Configured articulated model.
    """
    art = novaphy.Articulation()
    joints = []
    bodies = []

    for i in range(num_segments):
        j = novaphy.Joint()
        j.type = novaphy.JointType.Revolute
        j.axis = np.array([0, 0, 1], dtype=np.float32)

        if i == 0:
            j.parent = -1
            j.parent_to_joint = novaphy.Transform.identity()
        else:
            j.parent = i - 1
            j.parent_to_joint = novaphy.Transform.from_translation(
                np.array([seg_length, 0, 0], dtype=np.float32))

        joints.append(j)

        body = novaphy.RigidBody()
        body.mass = seg_mass
        body.com = np.array([seg_length / 2, 0, 0], dtype=np.float32)
        # Thin rod inertia about one end
        c = seg_mass / 12.0
        body.inertia = np.diag([c * 0.01,
                                c * seg_length * seg_length,
                                c * seg_length * seg_length]).astype(np.float32)
        bodies.append(body)

    art.joints = joints
    art.bodies = bodies
    art.build_spatial_inertias()
    return art


def run_headless(steps=600):
    """Runs the rope-bridge demo without visualization and logs tip state.

    Args:
        steps (int): Number of simulation steps.

    Returns:
        None
    """
    num_seg = 10
    seg_len = 0.5
    art = build_rope_bridge(num_seg, seg_len)
    nq = art.total_q()
    q = np.zeros(nq, dtype=np.float32)
    qd = np.zeros(nq, dtype=np.float32)
    tau = np.zeros(nq, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)

    solver = novaphy.ArticulatedSolver()
    print("=== Rope Bridge ===")
    for i in range(steps):
        q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 120.0)
        if i % 100 == 0:
            transforms = novaphy.forward_kinematics(art, q)
            tip = transforms[-1]
            pos = tip.position
            print(f"  step {i}: tip pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")


def run_visual():
    """Runs the interactive Polyscope rope-bridge visualization.

    Returns:
        None
    """
    if not HAS_POLYSCOPE:
        run_headless()
        return

    ps.init()
    ps.set_program_name("NovaPhy - Rope Bridge")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")

    num_seg = 10
    seg_len = 0.5
    art = build_rope_bridge(num_seg, seg_len)
    nq = art.total_q()
    q = np.zeros(nq, dtype=np.float32)
    qd = np.zeros(nq, dtype=np.float32)
    tau = np.zeros(nq, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)
    solver = novaphy.ArticulatedSolver()

    # Create segment meshes
    seg_half = np.array([seg_len / 2, 0.03, 0.08], dtype=np.float32)
    seg_verts_local, seg_faces = make_box_mesh(seg_half)
    seg_verts_local[:, 0] += seg_len / 2  # offset so left edge at joint

    def callback():
        nonlocal q, qd
        for _ in range(2):
            q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 240.0)

        transforms = novaphy.forward_kinematics(art, q)
        for i in range(num_seg):
            pos = np.array(transforms[i].position, dtype=np.float32)
            rot = quat_to_rotation_matrix(np.array(transforms[i].rotation))
            world_v = (seg_verts_local @ rot.T) + pos
            name = f"seg_{i}"
            if ps.has_surface_mesh(name):
                ps.get_surface_mesh(name).update_vertex_positions(world_v)
            else:
                m = ps.register_surface_mesh(name, world_v, seg_faces)
                m.set_color((0.6, 0.4, 0.2))

    ps.register_point_cloud("anchor", np.array([[0, 0, 0]], dtype=np.float32))
    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    run_visual()
