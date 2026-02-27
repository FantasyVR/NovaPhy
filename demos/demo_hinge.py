"""Demo: Door swinging on a hinge joint.

A single revolute joint with a large flat body representing a door.
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


def build_hinge_door(width=1.5, height=2.0, thickness=0.08, mass=5.0):
    """Builds a single-link revolute articulation representing a door.

    Args:
        width (float): Door width in meters.
        height (float): Door height in meters.
        thickness (float): Door thickness in meters.
        mass (float): Door mass in kilograms.

    Returns:
        tuple[novaphy.Articulation, tuple[float, float, float]]: Articulation
        model and `(width, height, thickness)` dimensions.
    """
    art = novaphy.Articulation()

    # Revolute joint about Y axis (vertical hinge)
    j = novaphy.Joint()
    j.type = novaphy.JointType.Revolute
    j.axis = np.array([0, 1, 0], dtype=np.float32)
    j.parent = -1
    j.parent_to_joint = novaphy.Transform.identity()

    art.joints = [j]

    # Door body: COM offset from hinge
    body = novaphy.RigidBody()
    body.mass = mass
    body.com = np.array([width / 2, 0, 0], dtype=np.float32)
    # Box inertia about COM
    w, h, d = width, height, thickness
    c = mass / 12.0
    body.inertia = np.diag([c * (h*h + d*d),
                            c * (w*w + d*d),
                            c * (w*w + h*h)]).astype(np.float32)

    art.bodies = [body]
    art.build_spatial_inertias()
    return art, (width, height, thickness)


def run_headless(steps=600):
    """Runs the hinge demo without visualization and logs angle samples.

    Args:
        steps (int): Number of simulation steps.

    Returns:
        None
    """
    art, dims = build_hinge_door()
    q = np.array([np.pi / 6], dtype=np.float32)  # start slightly open
    qd = np.array([2.0], dtype=np.float32)        # initial angular velocity
    tau = np.zeros(1, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)

    solver = novaphy.ArticulatedSolver()
    print("=== Hinge Door ===")
    for i in range(steps):
        q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 120.0)
        if i % 100 == 0:
            print(f"  step {i}: angle={np.degrees(q[0]):.1f}°, "
                  f"angular_vel={qd[0]:.3f}")


def run_visual():
    """Runs the interactive Polyscope hinge-door visualization.

    Returns:
        None
    """
    if not HAS_POLYSCOPE:
        run_headless()
        return

    ps.init()
    ps.set_program_name("NovaPhy - Hinge Door")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")

    art, (width, height, thickness) = build_hinge_door()
    q = np.array([np.pi / 6], dtype=np.float32)
    qd = np.array([2.0], dtype=np.float32)
    tau = np.zeros(1, dtype=np.float32)
    gravity = np.array([0, -9.81, 0], dtype=np.float32)
    solver = novaphy.ArticulatedSolver()

    # Door mesh: offset so hinge edge is at origin
    door_half = np.array([width / 2, height / 2, thickness / 2], dtype=np.float32)
    door_verts, door_faces = make_box_mesh(door_half)
    door_verts[:, 0] += width / 2  # shift so left edge is at hinge

    def callback():
        nonlocal q, qd
        for _ in range(2):
            q, qd = solver.step(art, q, qd, tau, gravity, 1.0 / 240.0)

        transforms = novaphy.forward_kinematics(art, q)
        pos = np.array(transforms[0].position, dtype=np.float32)
        rot = quat_to_rotation_matrix(np.array(transforms[0].rotation))
        world_v = (door_verts @ rot.T) + pos

        if ps.has_surface_mesh("door"):
            ps.get_surface_mesh("door").update_vertex_positions(world_v)
        else:
            m = ps.register_surface_mesh("door", world_v, door_faces)
            m.set_color((0.55, 0.35, 0.2))

    # Hinge axis visualization
    ps.register_point_cloud("hinge", np.array([[0, 0, 0]], dtype=np.float32))

    ps.set_user_callback(callback)
    ps.show()


if __name__ == "__main__":
    run_visual()
