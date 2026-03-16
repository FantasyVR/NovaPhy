"""GUI: demo3d-style joint scenes (rope, heavy rope, bridge, breakable, soft-body joints).

No ground is added by default for rope-like scenes per user preference.
Scene switching fully clears Polyscope structures to avoid leftovers.
"""

import os
import sys
import argparse
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import novaphy

_BACKEND = "cpu"

try:
    import polyscope as ps
    import polyscope.imgui as psim
    HAS_POLYSCOPE = True
except ImportError:
    HAS_POLYSCOPE = False


def _box(builder, full_size, density, friction, pos):
    half = (np.array(full_size, dtype=np.float32) * 0.5).astype(np.float32)
    body = novaphy.RigidBody.from_box(float(density), half)
    t = novaphy.Transform.from_translation(np.array(pos, dtype=np.float32))
    bi = builder.add_body(body, t)
    builder.add_shape(novaphy.CollisionShape.make_box(half, bi, novaphy.Transform.identity(), float(friction), 0.0))
    return bi


def _static_box(builder, full_size, friction, pos):
    half = (np.array(full_size, dtype=np.float32) * 0.5).astype(np.float32)
    body = novaphy.RigidBody.make_static()
    t = novaphy.Transform.from_translation(np.array(pos, dtype=np.float32))
    bi = builder.add_body(body, t)
    builder.add_shape(novaphy.CollisionShape.make_box(half, bi, novaphy.Transform.identity(), float(friction), 0.0))
    return bi


def _make_world(model):
    cfg = novaphy.VBDConfig()
    cfg.dt = 1.0 / 60.0
    cfg.iterations = 10
    cfg.gravity = np.array([0.0, -10.0, 0.0], dtype=np.float32)
    cfg.alpha = 0.99
    cfg.gamma = 0.999
    cfg.beta_linear = 10000.0
    cfg.beta_angular = 100.0
    cfg.primal_relaxation = 0.9
    cfg.lhs_regularization = 1e-6
    if _BACKEND.lower() == "cuda":
        cfg.backend = novaphy.VbdBackend.CUDA
    else:
        cfg.backend = novaphy.VbdBackend.CPU
    return novaphy.VBDWorld(model, cfg)


def build_scene(name: str):
    name = name.lower()
    b = novaphy.ModelBuilder()

    joints = []  # for visualization: list of (body_a, body_b, rA, rB)
    ignores = []

    def add_joint(a, bb, rA, rB, Klin=float("inf"), Kang=0.0, fracture=float("inf")):
        joints.append((a, bb, np.array(rA, np.float32), np.array(rB, np.float32), Klin, Kang, fracture))
        # demo3d broadphase skips collisions between constrained bodies
        if a >= 0 and bb >= 0:
            add_ignore(a, bb)

    def add_ignore(a, bb):
        ignores.append((a, bb))

    if name == "rope":
        prev = None
        for i in range(20):
            dens = 0.0 if i == 0 else 1.0
            curr = _box(b, (1, 0.5, 0.5), dens, 0.5, (float(i), 10.0, 0.0))
            if prev is not None:
                add_joint(prev, curr, (0.5, 0.0, 0.0), (-0.5, 0.0, 0.0))
            prev = curr

    elif name == "heavy_rope":
        N = 20
        SIZE = 5.0
        prev = None
        for i in range(N):
            size = (SIZE, SIZE, SIZE) if i == N - 1 else (1, 0.5, 0.5)
            dens = 0.0 if i == 0 else 1.0
            x = float(i) + (SIZE / 2 if i == N - 1 else 0.0)
            curr = _box(b, size, dens, 0.5, (x, 10.0, 0.0))
            if prev is not None:
                rB = (-SIZE / 2, 0.0, 0.0) if i == N - 1 else (-0.5, 0.0, 0.0)
                add_joint(prev, curr, (0.5, 0.0, 0.0), rB)
            prev = curr

    elif name == "bridge":
        # Add ground only for bridge / breakable / soft_body where it matters visually.
        _static_box(b, (100, 1, 100), 0.5, (0, 0.0, 0))
        N = 40
        plank_len, plank_w, plank_h = 1.0, 4.0, 0.5
        half_len, half_w = plank_len * 0.5, plank_w * 0.5
        prev = None
        for i in range(N):
            dens = 0.0 if i == 0 or i == N - 1 else 1.0
            curr = _box(b, (plank_len, plank_h, plank_w), dens, 0.5, (i - N / 2.0, 10.0, 0.0))
            if prev is not None:
                add_joint(prev, curr, (half_len, 0.0, half_w), (-half_len, 0.0, half_w), float("inf"), 0.0)
                add_joint(prev, curr, (half_len, 0.0, -half_w), (-half_len, 0.0, -half_w), float("inf"), 0.0)
            prev = curr
        for x in range(N // 4):
            for y in range(N // 8):
                _box(b, (1, 1, 1), 1.0, 0.5, (x - N / 8.0, 12.0 + y, 0.0))

    elif name == "breakable":
        _static_box(b, (100, 1, 100), 0.5, (0, 0.0, 0))
        N, M = 10, 5
        # Less fragile than demo3d default (90): easier to observe impacts before fracture.
        break_force = 140.0
        prev = None
        for i in range(N + 1):
            curr = _box(b, (1, 0.5, 1), 1.0, 0.5, (i - N / 2.0, 6.0, 0.0))
            if prev is not None:
                add_joint(prev, curr, (0.5, 0.0, 0.0), (-0.5, 0.0, 0.0), float("inf"), float("inf"), break_force)
            prev = curr
        _static_box(b, (1, 5, 1), 0.5, (-N / 2.0, 2.5, 0.0))
        _static_box(b, (1, 5, 1), 0.5, (N / 2.0, 2.5, 0.0))
        # Stack 4 blocks vertically (like demo3d), so they drop as a pile.
        base_y = 8.0
        for i in range(5):
            _box(b, (2, 1, 1), 1.0, 0.5, (0.0, base_y + i * 1.01, 0.0))

    elif name == "soft_body":
        _static_box(b, (100, 1, 100), 0.5, (0, 0.0, 0))
        Klin, Kang = 1000.0, 250.0
        W, D, H = 4, 4, 4
        N = 3
        size = 0.8
        half = size * 0.5
        baseY = 8.0
        stack_gap = 2.0
        for i in range(N):
            grid = [[[None for _ in range(H)] for _ in range(D)] for _ in range(W)]
            stackY = i * (H * size + stack_gap)
            for x in range(W):
                for y in range(D):
                    for z in range(H):
                        px = (x - (W - 1) * 0.5) * size
                        pz = (y - (D - 1) * 0.5) * size
                        py = baseY + stackY + z * size
                        grid[x][y][z] = _box(b, (size, size, size), 1.0, 0.5, (px, py, pz))
            for x in range(1, W):
                for y in range(D):
                    for z in range(H):
                        add_joint(grid[x - 1][y][z], grid[x][y][z], (half, 0, 0), (-half, 0, 0), Klin, Kang)
            for x in range(W):
                for y in range(1, D):
                    for z in range(H):
                        add_joint(grid[x][y - 1][z], grid[x][y][z], (0, 0, half), (0, 0, -half), Klin, Kang)
            for x in range(W):
                for y in range(D):
                    for z in range(1, H):
                        add_joint(grid[x][y][z - 1], grid[x][y][z], (0, half, 0), (0, -half, 0), Klin, Kang)
            for x in range(1, W):
                for y in range(D):
                    for z in range(1, H):
                        add_ignore(grid[x - 1][y][z - 1], grid[x][y][z])
                        add_ignore(grid[x][y][z - 1], grid[x - 1][y][z])
            for x in range(W):
                for y in range(1, D):
                    for z in range(1, H):
                        add_ignore(grid[x][y - 1][z - 1], grid[x][y][z])
                        add_ignore(grid[x][y][z - 1], grid[x][y - 1][z])
            for x in range(1, W):
                for y in range(1, D):
                    for z in range(H):
                        add_ignore(grid[x - 1][y - 1][z], grid[x][y][z])
                        add_ignore(grid[x][y - 1][z], grid[x - 1][y][z])
    else:
        raise ValueError(name)

    world = _make_world(b.build())
    world.clear_forces()
    for a, bb in ignores:
        world.add_ignore_collision(int(a), int(bb))
    for a, bb, rA, rB, Klin, Kang, fracture in joints:
        world.add_joint(int(a), int(bb), rA, rB, float(Klin), float(Kang), float(fracture))
    joint_defs = [(int(a), int(bb), rA.copy(), rB.copy()) for a, bb, rA, rB, *_ in joints]
    return world, joint_defs


class _Adapter:
    def __init__(self, w):
        self._w = w

    @property
    def model(self):
        return self._w.model

    @property
    def state(self):
        return self._w.state

    def step(self, dt):
        del dt
        self._w.step()


def main():
    if not HAS_POLYSCOPE:
        raise RuntimeError("polyscope not installed")

    scenes = ["rope", "heavy_rope", "bridge", "breakable", "soft_body"]
    state = {"scene_idx": 0, "paused": False, "step_once": False, "need_rebuild": True, "w": None, "viz": None}

    ps.init()
    ps.set_program_name("NovaPhy - VBD joint scenes")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")
    ps.look_at((20.0, 14.0, 20.0), (8.0, 6.0, 0.0))

    from novaphy.viz import SceneVisualizer

    def joint_nodes(adapter):
        defs = getattr(adapter, "joint_defs", [])
        nodes = []
        edges = []
        for i, (a, bb, rA, rB) in enumerate(defs):
            if a < 0 or bb < 0:
                continue
            ta = adapter.state.transforms[a]
            tb = adapter.state.transforms[bb]
            pA = np.array(ta.transform_point(rA), dtype=np.float32)
            pB = np.array(tb.transform_point(rB), dtype=np.float32)
            nodes.append(pA)
            nodes.append(pB)
            edges.append([2 * i, 2 * i + 1])
        if not nodes:
            nodes = [np.zeros(3, np.float32)]
            edges = "segments"
        return np.vstack(nodes).astype(np.float32), np.array(edges, dtype=np.int32) if isinstance(edges, list) else edges

    def rebuild():
        ps.remove_all_structures()
        ps.remove_all_groups()
        ps.remove_all_slice_planes()
        ps.remove_all_transformation_gizmos()
        world, joint_defs = build_scene(scenes[state["scene_idx"]])
        state["w"] = _Adapter(world)
        state["w"].joint_defs = joint_defs
        state["viz"] = SceneVisualizer(state["w"], 40.0)
        # (no extra joint decoration; only rigid meshes)
        state["need_rebuild"] = False

    def cb():
        psim.PushItemWidth(260)
        changed, new_idx = psim.Combo("scene", state["scene_idx"], scenes)
        psim.PopItemWidth()
        if changed:
            state["scene_idx"] = new_idx
            state["need_rebuild"] = True

        psim.SameLine()
        if psim.Button("reset"):
            state["need_rebuild"] = True

        _, paused_val = psim.Checkbox("paused", bool(state["paused"]))
        state["paused"] = paused_val
        psim.SameLine()
        if psim.Button("step"):
            state["step_once"] = True

        if state["need_rebuild"] or state["w"] is None or state["viz"] is None:
            rebuild()

        if not state["paused"] or state["step_once"]:
            state["w"].step(0.0)
            state["step_once"] = False
        # (no joint curve network update)
        state["viz"].update()

    ps.set_user_callback(cb)
    ps.show()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="VBD joint scenes (rope, bridge, etc.)")
    ap.add_argument("--backend", default="cuda", choices=("cpu", "cuda"), help="VBD solver backend")
    args = ap.parse_args()
    _BACKEND = args.backend
    main()

