"""GUI: demo3d-style spring scenes.

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
    springs = []  # for visualization: list of (body_a, body_b, rA, rB)

    def add_spring(a, bb, rA, rB, k, rest=-1.0):
        springs.append((a, bb, np.array(rA, np.float32), np.array(rB, np.float32), float(k), float(rest)))

    if name == "spring":
        _static_box(b, (100, 1, 100), 0.5, (0, 0.0, 0))
        anchor = _box(b, (1, 1, 1), 0.0, 0.5, (0, 14.0, 0))
        block = _box(b, (2, 2, 2), 1.0, 0.5, (0, 8.0, 0))
        add_spring(anchor, block, (0, 0, 0), (0, 0, 0), 100.0, 4.0)

    elif name == "springs_ratio":
        N = 8
        _static_box(b, (100, 1, 100), 0.5, (0, -10.0, 0))
        prev = None
        for i in range(N):
            x = (i - (N - 1) * 0.5) * 3.0
            dens = 0.0 if i == 0 or i == N - 1 else 1.0
            curr = _box(b, (1, 0.75, 0.75), dens, 0.5, (x, 12.0, 0.0))
            if prev is not None:
                k = 10.0 if i % 2 == 0 else 10000.0
                add_spring(prev, curr, (0.5, 0.0, 0.0), (-0.5, 0.0, 0.0), k, 3.0)
            prev = curr
    else:
        raise ValueError(name)

    world = _make_world(b.build())
    world.clear_forces()
    for a, bb, rA, rB, k, rest in springs:
        world.add_spring(int(a), int(bb), rA, rB, float(k), float(rest))
    spring_defs = [(int(a), int(bb), rA.copy(), rB.copy()) for a, bb, rA, rB, _, _ in springs]
    return world, spring_defs


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

    scenes = ["spring", "springs_ratio"]
    state = {"scene_idx": 0, "paused": False, "step_once": False, "need_rebuild": True, "w": None, "viz": None}

    ps.init()
    ps.set_program_name("NovaPhy - VBD spring scenes")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")
    ps.look_at((16.0, 10.0, 16.0), (0.0, 8.0, 0.0))

    from novaphy.viz import SceneVisualizer

    def spring_nodes(adapter):
        defs = getattr(adapter, "spring_defs", [])
        nodes = []
        edges = []
        for i, (a, bb, rA, rB) in enumerate(defs):
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
        world, spring_defs = build_scene(scenes[state["scene_idx"]])
        state["w"] = _Adapter(world)
        state["w"].spring_defs = spring_defs
        state["viz"] = SceneVisualizer(state["w"], 30.0)
        # Spring visualization as curve network
        nodes, edges = spring_nodes(state["w"])
        import polyscope as _ps
        if _ps.has_curve_network("springs"):
            _ps.remove_curve_network("springs", error_if_absent=False)
        # Thinner spring lines for nicer look.
        cn = _ps.register_curve_network("springs", nodes, edges, radius=0.0008, color=(0.9, 0.2, 0.2))
        cn.set_radius(0.0001)
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
        # Update spring curve nodes
        adapter = state["w"]
        defs = getattr(adapter, "spring_defs", [])
        if defs and ps.has_curve_network("springs"):
            nodes = []
            for (a, bb, rA, rB) in defs:
                ta = adapter.state.transforms[a]
                tb = adapter.state.transforms[bb]
                nodes.append(np.array(ta.transform_point(rA), dtype=np.float32))
                nodes.append(np.array(tb.transform_point(rB), dtype=np.float32))
            ps.get_curve_network("springs").update_node_positions(np.vstack(nodes).astype(np.float32))
        state["viz"].update()

    ps.set_user_callback(cb)
    ps.show()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="VBD spring scenes")
    ap.add_argument("--backend", default="cuda", choices=("cpu", "cuda"), help="VBD solver backend")
    args = ap.parse_args()
    _BACKEND = args.backend
    main()

