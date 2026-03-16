"""GUI: demo3d-style rigid/contact scenes (no joints/springs).

Keeps the demo directory cleaner by grouping similar scenes together.
Scene switching fully clears Polyscope structures to avoid leftovers.
"""

import os
import sys
import argparse
import time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import novaphy

try:
    import polyscope as ps
    import polyscope.imgui as psim
    HAS_POLYSCOPE = True
except ImportError:
    HAS_POLYSCOPE = False


# Global backend selector for VBDWorld used in this demo.
# Can be overridden via command-line argument.
_BACKEND = "cpu"


def _box(builder, full_size, density, friction, pos):
    half = (np.array(full_size, dtype=np.float32) * 0.5).astype(np.float32)
    body = novaphy.RigidBody.from_box(float(density), half)
    t = novaphy.Transform.from_translation(np.array(pos, dtype=np.float32))
    bi = builder.add_body(body, t)
    builder.add_shape(novaphy.CollisionShape.make_box(half, bi, novaphy.Transform.identity(), float(friction), 0.0))
    return bi


def _static_box(builder, full_size, friction, pos, rot=None):
    half = (np.array(full_size, dtype=np.float32) * 0.5).astype(np.float32)
    body = novaphy.RigidBody.make_static()
    t = novaphy.Transform.from_translation(np.array(pos, dtype=np.float32))
    if rot is not None:
        t = t * rot
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
    # Select backend based on global switch.
    if _BACKEND.lower() == "cuda":
        cfg.backend = novaphy.VbdBackend.CUDA
    else:
        cfg.backend = novaphy.VbdBackend.CPU
    return novaphy.VBDWorld(model, cfg)


def build_scene(name: str):
    name = name.lower()
    b = novaphy.ModelBuilder()
    init_lin_vel = {}

    if name == "ground":
        _static_box(b, (100, 1, 100), 0.5, (0, 0.0, 0))
        _box(b, (1, 1, 1), 1.0, 0.5, (0, 4.0, 0))

    elif name == "dynamic_friction":
        _static_box(b, (100, 1, 100), 0.5, (0, 0.0, 0))
        for x in range(0, 11):
            fr = 5.0 - (x / 10.0 * 5.0)
            bi = _box(b, (1, 0.5, 1), 1.0, fr, (0, 0.75, -30.0 + x * 2.0))
            init_lin_vel[bi] = np.array([10.0, 0.0, 0.0], dtype=np.float32)

    elif name == "static_friction":
        _static_box(b, (100, 1, 100), 0.5, (0, 0.0, 0))
        # Flip slope direction: make the previous high side become low.
        angle = -np.deg2rad(30.0)
        # demo3d rotates ramp about world-Y (with Z-up). Mapping to our Y-up: demo-Y -> our-Z.
        ramp_rot = novaphy.Transform.from_axis_angle(np.array([0.0, 0.0, 1.0], np.float32), float(angle))
        # Lift the ramp up so blocks never start below the visible ground plane.
        ramp_pos = np.array([0.0, 10.0, 0.0], dtype=np.float32)
        _static_box(b, (40, 1, 24), 1.0, tuple(ramp_pos.tolist()), rot=ramp_rot)

        # demo3d placement:
        # rampTangent = rotate(q, (1,0,0))
        # rampNormal  = rotate(q, (0,0,1))
        # pos = rampPos + rampTangent * -12 + (0, y, 0) + rampNormal * 1.05
        # Here: demo3d Y axis maps to our Z axis (Y-up), so (0,y,0) -> (0,0,y).
        R = ramp_rot.rotation_matrix()
        ramp_tangent = (R @ np.array([1.0, 0.0, 0.0], dtype=np.float32)).astype(np.float32)
        # demo3d rampNormal uses (0,0,1) in Z-up -> maps to our (0,1,0) in Y-up
        ramp_normal = (R @ np.array([0.0, 1.0, 0.0], dtype=np.float32)).astype(np.float32)

        for i in range(0, 11):
            fr = i / 10.0 * 0.25 + 0.25
            y_demo = -10.0 + i * 2.0
            # Place blocks close to the ramp surface to avoid an initial drop/bounce which can kick them off.
            pos = ramp_pos + ramp_tangent * -12.0 + np.array([0.0, 0.0, y_demo], dtype=np.float32) + ramp_normal * 1.10
            _box(b, (1, 1, 1), 1.0, fr, (float(pos[0]), float(pos[1]), float(pos[2])))

    elif name == "stack":
        _static_box(b, (100, 1, 100), 0.5, (0, 0.0, 0))
        for i in range(10):
            _box(b, (1, 1, 1), 1.0, 0.5, (0, i * 1.5 + 1.0, 0))

    elif name == "stack_ratio":
        ground_thick = 1.0
        _static_box(b, (100, ground_thick, 100), 0.5, (0, 0.0, 0))
        top = ground_thick * 0.5
        s = 1.0
        for _ in range(4):
            half = s * 0.5
            center = top + half
            _box(b, (s, s, s), 1.0, 0.5, (0, center, 0))
            top = center + half
            s *= 2.0

    elif name == "pyramid":
        SIZE = 16
        _static_box(b, (100, 1, 100), 0.5, (0.0, -0.5, 0.0))
        for y in range(SIZE):
            for x in range(SIZE - y):
                px = x * 1.01 + y * 0.5 - SIZE / 2.0
                py = y * 0.85 + 0.5
                _box(b, (1, 0.5, 0.5), 1.0, 0.5, (px, py, 0.0))

    else:
        raise ValueError(name)

    world = _make_world(b.build())
    # initial velocities if supported
    st = getattr(world, "state_mut", None)
    if st is not None:
        for bi, v in init_lin_vel.items():
            st.set_linear_velocity(int(bi), v)
    return world


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


def _run_headless_compare(scene: str = "pyramid", steps: int = 300):
    """Headless CPU vs CUDA comparison for a given scene.

    Runs the same scene with CPU and CUDA backends and prints basic
    trajectory statistics to the console.
    """

    def _run_once(backend: str, collect_all=True):
        global _BACKEND
        old = _BACKEND
        _BACKEND = backend
        world = build_scene(scene)
        _BACKEND = old

        ys = []
        model = world.model
        t0 = time.perf_counter()
        for i in range(steps):
            world.step()
            st = world.state
            ys.append([st.transforms[b].position[1] for b in range(model.num_bodies)])
        elapsed = time.perf_counter() - t0
        ms_per_step = (elapsed / steps) * 1000.0
        fps = steps / elapsed if elapsed > 0 else 0.0
        print(f"  [{backend}] {elapsed:.2f}s total, {ms_per_step:.1f} ms/step, {fps:.1f} FPS")
        return np.asarray(ys, dtype=np.float32), elapsed

    print(f"[VBD contact scene: {scene}] steps={steps}")
    ys_cpu, _ = _run_once("cpu")
    ys_cuda, _ = _run_once("cuda")

    if ys_cpu.shape != ys_cuda.shape:
        print("CPU/CUDA shapes differ:", ys_cpu.shape, ys_cuda.shape)
        return

    diff = np.abs(ys_cpu - ys_cuda)
    print("  bodies:", ys_cpu.shape[1])
    print("  max abs position diff (all steps):", float(diff.max()))
    print("  mean abs position diff:", float(diff.mean()))
    # Show when divergence starts (sample at step 0, 1, 9, 49 to see early vs late).
    for step_idx in [0, 1, 9, 49]:
        if step_idx < diff.shape[0]:
            d = diff[step_idx]
            print(f"  at step {step_idx}: max_diff={float(d.max()):.6f}, mean_diff={float(d.mean()):.6f}")


def main():
    parser = argparse.ArgumentParser(description="VBD contact scenes (GUI/headless compare)")
    parser.add_argument(
        "--backend",
        choices=["cpu", "cuda"],
        default="cuda",
        help="Backend for VBD solver used in GUI mode.",
    )
    parser.add_argument(
        "--headless-compare",
        action="store_true",
        help="Run a headless CPU vs CUDA comparison (no GUI) on the pyramid scene.",
    )
    parser.add_argument(
        "--scene",
        choices=["ground", "dynamic_friction", "static_friction", "stack", "stack_ratio", "pyramid"],
        default="ground",
        help="Scene name for headless comparison.",
    )
    args, _ = parser.parse_known_args()

    if args.headless_compare:
        _run_headless_compare(scene=args.scene, steps=300)
        return

    if not HAS_POLYSCOPE:
        raise RuntimeError("polyscope not installed")

    global _BACKEND
    _BACKEND = args.backend

    scenes = ["ground", "dynamic_friction", "static_friction", "stack", "stack_ratio", "pyramid"]
    state = {
        "scene_idx": 3,
        "paused": False,
        "step_once": False,
        "need_rebuild": True,
        "w": None,
        "viz": None,
        "status": "",
        "last_scene_idx": None,
        "frame_times": [],  # rolling window of step durations (seconds)
        "max_frame_times": 60,
        "warmup_steps": 2,  # run a few untimed steps after rebuild (CUDA context/JIT)
        "target_fps": 60,
    }

    ps.init()
    ps.set_program_name("NovaPhy - VBD contact scenes")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")
    ps.look_at((16.0, 10.0, 16.0), (0.0, 3.0, 0.0))

    from novaphy.viz import SceneVisualizer

    def apply_camera_preset(scene_name: str):
        # Only called on demand (not on reset) so user camera is preserved.
        if scene_name == "static_friction":
            # Put camera in front/side of ramp (not behind).
            ps.look_at((22.0, 18.0, 18.0), (0.0, 11.0, 0.0))
        else:
            ps.look_at((16.0, 10.0, 16.0), (0.0, 3.0, 0.0))

    def rebuild():
        ps.remove_all_structures()
        ps.remove_all_groups()
        ps.remove_all_slice_planes()
        ps.remove_all_transformation_gizmos()
        scene_name = scenes[state["scene_idx"]]
        world = build_scene(scene_name)
        state["w"] = _Adapter(world)
        state["viz"] = SceneVisualizer(state["w"], 30.0)
        state["need_rebuild"] = False
        # Reset timing window on rebuild/reset; first CUDA step can include context/JIT overhead.
        state["frame_times"].clear()
        state["warmup_steps"] = 2
        # Do not touch camera here; preserve user view. Camera preset is applied only when switching scenes
        # (first entry) or when user clicks the button.
        if state["last_scene_idx"] != state["scene_idx"]:
            apply_camera_preset(scene_name)
            state["last_scene_idx"] = state["scene_idx"]

        if scene_name == "dynamic_friction" and not hasattr(world, "state_mut"):
            state["status"] = "warning: rebuild extension to enable initial velocities (state_mut)"
        else:
            state["status"] = ""

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

        psim.SameLine()
        if psim.Button("camera preset"):
            apply_camera_preset(scenes[state["scene_idx"]])

        _, paused_val = psim.Checkbox("paused", bool(state["paused"]))
        state["paused"] = paused_val
        psim.SameLine()
        if psim.Button("step"):
            state["step_once"] = True

        if state["status"]:
            psim.TextUnformatted(state["status"])

        if state["need_rebuild"] or state["w"] is None or state["viz"] is None:
            rebuild()

        if not state["paused"] or state["step_once"]:
            t0 = time.perf_counter()
            state["w"].step(0.0)
            elapsed = time.perf_counter() - t0
            if state.get("warmup_steps", 0) > 0:
                state["warmup_steps"] -= 1
            else:
                ft = state["frame_times"]
                ft.append(elapsed)
                if len(ft) > state["max_frame_times"]:
                    ft.pop(0)
            state["step_once"] = False

            # Cap to ~60 FPS so frame time is predictable and system isn't maxed (reduces stutter)
            target_fps = state.get("target_fps", 60)
            if target_fps > 0 and not state["paused"]:
                frame_budget = 1.0 / float(target_fps)
                sleep_time = frame_budget - elapsed
                if sleep_time > 0.001:  # sleep at least 1ms to avoid busy spin
                    time.sleep(sleep_time)

        state["viz"].update()

        # FPS / frame time (ms) from rolling window; show max to spot hitches
        ft = state["frame_times"]
        if ft:
            avg_ms = (sum(ft) / len(ft)) * 1000.0
            max_ms = max(ft) * 1000.0
            fps = 1.0 / (sum(ft) / len(ft)) if ft else 0.0
            psim.TextUnformatted(f"step: {avg_ms:.1f} ms (max {max_ms:.0f} ms)  |  FPS: {fps:.1f}")

    ps.set_user_callback(cb)
    ps.show()


if __name__ == "__main__":
    main()

