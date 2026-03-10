"""Demo: box stacking with VBDWorld (3D AVBD pipeline).

Mirrors `demo_stack.py` but uses `novaphy.VBDWorld` (AVBD solver).
Supports both headless and 3D Polyscope GUI, like `demo_stack.py` and
`demo_ipc_stack.py`.

Usage:
    python demos/demo_vbd_stack.py          # 3D window if Polyscope installed
    python demos/demo_vbd_stack.py --headless   # print y positions only
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


def build_vbd_world():
    """Build a simple box-stack scene and return a VBDWorld instance."""
    builder = novaphy.ModelBuilder()

    # Ground plane (y=0, friction 0.5)
    builder.add_ground_plane(0.0, 0.5, 0.0)

    # Stack of boxes (rigid bodies)
    half = np.array([0.5, 0.5, 0.5], dtype=np.float32)
    for i in range(15):
        body = novaphy.RigidBody.from_box(1.0, half)
        y = 1.5 + i * 1.2
        t = novaphy.Transform.from_translation(
            np.array([0.0, y, 0.0], dtype=np.float32)
        )
        body_idx = builder.add_body(body, t)
        shape = novaphy.CollisionShape.make_box(
            half, body_idx, novaphy.Transform.identity(), 0.9, 0.0
        )
        builder.add_shape(shape)

    model = builder.build()

    # 与 avbd-demo3d defaultParams 对齐
    cfg = novaphy.VBDConfig()
    # IMPORTANT: VBDWorld.step() uses cfg.dt, the adapter's `dt` argument is ignored.
    # Use a smaller step for better contact stability in tall stacks.
    cfg.dt = 1.0 / 120.0
    cfg.iterations = 20
    cfg.gravity = np.array([0.0, -10.0, 0.0], dtype=np.float32)
    cfg.alpha = 0.99
    cfg.gamma = 0.999
    cfg.beta_linear = 10000.0
    cfg.beta_angular = 100.0

    world = novaphy.VBDWorld(model, cfg)
    return world


class _VBDWorldAdapter:
    """Adapter so VBDWorld can be used with DemoApp: step(dt) calls world.step()."""
    def __init__(self, vbd_world):
        self._w = vbd_world

    @property
    def model(self):
        return self._w.model

    @property
    def state(self):
        return self._w.state

    def step(self, dt):
        del dt  # VBDWorld uses config.dt
        self._w.step()


class VBDStackDemo:
    """VBD box stack demo with optional 3D Polyscope window (same pattern as demo_stack)."""
    def __init__(self):
        self.title = "NovaPhy - VBD/AVBD Box Stack"
        # Kept only for GUI update cadence; physics dt is cfg.dt inside VBDWorld.
        self.dt = 1.0 / 120.0
        self.ground_size = 20.0
        self.world = None
        self.viz = None

    def build_scene(self):
        vbd_world = build_vbd_world()
        self.world = _VBDWorldAdapter(vbd_world)

    def run(self, headless=False):
        self.build_scene()
        assert self.world is not None

        if headless:
            self._run_headless()
            return

        if not HAS_POLYSCOPE:
            print("Polyscope not available. Running headless...")
            self._run_headless()
            return

        ps.init()
        ps.set_program_name(self.title)
        ps.set_up_dir("y_up")
        ps.set_ground_plane_mode("shadow_only")

        from novaphy.viz import SceneVisualizer
        self.viz = SceneVisualizer(self.world, self.ground_size)

        def callback():
            self.world.step(self.dt)
            self.viz.update()

        ps.set_user_callback(callback)
        ps.show()

    def _run_headless(self, steps=240):
        print("Running VBD box stack demo (headless)...")
        for step in range(steps):
            self.world.step(self.dt)
            if step % 40 == 0 or step == steps - 1:
                state = self.world.state
                n = self.world.model.num_bodies
                ys = [state.transforms[i].position[1] for i in range(n)]
                print(f"step {step:4d}: y = {[f'{y:.3f}' for y in ys]}")
        print("Done.")


if __name__ == "__main__":
    headless = "--headless" in sys.argv
    demo = VBDStackDemo()
    demo.run(headless=headless)

