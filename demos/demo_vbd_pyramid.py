"""Demo: pyramid of boxes with VBDWorld (5-layer pyramid, avbd-demo3d scenePyramid layout).

Mirrors avbd-demo3d scenes.h scenePyramid():
  - SIZE rows; for row index y: (SIZE - y) boxes.
  - Position (demo3d Z-up): x*1.01 + y*0.5 - SIZE/2, 0, y*0.85+0.5.
  - Box full size (1, 0.5, 0.5) → half-extents (0.5, 0.25, 0.25).
  - NovaPhy Y-up: X = same, Y = height (demo3d Z), Z = 0.

Usage:
    python demos/demo_vbd_pyramid.py
    python demos/demo_vbd_pyramid.py --headless
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


def build_vbd_pyramid_world():
    """Build a fixed 5-layer pyramid scene matching avbd-demo3d scenePyramid."""
    SIZE = 16
    builder = novaphy.ModelBuilder()


   # Ground box (match avbd-demo3d: large static box instead of infinite plane)
    half_ground = np.array([50.0, 0.5, 50.0], dtype=np.float32)
    ground_body = novaphy.RigidBody.make_static()
    ground_t = novaphy.Transform.from_translation(
        np.array([0.0, -0.5, 0.0], dtype=np.float32)
    )
    ground_idx = builder.add_body(ground_body, ground_t)
    ground_shape = novaphy.CollisionShape.make_box(
        half_ground, ground_idx, novaphy.Transform.identity(), 0.9, 0.0
    )
    builder.add_shape(ground_shape)
    # Box full size (1, 0.5, 0.5) → half-extents
    half = np.array([0.5, 0.25, 0.25], dtype=np.float32)
    for row in range(SIZE):
        for x in range(SIZE - row):
            # demo3d: pos (x*1.01 + y*0.5 - SIZE/2, 0, y*0.85 + 0.5); Y-up → (X, Y=height, Z=0)
            px = x * 1.01 + row * 0.5 - SIZE / 2.0
            py = row * 0.85 + 0.5
            pz = 0.0
            body = novaphy.RigidBody.from_box(1.0, half)
            t = novaphy.Transform.from_translation(
                np.array([px, py, pz], dtype=np.float32)
            )
            body_idx = builder.add_body(body, t)
            shape = novaphy.CollisionShape.make_box(
                half, body_idx, novaphy.Transform.identity(), 0.5, 0.0
            )
            builder.add_shape(shape)

    model = builder.build()

    cfg = novaphy.VBDConfig()
    # Use a smaller dt for better stacking stability (especially when SIZE is increased).
    cfg.dt = 1.0 / 120.0
    cfg.iterations = 30
    cfg.max_contacts_per_pair = 8  
    cfg.gravity = np.array([0.0, -10.0, 0.0], dtype=np.float32)
    cfg.alpha = 0.99
    cfg.gamma = 0.999
    cfg.beta_linear = 10000.0
    cfg.beta_angular = 100.0

    world = novaphy.VBDWorld(model, cfg)
    return world


class _VBDWorldAdapter:
    def __init__(self, vbd_world):
        self._w = vbd_world

    @property
    def model(self):
        return self._w.model

    @property
    def state(self):
        return self._w.state

    def step(self, dt):
        del dt
        self._w.step()


class VBDPyramidDemo:
    """VBD 5-layer pyramid demo (avbd-demo3d scenePyramid layout) with optional Polyscope."""
    def __init__(self):
        self.title = "NovaPhy - VBD Pyramid (demo3d scenePyramid)"
        # GUI update cadence only; VBDWorld physics dt is cfg.dt.
        self.dt = 1.0 / 120.0
        self.ground_size = 20.0
        self.world = None
        self.viz = None

    def build_scene(self):
        self.world = _VBDWorldAdapter(build_vbd_pyramid_world())

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

    def _run_headless(self, steps=300):
        print("Running VBD pyramid demo (headless, avbd-demo3d scenePyramid layout)...")
        n = self.world.model.num_bodies
        for step in range(steps):
            self.world.step(self.dt)
            if step % 80 == 0 or step == steps - 1:
                state = self.world.state
                ys = [state.transforms[i].position[1] for i in range(n)]
                # print first 8 and last 4 y positions to avoid huge output
                head = [f"{y:.3f}" for y in ys[:8]]
                tail = [f"{y:.3f}" for y in ys[-4:]] if n > 12 else []
                part = head + (["..."] if n > 12 else []) + tail
                print(f"step {step:4d}: n={n} y = {part}")
        print("Done.")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="VBD 5-layer pyramid (avbd-demo3d scenePyramid)")
    p.add_argument("--headless", action="store_true", help="No GUI, print y positions")
    args = p.parse_args()
    demo = VBDPyramidDemo()
    demo.run(headless=args.headless)
