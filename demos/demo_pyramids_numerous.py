"""Demo: Multiple pyramids with configurable layers and count.

Demonstrates stacking with multiple contact constraints across many pyramids.

Performance notes
-----------------
* AsyncPhysicsDriver uses double-buffering to separate physics and rendering.
* Physics thread writes to one buffer slot while render reads from the other.
* Zero-copy: buffer swap via _ready_idx index exchange, no per-frame .copy().
* GIL released during physics step — true CPU parallelism between threads.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import threading
import math
import numpy as np
import novaphy
from novaphy.viz import GeneralBatchedVisualizer

try:
    import polyscope as ps
    HAS_POLYSCOPE = True
except ImportError:
    HAS_POLYSCOPE = False


# -- Simulation parameters ---------------------------------------------------
DT                  = 1.0 / 120.0  # physics timestep (120 Hz)
VELOCITY_ITERATIONS = 30
WARM_STARTING       = True

# -- Pyramid geometry --------------------------------------------------------
NUM_LAYERS      = 4      # layers per pyramid
NUM_PYRAMIDS    = 10000   # number of pyramids
BOX_HALF        = np.array([0.5, 0.5, 0.5], dtype=np.float32)
BOX_SPACING     = 1.05   # centre-to-centre spacing

def build_world(num_layers: int = NUM_LAYERS, num_pyramids: int = NUM_PYRAMIDS):
    """Build a scene with multiple pyramids over a ground plane.

    Args:
        num_layers: Number of layers in each pyramid.
        num_pyramids: Number of pyramids to create.

    Returns:
        novaphy.World: The constructed physics world.
    """
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0, friction=0.6)

    half = BOX_HALF
    spacing = BOX_SPACING

    # Calculate grid layout for the pyramids
    pyramids_per_row = int(math.ceil(math.sqrt(num_pyramids)))
    pyramid_spacing_x = num_layers * 1.5  # horizontal spacing between pyramids
    pyramid_spacing_z = num_layers * 1.5  # depth spacing between pyramids

    for py_idx in range(num_pyramids):
        row = py_idx // pyramids_per_row
        col = py_idx % pyramids_per_row

        # Base position for this pyramid
        base_x = (col - pyramids_per_row / 2.0) * pyramid_spacing_x
        base_z = (row - pyramids_per_row / 2.0) * pyramid_spacing_z

        # Build pyramid layers: num_layers, num_layers-1, ..., 1
        for layer_idx in range(num_layers):
            count = num_layers - layer_idx
            y = 0.5 + layer_idx * 1.0
            # Center the row
            start_x = base_x - (count - 1) * spacing / 2.0
            for i in range(count):
                x = start_x + i * spacing
                z = base_z
                body = novaphy.RigidBody.from_box(1.0, half)
                t = novaphy.Transform.from_translation(
                    np.array([x, y, z], dtype=np.float32))
                idx = builder.add_body(body, t)

                shape = novaphy.CollisionShape.make_box(
                    half, idx, novaphy.Transform.identity(), 0.5, 0.0)
                builder.add_shape(shape)

    model = builder.build()
    settings = novaphy.SolverSettings()
    settings.velocity_iterations = VELOCITY_ITERATIONS
    settings.warm_starting = WARM_STARTING
    return novaphy.World(model, settings)


class AsyncPhysicsDriver:
    """Runs physics in a background thread; render callback reads last snapshot.

    Uses double-buffering: physics writes to one buffer slot while render reads
    from the other, eliminating per-step .copy() allocations.
    """

    def __init__(self, world, viz, dt):
        self.world    = world
        self.viz      = viz
        self.dt       = dt
        self._running = False
        N = world.model.num_bodies
        # Double buffer — physics writes to _bufs[_write_idx], render reads _bufs[_ready_idx]
        self._bufs = [
            (np.zeros((N, 3), dtype=np.float32), np.zeros((N, 4), dtype=np.float32)),
            (np.zeros((N, 3), dtype=np.float32), np.zeros((N, 4), dtype=np.float32)),
        ]
        self._write_idx = 0
        self._ready_idx = None  # index of last fully-written buffer (atomic int under GIL)
        self._new_frame = False
        # Timing stats
        self._render_times: list = []
        self._last_hud = 0.0

    def start(self):
        self._running = True
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def stop(self):
        self._running = False

    def _loop(self):
        state = self.world.state
        while self._running:
            self.world.step(self.dt)          # GIL released — true CPU parallelism
            # GIL re-acquired; C++ state fully written. Zero-copy into write slot:
            w = self._write_idx
            state.get_transforms_into(self._bufs[w][0], self._bufs[w][1])
            self._ready_idx = w               # atomic int assign under CPython GIL
            self._write_idx = 1 - w           # flip for next step
            self._new_frame = True

    def render(self):
        """Call from ps.set_user_callback — runs on Polyscope main thread."""
        if self._new_frame and self._ready_idx is not None:
            self._new_frame = False
            self.viz.update_from_arrays(*self._bufs[self._ready_idx])


def main(num_layers: int = NUM_LAYERS, num_pyramids: int = NUM_PYRAMIDS):
    """Run the multi-pyramid demo with async rendering.

    Args:
        num_layers: Number of layers in each pyramid.
        num_pyramids: Number of pyramids to create.
    """

    print(f"Building scene: {num_pyramids} pyramids x {num_layers} layers ...")

    world = build_world(num_layers, num_pyramids)
    print(f"Scene ready: {world.model.num_bodies} bodies")

    if not HAS_POLYSCOPE:
        print("Polyscope not found -- running 500 headless steps.")
        for i in range(500):
            world.step(DT)
            if i % 100 == 0:
                print(f"  step {i}")
        return

    ps.init()
    ps.set_program_name(f"NovaPhy - {num_pyramids} Pyramids ({num_layers} layers each)")
    ps.set_up_dir("y_up")
    ps.set_ground_plane_mode("shadow_only")
    ps.set_navigation_style("free")

    # Calculate ground size based on the pyramid grid layout
    pyramids_per_row = math.ceil(math.sqrt(num_pyramids))
    pyramid_spacing = 3
    ground_size = pyramids_per_row * pyramid_spacing + 5
    viz = GeneralBatchedVisualizer(
        world,
        ground_size=ground_size,
        sphere_lat=8,
        sphere_lon=16,
        colors={"Box": (0.75, 0.55, 0.30), "Sphere": (0.20, 0.50, 0.90)},
    )

    driver = AsyncPhysicsDriver(world, viz, DT)
    driver.start()
    ps.set_user_callback(driver.render)
    ps.show()
    driver.stop()

    driver.viz = None
    viz._batches.clear()
    del viz
    del driver

    ps.remove_all_structures()
    ps.shutdown()


if __name__ == "__main__":
    main()
    
