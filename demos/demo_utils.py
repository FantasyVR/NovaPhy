"""Shared demo utilities for NovaPhy demos."""

import sys
import numpy as np

try:
    import polyscope as ps
    HAS_POLYSCOPE = True
except ImportError:
    HAS_POLYSCOPE = False

import novaphy
from novaphy.viz import SceneVisualizer


class DemoApp:
    """Base class for NovaPhy demos with optional Polyscope visualization."""

    def __init__(self, title="NovaPhy Demo", dt=1.0/120.0, ground_size=20.0):
        """Initializes shared demo runtime parameters.

        Args:
            title (str): Window/application title for the demo.
            dt (float): Fixed simulation step size in seconds.
            ground_size (float): Half-size of the rendered ground plane (m).
        """
        self.title = title
        self.dt = dt
        self.ground_size = ground_size
        self.world = None
        self.viz = None

    def build_scene(self):
        """Builds the simulation scene and assigns `self.world`.

        Raises:
            NotImplementedError: Must be implemented by subclasses.
        """
        raise NotImplementedError

    def run(self, headless_steps=0):
        """Run the demo.

        Args:
            headless_steps (int): If positive, runs this many simulation steps
                without creating a visualization window.

        Returns:
            None
        """
        self.build_scene()
        assert self.world is not None, "build_scene() must set self.world"

        if headless_steps > 0:
            for _ in range(headless_steps):
                self.world.step(self.dt)
            return

        if not HAS_POLYSCOPE:
            print("Polyscope not available. Running 500 steps headless...")
            for i in range(500):
                self.world.step(self.dt)
                if i % 100 == 0:
                    state = self.world.state
                    for j in range(self.world.model.num_bodies):
                        pos = state.transforms[j].position
                        print(f"  step {i}, body {j}: pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
            print("Done.")
            return

        # Initialize Polyscope
        ps.init()
        ps.set_program_name(self.title)
        ps.set_up_dir("y_up")
        ps.set_ground_plane_mode("shadow_only")

        # Create visualizer
        self.viz = SceneVisualizer(self.world, self.ground_size)

        # Simulation callback
        def callback():
            # Step physics
            self.world.step(self.dt)
            # Update meshes
            self.viz.update()

        ps.set_user_callback(callback)
        ps.show()
