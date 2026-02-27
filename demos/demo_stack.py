"""Demo: 3 boxes falling and stacking on a ground plane.

This is the first NovaPhy visual demo, demonstrating:
- Gravity
- Box-plane collision
- Box-box collision and stacking
- Sequential Impulse contact solver with warm starting
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import novaphy
from demos.demo_utils import DemoApp


class BoxStackDemo(DemoApp):
    def __init__(self):
        """Initializes the box stacking demo configuration."""
        super().__init__(title="NovaPhy - Box Stack", dt=1.0/120.0)

    def build_scene(self):
        """Builds a 3-box stack on a static ground plane.

        Returns:
            None
        """
        builder = novaphy.ModelBuilder()

        # Ground plane
        builder.add_ground_plane(y=0.0, friction=0.6, restitution=0.0)

        # Stack of 3 boxes
        half = np.array([0.5, 0.5, 0.5], dtype=np.float32)

        for i in range(3):
            body = novaphy.RigidBody.from_box(1.0, half)
            y = 1.5 + i * 1.2  # stack with small gap
            t = novaphy.Transform.from_translation(
                np.array([0.0, y, 0.0], dtype=np.float32))
            body_idx = builder.add_body(body, t)

            shape = novaphy.CollisionShape.make_box(
                half, body_idx, novaphy.Transform.identity(), 0.5, 0.0)
            builder.add_shape(shape)

        model = builder.build()

        settings = novaphy.SolverSettings()
        settings.velocity_iterations = 30
        settings.warm_starting = True

        self.world = novaphy.World(model, settings)


if __name__ == "__main__":
    demo = BoxStackDemo()
    demo.run()
