"""Demo: 20 thin box dominoes in a line.

Demonstrates chain reaction: first domino is pushed, toppling all others.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import novaphy
from demos.demo_utils import DemoApp


class DominoesDemo(DemoApp):
    def __init__(self):
        """Initializes the domino chain reaction demo configuration."""
        super().__init__(title="NovaPhy - Dominoes", dt=1.0/120.0)

    def build_scene(self):
        """Builds a line of dominoes and applies an initial angular kick.

        Returns:
            None
        """
        builder = novaphy.ModelBuilder()
        builder.add_ground_plane(y=0.0, friction=0.5)

        # Thin domino dimensions
        domino_half = np.array([0.1, 0.5, 0.3], dtype=np.float32)
        num_dominoes = 20
        spacing = 0.6  # distance between domino centers along x

        for i in range(num_dominoes):
            body = novaphy.RigidBody.from_box(0.3, domino_half)
            x = i * spacing
            t = novaphy.Transform.from_translation(
                np.array([x, 0.5, 0.0], dtype=np.float32))
            idx = builder.add_body(body, t)

            shape = novaphy.CollisionShape.make_box(
                domino_half, idx,
                novaphy.Transform.identity(),
                friction=0.4, restitution=0.1)
            builder.add_shape(shape)

        model = builder.build()
        settings = novaphy.SolverSettings()
        settings.velocity_iterations = 15
        self.world = novaphy.World(model, settings)

        # Push the first domino with an initial angular velocity (tilt forward)
        self.world.state.set_angular_velocity(
            0, np.array([0.0, 0.0, -3.0], dtype=np.float32))


if __name__ == "__main__":
    demo = DominoesDemo()
    demo.run()
