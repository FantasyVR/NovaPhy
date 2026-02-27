"""Demo: 3 boxes on a 30-degree ramp with different friction coefficients.

Demonstrates Coulomb friction: high-friction box stays put, medium slides slowly,
low-friction box slides off quickly.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import novaphy
from demos.demo_utils import DemoApp


class FrictionRampDemo(DemoApp):
    def __init__(self):
        """Initializes the friction ramp demo configuration."""
        super().__init__(title="NovaPhy - Friction Ramp", dt=1.0/120.0)

    def build_scene(self):
        """Builds an inclined ramp scene with three friction variants.

        Returns:
            None
        """
        builder = novaphy.ModelBuilder()
        builder.add_ground_plane(y=-2.0, friction=0.5)

        # Ramp: a large thin box tilted 30 degrees
        ramp_half = np.array([5.0, 0.1, 3.0], dtype=np.float32)
        ramp_body = novaphy.RigidBody.make_static()
        angle = np.radians(30.0)
        ramp_t = novaphy.Transform.from_axis_angle(
            np.array([0, 0, 1], dtype=np.float32), angle)
        ramp_t.position = np.array([0.0, 1.5, 0.0], dtype=np.float32)
        ramp_idx = builder.add_body(ramp_body, ramp_t)

        ramp_shape = novaphy.CollisionShape.make_box(
            ramp_half, ramp_idx,
            novaphy.Transform.identity(),
            friction=0.8, restitution=0.0)
        builder.add_shape(ramp_shape)

        # Three boxes with different friction
        box_half = np.array([0.3, 0.3, 0.3], dtype=np.float32)
        frictions = [0.8, 0.4, 0.1]
        colors_desc = ["high", "medium", "low"]

        for i, (fric, desc) in enumerate(zip(frictions, colors_desc)):
            body = novaphy.RigidBody.from_box(1.0, box_half)

            # Place boxes on the ramp surface
            # Ramp surface at angle, offset each box in z
            ramp_up = np.array([-np.sin(angle), np.cos(angle), 0], dtype=np.float32)
            ramp_along = np.array([np.cos(angle), np.sin(angle), 0], dtype=np.float32)
            pos = (np.array([0.0, 1.5, 0.0], dtype=np.float32) +
                   ramp_up * 0.5 +
                   ramp_along * 1.0 +
                   np.array([0, 0, (i - 1) * 1.0], dtype=np.float32))

            t = novaphy.Transform.from_translation(pos.astype(np.float32))
            idx = builder.add_body(body, t)

            shape = novaphy.CollisionShape.make_box(
                box_half, idx,
                novaphy.Transform.identity(),
                friction=fric, restitution=0.0)
            builder.add_shape(shape)

        model = builder.build()
        settings = novaphy.SolverSettings()
        settings.velocity_iterations = 15
        self.world = novaphy.World(model, settings)


if __name__ == "__main__":
    demo = FrictionRampDemo()
    demo.run()
