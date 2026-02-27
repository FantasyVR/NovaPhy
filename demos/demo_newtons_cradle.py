"""Demo: Newton's cradle with 5 spheres.

Demonstrates elastic collision with high restitution (1.0).
The spheres are placed in a line. The leftmost sphere is pulled back
and released.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import novaphy
from demos.demo_utils import DemoApp


class NewtonsCradleDemo(DemoApp):
    def __init__(self):
        """Initializes the Newton's cradle demo configuration."""
        super().__init__(title="NovaPhy - Newton's Cradle", dt=1.0/120.0)

    def build_scene(self):
        """Builds a 5-sphere cradle with near-elastic collisions.

        Returns:
            None
        """
        builder = novaphy.ModelBuilder()
        builder.add_ground_plane(y=-2.0, friction=0.3)

        radius = 0.5
        num_spheres = 5
        spacing = radius * 2.0  # touching

        for i in range(num_spheres):
            body = novaphy.RigidBody.from_sphere(1.0, radius)
            x = (i - num_spheres // 2) * spacing
            t = novaphy.Transform.from_translation(
                np.array([x, 0.0, 0.0], dtype=np.float32))
            idx = builder.add_body(body, t)

            shape = novaphy.CollisionShape.make_sphere(
                radius, idx,
                novaphy.Transform.identity(),
                friction=0.0, restitution=1.0)
            builder.add_shape(shape)

        model = builder.build()
        settings = novaphy.SolverSettings()
        settings.velocity_iterations = 20
        self.world = novaphy.World(model, settings)
        self.world.set_gravity(np.array([0, 0, 0], dtype=np.float32))

        # Give the first sphere an initial velocity
        self.world.state.set_linear_velocity(
            0, np.array([5.0, 0.0, 0.0], dtype=np.float32))


if __name__ == "__main__":
    demo = NewtonsCradleDemo()
    demo.run()
