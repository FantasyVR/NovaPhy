"""Demo: 5x5 box wall hit by a sphere projectile.

Demonstrates dynamic collision response with many interacting bodies.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import novaphy
from demos.demo_utils import DemoApp


class WallBreakDemo(DemoApp):
    def __init__(self):
        """Initializes the projectile-versus-wall demo configuration."""
        super().__init__(title="NovaPhy - Wall Break", dt=1.0/120.0)

    def build_scene(self):
        """Builds a box wall and launches a sphere projectile at it.

        Returns:
            None
        """
        builder = novaphy.ModelBuilder()
        builder.add_ground_plane(y=0.0, friction=0.5)

        # Build a 5x5 wall of boxes
        box_half = np.array([0.4, 0.4, 0.4], dtype=np.float32)
        wall_x = 3.0  # wall position along x
        rows = 5
        cols = 5

        for row in range(rows):
            for col in range(cols):
                y = 0.4 + row * 0.8
                z = (col - cols // 2) * 0.85
                body = novaphy.RigidBody.from_box(0.5, box_half)
                t = novaphy.Transform.from_translation(
                    np.array([wall_x, y, z], dtype=np.float32))
                idx = builder.add_body(body, t)

                shape = novaphy.CollisionShape.make_box(
                    box_half, idx,
                    novaphy.Transform.identity(),
                    friction=0.4, restitution=0.1)
                builder.add_shape(shape)

        # Sphere projectile
        sphere_radius = 0.5
        sphere_body = novaphy.RigidBody.from_sphere(5.0, sphere_radius)
        sphere_t = novaphy.Transform.from_translation(
            np.array([-3.0, 1.5, 0.0], dtype=np.float32))
        sphere_idx = builder.add_body(sphere_body, sphere_t)

        sphere_shape = novaphy.CollisionShape.make_sphere(
            sphere_radius, sphere_idx,
            novaphy.Transform.identity(),
            friction=0.3, restitution=0.3)
        builder.add_shape(sphere_shape)

        model = builder.build()
        settings = novaphy.SolverSettings()
        settings.velocity_iterations = 15
        self.world = novaphy.World(model, settings)

        # Give the sphere an initial velocity toward the wall
        self.world.state.set_linear_velocity(
            sphere_idx, np.array([15.0, 2.0, 0.0], dtype=np.float32))


if __name__ == "__main__":
    demo = WallBreakDemo()
    demo.run()
