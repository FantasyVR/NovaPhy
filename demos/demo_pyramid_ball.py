"""Demo: 4-3-2-1 box pyramid on a ground plane with a sphere projectile.

Demonstrates stable stacking with multiple contact constraints,
and dynamic collision response when a sphere hits the pyramid.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import novaphy
from demos.demo_utils import DemoApp


class PyramidDemo(DemoApp):
    def __init__(self):
        """Initializes the pyramid stacking demo configuration."""
        super().__init__(title="NovaPhy - Pyramid", dt=1.0/120.0)

        self.ball_idx = None  # Will be set in build_scene

    def build_scene(self):
        """Builds a 4-3-2-1 box pyramid over a ground plane.

        Returns:
            None
        """
        builder = novaphy.ModelBuilder()
        builder.add_ground_plane(y=0.0, friction=0.6)

        half = np.array([0.5, 0.5, 0.5], dtype=np.float32)
        spacing = 1.05  # slight gap between boxes

        # Build pyramid layers: 4-3-2-1
        layers = [4, 3, 2, 1]
        for layer_idx, count in enumerate(layers):
            y = 0.5 + layer_idx * 1.0
            # Center the row
            start_x = -(count - 1) * spacing / 2.0
            for i in range(count):
                x = start_x + i * spacing
                body = novaphy.RigidBody.from_box(1.0, half)
                t = novaphy.Transform.from_translation(
                    np.array([x, y, 0.0], dtype=np.float32))
                idx = builder.add_body(body, t)

                shape = novaphy.CollisionShape.make_box(
                    half, idx, novaphy.Transform.identity(), 0.5, 0.0)
                builder.add_shape(shape)

        # Sphere projectile to hit the pyramid
        sphere_radius = 0.4
        sphere_mass = 3.0
        sphere_body = novaphy.RigidBody.from_sphere(sphere_mass, sphere_radius)
        sphere_t = novaphy.Transform.from_translation(
            np.array([-4.0, 0.4, 0.0], dtype=np.float32))
        self.ball_idx = builder.add_body(sphere_body, sphere_t)

        sphere_shape = novaphy.CollisionShape.make_sphere(
            sphere_radius, self.ball_idx,
            novaphy.Transform.identity())
        builder.add_shape(sphere_shape)

        model = builder.build()
        settings = novaphy.SolverSettings()
        settings.velocity_iterations = 30
        settings.warm_starting = True

        # Sleep mechanism settings to prevent drift
        settings.sleep_enabled = True
        settings.sleep_energy_threshold = 0.01  # Universal energy threshold
        settings.sleep_time_required = 0.5      # 0.5 seconds below threshold
        settings.sleep_ema_alpha = 0.8          # EMA smoothing factor

        self.world = novaphy.World(model, settings)

        # Give the ball an initial velocity toward the pyramid
        self.world.state.set_linear_velocity(
            self.ball_idx, np.array([5.0, 0, 0.0], dtype=np.float32))


if __name__ == "__main__":
    demo = PyramidDemo()
    demo.run()
