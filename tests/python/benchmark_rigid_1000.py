import json
import time
from pathlib import Path

import numpy as np

import novaphy


def _serialize_phase_stats(phase_stats, limit=8):
    ranked = sorted(phase_stats, key=lambda stat: stat.avg_ms, reverse=True)
    return [
        {
            "name": stat.name,
            "last_ms": stat.last_ms,
            "avg_ms": stat.avg_ms,
            "max_ms": stat.max_ms,
            "samples": stat.samples,
        }
        for stat in ranked[:limit]
    ]


def _serialize_metrics(metrics):
    return {metric.name: metric.value for metric in metrics}


def run_benchmark(steps=300, dt=1.0 / 120.0, profile=False):
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0)
    half = np.array([0.2, 0.2, 0.2], dtype=np.float32)

    side = 10
    layers = 10
    count = side * side * layers
    for k in range(layers):
        for i in range(side):
            for j in range(side):
                body = novaphy.RigidBody.from_box(1.0, half)
                pos = np.array(
                    [i * 0.45 - 2.0, 0.5 + k * 0.45, j * 0.45 - 2.0], dtype=np.float32
                )
                idx = builder.add_body(body, novaphy.Transform.from_translation(pos))
                builder.add_shape(novaphy.CollisionShape.make_box(half, idx))

    world = novaphy.World(builder.build())
    if profile:
        world.performance_monitor.enabled = True

    start = time.perf_counter()
    for _ in range(steps):
        world.step(dt)
    elapsed = time.perf_counter() - start
    fps = steps / elapsed

    result = {
        "bodies": count,
        "steps": steps,
        "dt": dt,
        "elapsed_sec": elapsed,
        "fps": fps,
    }

    if profile:
        result["top_phase_stats"] = _serialize_phase_stats(
            world.performance_monitor.phase_stats()
        )
        result["last_frame_metrics"] = _serialize_metrics(
            world.performance_monitor.last_frame_metrics()
        )

    return result


if __name__ == "__main__":
    result = run_benchmark(profile=True)
    output = Path("build") / "benchmark_rigid_1000.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
