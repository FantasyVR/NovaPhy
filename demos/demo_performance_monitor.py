"""Headless demo for NovaPhy runtime performance profiling.

This demo runs either a rigid-body or fluid benchmark scene with the built-in
performance monitor enabled, prints the slowest phases, and optionally exports
summary and Perfetto trace JSON files.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import novaphy


@dataclass
class DemoConfig:
    scene: str = "rigid"
    dt: float = 1.0 / 120.0
    warmup_steps: int = 30
    measured_steps: int = 120
    trace_enabled: bool = True
    trace_frame_capacity: int = 120
    top_n: int = 8
    output_dir: str = "build/demo_performance_monitor"


def _build_rigid_world() -> novaphy.World:
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0, friction=0.6, restitution=0.0)

    half = np.array([0.2, 0.2, 0.2], dtype=np.float32)
    side = 10
    layers = 10

    for k in range(layers):
        for i in range(side):
            for j in range(side):
                body = novaphy.RigidBody.from_box(1.0, half)
                pos = np.array(
                    [i * 0.45 - 2.0, 0.5 + k * 0.45, j * 0.45 - 2.0],
                    dtype=np.float32,
                )
                idx = builder.add_body(
                    body, novaphy.Transform.from_translation(pos)
                )
                builder.add_shape(novaphy.CollisionShape.make_box(half, idx))

    settings = novaphy.SolverSettings()
    settings.velocity_iterations = 30
    settings.warm_starting = True
    return novaphy.World(builder.build(), settings)


def _build_fluid_world() -> novaphy.FluidWorld:
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0)

    block = novaphy.FluidBlockDef()
    block.lower = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    block.upper = np.array([0.4, 0.6, 0.4], dtype=np.float32)
    block.particle_spacing = 0.025
    block.rest_density = 1000.0

    pbf = novaphy.PBFSettings()
    pbf.rest_density = 1000.0
    pbf.kernel_radius = block.particle_spacing * 4.0
    pbf.solver_iterations = 6
    pbf.xsph_viscosity = 0.1
    pbf.epsilon = 100.0
    pbf.particle_radius = block.particle_spacing * 0.5
    pbf.use_domain_bounds = True
    pbf.domain_lower = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    pbf.domain_upper = np.array([1.0, 1.0, 0.4], dtype=np.float32)

    return novaphy.FluidWorld(builder.build(), [block], novaphy.SolverSettings(), pbf)


def _phase_stats_payload(stats: List[novaphy.PerformancePhaseStat]) -> List[Dict[str, float]]:
    return [
        {
            "name": stat.name,
            "last_ms": stat.last_ms,
            "avg_ms": stat.avg_ms,
            "max_ms": stat.max_ms,
            "samples": stat.samples,
        }
        for stat in stats
    ]


def _metric_payload(metrics: List[novaphy.PerformanceMetric]) -> Dict[str, float]:
    return {metric.name: metric.value for metric in metrics}


def _make_summary(
    config: DemoConfig,
    elapsed_sec: float,
    phase_stats: List[novaphy.PerformancePhaseStat],
    metrics: List[novaphy.PerformanceMetric],
) -> Dict[str, object]:
    ranked = sorted(phase_stats, key=lambda stat: stat.avg_ms, reverse=True)
    return {
        "config": asdict(config),
        "elapsed_sec": elapsed_sec,
        "fps": config.measured_steps / elapsed_sec if elapsed_sec > 0 else 0.0,
        "top_phase_stats": _phase_stats_payload(ranked[: config.top_n]),
        "all_phase_stats": _phase_stats_payload(ranked),
        "last_frame_metrics": _metric_payload(metrics),
    }


def _print_summary(summary: Dict[str, object]) -> None:
    print(
        f"Scene={summary['config']['scene']} "
        f"steps={summary['config']['measured_steps']} "
        f"fps={summary['fps']:.2f}"
    )

    print("Top phases:")
    for stat in summary["top_phase_stats"]:
        print(
            f"  {stat['name']:<36} "
            f"avg={stat['avg_ms']:.4f} ms "
            f"max={stat['max_ms']:.4f} ms"
        )

    print("Last-frame metrics:")
    for name, value in summary["last_frame_metrics"].items():
        print(f"  {name}={value}")


def run_demo(config: DemoConfig) -> Dict[str, str]:
    if config.scene == "rigid":
        world = _build_rigid_world()
    elif config.scene == "fluid":
        world = _build_fluid_world()
    else:
        raise ValueError(f"Unsupported scene: {config.scene}")

    output_dir = Path(config.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    monitor = world.performance_monitor
    monitor.enabled = True
    monitor.trace_enabled = config.trace_enabled
    monitor.trace_frame_capacity = config.trace_frame_capacity

    for _ in range(config.warmup_steps):
        world.step(config.dt)

    monitor.reset()
    monitor.enabled = True
    monitor.trace_enabled = config.trace_enabled
    monitor.trace_frame_capacity = config.trace_frame_capacity

    start = time.perf_counter()
    for _ in range(config.measured_steps):
        world.step(config.dt)
    elapsed_sec = time.perf_counter() - start

    phase_stats = monitor.phase_stats()
    metrics = monitor.last_frame_metrics()
    summary = _make_summary(config, elapsed_sec, phase_stats, metrics)
    _print_summary(summary)

    summary_path = output_dir / f"{config.scene}_profile_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    outputs = {"summary_json": str(summary_path)}
    if config.trace_enabled:
        trace_path = output_dir / f"{config.scene}_profile_trace.json"
        monitor.write_trace_json(trace_path)
        outputs["trace_json"] = str(trace_path)

    return outputs


def _parse_args(argv: List[str]) -> DemoConfig:
    parser = argparse.ArgumentParser(description="Run NovaPhy profiling demo.")
    parser.add_argument(
        "--scene",
        choices=["rigid", "fluid"],
        default="rigid",
        help="Select which profiling scene to run.",
    )
    parser.add_argument("--dt", type=float, default=1.0 / 120.0)
    parser.add_argument("--warmup-steps", type=int, default=30)
    parser.add_argument("--measured-steps", type=int, default=120)
    parser.add_argument(
        "--no-trace",
        action="store_true",
        help="Disable Perfetto trace export.",
    )
    parser.add_argument("--trace-frame-capacity", type=int, default=120)
    parser.add_argument("--top-n", type=int, default=8)
    parser.add_argument(
        "--output-dir",
        default="build/demo_performance_monitor",
        help="Directory for summary and trace outputs.",
    )
    args = parser.parse_args(argv)
    return DemoConfig(
        scene=args.scene,
        dt=args.dt,
        warmup_steps=args.warmup_steps,
        measured_steps=args.measured_steps,
        trace_enabled=not args.no_trace,
        trace_frame_capacity=args.trace_frame_capacity,
        top_n=args.top_n,
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    run_demo(_parse_args(sys.argv[1:]))
