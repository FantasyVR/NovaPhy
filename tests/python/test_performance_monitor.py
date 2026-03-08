import json
from pathlib import Path

import numpy as np
import novaphy


def _phase_map(monitor):
    return {stat.name: stat for stat in monitor.phase_stats()}


def _metric_map(monitor):
    return {metric.name: metric.value for metric in monitor.last_frame_metrics()}


def _make_contact_world():
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0)

    half = np.array([0.5, 0.5, 0.5], dtype=np.float32)
    for y in (0.5, 1.5):
        body = novaphy.RigidBody.from_box(1.0, half)
        transform = novaphy.Transform.from_translation(
            np.array([0.0, y, 0.0], dtype=np.float32)
        )
        body_index = builder.add_body(body, transform)
        builder.add_shape(novaphy.CollisionShape.make_box(half, body_index))

    settings = novaphy.SolverSettings()
    settings.velocity_iterations = 10
    return novaphy.World(builder.build(), settings)


def _make_profiled_fluid_world():
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(y=0.0)

    block = novaphy.FluidBlockDef()
    block.lower = np.array([0.0, 0.8, 0.0], dtype=np.float32)
    block.upper = np.array([0.15, 0.95, 0.15], dtype=np.float32)
    block.particle_spacing = 0.05

    pbf = novaphy.PBFSettings()
    pbf.kernel_radius = 0.2
    pbf.solver_iterations = 2

    return novaphy.FluidWorld(builder.build(), [block], novaphy.SolverSettings(), pbf)


def test_performance_monitor_disabled_by_default():
    world = _make_contact_world()
    monitor = world.performance_monitor

    assert monitor.enabled is False
    assert monitor.trace_enabled is False
    assert monitor.frame_count == 0
    assert monitor.last_frame_total_ms == 0.0
    assert monitor.phase_stats() == []
    assert monitor.last_frame_metrics() == []

    world.step(1.0 / 120.0)

    assert monitor.frame_count == 0
    assert monitor.phase_stats() == []
    assert monitor.last_frame_metrics() == []
    assert monitor.last_frame_total_ms == 0.0


def test_world_monitor_reports_phases_and_metrics():
    world = _make_contact_world()
    monitor = world.performance_monitor
    monitor.enabled = True

    world.step(1.0 / 120.0)

    phases = _phase_map(monitor)
    assert "world.total" in phases
    assert "world.broadphase.sap" in phases
    assert "world.solver.total" in phases
    assert phases["world.total"].last_ms > 0.0
    assert phases["world.broadphase.sap"].last_ms > 0.0
    assert phases["world.solver.total"].last_ms > 0.0

    metrics = _metric_map(monitor)
    assert metrics["bodies"] == 2.0
    assert metrics["dynamic_bodies"] == 2.0
    assert metrics["shapes"] == 3.0
    assert metrics["candidate_pairs"] >= 1.0
    assert metrics["contacts"] >= 1.0
    assert metrics["solver_iterations"] == 10.0
    assert monitor.frame_count == 1
    assert monitor.last_frame_total_ms > 0.0


def test_performance_monitor_reset_clears_state(tmp_path):
    world = _make_contact_world()
    monitor = world.performance_monitor
    monitor.enabled = True
    monitor.trace_enabled = True

    world.step(1.0 / 120.0)
    assert monitor.phase_stats()
    assert monitor.last_frame_metrics()
    assert monitor.frame_count == 1

    monitor.reset()

    assert monitor.phase_stats() == []
    assert monitor.last_frame_metrics() == []
    assert monitor.frame_count == 0
    assert monitor.last_frame_total_ms == 0.0

    output_path = Path(tmp_path) / "trace_after_reset.json"
    monitor.write_trace_json(str(output_path))
    trace_data = json.loads(output_path.read_text(encoding="utf-8"))
    assert trace_data["traceEvents"] == []


def test_fluid_world_monitor_reports_rigid_and_fluid_phases():
    world = _make_profiled_fluid_world()
    monitor = world.performance_monitor
    monitor.enabled = True

    world.step(1.0 / 60.0)

    phases = _phase_map(monitor)
    assert "fluid.total" in phases
    assert "fluid.pbf.total" in phases
    assert "fluid.rigid_step" in phases
    assert "world.total" in phases
    assert "world.broadphase.sap" in phases

    metrics = _metric_map(monitor)
    assert metrics["fluid_particles"] > 0.0
    assert metrics["boundary_particles"] >= 0.0
    assert metrics["pbf_solver_iterations"] == 2.0
    assert abs(metrics["kernel_radius"] - 0.2) < 1e-6


def test_write_trace_json_exports_duration_and_counter_events(tmp_path):
    world = _make_contact_world()
    monitor = world.performance_monitor
    monitor.enabled = True
    monitor.trace_enabled = True
    monitor.trace_frame_capacity = 4

    world.step(1.0 / 120.0)

    output_path = Path(tmp_path) / "perf_trace.json"
    monitor.write_trace_json(str(output_path))

    data = json.loads(output_path.read_text(encoding="utf-8"))
    assert "traceEvents" in data
    assert data["traceEvents"]
    assert any(event["ph"] == "X" for event in data["traceEvents"])
    assert any(event["ph"] == "C" for event in data["traceEvents"])
