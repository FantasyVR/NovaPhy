import argparse
import csv
import json
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import re

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import novaphy

try:
    import polyscope as ps
    import polyscope.imgui as psim
    HAS_POLYSCOPE = True
except ImportError:
    ps = None
    psim = None
    HAS_POLYSCOPE = False

from novaphy.viz import SceneVisualizer


@dataclass
class DemoConfig:
    urdf_path: str = "demos/data/robot_two_link.urdf"
    usd_path: str = "demos/data/robot_env.usda"
    export_dir: str = "build/demo_robot_outputs"
    dt: float = 1.0 / 120.0
    steps: int = 600
    visual: bool = True
    steps_per_frame: int = 2
    ground_size: float = 8.0
    gravity_x: float = 0.0
    gravity_y: float = -9.81
    gravity_z: float = 0.0
    drive_amp: float = 2.0
    drive_freq: float = 1.2
    usd_only: bool = False
    usd_min_supported_version: float = 1.0
    usd_proxy_scale: float = 1.0

    @staticmethod
    def from_json(path: str) -> "DemoConfig":
        cfg = DemoConfig()
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        for k, v in payload.items():
            if hasattr(cfg, k):
                setattr(cfg, k, v)
        return cfg


def clone_shape(shape: novaphy.CollisionShape, body_offset: int) -> novaphy.CollisionShape:
    dst = novaphy.CollisionShape()
    dst.type = shape.type
    dst.local_transform = shape.local_transform
    dst.friction = float(shape.friction)
    dst.restitution = float(shape.restitution)
    dst.body_index = int(shape.body_index + body_offset) if shape.body_index >= 0 else -1
    dst.box_half_extents = np.array(shape.box_half_extents, dtype=np.float32)
    dst.sphere_radius = float(shape.sphere_radius)
    dst.plane_normal = np.array(shape.plane_normal, dtype=np.float32)
    dst.plane_offset = float(shape.plane_offset)
    return dst


def merge_models(
    robot_model: novaphy.Model, env_model: novaphy.Model
) -> Tuple[novaphy.Model, int, int]:
    builder = novaphy.ModelBuilder()

    robot_world = novaphy.World(robot_model)
    env_world = novaphy.World(env_model)
    robot_transforms = robot_world.state.transforms
    env_transforms = env_world.state.transforms

    for i, body in enumerate(robot_model.bodies):
        builder.add_body(body, robot_transforms[i])
    for s in robot_model.shapes:
        builder.add_shape(clone_shape(s, 0))

    body_offset = robot_model.num_bodies
    for i, body in enumerate(env_model.bodies):
        builder.add_body(body, env_transforms[i])
    for s in env_model.shapes:
        builder.add_shape(clone_shape(s, body_offset))

    return builder.build(), robot_model.num_bodies, env_model.num_bodies


def extract_materials_and_lights(stage: novaphy.UsdStageData) -> Tuple[List[str], List[str]]:
    materials = []
    lights = []
    for prim in stage.prims:
        if prim.material_binding:
            materials.append(prim.material_binding)
        if "Light" in prim.type_name:
            lights.append(prim.path)
    materials = sorted(set(materials))
    lights = sorted(set(lights))
    return materials, lights


def write_joint_trajectory_csv(path: Path, rows: List[Dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    keys = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def write_ee_pose_csv(path: Path, rows: List[Dict[str, float]]) -> None:
    write_joint_trajectory_csv(path, rows)


def _seed_free_joint_quaternions(art: novaphy.Articulation, q: np.ndarray) -> np.ndarray:
    if q.size == 0:
        return q
    for link_idx, joint in enumerate(art.joints):
        if joint.type == novaphy.JointType.Free:
            qi = art.q_start(link_idx)
            q[qi + 6] = 1.0
    return q


def _parse_tuple3(text: str, fallback: np.ndarray) -> np.ndarray:
    m = re.search(r"\(([-+0-9eE\.]+)\s*,\s*([-+0-9eE\.]+)\s*,\s*([-+0-9eE\.]+)\)", text)
    if not m:
        return fallback
    return np.array([float(m.group(1)), float(m.group(2)), float(m.group(3))], dtype=np.float32)


def _parse_quat_wxyz(text: str, fallback_xyzw: np.ndarray) -> np.ndarray:
    m = re.search(r"\(([-+0-9eE\.]+)\s*,\s*([-+0-9eE\.]+)\s*,\s*([-+0-9eE\.]+)\s*,\s*([-+0-9eE\.]+)\)", text)
    if not m:
        return fallback_xyzw
    w = float(m.group(1))
    x = float(m.group(2))
    y = float(m.group(3))
    z = float(m.group(4))
    return np.array([x, y, z, w], dtype=np.float32)


def _convert_up_axis(v: np.ndarray, up_axis: str) -> np.ndarray:
    axis = up_axis.upper()
    if axis == "Z":
        return np.array([v[0], v[2], v[1]], dtype=np.float32)
    return np.array(v, dtype=np.float32)


def _extract_balanced_block(text: str, open_brace_index: int) -> Tuple[int, int]:
    depth = 0
    for i in range(open_brace_index, len(text)):
        ch = text[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return open_brace_index, i
    return open_brace_index, len(text) - 1


def _build_usd_proxy_model(config: DemoConfig, usd_stage: novaphy.UsdStageData) -> novaphy.Model:
    text = Path(config.usd_path).read_text(encoding="utf-8")
    up_axis = usd_stage.up_axis if usd_stage.up_axis else "Y"
    builder = novaphy.ModelBuilder()
    builder.add_ground_plane(0.0)

    idx = 0
    body_count = 0
    while True:
        m = re.search(r'def\s+Xform\s+"([^"]+)"\s*\(', text[idx:])
        if not m:
            break
        name = m.group(1)
        start = idx + m.start()
        brace = text.find("{", start)
        if brace < 0:
            idx = start + 1
            continue
        b0, b1 = _extract_balanced_block(text, brace)
        block = text[start : b1 + 1]
        header = text[start:brace]
        idx = b1 + 1
        if "PhysicsRigidBodyAPI" not in header:
            continue

        trans_m = re.search(r'(?:double3|float3)\s+xformOp:translate\s*=\s*\([^\)]*\)', block)
        ori_m = re.search(r'quatf\s+xformOp:orient\s*=\s*\([^\)]*\)', block)
        den_m = re.search(r'float\s+physics:density\s*=\s*([-+0-9eE\.]+)', block)
        h_m = re.search(r'double\s+height\s*=\s*([-+0-9eE\.]+)', block)
        r_m = re.search(r'double\s+radius\s*=\s*([-+0-9eE\.]+)', block)

        pos = _parse_tuple3(trans_m.group(0), np.zeros(3, dtype=np.float32)) if trans_m else np.zeros(3, dtype=np.float32)
        pos = _convert_up_axis(pos, up_axis)
        rot = _parse_quat_wxyz(ori_m.group(0), np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)) if ori_m else np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        density = float(den_m.group(1)) if den_m else 1.0
        cap_h = float(h_m.group(1)) if h_m else 0.3
        cap_r = float(r_m.group(1)) if r_m else 0.08
        cap_h *= float(config.usd_proxy_scale)
        cap_r *= float(config.usd_proxy_scale)

        volume = np.pi * cap_r * cap_r * max(cap_h, 1e-4)
        mass = max(0.2, density * volume)
        body = novaphy.RigidBody.from_box(mass, np.array([cap_h * 0.5 + cap_r, cap_r, cap_r], dtype=np.float32))
        t = novaphy.Transform(pos, rot)
        bidx = builder.add_body(body, t)
        builder.add_shape(
            novaphy.CollisionShape.make_box(
                np.array([cap_h * 0.5 + cap_r, cap_r, cap_r], dtype=np.float32),
                bidx,
            )
        )
        body_count += 1

    if body_count == 0:
        for prim in usd_stage.prims:
            if prim.type_name == "Xform":
                body = novaphy.RigidBody.from_box(1.0, np.array([0.12, 0.08, 0.08], dtype=np.float32))
                pos = _convert_up_axis(np.array(prim.local_transform.position, dtype=np.float32), up_axis)
                t = novaphy.Transform.from_translation(pos)
                bidx = builder.add_body(body, t)
                builder.add_shape(novaphy.CollisionShape.make_box(np.array([0.12, 0.08, 0.08], dtype=np.float32), bidx))
                body_count += 1
                if body_count >= 24:
                    break

    return builder.build()


def _extract_stage_gravity(config: DemoConfig) -> np.ndarray:
    text = Path(config.usd_path).read_text(encoding="utf-8")
    dir_m = re.search(r'physics:gravityDirection\s*=\s*\([^\)]*\)', text)
    mag_m = re.search(r'physics:gravityMagnitude\s*=\s*([-+0-9eE\.]+)', text)
    up_m = re.search(r'upAxis\s*=\s*"([^"]+)"', text)
    gdir = _parse_tuple3(dir_m.group(0), np.array([0.0, -1.0, 0.0], dtype=np.float32)) if dir_m else np.array([0.0, -1.0, 0.0], dtype=np.float32)
    mag = float(mag_m.group(1)) if mag_m else 9.81
    up_axis = up_m.group(1) if up_m else "Y"
    gy = _convert_up_axis(gdir, up_axis)
    if np.linalg.norm(gy) > 1e-8:
        gy = gy / np.linalg.norm(gy)
    return gy * mag


def run_demo(config: DemoConfig) -> Dict[str, str]:
    urdf_path = Path(config.urdf_path)
    usd_path = Path(config.usd_path)
    urdf_parser = novaphy.UrdfParser()
    usd_importer = novaphy.OpenUsdImporter(float(config.usd_min_supported_version))
    builder = novaphy.SceneBuilderEngine()
    usd_stage = usd_importer.import_file(usd_path)
    materials, lights = extract_materials_and_lights(usd_stage)

    urdf_model = None
    robot_scene = None
    robot_body_count = 0
    if not config.usd_only:
        urdf_model = urdf_parser.parse_file(urdf_path)
        robot_scene = builder.build_from_urdf(urdf_model)
        robot_body_count = robot_scene.model.num_bodies

    env_scene = builder.build_from_openusd(usd_stage)
    if env_scene.model.num_bodies == 0:
        proxy_model = _build_usd_proxy_model(config, usd_stage)
        env_scene = novaphy.SceneBuildResult()
        env_scene.model = proxy_model
        env_scene.articulation = novaphy.Articulation()
        env_scene.warnings = ["usd_stage_fallback_proxy_model"]

    if robot_scene is not None:
        merged_model, robot_body_count, _ = merge_models(robot_scene.model, env_scene.model)
    else:
        merged_model = env_scene.model

    world = novaphy.World(merged_model)
    if config.usd_only:
        world.set_gravity(_extract_stage_gravity(config).astype(np.float32))
    else:
        world.set_gravity(
            np.array([config.gravity_x, config.gravity_y, config.gravity_z], dtype=np.float32)
        )

    exporter = novaphy.SimulationExporter()
    art = robot_scene.articulation if robot_scene is not None else novaphy.Articulation()
    has_art = len(art.joints) > 0 and len(art.bodies) > 0
    q = (
        _seed_free_joint_quaternions(art, np.zeros(art.total_q(), dtype=np.float32))
        if has_art
        else np.zeros(0, dtype=np.float32)
    )
    qd = np.zeros(art.total_qd(), dtype=np.float32) if has_art else np.zeros(0, dtype=np.float32)
    solver = novaphy.ArticulatedSolver() if has_art else None

    joint_rows: List[Dict[str, float]] = []
    ee_rows: List[Dict[str, float]] = []
    contact_rows: List[Dict[str, float]] = []
    t = 0.0
    run_flag = True

    def step_once():
        nonlocal t, q, qd, run_flag
        if not run_flag:
            return
        if has_art and solver is not None:
            tau = np.zeros(art.total_qd(), dtype=np.float32)
            for i in range(tau.shape[0]):
                tau[i] = config.drive_amp * np.sin(2.0 * np.pi * config.drive_freq * t + 0.3 * i)
            q, qd = solver.step(
                art,
                q,
                qd,
                tau,
                np.array([config.gravity_x, config.gravity_y, config.gravity_z], dtype=np.float32),
                config.dt,
            )
            fk = novaphy.forward_kinematics(art, q)
            if len(fk) > 0:
                ee = fk[-1]
                ee_rows.append(
                    {
                        "time": t,
                        "px": float(ee.position[0]),
                        "py": float(ee.position[1]),
                        "pz": float(ee.position[2]),
                        "qx": float(ee.rotation[0]),
                        "qy": float(ee.rotation[1]),
                        "qz": float(ee.rotation[2]),
                        "qw": float(ee.rotation[3]),
                    }
                )
            row = {"time": t}
            for i in range(q.shape[0]):
                row[f"q_{i}"] = float(q[i])
            for i in range(qd.shape[0]):
                row[f"qd_{i}"] = float(qd[i])
            joint_rows.append(row)

        world.step(config.dt)
        exporter.capture_frame(world, t)
        for c in world.contacts:
            contact_rows.append(
                {
                    "time": t,
                    "body_a": int(c.body_a),
                    "body_b": int(c.body_b),
                    "px": float(c.position[0]),
                    "py": float(c.position[1]),
                    "pz": float(c.position[2]),
                    "nx": float(c.normal[0]),
                    "ny": float(c.normal[1]),
                    "nz": float(c.normal[2]),
                    "penetration": float(c.penetration),
                }
            )
        t += config.dt

    if config.visual and HAS_POLYSCOPE:
        ps.init()
        ps.set_program_name("NovaPhy Robot Simulation Demo")
        ps.set_up_dir("y_up")
        ps.set_ground_plane_mode("shadow_only")
        viz = SceneVisualizer(world, config.ground_size)
        ee_cloud = ps.register_point_cloud("end_effector", np.array([[0.0, 0.0, 0.0]], dtype=np.float32))
        ee_cloud.set_radius(0.01, relative=False)
        frame_count = {"v": 0}

        def callback():
            nonlocal run_flag
            changed, run_flag = psim.Checkbox("run", run_flag)
            changed_dt, new_dt = psim.InputFloat("dt", config.dt)
            if changed_dt and new_dt > 0.0:
                config.dt = new_dt
            changed_amp, new_amp = psim.InputFloat("drive_amp", config.drive_amp)
            if changed_amp:
                config.drive_amp = new_amp
            changed_freq, new_freq = psim.InputFloat("drive_freq", config.drive_freq)
            if changed_freq:
                config.drive_freq = new_freq
            for _ in range(config.steps_per_frame):
                if frame_count["v"] >= config.steps:
                    run_flag = False
                    break
                step_once()
                frame_count["v"] += 1
            viz.update()
            if ee_rows:
                p = np.array([[ee_rows[-1]["px"], ee_rows[-1]["py"], ee_rows[-1]["pz"]]], dtype=np.float32)
                ps.get_point_cloud("end_effector").update_point_positions(p)

        ps.set_user_callback(callback)
        ps.show()
    else:
        for _ in range(config.steps):
            step_once()

    export_dir = Path(config.export_dir)
    export_dir.mkdir(parents=True, exist_ok=True)
    keyframe_csv = export_dir / "keyframes.csv"
    collision_csv = export_dir / "collisions.csv"
    reaction_csv = export_dir / "constraint_reactions.csv"
    joint_csv = export_dir / "joint_trajectory.csv"
    ee_csv = export_dir / "end_effector_pose.csv"
    out_urdf = export_dir / "robot_out.urdf"
    out_usda = export_dir / "sim_anim.usda"
    config_json = export_dir / "run_config.json"
    scene_meta = export_dir / "scene_meta.json"
    contact_trace_csv = export_dir / "contact_trace.csv"

    exporter.write_keyframes_csv(keyframe_csv)
    exporter.write_collision_log_csv(collision_csv)
    exporter.write_constraint_reactions_csv(reaction_csv)
    if urdf_model is not None:
        exporter.write_urdf(urdf_model, out_urdf)
    exporter.write_openusd_animation_layer(out_usda)
    write_joint_trajectory_csv(joint_csv, joint_rows)
    write_ee_pose_csv(ee_csv, ee_rows)
    write_joint_trajectory_csv(contact_trace_csv, contact_rows)
    config_json.write_text(json.dumps(asdict(config), indent=2), encoding="utf-8")
    scene_meta.write_text(
        json.dumps(
            {
                "robot_name": urdf_model.name if urdf_model is not None else "usd_only_robot",
                "robot_links": len(urdf_model.links) if urdf_model is not None else 0,
                "robot_joints": len(urdf_model.joints) if urdf_model is not None else 0,
                "robot_body_count": robot_body_count,
                "stage_prims": len(usd_stage.prims),
                "materials": materials,
                "lights": lights,
                "usd_only": config.usd_only,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    return {
        "keyframes_csv": str(keyframe_csv),
        "collisions_csv": str(collision_csv),
        "constraint_reactions_csv": str(reaction_csv),
        "joint_trajectory_csv": str(joint_csv),
        "end_effector_pose_csv": str(ee_csv),
        "contact_trace_csv": str(contact_trace_csv),
        "urdf_out": str(out_urdf) if urdf_model is not None else "",
        "usd_anim_out": str(out_usda),
        "run_config": str(config_json),
        "scene_meta": str(scene_meta),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser("NovaPhy robot simulation pipeline demo")
    parser.add_argument("--config", type=str, default=None)
    parser.add_argument("--urdf", type=str, default=None)
    parser.add_argument("--usd", type=str, default=None)
    parser.add_argument("--export-dir", type=str, default=None)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--dt", type=float, default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--usd-only", action="store_true")
    parser.add_argument("--usd-min-version", type=float, default=None)
    parser.add_argument("--usd-proxy-scale", type=float, default=None)
    return parser.parse_args()


def build_config_from_args(args: argparse.Namespace) -> DemoConfig:
    cfg = DemoConfig()
    if args.config:
        cfg = DemoConfig.from_json(args.config)
    if args.urdf:
        cfg.urdf_path = args.urdf
    if args.usd:
        cfg.usd_path = args.usd
    if args.export_dir:
        cfg.export_dir = args.export_dir
    if args.steps is not None:
        cfg.steps = args.steps
    if args.dt is not None:
        cfg.dt = args.dt
    if args.headless:
        cfg.visual = False
    if args.usd_only:
        cfg.usd_only = True
    if args.usd_min_version is not None:
        cfg.usd_min_supported_version = args.usd_min_version
    if args.usd_proxy_scale is not None:
        cfg.usd_proxy_scale = args.usd_proxy_scale
    return cfg


def main() -> None:
    args = parse_args()
    cfg = build_config_from_args(args)
    outputs = run_demo(cfg)
    print(json.dumps(outputs, indent=2))


if __name__ == "__main__":
    main()
