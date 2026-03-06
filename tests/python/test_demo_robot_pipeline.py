import importlib.util
from pathlib import Path


def _load_demo_module():
    root = Path(__file__).resolve().parents[2]
    demo_path = root / "demos" / "demo_robot_sim_pipeline.py"
    spec = importlib.util.spec_from_file_location("demo_robot_sim_pipeline", demo_path)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None and spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_robot_demo_headless_outputs(tmp_path):
    module = _load_demo_module()
    cfg = module.DemoConfig()
    cfg.visual = False
    cfg.steps = 20
    cfg.export_dir = str(tmp_path / "outputs")
    outputs = module.run_demo(cfg)

    for key in [
        "keyframes_csv",
        "collisions_csv",
        "joint_trajectory_csv",
        "end_effector_pose_csv",
        "contact_trace_csv",
        "urdf_out",
        "usd_anim_out",
    ]:
        assert Path(outputs[key]).exists()

    joint_traj = Path(outputs["joint_trajectory_csv"]).read_text(encoding="utf-8").lower()
    ee_pose = Path(outputs["end_effector_pose_csv"]).read_text(encoding="utf-8").lower()
    assert "nan" not in joint_traj
    assert "nan" not in ee_pose

    content = Path(outputs["scene_meta"]).read_text(encoding="utf-8")
    assert "robot_links" in content


def test_robot_demo_ant_usda_headless(tmp_path):
    module = _load_demo_module()
    cfg = module.DemoConfig()
    cfg.visual = False
    cfg.usd_only = True
    cfg.steps = 30
    cfg.usd_path = "demos/data/ant.usda"
    cfg.export_dir = str(tmp_path / "ant_outputs")
    outputs = module.run_demo(cfg)

    assert Path(outputs["keyframes_csv"]).exists()
    assert Path(outputs["usd_anim_out"]).exists()
    meta = Path(outputs["scene_meta"]).read_text(encoding="utf-8")
    assert "\"usd_only\": true" in meta
