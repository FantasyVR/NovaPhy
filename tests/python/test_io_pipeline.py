import numpy as np

import novaphy


URDF_SAMPLE = """<robot name="two_link">
  <link name="base">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <link name="tip">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <collision>
      <origin xyz="0 0.5 0" rpy="0 0 0"/>
      <geometry><sphere radius="0.2"/></geometry>
    </collision>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <origin xyz="0 1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
  </joint>
</robot>
"""


URDF_UNORDERED_LINKS_SAMPLE = """<robot name="unordered_two_link">
  <link name="tip">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <collision>
      <origin xyz="0 0.5 0" rpy="0 0 0"/>
      <geometry><sphere radius="0.2"/></geometry>
    </collision>
  </link>
  <link name="base">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="1 1 1"/></geometry>
    </collision>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <origin xyz="0 1 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
  </joint>
</robot>
"""


USDA_SAMPLE = """#usda 21.08
(
    defaultPrim = "World"
    upAxis = "Y"
    metersPerUnit = 1
)

def Xform "World"
{
    def Xform "RigidA"
    {
        double3 xformOp:translate = (0, 2, 0)
        quatf xformOp:orient = (1, 0, 0, 0)
        float physics:mass = 3
        float3 novaphy:boxHalfExtents = (0.5, 0.5, 0.5)
    }
}
"""


def test_urdf_parse_build_and_write(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(URDF_SAMPLE, encoding="utf-8")

    parser = novaphy.UrdfParser()
    model_data = parser.parse_file(str(urdf_path))

    assert model_data.name == "two_link"
    assert len(model_data.links) == 2
    assert len(model_data.joints) == 1
    assert model_data.links[0].collisions[0].geometry.type == novaphy.UrdfGeometryType.Box

    builder = novaphy.SceneBuilderEngine()
    build_result = builder.build_from_urdf(model_data)
    assert build_result.model.num_bodies == 2
    assert build_result.model.num_shapes >= 2

    output_urdf = tmp_path / "robot_out.urdf"
    parser.write_file(model_data, str(output_urdf))
    assert output_urdf.exists()
    assert "two_link" in output_urdf.read_text(encoding="utf-8")


def test_usd_import_build_and_sim_export(tmp_path):
    usd_path = tmp_path / "scene.usda"
    usd_path.write_text(USDA_SAMPLE, encoding="utf-8")

    importer = novaphy.OpenUsdImporter()
    stage = importer.import_file(str(usd_path))

    assert stage.up_axis == "Y"
    assert len(stage.prims) >= 2
    rigid = [p for p in stage.prims if p.name == "RigidA"][0]
    np.testing.assert_allclose(rigid.local_transform.position, [0.0, 2.0, 0.0], atol=1e-6)

    builder = novaphy.SceneBuilderEngine()
    result = builder.build_from_openusd(stage)
    assert result.model.num_bodies >= 1
    world = novaphy.World(result.model)

    exporter = novaphy.SimulationExporter()
    dt = 1.0 / 120.0
    for i in range(5):
        world.step(dt)
        exporter.capture_frame(world, (i + 1) * dt)

    key_csv = tmp_path / "keyframes.csv"
    col_csv = tmp_path / "collisions.csv"
    usda_anim = tmp_path / "anim.usda"

    exporter.write_keyframes_csv(str(key_csv))
    exporter.write_collision_log_csv(str(col_csv))
    exporter.write_openusd_animation_layer(str(usda_anim))

    assert key_csv.exists()
    assert col_csv.exists()
    assert usda_anim.exists()
    assert "xformOp:translate.timeSamples" in usda_anim.read_text(encoding="utf-8")


def test_urdf_articulation_is_topologically_ordered(tmp_path):
    urdf_path = tmp_path / "unordered_robot.urdf"
    urdf_path.write_text(URDF_UNORDERED_LINKS_SAMPLE, encoding="utf-8")

    parser = novaphy.UrdfParser()
    model_data = parser.parse_file(str(urdf_path))
    assert [link.name for link in model_data.links] == ["tip", "base"]

    builder = novaphy.SceneBuilderEngine()
    build_result = builder.build_from_urdf(model_data)
    joints = build_result.articulation.joints
    assert len(joints) == 2

    root_indices = [idx for idx, joint in enumerate(joints) if joint.parent < 0]
    child_indices = [idx for idx, joint in enumerate(joints) if joint.parent >= 0]
    assert len(root_indices) == 1
    assert len(child_indices) == 1
    assert joints[child_indices[0]].parent == root_indices[0]
    assert root_indices[0] < child_indices[0]


def test_feature_completeness_checker():
    checker = novaphy.FeatureCompletenessChecker()
    report = checker.run_check()
    assert len(report.items) >= 4
    assert report.all_aligned is False
