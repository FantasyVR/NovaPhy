#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl.h>

#include "novaphy/io/openusd_importer.h"
#include "novaphy/io/feature_completeness.h"
#include "novaphy/io/scene_builder.h"
#include "novaphy/io/simulation_exporter.h"
#include "novaphy/io/urdf_parser.h"

namespace py = pybind11;
using namespace novaphy;

void bind_io(py::module_& m) {
    py::enum_<UrdfGeometryType>(m, "UrdfGeometryType")
        .value("Box", UrdfGeometryType::Box)
        .value("Sphere", UrdfGeometryType::Sphere)
        .value("Cylinder", UrdfGeometryType::Cylinder)
        .value("Mesh", UrdfGeometryType::Mesh)
        .value("Unknown", UrdfGeometryType::Unknown);

    py::class_<UrdfGeometry>(m, "UrdfGeometry")
        .def(py::init<>())
        .def_readwrite("type", &UrdfGeometry::type)
        .def_readwrite("size", &UrdfGeometry::size)
        .def_readwrite("radius", &UrdfGeometry::radius)
        .def_readwrite("length", &UrdfGeometry::length)
        .def_readwrite("mesh_filename", &UrdfGeometry::mesh_filename)
        .def_readwrite("mesh_scale", &UrdfGeometry::mesh_scale);

    py::class_<UrdfVisual>(m, "UrdfVisual")
        .def(py::init<>())
        .def_readwrite("origin", &UrdfVisual::origin)
        .def_readwrite("geometry", &UrdfVisual::geometry)
        .def_readwrite("material_name", &UrdfVisual::material_name);

    py::class_<UrdfCollision>(m, "UrdfCollision")
        .def(py::init<>())
        .def_readwrite("origin", &UrdfCollision::origin)
        .def_readwrite("geometry", &UrdfCollision::geometry)
        .def_readwrite("friction", &UrdfCollision::friction)
        .def_readwrite("restitution", &UrdfCollision::restitution);

    py::class_<UrdfInertial>(m, "UrdfInertial")
        .def(py::init<>())
        .def_readwrite("origin", &UrdfInertial::origin)
        .def_readwrite("mass", &UrdfInertial::mass)
        .def_readwrite("inertia", &UrdfInertial::inertia);

    py::class_<UrdfLink>(m, "UrdfLink")
        .def(py::init<>())
        .def_readwrite("name", &UrdfLink::name)
        .def_readwrite("inertial", &UrdfLink::inertial)
        .def_readwrite("visuals", &UrdfLink::visuals)
        .def_readwrite("collisions", &UrdfLink::collisions);

    py::class_<UrdfJoint>(m, "UrdfJoint")
        .def(py::init<>())
        .def_readwrite("name", &UrdfJoint::name)
        .def_readwrite("type", &UrdfJoint::type)
        .def_readwrite("parent_link", &UrdfJoint::parent_link)
        .def_readwrite("child_link", &UrdfJoint::child_link)
        .def_readwrite("origin", &UrdfJoint::origin)
        .def_readwrite("axis", &UrdfJoint::axis)
        .def_readwrite("lower_limit", &UrdfJoint::lower_limit)
        .def_readwrite("upper_limit", &UrdfJoint::upper_limit)
        .def_readwrite("effort_limit", &UrdfJoint::effort_limit)
        .def_readwrite("velocity_limit", &UrdfJoint::velocity_limit);

    py::class_<UrdfModelData>(m, "UrdfModelData")
        .def(py::init<>())
        .def_readwrite("name", &UrdfModelData::name)
        .def_readwrite("links", &UrdfModelData::links)
        .def_readwrite("joints", &UrdfModelData::joints);

    py::class_<UsdAnimationTrack>(m, "UsdAnimationTrack")
        .def(py::init<>())
        .def_readwrite("property_name", &UsdAnimationTrack::property_name)
        .def_readwrite("vec3_samples", &UsdAnimationTrack::vec3_samples)
        .def_readwrite("vec4_samples", &UsdAnimationTrack::vec4_samples);

    py::class_<UsdPrim>(m, "UsdPrim")
        .def(py::init<>())
        .def_readwrite("path", &UsdPrim::path)
        .def_readwrite("name", &UsdPrim::name)
        .def_readwrite("type_name", &UsdPrim::type_name)
        .def_readwrite("parent_path", &UsdPrim::parent_path)
        .def_readwrite("local_transform", &UsdPrim::local_transform)
        .def_readwrite("mass", &UsdPrim::mass)
        .def_readwrite("density", &UsdPrim::density)
        .def_readwrite("material_binding", &UsdPrim::material_binding)
        .def_readwrite("box_half_extents", &UsdPrim::box_half_extents)
        .def_readwrite("sphere_radius", &UsdPrim::sphere_radius)
        .def_readwrite("tracks", &UsdPrim::tracks);

    py::class_<UsdStageData>(m, "UsdStageData")
        .def(py::init<>())
        .def_readwrite("default_prim", &UsdStageData::default_prim)
        .def_readwrite("up_axis", &UsdStageData::up_axis)
        .def_readwrite("meters_per_unit", &UsdStageData::meters_per_unit)
        .def_readwrite("prims", &UsdStageData::prims);

    py::class_<SceneBuildResult>(m, "SceneBuildResult")
        .def(py::init<>())
        .def_readwrite("model", &SceneBuildResult::model)
        .def_readwrite("articulation", &SceneBuildResult::articulation)
        .def_readwrite("warnings", &SceneBuildResult::warnings);

    py::class_<FeatureCheckItem>(m, "FeatureCheckItem")
        .def(py::init<>())
        .def_readwrite("name", &FeatureCheckItem::name)
        .def_readwrite("available", &FeatureCheckItem::available)
        .def_readwrite("backend", &FeatureCheckItem::backend);

    py::class_<FeatureCheckReport>(m, "FeatureCheckReport")
        .def(py::init<>())
        .def_readwrite("items", &FeatureCheckReport::items)
        .def_readwrite("all_aligned", &FeatureCheckReport::all_aligned);

    py::class_<RecordedKeyframe>(m, "RecordedKeyframe")
        .def(py::init<>())
        .def_readwrite("time", &RecordedKeyframe::time)
        .def_readwrite("body_index", &RecordedKeyframe::body_index)
        .def_readwrite("position", &RecordedKeyframe::position)
        .def_readwrite("rotation", &RecordedKeyframe::rotation)
        .def_readwrite("linear_velocity", &RecordedKeyframe::linear_velocity)
        .def_readwrite("angular_velocity", &RecordedKeyframe::angular_velocity);

    py::class_<RecordedCollisionEvent>(m, "RecordedCollisionEvent")
        .def(py::init<>())
        .def_readwrite("time", &RecordedCollisionEvent::time)
        .def_readwrite("body_a", &RecordedCollisionEvent::body_a)
        .def_readwrite("body_b", &RecordedCollisionEvent::body_b)
        .def_readwrite("position", &RecordedCollisionEvent::position)
        .def_readwrite("normal", &RecordedCollisionEvent::normal)
        .def_readwrite("penetration", &RecordedCollisionEvent::penetration);

    py::class_<RecordedConstraintReaction>(m, "RecordedConstraintReaction")
        .def(py::init<>())
        .def_readwrite("time", &RecordedConstraintReaction::time)
        .def_readwrite("joint_name", &RecordedConstraintReaction::joint_name)
        .def_readwrite("wrench", &RecordedConstraintReaction::wrench);

    py::class_<UrdfParser>(m, "UrdfParser")
        .def(py::init<>())
        .def("parse_file", &UrdfParser::parse_file, py::arg("urdf_path"))
        .def("write_string", &UrdfParser::write_string, py::arg("model"))
        .def("write_file", &UrdfParser::write_file, py::arg("model"), py::arg("urdf_path"));

    py::class_<OpenUsdImporter>(m, "OpenUsdImporter")
        .def(py::init<float>(), py::arg("min_supported_version") = 21.08f)
        .def("import_file", &OpenUsdImporter::import_file, py::arg("usd_path"))
        .def_property_readonly("min_supported_version", &OpenUsdImporter::min_supported_version);

    py::class_<SceneBuilderEngine>(m, "SceneBuilderEngine")
        .def(py::init<>())
        .def("build_from_urdf", &SceneBuilderEngine::build_from_urdf, py::arg("urdf_model"))
        .def("build_from_openusd", &SceneBuilderEngine::build_from_openusd, py::arg("stage"));

    py::class_<SimulationExporter>(m, "SimulationExporter")
        .def(py::init<>())
        .def("capture_frame", &SimulationExporter::capture_frame, py::arg("world"), py::arg("time_seconds"))
        .def("add_constraint_reaction", &SimulationExporter::add_constraint_reaction, py::arg("reaction"))
        .def_property_readonly("keyframes", &SimulationExporter::keyframes)
        .def_property_readonly("collision_events", &SimulationExporter::collision_events)
        .def_property_readonly("constraint_reactions", &SimulationExporter::constraint_reactions)
        .def("write_keyframes_csv", &SimulationExporter::write_keyframes_csv, py::arg("output_path"))
        .def("write_collision_log_csv", &SimulationExporter::write_collision_log_csv, py::arg("output_path"))
        .def("write_constraint_reactions_csv", &SimulationExporter::write_constraint_reactions_csv, py::arg("output_path"))
        .def("write_urdf", &SimulationExporter::write_urdf, py::arg("model"), py::arg("output_path"))
        .def("write_openusd_animation_layer", &SimulationExporter::write_openusd_animation_layer, py::arg("output_path"));

    py::class_<FeatureCompletenessChecker>(m, "FeatureCompletenessChecker")
        .def(py::init<>())
        .def("run_check", &FeatureCompletenessChecker::run_check)
        .def("require_full_alignment", &FeatureCompletenessChecker::require_full_alignment);
}
