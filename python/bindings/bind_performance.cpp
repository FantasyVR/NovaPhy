#include <pybind11/pybind11.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl.h>

#include "novaphy/sim/performance_monitor.h"

namespace py = pybind11;
using namespace novaphy;

void bind_performance(py::module_& m) {
    py::class_<PerformancePhaseStat>(m, "PerformancePhaseStat", R"pbdoc(
        Aggregate timing statistics for one named phase.
    )pbdoc")
        .def(py::init<>())
        .def_readonly("name", &PerformancePhaseStat::name)
        .def_readonly("last_ms", &PerformancePhaseStat::last_ms)
        .def_readonly("avg_ms", &PerformancePhaseStat::avg_ms)
        .def_readonly("max_ms", &PerformancePhaseStat::max_ms)
        .def_readonly("samples", &PerformancePhaseStat::samples);

    py::class_<PerformanceMetric>(m, "PerformanceMetric", R"pbdoc(
        Numeric metric captured for the most recent profiled frame.
    )pbdoc")
        .def(py::init<>())
        .def_readonly("name", &PerformanceMetric::name)
        .def_readonly("value", &PerformanceMetric::value);

    py::class_<PerformanceMonitor>(m, "PerformanceMonitor", R"pbdoc(
        Opt-in runtime performance monitor for simulation stepping.
    )pbdoc")
        .def(py::init<>())
        .def_property("enabled",
                      &PerformanceMonitor::enabled,
                      &PerformanceMonitor::set_enabled,
                      R"pbdoc(
                          bool: Enable aggregate performance capture for future steps.
                      )pbdoc")
        .def_property("trace_enabled",
                      &PerformanceMonitor::trace_enabled,
                      &PerformanceMonitor::set_trace_enabled,
                      R"pbdoc(
                          bool: Enable Perfetto/Chrome trace capture for future steps.
                      )pbdoc")
        .def_property("trace_frame_capacity",
                      &PerformanceMonitor::trace_frame_capacity,
                      &PerformanceMonitor::set_trace_frame_capacity,
                      R"pbdoc(
                          int: Number of recent profiled frames retained for trace export.
                      )pbdoc")
        .def_property_readonly("frame_count", &PerformanceMonitor::frame_count, R"pbdoc(
            int: Number of profiled frames captured since the last reset.
        )pbdoc")
        .def_property_readonly("last_frame_total_ms",
                               &PerformanceMonitor::last_frame_total_ms,
                               R"pbdoc(
                                   float: Total wall-clock time for the most recent profiled frame.
                               )pbdoc")
        .def("reset", &PerformanceMonitor::reset, R"pbdoc(
            Clear all aggregate stats, last-frame metrics, and buffered trace data.
        )pbdoc")
        .def("phase_stats", &PerformanceMonitor::phase_stats, R"pbdoc(
            Returns:
                list[PerformancePhaseStat]: Aggregate timings for recorded phases.
        )pbdoc")
        .def("last_frame_metrics", &PerformanceMonitor::last_frame_metrics, R"pbdoc(
            Returns:
                list[PerformanceMetric]: Metrics captured for the most recent profiled frame.
        )pbdoc")
        .def("write_trace_json", &PerformanceMonitor::write_trace_json,
             py::arg("output_path"),
             R"pbdoc(
                 Export buffered trace data as Chrome/Perfetto-compatible JSON.
             )pbdoc");
}
