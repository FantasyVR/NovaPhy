#include "novaphy/sim/performance_monitor.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace novaphy {
namespace {

thread_local PerformanceMonitor* g_active_monitor = nullptr;

std::string escape_json(const std::string& value) {
    std::ostringstream out;
    for (unsigned char ch : value) {
        switch (ch) {
            case '"':
                out << "\\\"";
                break;
            case '\\':
                out << "\\\\";
                break;
            case '\b':
                out << "\\b";
                break;
            case '\f':
                out << "\\f";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                if (ch < 0x20) {
                    out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(ch) << std::dec;
                } else {
                    out << static_cast<char>(ch);
                }
                break;
        }
    }
    return out.str();
}

void ensure_parent_directory(const std::string& output_path) {
    const std::filesystem::path path(output_path);
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
}

}  // namespace

void PerformanceMonitor::set_trace_frame_capacity(int capacity) {
    trace_frame_capacity_ = std::max(0, capacity);
    trim_trace_history();
}

void PerformanceMonitor::reset() {
    frame_count_ = 0;
    last_frame_total_ms_ = 0.0;
    epoch_ = Clock::now();
    frame_start_ = Clock::time_point{};
    current_frame_active_ = false;

    current_frame_phase_totals_.clear();
    current_frame_metrics_.clear();
    current_metric_indices_.clear();
    current_trace_events_.clear();

    aggregate_phase_stats_.clear();
    last_frame_metrics_.clear();
    trace_frames_.clear();
}

std::vector<PerformancePhaseStat> PerformanceMonitor::phase_stats() const {
    std::vector<PerformancePhaseStat> stats;
    stats.reserve(aggregate_phase_stats_.size());
    for (const auto& [name, aggregate] : aggregate_phase_stats_) {
        PerformancePhaseStat stat;
        stat.name = name;
        stat.last_ms = aggregate.last_ms;
        stat.avg_ms = (aggregate.samples > 0) ?
            (aggregate.total_ms / static_cast<double>(aggregate.samples)) : 0.0;
        stat.max_ms = aggregate.max_ms;
        stat.samples = aggregate.samples;
        stats.push_back(stat);
    }

    std::sort(stats.begin(), stats.end(),
              [](const PerformancePhaseStat& a, const PerformancePhaseStat& b) {
                  return a.name < b.name;
              });
    return stats;
}

std::vector<PerformanceMetric> PerformanceMonitor::last_frame_metrics() const {
    return last_frame_metrics_;
}

void PerformanceMonitor::write_trace_json(const std::string& output_path) const {
    ensure_parent_directory(output_path);

    std::ofstream out(output_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open output file: " + output_path);
    }

    out << "{\n  \"traceEvents\": [\n";
    bool first_event = true;

    auto write_separator = [&]() {
        if (!first_event) {
            out << ",\n";
        }
        first_event = false;
    };

    for (const TraceFrame& frame : trace_frames_) {
        for (const TraceDurationEvent& event : frame.duration_events) {
            write_separator();
            out << "    {"
                << "\"name\":\"" << escape_json(event.name) << "\","
                << "\"ph\":\"X\","
                << "\"ts\":" << event.start_us << ","
                << "\"dur\":" << event.duration_us << ","
                << "\"pid\":1,"
                << "\"tid\":0"
                << "}";
        }

        for (const TraceCounterEvent& event : frame.counter_events) {
            write_separator();
            out << "    {"
                << "\"name\":\"" << escape_json(event.name) << "\","
                << "\"ph\":\"C\","
                << "\"ts\":" << event.timestamp_us << ","
                << "\"pid\":1,"
                << "\"tid\":0,"
                << "\"args\":{";
            for (size_t i = 0; i < event.metrics.size(); ++i) {
                if (i > 0) {
                    out << ",";
                }
                out << "\"" << escape_json(event.metrics[i].name) << "\":"
                    << std::setprecision(17) << event.metrics[i].value;
            }
            out << "}}";
        }
    }

    out << "\n  ]\n}\n";
}

void PerformanceMonitor::begin_frame() {
    current_frame_phase_totals_.clear();
    current_frame_metrics_.clear();
    current_metric_indices_.clear();
    current_trace_events_.clear();

    current_frame_active_ = is_capturing_enabled();
    if (current_frame_active_) {
        frame_start_ = Clock::now();
    }
}

void PerformanceMonitor::end_frame() {
    if (!current_frame_active_) {
        return;
    }

    const Clock::time_point frame_end = Clock::now();
    last_frame_total_ms_ = std::chrono::duration<double, std::milli>(frame_end - frame_start_).count();

    for (const auto& [name, duration_ms] : current_frame_phase_totals_) {
        AggregatePhase& aggregate = aggregate_phase_stats_[name];
        aggregate.total_ms += duration_ms;
        aggregate.last_ms = duration_ms;
        aggregate.max_ms = std::max(aggregate.max_ms, duration_ms);
        aggregate.samples += 1;
    }

    last_frame_metrics_ = current_frame_metrics_;
    frame_count_ += 1;

    if (trace_enabled_ && trace_frame_capacity_ > 0) {
        TraceFrame frame;
        frame.duration_events = current_trace_events_;

        TraceCounterEvent counters;
        counters.name = "frame.metrics";
        counters.timestamp_us = to_microseconds(frame_end);
        counters.metrics = last_frame_metrics_;
        counters.metrics.push_back({"frame_count", static_cast<double>(frame_count_)});
        counters.metrics.push_back({"frame_total_ms", last_frame_total_ms_});
        frame.counter_events.push_back(std::move(counters));

        trace_frames_.push_back(std::move(frame));
        trim_trace_history();
    }

    current_frame_active_ = false;
}

void PerformanceMonitor::record_metric(const std::string& name, double value) {
    if (!current_frame_active_) {
        return;
    }

    auto it = current_metric_indices_.find(name);
    if (it == current_metric_indices_.end()) {
        current_metric_indices_[name] = current_frame_metrics_.size();
        current_frame_metrics_.push_back({name, value});
        return;
    }

    current_frame_metrics_[it->second].value = value;
}

long long PerformanceMonitor::to_microseconds(Clock::time_point t) const {
    return std::chrono::duration_cast<std::chrono::microseconds>(t - epoch_).count();
}

void PerformanceMonitor::record_phase(const char* name,
                                      Clock::time_point start,
                                      Clock::time_point end) {
    if (!current_frame_active_ || name == nullptr) {
        return;
    }

    const double duration_ms = std::chrono::duration<double, std::milli>(end - start).count();
    current_frame_phase_totals_[name] += duration_ms;

    if (trace_enabled_ && trace_frame_capacity_ > 0) {
        TraceDurationEvent event;
        event.name = name;
        event.start_us = to_microseconds(start);
        event.duration_us =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        current_trace_events_.push_back(std::move(event));
    }
}

void PerformanceMonitor::trim_trace_history() {
    while (static_cast<int>(trace_frames_.size()) > trace_frame_capacity_) {
        trace_frames_.pop_front();
    }
}

namespace detail {

PerformanceMonitor* current_performance_monitor() {
    return g_active_monitor;
}

ScopedPerformanceCaptureContext::ScopedPerformanceCaptureContext(PerformanceMonitor* monitor)
    : previous_(g_active_monitor) {
    g_active_monitor = monitor;
}

ScopedPerformanceCaptureContext::~ScopedPerformanceCaptureContext() {
    g_active_monitor = previous_;
}

PerformancePhaseScope::PerformancePhaseScope(PerformanceMonitor* monitor, const char* name)
    : monitor_(monitor), name_(name) {
    if (monitor_ != nullptr && monitor_->is_current_frame_active()) {
        active_ = true;
        start_ = PerformanceMonitor::Clock::now();
    }
}

PerformancePhaseScope::~PerformancePhaseScope() {
    if (!active_ || monitor_ == nullptr) {
        return;
    }

    monitor_->record_phase(name_, start_, PerformanceMonitor::Clock::now());
}

}  // namespace detail

}  // namespace novaphy
