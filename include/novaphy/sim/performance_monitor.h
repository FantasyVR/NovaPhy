#pragma once

#include <chrono>
#include <deque>
#include <filesystem>
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace novaphy {

namespace detail {
class PerformancePhaseScope;
class ScopedPerformanceCaptureContext;

struct StringKeyHash {
    using is_transparent = void;

    size_t operator()(std::string_view value) const noexcept {
        return std::hash<std::string_view>{}(value);
    }

    size_t operator()(const std::string& value) const noexcept {
        return (*this)(std::string_view(value));
    }

    size_t operator()(const char* value) const noexcept {
        return (*this)(std::string_view(value));
    }
};
}

struct PerformancePhaseStat {
    std::string name;
    double last_ms = 0.0;
    double avg_ms = 0.0;
    double max_ms = 0.0;
    int samples = 0;
};

struct PerformanceMetric {
    std::string name;
    double value = 0.0;
};

class PerformanceMonitor {
public:
    using Clock = std::chrono::steady_clock;

    bool enabled() const { return enabled_; }
    void set_enabled(bool enabled) { enabled_ = enabled; }

    bool trace_enabled() const { return trace_enabled_; }
    void set_trace_enabled(bool enabled) { trace_enabled_ = enabled; }

    int trace_frame_capacity() const { return trace_frame_capacity_; }
    void set_trace_frame_capacity(int capacity);

    int frame_count() const { return frame_count_; }
    double last_frame_total_ms() const { return last_frame_total_ms_; }

    void reset();
    std::vector<PerformancePhaseStat> phase_stats() const;
    std::vector<PerformanceMetric> last_frame_metrics() const;
    void write_trace_json(const std::filesystem::path& output_path) const;

    void begin_frame();
    void end_frame();
    void record_metric(std::string_view name, double value);

private:
    friend class detail::PerformancePhaseScope;
    friend class detail::ScopedPerformanceCaptureContext;

    struct AggregatePhase {
        double total_ms = 0.0;
        double last_ms = 0.0;
        double max_ms = 0.0;
        int samples = 0;
    };

    struct TraceDurationEvent {
        std::string name;
        long long start_us = 0;
        long long duration_us = 0;
    };

    struct TraceCounterEvent {
        std::string name;
        long long timestamp_us = 0;
        std::vector<PerformanceMetric> metrics;
    };

    struct TraceFrame {
        std::vector<TraceDurationEvent> duration_events;
        std::vector<TraceCounterEvent> counter_events;
    };

    bool is_capturing_enabled() const { return enabled_ || trace_enabled_; }
    bool is_current_frame_active() const { return current_frame_active_; }
    long long to_microseconds(Clock::time_point t) const;
    void record_phase(const char* name, Clock::time_point start, Clock::time_point end);
    void trim_trace_history();

    bool enabled_ = false;
    bool trace_enabled_ = false;
    int trace_frame_capacity_ = 120;
    int frame_count_ = 0;
    double last_frame_total_ms_ = 0.0;

    Clock::time_point epoch_ = Clock::now();
    Clock::time_point frame_start_{};
    bool current_frame_active_ = false;

    std::unordered_map<std::string, double, detail::StringKeyHash, std::equal_to<>> current_frame_phase_totals_;
    std::vector<PerformanceMetric> current_frame_metrics_;
    std::unordered_map<std::string, size_t, detail::StringKeyHash, std::equal_to<>> current_metric_indices_;
    std::vector<TraceDurationEvent> current_trace_events_;

    std::unordered_map<std::string, AggregatePhase, detail::StringKeyHash, std::equal_to<>> aggregate_phase_stats_;
    std::vector<PerformanceMetric> last_frame_metrics_;
    std::deque<TraceFrame> trace_frames_;
};

namespace detail {

PerformanceMonitor* current_performance_monitor();

class ScopedPerformanceCaptureContext {
public:
    explicit ScopedPerformanceCaptureContext(PerformanceMonitor* monitor);
    ~ScopedPerformanceCaptureContext();

    ScopedPerformanceCaptureContext(const ScopedPerformanceCaptureContext&) = delete;
    ScopedPerformanceCaptureContext& operator=(const ScopedPerformanceCaptureContext&) = delete;

private:
    PerformanceMonitor* previous_ = nullptr;
};

class PerformancePhaseScope {
public:
    PerformancePhaseScope(PerformanceMonitor* monitor, const char* name);
    ~PerformancePhaseScope();

    PerformancePhaseScope(const PerformancePhaseScope&) = delete;
    PerformancePhaseScope& operator=(const PerformancePhaseScope&) = delete;

private:
    PerformanceMonitor* monitor_ = nullptr;
    const char* name_ = nullptr;
    PerformanceMonitor::Clock::time_point start_{};
    bool active_ = false;
};

}  // namespace detail

}  // namespace novaphy
