#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace timestamp_worker {
struct Result {
    std::uint64_t sequence, parameter_version;
    std::int64_t timestamp_ns;
    std::size_t count;
    double mean, rms;
};
struct Submission { std::uint64_t sequence; bool accepted; };
struct Snapshot {
    std::uint64_t attempted, accepted, rejected, completed, failed, cancelled;
    std::uint64_t result_evicted, parameter_version;
    std::uint64_t processing_ns;
    std::size_t queued, retained_results, drainers;
    bool active, running, closed, gated;
    std::string error;
};
class Worker {
    struct Impl;
    std::unique_ptr<Impl> impl_;
public:
    Worker(std::size_t queue_capacity, std::size_t result_capacity,
           std::size_t max_samples);
    ~Worker();
    Worker(const Worker&) = delete;
    Worker& operator=(const Worker&) = delete;
    void start();
    void stop();
    Submission submit(std::int64_t timestamp_ns, const std::vector<double>& values);
    std::uint64_t update_scale(double scale);
    Snapshot snapshot() const;
    std::vector<Result> take_results();
    void set_gate(bool closed);
    void drain(std::uint64_t timeout_ms);
    std::function<void()> drain_call(std::uint64_t timeout_ms);
    std::function<void()> stop_call();
};
}
