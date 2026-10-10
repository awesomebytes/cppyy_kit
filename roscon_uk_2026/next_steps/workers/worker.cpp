#include "worker.hpp"
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace timestamp_worker {
struct Worker::Impl {
    struct Input {
        std::uint64_t sequence, version;
        std::int64_t timestamp;
        double scale;
        std::vector<double> values;
    };
    const std::size_t queue_capacity, result_capacity, max_samples;
    mutable std::mutex mutex;
    std::mutex lifecycle;
    std::condition_variable changed;
    std::thread thread;
    std::deque<Input> inputs;
    std::deque<Result> results;
    Snapshot state{};
    double scale = 1.0;
    std::int64_t last_timestamp = -1;

    Impl(std::size_t q, std::size_t r, std::size_t n)
        : queue_capacity(q), result_capacity(r), max_samples(n) {
        if (!q || !r || !n) throw std::invalid_argument("capacities must be positive");
    }
    void run() noexcept {
        std::unique_lock<std::mutex> lock(mutex);
        for (;;) {
            changed.wait(lock, [this] {
                return state.closed || (!state.gated && !inputs.empty());
            });
            if (inputs.empty() && state.closed) break;
            Input input = std::move(inputs.front());
            inputs.pop_front();
            state.active = true;
            lock.unlock();
            try {
                const auto started = std::chrono::steady_clock::now();
                double sum = 0.0, sum_sq = 0.0;
                for (double value : input.values) {
                    const double scaled = input.scale * value;
                    sum += scaled;
                    sum_sq += scaled * scaled;
                }
                if (!std::isfinite(sum) || !std::isfinite(sum_sq))
                    throw std::overflow_error("scaled summary overflow");
                Result result{input.sequence, input.version, input.timestamp,
                              input.values.size(), sum / input.values.size(),
                              std::sqrt(sum_sq / input.values.size())};
                const auto processing_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - started).count();
                lock.lock();
                // Increment completion only after storing the result succeeds.
                results.push_back(result);
                ++state.completed;
                state.processing_ns += processing_ns;
                if (results.size() > result_capacity) {
                    results.pop_front();
                    ++state.result_evicted;
                }
                state.active = false;
                changed.notify_all();
            } catch (const std::exception& error) {
                if (!lock.owns_lock()) lock.lock();
                state.error = error.what();
                ++state.failed;
                state.cancelled += inputs.size();
                inputs.clear();
                state.active = false;
                state.closed = true;
                state.gated = false;
                changed.notify_all();
                break;
            } catch (...) {
                if (!lock.owns_lock()) lock.lock();
                state.error = "unknown native worker failure";
                ++state.failed;
                state.cancelled += inputs.size();
                inputs.clear();
                state.active = false;
                state.closed = true;
                state.gated = false;
                changed.notify_all();
                break;
            }
        }
        state.running = false;
        changed.notify_all();
    }
};

Worker::Worker(std::size_t q, std::size_t r, std::size_t n)
    : impl_(new Impl(q, r, n)) {}
Worker::~Worker() { stop(); }
void Worker::start() {
    auto& p = *impl_;
    std::lock_guard<std::mutex> lifecycle(p.lifecycle);
    std::lock_guard<std::mutex> lock(p.mutex);
    if (p.state.closed) throw std::runtime_error("worker is closed");
    if (p.state.running) return;
    p.state.running = true;
    try { p.thread = std::thread([&p] { p.run(); }); }
    catch (...) { p.state.running = false; throw; }
}
void Worker::stop() {
    auto& p = *impl_;
    std::lock_guard<std::mutex> lifecycle(p.lifecycle);
    {
        std::lock_guard<std::mutex> lock(p.mutex);
        p.state.closed = true;
        p.state.gated = false;
        p.changed.notify_all();
    }
    if (p.thread.joinable()) p.thread.join();
}
Submission Worker::submit(std::int64_t timestamp, const std::vector<double>& values) {
    auto& p = *impl_;
    std::lock_guard<std::mutex> lock(p.mutex);
    if (!p.state.running || p.state.closed)
        throw std::runtime_error("worker is not accepting inputs");
    if (timestamp < 0 || timestamp <= p.last_timestamp)
        throw std::invalid_argument("timestamps must be nonnegative and strictly increasing");
    if (values.empty() || values.size() > p.max_samples)
        throw std::invalid_argument("sample count outside configured bounds");
    for (double value : values)
        if (!std::isfinite(value)) throw std::invalid_argument("samples must be finite");
    const auto sequence = p.state.attempted;
    if (p.inputs.size() >= p.queue_capacity) {
        ++p.state.attempted;
        ++p.state.rejected;
        p.last_timestamp = timestamp;
        return {sequence, false};
    }
    p.inputs.push_back({sequence, p.state.parameter_version, timestamp, p.scale, values});
    ++p.state.attempted;
    ++p.state.accepted;
    p.last_timestamp = timestamp;
    p.changed.notify_all();
    return {sequence, true};
}
std::uint64_t Worker::update_scale(double scale) {
    auto& p = *impl_;
    std::lock_guard<std::mutex> lock(p.mutex);
    if (p.state.closed) throw std::runtime_error("worker is closed");
    if (!std::isfinite(scale)) throw std::invalid_argument("scale must be finite");
    p.scale = scale;
    return ++p.state.parameter_version;
}
Snapshot Worker::snapshot() const {
    auto& p = *impl_;
    std::lock_guard<std::mutex> lock(p.mutex);
    auto state = p.state;
    state.queued = p.inputs.size();
    state.retained_results = p.results.size();
    return state;
}
std::vector<Result> Worker::take_results() {
    auto& p = *impl_;
    std::lock_guard<std::mutex> lock(p.mutex);
    std::vector<Result> results(p.results.begin(), p.results.end());
    p.results.clear();
    return results;
}
void Worker::set_gate(bool closed) {
    auto& p = *impl_;
    std::lock_guard<std::mutex> lock(p.mutex);
    if (p.state.closed) throw std::runtime_error("worker is closed");
    p.state.gated = closed;
    p.changed.notify_all();
}
void Worker::drain(std::uint64_t timeout_ms) {
    auto& p = *impl_;
    std::unique_lock<std::mutex> lock(p.mutex);
    if (!p.state.running && !p.state.closed) throw std::runtime_error("worker was not started");
    ++p.state.drainers;
    p.changed.notify_all();
    bool idle = p.changed.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&p] {
        return p.inputs.empty() && !p.state.active;
    });
    --p.state.drainers;
    if (!idle) throw std::runtime_error("drain timed out");
    if (!p.state.error.empty()) throw std::runtime_error(p.state.error);
}
std::function<void()> Worker::drain_call(std::uint64_t ms) {
    return [this, ms] { drain(ms); };
}
std::function<void()> Worker::stop_call() { return [this] { stop(); }; }
}
