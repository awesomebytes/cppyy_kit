#include "pose_filter.hpp"
#include <cmath>
#include <fstream>
#include <limits>
#include <regex>
#include <stdexcept>
namespace reverse_demo {
void Config::validate() const {
    if (!std::isfinite(time_constant_s) || time_constant_s <= 0)
        throw std::invalid_argument("tau_s must be finite and positive seconds");
    if (!std::isfinite(reset_gap_s) || reset_gap_s < time_constant_s)
        throw std::invalid_argument("max_gap_s must be finite and >= tau_s");
    if (output_frame.size() > 128 ||
        !std::regex_match(output_frame, std::regex("[A-Za-z][A-Za-z0-9_/]*")))
        throw std::invalid_argument("frame_id has invalid characters or length");
}
PoseEstimator::PoseEstimator(const Config& config) : config_(config) { config_.validate(); }
void PoseEstimator::reset() { state_ = Snapshot{}; }
Snapshot PoseEstimator::snapshot() const { return state_; }
const Config& PoseEstimator::config() const { return config_; }
void PoseEstimator::process(const std::int64_t* t, const double* x,
                            std::size_t n, double* out) {
    if (n == 0) return;
    if (!t || !x || !out) throw std::invalid_argument("null batch buffer");
    if (n > std::numeric_limits<std::uint64_t>::max() - state_.samples_processed)
        throw std::overflow_error("sample counter overflow");
    for (std::size_t i = 0; i < n; ++i) {
        if ((i && t[i] <= t[i-1]) || (!i && state_.initialized && t[i] <= state_.last_timestamp_ns))
            throw std::invalid_argument("timestamps_ns must strictly increase");
        for (int axis = 0; axis < 3; ++axis)
            if (!std::isfinite(x[3*i+axis]))
                throw std::invalid_argument("positions_m must be finite");
    }
    for (std::size_t i = 0; i < n; ++i) {
        const double dt = state_.initialized
            ? static_cast<double>((static_cast<long double>(t[i]) - state_.last_timestamp_ns) / 1000000000.0L) : 0;
        const bool gap = state_.initialized && dt > config_.reset_gap_s;
        if (!state_.initialized || gap) {
            for (int a = 0; a < 3; ++a) state_.position_m[a] = x[3*i+a];
            if (gap) ++state_.gap_resets;
        } else {
            const long double alpha = -std::expm1(-dt / config_.time_constant_s);
            for (int a = 0; a < 3; ++a) {
                // Extended precision prevents overflow for opposite finite inputs.
                const long double previous = state_.position_m[a];
                state_.position_m[a] = static_cast<double>(
                    previous + alpha * (static_cast<long double>(x[3*i+a]) - previous));
            }
        }
        state_.initialized = true;
        state_.last_timestamp_ns = t[i];
        ++state_.samples_processed;
        for (int a = 0; a < 3; ++a) out[3*i+a] = state_.position_m[a];
    }
}
static double read_number(const std::string& line, const std::string& prefix) {
    if (line.rfind(prefix, 0) != 0) throw std::invalid_argument("configuration key/order mismatch");
    const std::string value = line.substr(prefix.size());
    std::size_t consumed = 0;
    const double result = std::stod(value, &consumed);
    if (consumed != value.size()) throw std::invalid_argument("configuration trailing number text");
    return result;
}
Config load_config_file(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open configuration: " + path);
    std::string header, tau, gap, frame, extra;
    if (!std::getline(input, header) || !std::getline(input, tau) ||
        !std::getline(input, gap) || !std::getline(input, frame) ||
        std::getline(input, extra) || header != "pose_filter_config_v1" || frame.rfind("frame_id=",0) != 0)
        throw std::invalid_argument("expected exactly four resolved configuration lines");
    Config config;
    config.time_constant_s = read_number(tau, "tau_s=");
    config.reset_gap_s = read_number(gap, "max_gap_s=");
    config.output_frame = frame.substr(9);
    config.validate();
    return config;
}
}
