#include "smoother.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace component_example {
Smoother::Smoother(const Config& config) : weight_(config.smoothing_weight) {
    if (!std::isfinite(weight_) || weight_ <= 0.0 || weight_ > 1.0)
        throw std::invalid_argument("smoothing weight must be in (0, 1]");
}
Smoother::~Smoother() = default;
void Smoother::reset() { state_ = State{}; }
State Smoother::snapshot() const { return state_; }
std::vector<double> Smoother::process(const std::vector<double>& values) {
    if (values.size() > std::numeric_limits<std::uint64_t>::max() - state_.samples)
        throw std::overflow_error("sample counter overflow");
    // Check the whole batch before allocating outputs or changing state.
    for (double value : values)
        if (!std::isfinite(value) || std::abs(value) > 1e6)
            throw std::invalid_argument("samples must be finite and in [-1e6, 1e6]");
    std::vector<double> output;
    output.reserve(values.size());
    for (double value : values) {
        state_.value = state_.initialized
            ? state_.value + weight_ * (value - state_.value) : value;
        state_.initialized = true;
        ++state_.samples;
        output.push_back(state_.value);
    }
    return output;
}
}
