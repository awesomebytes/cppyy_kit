#include "faulty.hpp"
namespace generated_test_fault {
MissingResetEstimator::MissingResetEstimator(const reverse_demo::Config& config)
    : estimator_(config) {}
void MissingResetEstimator::reset() {
    // Deliberate native defect: omit estimator_.reset().
}
reverse_demo::Snapshot MissingResetEstimator::snapshot() const {
    return estimator_.snapshot();
}
void MissingResetEstimator::process(const std::int64_t* timestamps_ns,
    const double* positions_m, std::size_t samples, double* output_m) {
    estimator_.process(timestamps_ns, positions_m, samples, output_m);
}
}
