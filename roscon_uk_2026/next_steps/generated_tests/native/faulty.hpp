#ifndef GENERATED_TESTS_FAULTY_HPP
#define GENERATED_TESTS_FAULTY_HPP
#include "pose_filter.hpp"
namespace generated_test_fault {
// Test fixture only. Each process call forwards to the real compiled estimator.
class MissingResetEstimator {
public:
    explicit MissingResetEstimator(const reverse_demo::Config& config);
    void reset();
    reverse_demo::Snapshot snapshot() const;
    void process(const std::int64_t* timestamps_ns, const double* positions_m,
                 std::size_t samples, double* output_m);
private:
    reverse_demo::PoseEstimator estimator_;
};
}
#endif
