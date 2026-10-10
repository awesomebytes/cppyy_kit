#ifndef REVERSE_DEMO_POSE_FILTER_HPP
#define REVERSE_DEMO_POSE_FILTER_HPP
#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
namespace reverse_demo {
// Public deployment configuration; Python adapts to these fields explicitly.
struct Config {
    double time_constant_s = 0.08;
    double reset_gap_s = 0.5;
    std::string output_frame = "world";
    void validate() const;
};
struct Snapshot {
    bool initialized = false;
    std::int64_t last_timestamp_ns = 0;
    std::array<double, 3> position_m{};
    std::uint64_t samples_processed = 0;
    std::uint64_t gap_resets = 0;
};
class PoseEstimator {
public:
    explicit PoseEstimator(const Config& config);
    void reset();
    Snapshot snapshot() const;
    const Config& config() const;
    // Non-overlapping buffers. All inputs are validated before state mutation.
    void process(const std::int64_t* timestamps_ns, const double* positions_m,
                 std::size_t samples, double* output_m);
private:
    Config config_;
    Snapshot state_;
};
Config load_config_file(const std::string& path);
}
#endif
