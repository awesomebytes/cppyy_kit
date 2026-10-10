#pragma once
#include <cstdint>
#include <vector>
namespace trajectory_demo {
struct Sample {
    int result;
    bool new_calculation;
    double time, duration;
    std::uint64_t native_ns;
    std::vector<double> position, velocity, acceleration, jerk, command;
};
// The implementation owns Ruckig. This header does not expose its templates.
class Session {
    struct Impl;
    Impl* impl_;
public:
    Session(unsigned dofs, double dt, const std::vector<double>& vmax,
            const std::vector<double>& amax, const std::vector<double>& jmax,
            double kp = 12.0);
    ~Session();
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;
    void target(const std::vector<double>& p, const std::vector<double>& v,
                const std::vector<double>& a);
    void limits(const std::vector<double>& v, const std::vector<double>& a,
                const std::vector<double>& j);
    void reset(const std::vector<double>& p, const std::vector<double>& v,
               const std::vector<double>& a);
    Sample step(const std::vector<double>& measured);
    Sample state() const;
    Sample at_time(double t) const;
    std::vector<double> knots() const;
};
}
