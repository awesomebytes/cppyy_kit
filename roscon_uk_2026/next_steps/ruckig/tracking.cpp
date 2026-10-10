#include "tracking.hpp"
#include <ruckig/ruckig.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
namespace trajectory_demo {
struct Session::Impl {
    unsigned n;
    double dt, kp;
    ruckig::Ruckig<0> generator;
    ruckig::InputParameter<0> input;
    ruckig::OutputParameter<0> output;
    bool has_trajectory = false;
    int result = 0;
    std::vector<double> command;
    Impl(unsigned n, double dt, double kp): n(n), dt(dt), kp(kp), generator(n, dt),
        input(n), output(n), command(n, 0.0) {}
    void check(const std::vector<double>& x, bool positive = false) const {
        if (x.size() != n) throw std::invalid_argument("expected one finite value per DoF");
        for (double value : x)
            if (!std::isfinite(value) || (positive && value <= 0))
                throw std::invalid_argument("values must be finite; limits must be positive");
    }
    void validate(const ruckig::InputParameter<0>& x) const {
        try {
            if (!generator.validate_input(x, true, true))
                throw std::invalid_argument("state violates Ruckig feasibility limits");
        } catch (const ruckig::RuckigError& error) {
            throw std::invalid_argument(error.what());
        }
    }
    Sample snapshot() const {
        return {result, false, has_trajectory ? output.time : 0.0,
                has_trajectory ? output.trajectory.get_duration() : 0.0, 0,
                input.current_position, input.current_velocity, input.current_acceleration,
                has_trajectory ? output.new_jerk : std::vector<double>(n, 0.0), command};
    }
};
Session::Session(unsigned n, double dt, const std::vector<double>& v,
                 const std::vector<double>& a, const std::vector<double>& j, double kp): impl_(nullptr) {
    if ((n != 3 && n != 6) || !std::isfinite(dt) || dt <= 0 || !std::isfinite(kp) || kp < 0)
        throw std::invalid_argument("require 3 or 6 DoF, finite positive dt, finite nonnegative kp");
    auto* p = new Impl(n, dt, kp);
    try {
        p->check(v, true); p->check(a, true); p->check(j, true);
        p->input.max_velocity = v; p->input.max_acceleration = a; p->input.max_jerk = j;
        p->validate(p->input);
    } catch (...) { delete p; throw; }
    impl_ = p;
}
Session::~Session() { delete impl_; }
void Session::target(const std::vector<double>& p, const std::vector<double>& v,
                     const std::vector<double>& a) {
    impl_->check(p); impl_->check(v); impl_->check(a);
    auto candidate = impl_->input;
    candidate.target_position = p; candidate.target_velocity = v; candidate.target_acceleration = a;
    impl_->validate(candidate);
    impl_->input = candidate;
}
void Session::limits(const std::vector<double>& v, const std::vector<double>& a,
                     const std::vector<double>& j) {
    impl_->check(v, true); impl_->check(a, true); impl_->check(j, true);
    auto candidate = impl_->input;
    candidate.max_velocity = v; candidate.max_acceleration = a; candidate.max_jerk = j;
    impl_->validate(candidate);
    impl_->input = candidate;
}
void Session::reset(const std::vector<double>& p, const std::vector<double>& v,
                    const std::vector<double>& a) {
    impl_->check(p); impl_->check(v); impl_->check(a);
    auto candidate = impl_->input;
    candidate.current_position = p; candidate.current_velocity = v; candidate.current_acceleration = a;
    impl_->validate(candidate);
    impl_->input = candidate;
    impl_->output = ruckig::OutputParameter<0>(impl_->n);
    impl_->generator.reset();
    impl_->has_trajectory = false;
    impl_->result = 0;
    impl_->command = p;
}
Sample Session::step(const std::vector<double>& measured) {
    impl_->check(measured);
    auto start = std::chrono::steady_clock::now();
    // Keep the accepted output separate until generation and tracking both succeed.
    auto output = impl_->output;
    const auto result = impl_->generator.update(impl_->input, output);
    if (result < 0) {
        impl_->generator.reset();
        throw std::runtime_error("Ruckig update failed; accepted state retained");
    }
    std::vector<double> command(impl_->n);
    for (unsigned i = 0; i < impl_->n; ++i) {
        double rate = output.new_velocity[i] + impl_->kp * (output.new_position[i] - measured[i]);
        rate = std::clamp(rate, -impl_->input.max_velocity[i], impl_->input.max_velocity[i]);
        command[i] = measured[i] + impl_->dt * rate;
        if (!std::isfinite(command[i])) {
            impl_->generator.reset();
            throw std::runtime_error("tracking command overflow; state retained");
        }
    }
    impl_->output = output; impl_->command = std::move(command);
    output.pass_to_input(impl_->input);
    impl_->has_trajectory = true; impl_->result = static_cast<int>(result);
    Sample sample = impl_->snapshot();
    sample.new_calculation = output.new_calculation;
    sample.native_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - start).count();
    return sample;
}
Sample Session::state() const { return impl_->snapshot(); }
Sample Session::at_time(double t) const {
    if (!impl_->has_trajectory || !std::isfinite(t) || t < 0 || t > impl_->output.trajectory.get_duration())
        throw std::invalid_argument("sample time must lie in the last calculated trajectory");
    Sample sample = impl_->snapshot();
    size_t section;
    impl_->output.trajectory.at_time(t, sample.position, sample.velocity, sample.acceleration, sample.jerk, section);
    sample.time = t;
    return sample;
}
std::vector<double> Session::knots() const {
    if (!impl_->has_trajectory) throw std::logic_error("step once before reading trajectory phases");
    const double duration = impl_->output.trajectory.get_duration();
    std::vector<double> times{0.0, duration};
    const auto profiles = impl_->output.trajectory.get_profiles();
    for (const auto& profile : profiles[0]) {
        double t = 0.0;
        if (profile.brake.duration > 0) {
            for (double phase : profile.brake.t) { t += phase; times.push_back(t); }
        }
        for (double phase : profile.t) { t += phase; times.push_back(t); }
    }
    for (double& t : times) t = std::clamp(t, 0.0, duration);
    std::sort(times.begin(), times.end());
    times.erase(std::unique(times.begin(), times.end()), times.end());
    return times;
}
}
