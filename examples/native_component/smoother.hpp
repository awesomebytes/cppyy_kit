#ifndef CPPYY_KIT_EXAMPLE_SMOOTHER_HPP
#define CPPYY_KIT_EXAMPLE_SMOOTHER_HPP
#include <cstdint>
#include <vector>

namespace component_example {
struct Config {
    double smoothing_weight = 0.5;
};
struct State {
    bool initialized = false;
    double value = 0.0;
    std::uint64_t samples = 0;
};
class Smoother {
public:
    explicit Smoother(const Config& config);
    ~Smoother();
    std::vector<double> process(const std::vector<double>& values);
    void reset();
    State snapshot() const;
private:
    double weight_;
    State state_;
};
}
#endif
