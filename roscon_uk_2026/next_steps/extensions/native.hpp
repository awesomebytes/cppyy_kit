#pragma once
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace extension_example {
namespace ob = ompl::base;
namespace og = ompl::geometric;

class NativePolicy : public ob::StateValidityChecker {
public:
    mutable unsigned long calls = 0;
    double bias;
    NativePolicy(const ob::SpaceInformationPtr& si, double bias_x)
        : ob::StateValidityChecker(si), bias(bias_x) {
        if (!std::isfinite(bias)) throw std::invalid_argument("bias must be finite");
    }
    bool isValid(const ob::State* state) const override {
        ++calls;
        const auto* s = state->as<ob::RealVectorStateSpace::StateType>();
        const double x = (*s)[0], y = (*s)[1];
        if (!std::isfinite(x) || !std::isfinite(y) ||
            x < 0 || x > 1 || y < 0 || y > 1) return false;
        const double dx = x + bias - 0.5, dy = y - 0.5;
        return dx * dx + dy * dy > 0.25 * 0.25;
    }
};

struct SolveResult {
    bool exact = false;
    double seconds = 0;
    double length = 0;
    std::vector<double> xy;
};
struct BatchResult {
    unsigned long accepted = 0;
    double seconds = 0;
    std::vector<int> decisions;
};

// This owns an OMPL setup. It adds no virtual interface. The policy is the
// library's StateValidityChecker, installed into its native SpaceInformation.
class Engine {
    std::unique_ptr<og::SimpleSetup> setup_;
    void require_open() const {
        if (!setup_) throw std::runtime_error("engine is closed");
    }
public:
    Engine(double resolution = 0.001) {
        if (!std::isfinite(resolution) || resolution <= 0 || resolution > 1)
            throw std::invalid_argument("resolution must be finite and in (0, 1]");
        auto space = std::make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0.0);
        bounds.setHigh(1.0);
        space->setBounds(bounds);
        setup_.reset(new og::SimpleSetup(space));
        setup_->getSpaceInformation()->setStateValidityCheckingResolution(resolution);
        ob::ScopedState<ob::RealVectorStateSpace> start(space), goal(space);
        start[0] = 0.1; start[1] = 0.1;
        goal[0] = 0.9; goal[1] = 0.9;
        setup_->setStartAndGoalStates(start, goal);
        auto planner = std::make_shared<og::RRTConnect>(setup_->getSpaceInformation());
        planner->setRange(0.1);
        setup_->setPlanner(planner);
    }
    ob::SpaceInformationPtr space_information() const {
        require_open();
        return setup_->getSpaceInformation();
    }
    void attach(ob::StateValidityChecker* policy) {
        require_open();
        if (!policy) throw std::invalid_argument("null policy");
        // Python owns and pins policy. OMPL must never delete this pointer.
        setup_->setStateValidityChecker(ob::StateValidityCheckerPtr(
            policy, [](ob::StateValidityChecker*) {}));
        setup_->setup();
    }
    BatchResult check(const std::vector<double>& xy, unsigned long repeats) {
        require_open();
        if (xy.size() % 2) throw std::invalid_argument("xy requires pairs");
        auto si = setup_->getSpaceInformation();
        ob::ScopedState<ob::RealVectorStateSpace> state(si->getStateSpace());
        BatchResult result;
        result.decisions.reserve(xy.size() / 2);
        auto t = std::chrono::steady_clock::now();
        for (unsigned long r = 0; r < repeats; ++r) {
            for (std::size_t i = 0; i < xy.size(); i += 2) {
                state[0] = xy[i]; state[1] = xy[i + 1];
                const bool valid = si->isValid(state.get());
                result.accepted += valid;
                if (r == 0) result.decisions.push_back(valid);
            }
        }
        result.seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t).count();
        return result;
    }
    SolveResult solve(double timeout) {
        require_open();
        SolveResult result;
        auto t = std::chrono::steady_clock::now();
        result.exact = setup_->solve(timeout) == ob::PlannerStatus::EXACT_SOLUTION;
        result.seconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t).count();
        if (result.exact) {
            const auto& path = setup_->getSolutionPath();
            result.length = path.length();
            for (std::size_t i = 0; i < path.getStateCount(); ++i) {
                const auto* s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                result.xy.push_back((*s)[0]); result.xy.push_back((*s)[1]);
            }
        }
        return result;
    }
    void close() {
        if (!setup_) return;
        setup_->clear();
        // Detach the borrowed Python policy before its owner drops the pin.
        setup_->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(
            setup_->getSpaceInformation()));
        setup_.reset();
    }
    ~Engine() { close(); }
};
}
