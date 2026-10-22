#ifndef NAV2_KIT_PYTHON_CRITIC_HPP
#define NAV2_KIT_PYTHON_CRITIC_HPP
#include "dwb_core/trajectory_critic.hpp"
struct _object;
namespace nav2_kit_python {
// DWB retains its trajectory generator, collision critics and command selection.
class PythonCritic final : public dwb_core::TrajectoryCritic {
public:
  ~PythonCritic() override;
  void onInit() override;
  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override;
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &) override;
private:
  _object * policy_ = nullptr;
  _object * prepare_ = nullptr;
  _object * score_ = nullptr;
  double lookahead_ = 0.6;
  bool prepared_ = false;
};
}
#endif
