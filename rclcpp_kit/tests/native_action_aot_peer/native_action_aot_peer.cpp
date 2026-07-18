#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_msgs/action/lookup_transform.hpp>
#include <tf2_msgs/msg/tf2_error.hpp>

namespace {

using namespace std::chrono_literals;
using Action = tf2_msgs::action::LookupTransform;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

constexpr auto kTargetFrame = "aot-target-314159";
constexpr auto kSourceFrame = "managed-source-271828";
constexpr auto kResultFrame = "aot-result-frame-161803";
constexpr auto kResultChild = "aot-result-child-141421";
constexpr auto kResultText = "aot-action-result-173205";
constexpr std::size_t kFeedbackCount = 3;

class ActionPeer
{
public:
  ActionPeer(const std::shared_ptr<rclcpp::Node> & node, const std::string & action_name)
  : node_(node), feedback_topic_(action_name + "/_action/feedback")
  {
    server_ = rclcpp_action::create_server<Action>(
      node,
      action_name,
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const Action::Goal> goal) {
        const bool exact =
          goal->target_frame == kTargetFrame &&
          goal->source_frame == kSourceFrame &&
          goal->timeout.sec == 1 &&
          goal->timeout.nanosec == 234567890U &&
          !goal->advanced;
        goal_was_exact_.store(exact, std::memory_order_release);
        return exact ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
               rclcpp_action::GoalResponse::REJECT;
      },
      [](const std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        worker_ = std::thread([this, goal_handle]() { execute(goal_handle); });
      });
  }

  ActionPeer(const ActionPeer &) = delete;
  ActionPeer & operator=(const ActionPeer &) = delete;

  ~ActionPeer()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  bool completed() const
  {
    return completed_.load(std::memory_order_acquire);
  }

  bool goal_was_exact() const
  {
    return goal_was_exact_.load(std::memory_order_acquire);
  }

private:
  void execute(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
    while (node_->count_subscribers(feedback_topic_) == 0 &&
      std::chrono::steady_clock::now() < discovery_deadline)
    {
      std::this_thread::sleep_for(1ms);
    }
    if (node_->count_subscribers(feedback_topic_) == 0) {
      return;
    }
    for (std::size_t index = 0; index < kFeedbackCount; ++index) {
      goal_handle->publish_feedback(std::make_shared<Action::Feedback>());
      std::this_thread::sleep_for(20ms);
    }
    // Leave time for the independent feedback topic callbacks to reach the client.
    std::this_thread::sleep_for(50ms);

    auto result = std::make_shared<Action::Result>();
    result->transform.header.frame_id = kResultFrame;
    result->transform.child_frame_id = kResultChild;
    result->transform.transform.translation.x = 31.4159;
    result->transform.transform.translation.y = 27.1828;
    result->transform.transform.translation.z = 1.61803;
    result->transform.transform.rotation.w = 1.0;
    result->error.error = tf2_msgs::msg::TF2Error::NO_ERROR;
    result->error.error_string = kResultText;
    goal_handle->succeed(result);
    completed_.store(true, std::memory_order_release);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::string feedback_topic_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::thread worker_;
  std::atomic<bool> completed_{false};
  std::atomic<bool> goal_was_exact_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: native_action_aot_peer ACTION_NAME\n";
    return 2;
  }
  const std::string action_name = argv[1];
  rclcpp::init(0, nullptr);
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("native_action_aot_server");
    ActionPeer peer(node, action_name);
    std::cout << "AOT_ACTION_SERVER_READY " << action_name << std::endl;

    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (rclcpp::ok() && !peer.completed() &&
      std::chrono::steady_clock::now() < deadline)
    {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(1ms);
    }
    if (!peer.completed()) {
      std::cerr << "AOT action server timed out waiting for a completed goal\n";
      result = 10;
    } else if (!peer.goal_was_exact()) {
      std::cerr << "AOT action server received unexpected goal values\n";
      result = 11;
    } else {
      std::cout << "AOT_ACTION_SERVER_OK goal=" << kTargetFrame << ':' << kSourceFrame
                << " feedback=" << kFeedbackCount << " result=" << kResultText
                << std::endl;
    }
  } catch (const std::exception & exception) {
    std::cerr << "AOT action peer exception: " << exception.what() << '\n';
    result = 12;
  }
  rclcpp::shutdown();
  std::cout << "AOT_ACTION_PEER_TEARDOWN_OK" << std::endl;
  return result;
}
