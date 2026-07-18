#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

using namespace std::chrono_literals;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using State = lifecycle_msgs::msg::State;
using Transition = lifecycle_msgs::msg::Transition;

constexpr auto kServiceWait = 10s;
constexpr auto kResponseWait = 5s;

State get_state(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Client<GetState>::SharedPtr & client)
{
  auto request = std::make_shared<GetState::Request>();
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, kResponseWait) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("timed out waiting for get_state response");
  }
  return future.get()->current_state;
}

void expect_state(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Client<GetState>::SharedPtr & client,
  std::uint8_t expected_id,
  const std::string & expected_label,
  std::ostringstream & trace)
{
  const auto state = get_state(node, client);
  if (state.id != expected_id || state.label != expected_label) {
    std::ostringstream error;
    error << "unexpected lifecycle state: id=" << static_cast<unsigned int>(state.id)
          << " label=" << state.label << " expected_id="
          << static_cast<unsigned int>(expected_id)
          << " expected_label=" << expected_label;
    throw std::runtime_error(error.str());
  }
  if (trace.tellp() > 0) {
    trace << ',';
  }
  trace << static_cast<unsigned int>(state.id) << ':' << state.label;
}

void change_state(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Client<ChangeState>::SharedPtr & client,
  std::uint8_t transition_id)
{
  auto request = std::make_shared<ChangeState::Request>();
  request->transition.id = transition_id;
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, kResponseWait) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error("timed out waiting for change_state response");
  }
  if (!future.get()->success) {
    std::ostringstream error;
    error << "transition " << static_cast<unsigned int>(transition_id)
          << " was rejected";
    throw std::runtime_error(error.str());
  }
}

int run(const std::string & lifecycle_name)
{
  auto node = std::make_shared<rclcpp::Node>("native_lifecycle_aot_client");
  const std::string prefix = "/" + lifecycle_name;
  auto get_state_client = node->create_client<GetState>(prefix + "/get_state");
  auto change_state_client = node->create_client<ChangeState>(prefix + "/change_state");

  if (!get_state_client->wait_for_service(kServiceWait)) {
    std::cerr << "AOT lifecycle client timed out waiting for get_state\n";
    return 10;
  }
  if (!change_state_client->wait_for_service(kServiceWait)) {
    std::cerr << "AOT lifecycle client timed out waiting for change_state\n";
    return 11;
  }

  std::ostringstream trace;
  expect_state(
    node, get_state_client, State::PRIMARY_STATE_UNCONFIGURED, "unconfigured", trace);
  change_state(node, change_state_client, Transition::TRANSITION_CONFIGURE);
  expect_state(node, get_state_client, State::PRIMARY_STATE_INACTIVE, "inactive", trace);
  change_state(node, change_state_client, Transition::TRANSITION_ACTIVATE);
  expect_state(node, get_state_client, State::PRIMARY_STATE_ACTIVE, "active", trace);
  change_state(node, change_state_client, Transition::TRANSITION_DEACTIVATE);
  expect_state(node, get_state_client, State::PRIMARY_STATE_INACTIVE, "inactive", trace);
  change_state(node, change_state_client, Transition::TRANSITION_CLEANUP);
  expect_state(
    node, get_state_client, State::PRIMARY_STATE_UNCONFIGURED, "unconfigured", trace);

  std::cout << "AOT_LIFECYCLE_OK " << trace.str() << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: native_lifecycle_aot_peer LIFECYCLE_NODE_NAME\n";
    return 2;
  }

  rclcpp::init(0, nullptr);
  int result = 0;
  try {
    result = run(argv[1]);
  } catch (const std::exception & exception) {
    std::cerr << "AOT lifecycle client exception: " << exception.what() << '\n';
    result = 3;
  }
  rclcpp::shutdown();
  std::cout << "AOT_LIFECYCLE_TEARDOWN_OK" << std::endl;
  return result;
}
