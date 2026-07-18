#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace {

using namespace std::chrono_literals;
using Service = std_srvs::srv::SetBool;

constexpr auto kManagedResponse = "managed-native-service:enabled:314159";
constexpr auto kAotResponse = "aot-service:enabled:271828";

int run_client(const std::string & service_name)
{
  auto node = std::make_shared<rclcpp::Node>("native_service_aot_client");
  auto client = node->create_client<Service>(service_name);
  if (!client->wait_for_service(10s)) {
    std::cerr << "AOT client timed out waiting for " << service_name << '\n';
    return 10;
  }

  auto request = std::make_shared<Service::Request>();
  request->data = true;
  auto future = client->async_send_request(request);
  const auto status = rclcpp::spin_until_future_complete(node, future, 10s);
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    std::cerr << "AOT client timed out waiting for its response\n";
    return 11;
  }

  const auto response = future.get();
  if (!response->success || response->message != kManagedResponse) {
    std::cerr << "AOT client received an unexpected response: success="
              << response->success << " message=" << response->message << '\n';
    return 12;
  }
  std::cout << "AOT_CLIENT_OK " << response->message << std::endl;
  return 0;
}

int run_server(const std::string & service_name)
{
  auto node = std::make_shared<rclcpp::Node>("native_client_aot_server");
  std::atomic<bool> handled{false};
  auto service = node->create_service<Service>(
    service_name,
    [&handled](
      const std::shared_ptr<Service::Request> request,
      std::shared_ptr<Service::Response> response) {
      response->success = request->data;
      response->message = request->data ? kAotResponse : "aot-service:disabled";
      handled.store(true, std::memory_order_release);
    });
  (void)service;
  std::cout << "AOT_SERVER_READY " << service_name << std::endl;

  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (rclcpp::ok() && !handled.load(std::memory_order_acquire) &&
    std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(1ms);
  }
  if (!handled.load(std::memory_order_acquire)) {
    std::cerr << "AOT server timed out waiting for a request\n";
    return 20;
  }
  std::cout << "AOT_SERVER_OK " << kAotResponse << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: native_service_aot_peer client|server SERVICE_NAME\n";
    return 2;
  }
  const std::string mode = argv[1];
  const std::string service_name = argv[2];
  rclcpp::init(0, nullptr);
  int result = 0;
  try {
    if (mode == "client") {
      result = run_client(service_name);
    } else if (mode == "server") {
      result = run_server(service_name);
    } else {
      std::cerr << "unknown mode: " << mode << '\n';
      result = 3;
    }
  } catch (const std::exception & exception) {
    std::cerr << "AOT peer exception: " << exception.what() << '\n';
    result = 4;
  }
  rclcpp::shutdown();
  std::cout << "AOT_PEER_TEARDOWN_OK " << mode << std::endl;
  return result;
}
