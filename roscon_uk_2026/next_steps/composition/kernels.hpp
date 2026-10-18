#pragma once
#include <chrono>
#include <numeric>
#include <thread>
#include <vector>

namespace quick_composition {
// Element zero is the sequence number. Remaining elements are synthetic pixels.
inline std::vector<double> affine(const std::vector<double>& input) {
  auto output = input;
  for (std::size_t i = 1; i < output.size(); ++i) {
    output[i] = 2.0 * input[i] + 1.0;
  }
  return output;
}
inline std::vector<double> summary(const std::vector<double>& input) {
  return {input.at(0), std::accumulate(input.begin() + 1, input.end(), 0.0)};
}
inline void simulate_work() {
  // Deliberate latency, not a benchmark of image processing or CPU throughput.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}
}
