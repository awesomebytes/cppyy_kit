#pragma once
#include "image.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace pick_place {
inline std::string image_json(const sensor_msgs::msg::Image& input) {
  if (input.encoding != "rgb8" && input.encoding != "bgr8")
    throw std::invalid_argument("image encoding must be rgb8 or bgr8");
  if (input.step < input.width * 3 || input.data.size() != input.height * size_t(input.step))
    throw std::invalid_argument("invalid image step or payload length");
  cv::Mat mat(input.height, input.width, CV_8UC3,
              const_cast<unsigned char*>(input.data.data()), input.step);
  auto result = process_image(mat, input.encoding == "rgb8");
  std::ostringstream s;
  s << "{\"frame_id\":" << std::quoted(input.header.frame_id)
    << ",\"stamp_sec\":" << input.header.stamp.sec
    << ",\"stamp_nanosec\":" << input.header.stamp.nanosec << ",\"objects\":[";
  bool first = true;
  for (const auto& o : result.objects) {
    if (!first) s << ',';
    first = false;
    s << "{\"color\":" << std::quoted(o.color) << ",\"shape\":" << std::quoted(o.shape)
      << ",\"bbox\":[" << o.x << ',' << o.y << ',' << o.width << ',' << o.height
      << "],\"area\":" << o.area << '}';
  }
  return s.str() + "]}";
}
}
