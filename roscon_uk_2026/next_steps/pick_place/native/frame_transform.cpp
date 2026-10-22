#include "frame_transform.hpp"
#include <tf2/LinearMath/Transform.hpp>
#include <cstring>
#include <chrono>
#include <stdexcept>
#include <thread>
namespace pick_place {
namespace { tf2::BufferCore* frame_buffer = nullptr; }
void set_frame_buffer(tf2::BufferCore* buffer) { frame_buffer = buffer; }
bool transform_frame(const sensor_msgs::msg::PointCloud2& input,
                     sensor_msgs::msg::PointCloud2& output,
                     const std::string& target) {
  if (!frame_buffer) throw std::runtime_error("frame buffer not configured");
  auto stamp = tf2::TimePoint(std::chrono::seconds(input.header.stamp.sec) +
                              std::chrono::nanoseconds(input.header.stamp.nanosec));
  geometry_msgs::msg::TransformStamped transform;
  // Camera and TF arrive on independent ROS streams. Bound the wait for the
  // exact camera timestamp while the executor continues ingesting transforms.
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (true) {
    try {
      transform = frame_buffer->lookupTransform(target, input.header.frame_id, stamp);
      break;
    } catch (const tf2::TransformException&) {
      if (std::chrono::steady_clock::now() >= deadline) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  if (input.is_bigendian) throw std::invalid_argument("big-endian cloud unsupported");
  int offsets[3] = {-1, -1, -1};
  for (const auto& field : input.fields) {
    for (int k=0; k<3; ++k) {
      if (field.name == std::string(1, "xyz"[k])) {
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            field.offset + sizeof(float) > input.point_step)
          throw std::invalid_argument("cloud requires float32 XYZ fields");
        offsets[k] = static_cast<int>(field.offset);
      }
    }
  }
  for (int offset : offsets) if (offset < 0) throw std::invalid_argument("missing XYZ");
  if (input.row_step < input.width * input.point_step ||
      input.data.size() < static_cast<size_t>(input.row_step) * input.height)
    throw std::invalid_argument("truncated cloud");
  auto& t = transform.transform.translation;
  auto& q = transform.transform.rotation;
  tf2::Transform affine(tf2::Quaternion(q.x,q.y,q.z,q.w), tf2::Vector3(t.x,t.y,t.z));
  output = input;
  output.header.frame_id = target;
  for (size_t row=0; row<input.height; ++row) {
    for (size_t col=0; col<input.width; ++col) {
      auto base = row * input.row_step + col * input.point_step;
      float xyz[3];
      for(int k=0;k<3;++k) std::memcpy(&xyz[k], input.data.data()+base+offsets[k], 4);
      auto point = affine * tf2::Vector3(xyz[0], xyz[1], xyz[2]);
      for(int k=0;k<3;++k) {
        float value = point[k];
        std::memcpy(output.data.data()+base+offsets[k], &value, 4);
      }
    }
  }
  return true;
}
}
