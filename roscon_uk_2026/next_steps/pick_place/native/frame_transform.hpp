#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/buffer_core.hpp>
namespace pick_place {
// The Python supervisor owns the buffer until every pipeline worker has joined.
void set_frame_buffer(tf2::BufferCore* buffer);
bool transform_frame(const sensor_msgs::msg::PointCloud2& input,
                     sensor_msgs::msg::PointCloud2& output,
                     const std::string& target);
}
