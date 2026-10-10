#pragma once
#include <cstdint>
#include <string>
#include <vector>
namespace typed_mcap {
struct Batch {
  std::vector<uint64_t> log_ns, publish_ns, offsets;
  std::vector<int64_t> header_ns;
  std::vector<double> position, quaternion;
  std::vector<uint32_t> width, height, step;
  std::vector<uint8_t> data;
  std::vector<uint8_t> is_bigendian;
  std::string frame_id, encoding;
  double container_ms = 0, decode_ms = 0;
};
Batch extract(const std::string& path, const std::string& topic, uint64_t start,
              uint64_t end, bool image, bool headers_only,
              const std::string& expected_definition);
uint64_t count_speed_above(const uint64_t* timestamps, const double* positions,
                           size_t count, double threshold_m_s);
}
