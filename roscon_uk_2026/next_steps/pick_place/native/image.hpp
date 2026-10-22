#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>
namespace pick_place {
struct ImageObject {
  std::string color, shape;
  int x = 0, y = 0, width = 0, height = 0;
  double area = 0, circularity = 0;
};
struct ImageResult {
  // One owned mono8 mask per color, ordered red, blue, yellow.
  std::vector<cv::Mat> masks;
  std::vector<ImageObject> objects;
};
ImageResult process_image(const cv::Mat&, bool rgb = true, double min_area = 40);
}
