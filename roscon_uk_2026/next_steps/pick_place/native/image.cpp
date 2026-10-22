#include "image.hpp"
#include <opencv2/imgproc.hpp>
#include <stdexcept>
namespace pick_place {
ImageResult process_image(const cv::Mat& image, bool rgb, double min_area) {
  if (image.type() != CV_8UC3 || min_area <= 0)
    throw std::invalid_argument("expected uint8 three-channel image and positive minimum area");
  ImageResult result;
  cv::Mat hsv;
  cv::cvtColor(image, hsv, rgb ? cv::COLOR_RGB2HSV : cv::COLOR_BGR2HSV);
  const std::vector<std::string> colors = {"red", "blue", "yellow"};
  const int lows[] = {0, 95, 18}, highs[] = {10, 135, 38};
  for (int i = 0; i < 3; ++i) {
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(lows[i], 85, 45), cv::Scalar(highs[i], 255, 255), mask);
    if (i == 0) {
      cv::Mat high_red;
      cv::inRange(hsv, cv::Scalar(170, 85, 45), cv::Scalar(179, 255, 255), high_red);
      cv::bitwise_or(mask, high_red, mask);
    }
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    result.masks.push_back(mask);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
      double area = cv::contourArea(contour);
      if (area < min_area) continue;
      double perimeter = cv::arcLength(contour, true);
      std::vector<cv::Point> polygon;
      cv::approxPolyDP(contour, polygon, 0.035 * perimeter, true);
      auto bounds = cv::boundingRect(contour);
      ImageObject o;
      o.color = colors[i];
      o.area = area;
      o.circularity = perimeter > 0 ? 4 * CV_PI * area / (perimeter * perimeter) : 0;
      // These describe image silhouettes, not physical object classes.
      o.shape = polygon.size() == 3 ? "triangular" :
                polygon.size() == 4 ? "quadrilateral" :
                (polygon.size() == 5 || polygon.size() == 6) ? "polygon" :
                o.circularity > 0.78 ? "round" : "irregular";
      o.x = bounds.x; o.y = bounds.y; o.width = bounds.width; o.height = bounds.height;
      result.objects.push_back(o);
    }
  }
  return result;
}
}
