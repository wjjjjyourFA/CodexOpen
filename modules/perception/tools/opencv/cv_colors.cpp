#include "modules/perception/tools/opencv/cv_colors.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace perception_tools = jojo::perception::tools;

// 随机颜色生成器（或者预设颜色列表）
cv::Scalar GetColor(int index) {
  static const std::vector<cv::Scalar> colors = {
      cv::Scalar(255, 0, 0),  // Blue 蓝
      cv::Scalar(0, 255, 0),  // Green 绿
      cv::Scalar(0, 0, 255),  // Red 红
      cv::Scalar(255, 255, 0),  // Cyan 青
      cv::Scalar(255, 0, 255),  // Magenta 品红
      cv::Scalar(0, 255, 255),  // Yellow 黄
      cv::Scalar(128, 0, 255),  // Purple 紫
      cv::Scalar(255, 128, 0)  // Orange 橙
  };
  const auto color_count = static_cast<int>(colors.size());
  const auto normalized_index =
      ((index % color_count) + color_count) % color_count;
  return colors[static_cast<std::size_t>(normalized_index)];
}

cv::Scalar GetColorByDistance(float distance, float max_range,
                              float min_range) {
  return perception_tools::ToCvScalar(perception_tools::ColorByDistance(
      distance, perception_tools::ScalarRangeConfig{min_range, max_range}));
}

cv::Scalar GetColorByNorm(float t) {
  return perception_tools::ToCvScalar(
      perception_tools::ColorByNormalizedValue(t));
}

cv::Scalar GetColorByHeight(float height, float min_h, float max_h) {
  return perception_tools::ToCvScalar(perception_tools::ColorByHeight(
      height, perception_tools::ScalarRangeConfig{min_h, max_h}));
}

cv::Scalar HSV2RGB(cv::Scalar scalar) {
  if (!std::isfinite(scalar.val[0]) || !std::isfinite(scalar.val[1]) ||
      !std::isfinite(scalar.val[2])) {
    return cv::Scalar(0, 0, 0);
  }

  double hue = std::fmod(scalar.val[0], 360.0);
  if (hue < 0.0) hue += 360.0;
  const double saturation = std::min(std::max(scalar.val[1], 0.0), 1.0);
  const double value      = std::min(std::max(scalar.val[2], 0.0), 1.0);

  double R = 0.0, G = 0.0, B = 0.0;
  const int Hi   = static_cast<int>(hue / 60.0) % 6;
  const double f = hue / 60.0 - Hi;
  const double a = value * (1 - saturation);
  const double b = value * (1 - f * saturation);
  const double c = value * (1 - (1 - f) * saturation);

  switch (Hi) {
    case 0:
      R = value;
      G = c;
      B = a;
      break;
    case 1:
      R = b;
      G = value;
      B = a;
      break;
    case 2:
      R = a;
      G = value;
      B = c;
      break;
    case 3:
      R = a;
      G = b;
      B = value;
      break;
    case 4:
      R = c;
      G = a;
      B = value;
      break;
    case 5:
      R = value;
      G = a;
      B = b;
      break;
    default:
      break;
  }

  return cv::Scalar(R * 255, G * 255, B * 255);
}

void DrawText(cv::Mat& image, const cv::Point& anchor, const std::string& txt,
              const cv::Scalar& color, int offset_y, double font_scale,
              int thickness) {
  // 计算文本高度（取最大字体基准）
  // int baseline = 0;
  // cv::Size text_size =
  //     cv::getTextSize(text_distance, cv::FONT_HERSHEY_COMPLEX, label_scale,
  //                     box_thick * 2 / 3, &baseline);
  // int text_h = text_size.height;

  cv::Point pos;
  if (offset_y == 0) {
    pos = anchor;
  } else {
    pos = cv::Point(anchor.x, anchor.y + offset_y);
  }

  cv::putText(image, txt, pos, cv::FONT_HERSHEY_COMPLEX, font_scale, color,
              thickness, cv::LINE_AA);
};
