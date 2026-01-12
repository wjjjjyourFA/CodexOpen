#include "modules/perception/common/base/opencv_extra/cv_colors.h"

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
  return colors[index % colors.size()];
}

cv::Scalar GetColorByDistance(float distance) {
  static float max_range_ = 50.0;
  static float min_range_ = 0.5;

  // 将距离归一化到 [0, 1] 范围内
  double normalized_distance =
      (distance - min_range_) / (max_range_ - min_range_);
  normalized_distance = std::min(std::max(normalized_distance, 0.0), 1.0);

  // 使用HSV颜色空间：Hue从240°(蓝色)到0°(红色)
  double hue        = 240 * (1 - normalized_distance);  // 240°(蓝) -> 0°(红)
  double saturation = 1.0;
  double value      = 1.0;

  // 将HSV转换为BGR
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, saturation * 255, value * 255));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

  return cv::Scalar(bgr.data[0], bgr.data[1], bgr.data[2]);
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