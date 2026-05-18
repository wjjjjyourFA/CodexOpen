#include "modules/perception/tools/opencv/cv_colors.h"

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

cv::Scalar GetColorByDistance(float distance, float max_range,
                              float min_range) {
  // 将距离归一化到 [0, 1] 范围内
  double t = (distance - min_range) / (max_range - min_range);

  t = std::min(std::max(t, 0.0), 1.0);

  // 使用HSV颜色空间：Hue从240°(蓝色)到0°(红色)
  double hue        = 240 * (1 - t);  // 240°(蓝) -> 0°(红)
  double saturation = 1.0;
  double value      = 1.0;

  // 将HSV转换为BGR
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, saturation * 255, value * 255));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

  return cv::Scalar(bgr.data[0], bgr.data[1], bgr.data[2]);
}

cv::Scalar GetColorByNorm(float t) {
  static const std::array<cv::Scalar, 256> lut = [] {
    std::array<cv::Scalar, 256> table{};
    for (int i = 0; i < 256; ++i) {
      float t = i / 255.0f;

      uint8_t r = 0, g = 0, b = 0;

      if (t < 0.33f) {
        float k = t * 3.03f;
        g       = 255 * k;
        b       = 255 * (1 - k);
      } else if (t < 0.66f) {
        float k = (t - 0.33f) * 3.03f;
        r       = 255 * k;
        g       = 255;
      } else {
        float k = (t - 0.66f) * 2.94f;
        r       = 255;
        g       = 255 * (1 - k);
      }

      table[i] = cv::Scalar(b, g, r);  // BGR
    }
    return table;
  }();

  // t = std::clamp(t, 0.0f, 1.0f);
  t = std::min(std::max(t, 0.0f), 1.0f);

  int idx = static_cast<int>(t * 255.0f);
  return lut[idx];
}

cv::Scalar GetColorByHeight(float height, float min_h, float max_h) {
  float range = max_h - min_h;
  if (range < 1e-6f) return GetColorByNorm(0.0f);

  float t = (height - min_h) / range;
  return GetColorByNorm(t);
}

cv::Scalar HSV2RGB(cv::Scalar scalar) {
  double R, G, B;
  int Hi   = (int)abs(scalar.val[0] / 60.0);
  double f = scalar.val[0] / 60.0 - Hi;
  double a = scalar.val[2] * (1 - scalar.val[1]);
  double b = scalar.val[2] * (1 - f * scalar.val[1]);
  double c = scalar.val[2] * (1 - (1 - f) * scalar.val[1]);

  switch (Hi) {
    case 0:
      R = scalar.val[2];
      G = c;
      B = a;
      break;
    case 1:
      R = b;
      G = scalar.val[2];
      B = a;
      break;
    case 2:
      R = a;
      G = scalar.val[2];
      B = c;
      break;
    case 3:
      R = a;
      G = b;
      B = scalar.val[2];
      break;
    case 4:
      R = c;
      G = a;
      B = scalar.val[2];
      break;
    case 5:
      R = scalar.val[2];
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