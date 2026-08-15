#ifndef PERCEPTION_TOOLS_OPENCV_COMMON
#define PERCEPTION_TOOLS_OPENCV_COMMON

#include <opencv2/core.hpp>

#include "modules/perception/tools/common/visualization_config.h"

namespace jojo {
namespace perception {
namespace tools {

struct BgrColor {
  std::uint8_t blue  = 0;
  std::uint8_t green = 0;
  std::uint8_t red   = 0;

  bool operator==(const BgrColor& other) const {
    return blue == other.blue && green == other.green && red == other.red;
  }
};

inline cv::Scalar ToCvScalar(const BgrColor& color) {
  return cv::Scalar(color.blue, color.green, color.red);
}

inline bool IsValidBgrImage(const cv::Mat& image) {
  return !image.empty() && image.type() == CV_8UC3;
}

inline std::uint8_t ToByte(float value) {
  const float clamped = std::min(std::max(value, 0.0f), 255.0f);
  return static_cast<std::uint8_t>(clamped);
}

inline BgrColor ColorByNormalizedValue(float value) {
  if (!std::isfinite(value)) return {};

  const float t = std::min(std::max(value, 0.0f), 1.0f);
  BgrColor color;
  if (t < 0.33f) {
    const float k = t * 3.03f;
    color.green   = ToByte(255.0f * k);
    color.blue    = ToByte(255.0f * (1.0f - k));
  } else if (t < 0.66f) {
    const float k = (t - 0.33f) * 3.03f;
    color.red     = ToByte(255.0f * k);
    color.green   = 255;
  } else {
    const float k = (t - 0.66f) * 2.94f;
    color.red     = 255;
    color.green   = ToByte(255.0f * (1.0f - k));
  }
  return color;
}

inline BgrColor ColorByDistance(float distance,
                                const ScalarRangeConfig& range) {
  if (!std::isfinite(distance) || !range.IsValid()) return {};
  return ColorByNormalizedValue((distance - range.min) /
                                (range.max - range.min));
}

inline BgrColor ColorByHeight(float height, const ScalarRangeConfig& range) {
  if (!std::isfinite(height) || !range.IsValid()) return {};
  return ColorByNormalizedValue((height - range.min) / (range.max - range.min));
}

}  // namespace tools
}  // namespace perception
}  // namespace jojo

#endif  // PERCEPTION_TOOLS_OPENCV_COMMON
