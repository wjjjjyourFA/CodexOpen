#ifndef MODULES_PERCEPTION_TOOLS_VISUALIZATION_CONFIG_H_
#define MODULES_PERCEPTION_TOOLS_VISUALIZATION_CONFIG_H_

#include <cmath>

namespace jojo {
namespace perception {
namespace tools {

struct BevRenderConfig {
  int width = 1024;
  int height = 768;
  float pixels_per_meter = 5.0f;

  bool IsValid() const {
    return width > 0 && height > 0 && std::isfinite(pixels_per_meter) &&
           pixels_per_meter > 0.0f;
  }
};

struct ScalarRangeConfig {
  float min = 0.0f;
  float max = 1.0f;

  bool IsValid() const {
    return std::isfinite(min) && std::isfinite(max) && max > min;
  }
};

struct ViewerRuntimeConfig {
  int spin_once_ms = 10;
  int idle_sleep_ms = 10;

  bool IsValid() const { return spin_once_ms >= 0 && idle_sleep_ms >= 0; }
};

}  // namespace tools
}  // namespace perception
}  // namespace jojo

#endif  // MODULES_PERCEPTION_TOOLS_CONFIG_VISUALIZATION_CONFIG_H_
