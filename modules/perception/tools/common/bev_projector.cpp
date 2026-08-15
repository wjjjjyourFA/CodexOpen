#include "modules/perception/tools/common/bev_projector.h"

#include <cmath>

namespace jojo {
namespace perception {
namespace tools {

bool BevProjector::Project(float forward_x, float left_y,
                           BevPixel* pixel) const {
  if (pixel == nullptr || !config_.IsValid() || !std::isfinite(forward_x) ||
      !std::isfinite(left_y)) {
    return false;
  }

  pixel->x = static_cast<int>(-left_y * config_.pixels_per_meter +
                              static_cast<float>(config_.width) / 2.0f);
  pixel->y = static_cast<int>(static_cast<float>(config_.height) / 2.0f -
                              forward_x * config_.pixels_per_meter);
  return Contains(*pixel);
}

bool BevProjector::Contains(const BevPixel& pixel) const {
  return config_.IsValid() && pixel.x > 0 && pixel.y > 0 &&
         pixel.x < config_.width && pixel.y < config_.height;
}

}  // namespace tools
}  // namespace perception
}  // namespace jojo
