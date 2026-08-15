#ifndef MODULES_PERCEPTION_TOOLS_CORE_BEV_PROJECTOR_H_
#define MODULES_PERCEPTION_TOOLS_CORE_BEV_PROJECTOR_H_

#include "modules/perception/tools/common/visualization_config.h"

namespace jojo {
namespace perception {
namespace tools {

struct BevPixel {
  int x = 0;
  int y = 0;
};

class BevProjector {
 public:
  explicit BevProjector(const BevRenderConfig& config) : config_(config) {}

  bool Project(float forward_x, float left_y, BevPixel* pixel) const;
  bool Contains(const BevPixel& pixel) const;
  const BevRenderConfig& config() const { return config_; }

 private:
  BevRenderConfig config_;
};

}  // namespace tools
}  // namespace perception
}  // namespace jojo

#endif  // MODULES_PERCEPTION_TOOLS_CORE_BEV_PROJECTOR_H_
