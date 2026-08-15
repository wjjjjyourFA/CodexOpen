#ifndef MODULES_PERCEPTION_TOOLS_PCL_VIEWER_RUNNER_H_
#define MODULES_PERCEPTION_TOOLS_PCL_VIEWER_RUNNER_H_

#include <memory>

#include <pcl/visualization/pcl_visualizer.h>

#include "modules/perception/tools/common/visualization_config.h"
#include "modules/perception/tools/pcl/viewer_loop_state.h"

namespace jojo {
namespace perception {
namespace tools {

class ViewerRunner {
 public:
  ViewerRunner(pcl::visualization::PCLVisualizer::Ptr viewer,
               const ViewerRuntimeConfig& config = ViewerRuntimeConfig());
  virtual ~ViewerRunner() = default;

  bool IsValid() const;
  bool Tick();
  void Run();
  void RequestStop();

 private:
  pcl::visualization::PCLVisualizer::Ptr viewer_;
  ViewerRuntimeConfig config_;
  ViewerLoopState loop_state_;
};

}  // namespace tools
}  // namespace perception
}  // namespace jojo

#endif  // MODULES_PERCEPTION_TOOLS_PCL_VIEWER_RUNNER_H_
