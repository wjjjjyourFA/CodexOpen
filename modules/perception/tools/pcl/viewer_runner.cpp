#include "modules/perception/tools/pcl/viewer_runner.h"

#include <chrono>
#include <thread>

namespace jojo {
namespace perception {
namespace tools {

ViewerRunner::ViewerRunner(pcl::visualization::PCLVisualizer::Ptr viewer,
                           const ViewerRuntimeConfig& config)
    : viewer_(std::move(viewer)), config_(config) {}

bool ViewerRunner::IsValid() const {
  return viewer_ != nullptr && config_.IsValid() && loop_state_.IsOwnerThread();
}

bool ViewerRunner::Tick() {
  if (!IsValid() || !loop_state_.ShouldContinue(viewer_->wasStopped())) {
    return false;
  }
  viewer_->spinOnce(config_.spin_once_ms);
  return loop_state_.ShouldContinue(viewer_->wasStopped());
}

void ViewerRunner::Run() {
  while (Tick()) {
    if (config_.idle_sleep_ms > 0) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(config_.idle_sleep_ms));
    }
  }
}

void ViewerRunner::RequestStop() { loop_state_.RequestStop(); }

}  // namespace tools
}  // namespace perception
}  // namespace jojo
