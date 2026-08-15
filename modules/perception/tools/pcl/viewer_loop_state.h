#ifndef MODULES_PERCEPTION_TOOLS_RUNNER_VIEWER_LOOP_STATE_H_
#define MODULES_PERCEPTION_TOOLS_RUNNER_VIEWER_LOOP_STATE_H_

#include <atomic>
#include <thread>

namespace jojo {
namespace perception {
namespace tools {

class ViewerLoopState {
 public:
  ViewerLoopState() : owner_thread_(std::this_thread::get_id()) {}

  bool IsOwnerThread() const {
    return owner_thread_ == std::this_thread::get_id();
  }
  void RequestStop() { stop_requested_.store(true); }
  bool ShouldContinue(bool viewer_stopped) const {
    return !viewer_stopped && !stop_requested_.load();
  }

 private:
  std::thread::id owner_thread_;
  std::atomic<bool> stop_requested_{false};
};

}  // namespace tools
}  // namespace perception
}  // namespace jojo

#endif  // MODULES_PERCEPTION_TOOLS_RUNNER_VIEWER_LOOP_STATE_H_
