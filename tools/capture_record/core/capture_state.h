#pragma once

#include <atomic>
#include <cstddef>

#include "tools/capture_record/common/common.h"

namespace jojo {
namespace tools {
namespace capture_record {

enum class CaptureState { kClosed, kOpening, kStreaming, kError };

// 录像状态统一使用 common/common.h 中供 CameraDevice、UI 和状态机共享的
// ::RecordState；这里不再维护第二套同义枚举。
// 可被 GUI、采集和编码线程共同观察的最小会话状态机。
class CaptureSessionState {
 public:
  bool OpenStarted() noexcept;
  bool OpenSucceeded() noexcept;
  void OpenFailed() noexcept;
  void Close() noexcept;
  bool StartRecording(std::size_t segment_seconds) noexcept;
  bool WriterOpened() noexcept;
  void WriterFailed() noexcept;
  void StopRecording() noexcept;

  CaptureState capture_state() const noexcept {
    return capture_state_.load(std::memory_order_acquire);
  }
  ::RecordState record_state() const noexcept {
    return record_state_.load(std::memory_order_acquire);
  }
  std::size_t dropped_frames() const noexcept {
    return dropped_frames_.load(std::memory_order_relaxed);
  }
  void CountDroppedFrame() noexcept {
    dropped_frames_.fetch_add(1, std::memory_order_relaxed);
  }
  void ResetDroppedFrames() noexcept {
    dropped_frames_.store(0, std::memory_order_relaxed);
  }

 private:
  std::atomic<CaptureState> capture_state_{CaptureState::kClosed};
  std::atomic<::RecordState> record_state_{::RecordState::Stopped};
  std::atomic<std::size_t> dropped_frames_{0};
};

}  // namespace capture_record
}  // namespace tools
}  // namespace jojo
