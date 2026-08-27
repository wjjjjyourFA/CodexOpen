#include "tools/capture_record/core/capture_state.h"

namespace jojo {
namespace tools {
namespace capture_record {

bool CaptureSessionState::OpenStarted() noexcept {
  CaptureState expected = CaptureState::kClosed;
  return capture_state_.compare_exchange_strong(
      expected, CaptureState::kOpening, std::memory_order_acq_rel);
}

bool CaptureSessionState::OpenSucceeded() noexcept {
  CaptureState expected = CaptureState::kOpening;
  return capture_state_.compare_exchange_strong(
      expected, CaptureState::kStreaming, std::memory_order_acq_rel);
}

void CaptureSessionState::OpenFailed() noexcept {
  capture_state_.store(CaptureState::kError, std::memory_order_release);
  record_state_.store(::RecordState::Error, std::memory_order_release);
}

void CaptureSessionState::Close() noexcept {
  capture_state_.store(CaptureState::kClosed, std::memory_order_release);
  record_state_.store(::RecordState::Stopped, std::memory_order_release);
}

bool CaptureSessionState::StartRecording(
    std::size_t segment_seconds) noexcept {
  if (capture_state() != CaptureState::kStreaming || segment_seconds == 0) {
    return false;
  }
  ::RecordState expected = ::RecordState::Stopped;
  if (!record_state_.compare_exchange_strong(
          expected, ::RecordState::Starting, std::memory_order_acq_rel)) {
    return false;
  }
  ResetDroppedFrames();
  return true;
}

bool CaptureSessionState::WriterOpened() noexcept {
  ::RecordState expected = ::RecordState::Starting;
  return record_state_.compare_exchange_strong(
      expected, ::RecordState::Recording, std::memory_order_acq_rel);
}

void CaptureSessionState::WriterFailed() noexcept {
  record_state_.store(::RecordState::Error, std::memory_order_release);
}

void CaptureSessionState::StopRecording() noexcept {
  ::RecordState state = record_state();
  while (state == ::RecordState::Recording ||
         state == ::RecordState::Starting || state == ::RecordState::Error) {
    if (record_state_.compare_exchange_weak(
            state, ::RecordState::Stopping, std::memory_order_acq_rel)) {
      break;
    }
  }
  record_state_.store(::RecordState::Stopped, std::memory_order_release);
}

}  // namespace capture_record
}  // namespace tools
}  // namespace jojo
