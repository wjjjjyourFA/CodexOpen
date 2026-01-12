#pragma once

#if defined(__linux__) || defined(__FreeBSD__)
#define __BSDSOCKETS__
#endif

#include <chrono>
#include <ctime>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>

// For writing to spdlog files
// #include <spdlog/spdlog.h>
// #include <spdlog/sinks/basic_file_sink.h>

namespace jojo {
namespace common {
namespace monitor {

// FrequencyMonitor is a passive observer.
// It MUST NOT sleep or control loop timing.
class FrequencyMonitor {
 public:
  FrequencyMonitor() = default;
  explicit FrequencyMonitor(const std::string& name) : name_(name) {}
  virtual ~FrequencyMonitor() = default;

  void Start(const std::string& name = "");

  void End(int maxHz_ = -1);

  void LoopEnd(double expected_hz = -1.0);

 private:
  using Clock = std::chrono::steady_clock;

  std::string name_ = "default";

  bool started_{false};
  int max_hz_{50};
  double expected_hz_{20.0};

  Clock::time_point start_time_, end_time_;
};

/* 业务代码
FrequencyMonitor freq("image_tracking");

while (running) {
  freq.Start();

  DoImageTracking();
  DoDataAssociation();
  PublishResult();

  freq.LoopEnd(20.0);  // 希望 20Hz
}
*/

}  // namespace monitor
}  // namespace common
}  // namespace jojo
