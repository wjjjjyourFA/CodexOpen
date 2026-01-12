#include "modules/common/monitor_log/frequency_monitor.h"

namespace jojo {
namespace common {
namespace monitor {

void FrequencyMonitor::Start(const std::string& name) {
  if (!name_.empty()) name_ = name;
  start_time_ = Clock::now();
  started_    = true;
}

void FrequencyMonitor::End(int max_hz) {
  if (max_hz > 0) max_hz_ = max_hz;
  if (!started_) {
    std::cerr << name_ << " Error: Start() not called before End()."
              << std::endl;
    return;
  }

  end_time_ = Clock::now();

  const std::chrono::duration<double> elapsed = end_time_ - start_time_;

  if (elapsed.count() <= 0.0) {
    std::cerr << name_ << " Warning: invalid elapsed time." << std::endl;
    started_ = false;
    return;
  }

  // const double elapsed_ms = elapsed.count() * 1000.0;
  const double freq_hz = 1.0 / elapsed.count();
  std::cout << name_ << " run frequency: " << freq_hz << " Hz" << std::endl;

  if (freq_hz > max_hz_) {
    std::cerr << name_ << " Warning: frequency exceeds limit (" << freq_hz
              << " > " << max_hz_ << " Hz)" << std::endl;
  }

  started_ = false;
}

void FrequencyMonitor::LoopEnd(double expected_hz) {
  if (expected_hz > 0.0) expected_hz_ = expected_hz;
  if (!started_) {
    std::cerr << name_ << " Error: timeStart() before loopEnd()." << std::endl;
    return;
  }

  // 第一次记录当前时间（“工作完成时刻”）
  end_time_ = Clock::now();

  // 计算业务耗时
  const std::chrono::duration<double> elapsed = end_time_ - start_time_;

  const double desired_period_sec = 1.0 / expected_hz_;
  const std::chrono::duration<double> desired_period(desired_period_sec);

  if (elapsed < desired_period) {
    // 节奏控制应属于 scheduler，不是 monitor。
    // 如果跑得太快 → sleep 补齐周期
    // std::this_thread::sleep_for(desired_period - elapsed);
  } else {
    // 跑得太慢，给出 warning，方便调试性能瓶颈
    std::cerr << name_ << " Warning: loop overrun (" << elapsed.count() * 1000.0
              << " ms > " << desired_period.count() * 1000.0 << " ms)"
              << std::endl;
  }

  // 第二次取时间（“真实 loop 结束”）
  // end_time_ = Clock::now();

  const std::chrono::duration<double> total = end_time_ - start_time_;

  const double freq_hz = 1.0 / total.count();

  // 真实调度后的频率
  std::cout << name_ << " loop frequency: " << freq_hz << " Hz" << std::endl;

  started_ = false;
}

}  // namespace monitor
}  // namespace common
}  // namespace jojo
