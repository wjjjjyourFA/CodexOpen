#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

class SerialRS232 {
 public:
  explicit SerialRS232(const std::string& device = "/dev/ttyS0",
                       int baudrate              = 115200);

  ~SerialRS232();

  // 打开并配置串口
  void openPort();
  // 关闭串口
  void closePort();
  // 是否已打开
  bool isOpen() const;

  // 写数据
  ssize_t writeData(const void* data, size_t size);
  ssize_t writeString(const std::string& str);

  // ===== Trigger 接口 =====
  void startLowLevel();  // 开始维持低电平
  void stopLowLevel();  // 停止（释放 TX）
  void triggerHigh(int high_ms);  // 触发一次高电平

 private:
  bool configurePort();
  void triggerTxLoop();  // 后台线程

  int fd_{-1};
  std::string device_;
  int baudrate_;

  // ===== Trigger 相关 =====
  std::atomic<bool> trigger_running_{false};
  std::atomic<bool> trigger_high_{false};
  std::thread trigger_thread_;
  std::mutex write_mutex_;
  std::mutex trigger_pulse_mutex_;
};
