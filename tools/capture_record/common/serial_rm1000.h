#pragma once

#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "stdio.h"
#include "tools/capture_record/rm1000/source/gpio.h"
#include "tools/capture_record/rm1000/source/usb_device.h"

class SerialRM1000 {
 public:
  explicit SerialRM1000(int pin = 8, int rw = 1);

  ~SerialRM1000();

  // 打开并配置串口
  void openPort();
  // 关闭串口
  void closePort();
  // 是否已打开
  bool isOpen() const;

  // ===== Trigger 接口 =====
  void startLowLevel();  // 开始维持低电平
  void stopLowLevel();  // 停止（释放 TX）
  void triggerHigh(int high_ms);  // 触发一次高电平

 private:
  bool configurePort();
  void triggerTxLoop();  // 后台线程

  int ret_{-1};
  std::string device_ = "RM1000";
  int SerialNumbers[16]{};
  int pin_, rw_, state_{0};
  int sn{-1};

  // ===== Trigger 相关 =====
  std::atomic<bool> trigger_running_{false};
  std::atomic<bool> trigger_high_{false};
  std::atomic<bool> trigger_dirty_{false};
  std::thread trigger_thread_;
  std::mutex trigger_mutex_;
  std::condition_variable trigger_cv_;
  std::mutex trigger_pulse_mutex_;
};
