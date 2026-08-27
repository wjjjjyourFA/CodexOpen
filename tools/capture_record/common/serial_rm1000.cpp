#include "tools/capture_record/common/serial_rm1000.h"

SerialRM1000::SerialRM1000(int pin, int rw) : pin_(pin), rw_(rw) {
  this->openPort();
}

SerialRM1000::~SerialRM1000() {
  stopLowLevel();
  closePort();
}

void SerialRM1000::openPort() {
  if (ret_ >= 0) return;

  ret_ = UsbDevice_Scan(SerialNumbers);
  if (ret_ < 0) {
    throw std::runtime_error("Failed to SCAN " + device_ +
                             " , Please Check!!!");
  } else if (ret_ == 0) {
    throw std::runtime_error("Failed to open " + device_ +
                             " , Please Check!!!");
  } else {
    for (int i = 0; i < ret_; i++) {
      printf("Dev%d SN: %d\n", i, SerialNumbers[i]);
    }
  }

  // 选择第一个设备
  sn = SerialNumbers[0];

  if (!configurePort()) {
    closePort();
    throw std::runtime_error("Failed to configure " + device_);
  }
}

void SerialRM1000::closePort() {
  stopLowLevel();
  if (ret_ >= 0) {
    // close();
    ret_ = -1;
  }
}

bool SerialRM1000::isOpen() const { return ret_ >= 0; }

bool SerialRM1000::configurePort() {
  // 配置指定 pin 口
  ret_ = IO_InitPin(sn, pin_, rw_, 0);
  if (ret_ < 0) {
    printf("RM1000 Error: %d\n", ret_);
    return false;
  }
  // std::cout << "configurePort(): ret " << ret_ << std::endl;
  return true;
}

/*
ssize_t SerialRM1000::sendTrigger(int high_time_ms) {
  if (fd < 0) return;

  const uint8_t LOW  = 0x00;  // 数据位全 0 → TX 低
  const uint8_t HIGH = 0xFF;  // 数据位全 1 → TX 高

  // ===== 确保默认是低电平 =====
  for (int i = 0; i < 20; ++i) {
    write(fd, &LOW, 1);
  }
  tcdrain(fd);

  // ===== 上升沿 + 保持高电平 =====
  auto start = std::chrono::steady_clock::now();
  while (true) {
    write(fd, &HIGH, 1);

    auto now = std::chrono::steady_clock::now();
    int elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
            .count();

    if (elapsed_ms >= high_time_ms) break;
  }
  tcdrain(fd);

  // ===== 回归低电平 =====
  for (int i = 0; i < 20; ++i) {
    write(fd, &LOW, 1);
  }
  tcdrain(fd);
}
*/

void SerialRM1000::triggerTxLoop() {
  const uint8_t LOW  = 0;  // 持续压低
  const uint8_t HIGH = 1;  // 持续拉高

  std::unique_lock<std::mutex> lock(trigger_mutex_);

  while (trigger_running_) {
    trigger_cv_.wait(lock, [&] { return !trigger_running_ || trigger_dirty_; });

    if (!trigger_running_) break;

    uint8_t v = trigger_high_ ? HIGH : LOW;

    trigger_dirty_ = false;

    lock.unlock();  // 写 GPIO 时不要持锁
    state_ = IO_WritePin(sn, pin_, v);
    if (state_ < 0) {
      std::cerr << "RM1000 IO_WritePin failed: " << state_ << std::endl;
    }
    lock.lock();
  }
}

void SerialRM1000::startLowLevel() {
  if (!isOpen()) return;

  std::lock_guard<std::mutex> lock(trigger_mutex_);

  if (trigger_running_) return;

  trigger_running_ = true;
  trigger_high_    = false;
  trigger_dirty_   = true;  // 初始写一次 LOW

  trigger_thread_ = std::thread(&SerialRM1000::triggerTxLoop, this);
}

void SerialRM1000::triggerHigh(int high_ms) {
  if (high_ms <= 0) return;

  std::lock_guard<std::mutex> pulse_lock(trigger_pulse_mutex_);

  {
    std::lock_guard<std::mutex> lock(trigger_mutex_);
    if (!trigger_running_) return;

    trigger_high_  = true;
    trigger_dirty_ = true;
  }

  trigger_cv_.notify_one();

  std::this_thread::sleep_for(std::chrono::milliseconds(high_ms));

  {
    std::lock_guard<std::mutex> lock(trigger_mutex_);
    if (!trigger_running_) return;

    trigger_high_  = false;
    trigger_dirty_ = true;
  }

  trigger_cv_.notify_one();
}

void SerialRM1000::stopLowLevel() {
  {
    std::lock_guard<std::mutex> lock(trigger_mutex_);
    if (!trigger_running_) return;

    trigger_running_ = false;
    trigger_dirty_   = true;
  }

  trigger_cv_.notify_one();

  if (trigger_thread_.joinable()) trigger_thread_.join();

  if (isOpen()) {
    state_ = IO_WritePin(sn, pin_, 0);
    if (state_ < 0) {
      std::cerr << "RM1000 failed to restore LOW level: " << state_
                << std::endl;
    }
  }
}
