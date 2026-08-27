#include "tools/capture_record/common/serial_rs232.h"

SerialRS232::SerialRS232(const std::string& device, int baudrate)
    : device_(device), baudrate_(baudrate) {
  this->openPort();
}

SerialRS232::~SerialRS232() {
  stopLowLevel();
  closePort();
}

void SerialRS232::openPort() {
  if (fd_ >= 0) return;

  fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0) {
    throw std::runtime_error("Failed to open " + device_ +
                             " , Please Check!!!");
  }

  if (!configurePort()) {
    closePort();
    throw std::runtime_error("Failed to configure " + device_);
  }
}

void SerialRS232::closePort() {
  stopLowLevel();
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool SerialRS232::isOpen() const { return fd_ >= 0; }

static speed_t toBaud(int baudrate) {
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      return B115200;
  }
}

bool SerialRS232::configurePort() {
  // 读取当前串口配置，再修改指定项，避免误伤其他配置
  struct termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    return false;
  }

  // 设置波特率：115200；输出 / 输入波特率都设为 115200
  // 典型 TTL / RS232 测试速率
  speed_t speed = toBaud(baudrate_);
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  // 校验位：显式关闭
  tty.c_cflag &= ~(PARENB | PARODD);
  // 停止位：设置为 1
  tty.c_cflag &= ~CSTOPB;
  // 关闭硬件流控（RTS/CTS）
  tty.c_cflag &= ~CRTSCTS;

  // raw mode
  // 关闭软件流控（XON/XOFF）
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  // 原样输出；read() 收到什么字节就给什么字节
  tty.c_oflag = 0;
  // 非 canonical，不处理回显、Ctrl+C 等
  tty.c_lflag = 0;

  tty.c_cc[VMIN]  = 0;  // 无最小字符要求
  tty.c_cc[VTIME] = 5;  // 0.5s timeout

  // 立刻生效（不中断 I/O）
  return tcsetattr(fd_, TCSANOW, &tty) == 0;
}

ssize_t SerialRS232::writeData(const void* data, size_t size) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  if (fd_ < 0 || data == nullptr || size == 0) {
    return -1;
  }

  // 已成功写入的字节数
  ssize_t total_written = 0;
  // 转为字节指针
  const uint8_t* ptr = static_cast<const uint8_t*>(data);

  while (total_written < static_cast<ssize_t>(size)) {
    ssize_t ret = write(fd_, ptr + total_written, size - total_written);

    if (ret < 0) {
      if (errno == EINTR) continue;  // 被信号中断，重试
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // 非阻塞模式下缓冲区满，可以等待或返回
        usleep(1000);
        continue;
      }
      return -1;  // 真正的错误
    }

    if (ret == 0) {
      break;  // 不应该发生，但需要处理
    }

    /* 等“这次写”的数据真正发完
    if (ret > 0) {
      // tcdrain() 是 阻塞调用
      // 阻塞直到数据真正从硬件FIFO发出，如果调用 read()，可能永久阻塞
      // 会严重拖慢串口吞吐，在协议通信中几乎不用
      tcdrain(fd_);
    }
    */

    total_written += ret;
  }

  return total_written;
}

ssize_t SerialRS232::writeString(const std::string& str) {
  if (str.empty()) return 0;

  // debug
  // std::cerr << "SerialRS232::writeString: " << str << std::endl;
  // return true;

  return writeData(str.data(), str.size());
}

/*
ssize_t SerialRS232::sendTrigger(int high_time_ms) {
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

void SerialRS232::triggerTxLoop() {
  const uint8_t LOW  = 0x00;  // 持续压低
  const uint8_t HIGH = 0xFF;  // 持续拉高

  while (trigger_running_) {
    uint8_t v = trigger_high_ ? HIGH : LOW;
    if (writeData(&v, 1) != 1) {
      trigger_running_ = false;
      break;
    }

    // 不能太慢，否则 TX 会出现空闲高
    // 不能太快，否则 CPU 占用高
    usleep(50);  // 50us 是一个工程上很常用的折中值
  }
}

void SerialRS232::startLowLevel() {
  if (!isOpen()) return;
  if (trigger_running_) return;

  trigger_running_ = true;
  trigger_high_    = false;

  trigger_thread_ = std::thread(&SerialRS232::triggerTxLoop, this);
}

void SerialRS232::triggerHigh(int high_ms) {
  if (high_ms <= 0 || !trigger_running_) return;

  std::lock_guard<std::mutex> pulse_lock(trigger_pulse_mutex_);
  if (!trigger_running_) return;

  trigger_high_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(high_ms));
  trigger_high_ = false;
}

void SerialRS232::stopLowLevel() {
  trigger_running_ = false;

  if (trigger_thread_.joinable()) {
    trigger_thread_.join();
  }
}
