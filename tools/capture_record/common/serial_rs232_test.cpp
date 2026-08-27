#include "tools/capture_record/common/serial_rs232.h"

#include <chrono>
#include <memory>
#include <thread>

int main() {
  std::unique_ptr<SerialRS232> serialPtr = nullptr;

  if (serialPtr) {
    // serialPtr->closePort();
    return -1;
  }

  // for rs232 to ttl
  try {
    serialPtr = std::make_unique<SerialRS232>("/dev/ttyS0", 115200);
  } catch (const std::exception& e) {
    std::cerr << "Serial Error" << std::endl;
    return -1;
  }

  // 直觉上的方波
  // 0x55 = 01010101（二进制）
  uint8_t square = 0x55;

  std::cout << "Sending UART square wave (0x55)..." << std::endl;

  while (true) {
    /* way 1
    serialPtr->writeString("Hello COM1\r\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    */

    // way 2
    // sleep 会破坏方波连续性；不要加 sleep，保证连续比特流
    ssize_t ret = serialPtr->writeData(&square, 1);
    if (ret <= 0) {
      if (errno == EAGAIN) {
        // 缓冲区满，稍微等待（可选）
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        continue;
      }
      perror("write failed");
      break;
    }

    // 可选：每发送 N 字节打印一次状态
    static int count = 0;
    if (++count % 10000 == 0) {
      std::cout << "Sent " << count << " bytes\r" << std::flush;
    }
  }

  // 检测完释放掉
  if (serialPtr) {
    serialPtr->closePort();
  }

  return 0;
}
