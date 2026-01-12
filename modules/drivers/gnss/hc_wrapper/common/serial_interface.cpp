#include "serial_interface.h"

//******************* Serial **************************
SerialInterface::SerialInterface() { buf_size_ = 0; }

SerialInterface::~SerialInterface() {
  Close();

  if (buf_size_ > 0) {
    delete[] buf_;
  }
}

// return 1 success, 0 error
int SerialInterface::Init(const std::string &port, int buf_size, int speed,
                          int flow_ctrl, int databits, int stopbits,
                          int parity) {
  port_ = port;

  if (buf_size_ > 0 && buf_size_ != buf_size) {
    delete[] buf_;
  }

  if (buf_size_ != buf_size) {
    buf_size_ = buf_size;

    if (buf_size_ > 0) {
      buf_ = new char[buf_size_];
    }
  }

  serial_fd_ = SerialOpen(port.c_str());
  // serial_fd_ = SerialOpen("/dev/ttyS0");

  return SerialSet(serial_fd_, speed, flow_ctrl, databits, stopbits, parity);
}

void SerialInterface::Close() { SerialClose(serial_fd_); }

void SerialInterface::Send(const void *const buf, int n) {
  SerialSend(serial_fd_, buf, n);
}

void SerialInterface::RecvLoop() {
  if (buf_size_ > 0) {
    int n;
    buf_ = new char[buf_size_];

    while (loop_ok_ && buf_size_ > 0) {
      if (Receive(buf_, buf_size_, n) > 0) {
        resolve_func_(buf_, n);
      }

      usleep(10000);
    }
  }
}

int SerialInterface::Receive(char *buf, int buf_size, int &len) {
  len = SerialRecv(serial_fd_, buf, buf_size);

  if (buf_size > len) {
    buf[len] = '\0';
  }

  if (print_msg_ && len > 0) {
    printf("received %s : (%d) %s\n", port_.c_str(), len, buf);
    //        printf("%s", buf);
  }

  return len;
}