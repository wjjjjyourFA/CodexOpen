#ifndef SerialInterface_H
#define SerialInterface_H

#include <iostream>
#include <vector>
#include <functional>
#include <thread>
#include <sys/types.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include "serial_base.h"
#include "comm_interface.h"

class SerialInterface : public CommInterface {
 public:
  SerialInterface();
  ~SerialInterface();
  // serial port
  // buf size for recv data, [0] for send data only
  // bit rate
  // data flow control, [0] no flow control, [1] hardware flow control, [2] software flow control
  // data bits, set [7] or [8]
  // stop bits, set [1] or [2]
  // parity check, [n][N] no parity check, [o][O] odd check, [e][E] even check, [s][S] blank
  int Init(const std::string &port, int buf_size, int speed = 115200,
           int flow_ctrl = 0, int databits = 8, int stopbits = 1,
           int parity = 'N');
  void Close();
  void Send(const void *const buf, int n);

 protected:
  virtual void RecvLoop();
  int Receive(char *buf, int buf_size, int &len);

 protected:
  int serial_fd_;
  std::string port_;
  int buf_size_;
  char *buf_;
};

#endif
