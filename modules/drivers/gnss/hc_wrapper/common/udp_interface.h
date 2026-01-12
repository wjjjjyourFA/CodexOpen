#ifndef UdpInterface_H
#define UdpInterface_H

#include <iostream>
#include <vector>
#include <functional>
#include <thread>
#include <sys/types.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "comm_interface.h"

class UdpInterface : public CommInterface {
 public:
  UdpInterface();
  ~UdpInterface();
  void Init(const std::string &target_ip, int port, int buf_size = 0,
            bool nonblock = false);
  void Send(const void *const buf, int n);

 protected:
  virtual void RecvLoop();
  int Receive(char *buf, int buf_size, int &len);

 private:
  int socket_;
  struct sockaddr_in target_addr_;
  int buf_size_;
  char *buf_;
  bool non_block_;
};

#endif  //
