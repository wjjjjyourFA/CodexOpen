#ifndef CanInterface_H
#define CanInterface_H

#include <iostream>
#include <vector>
#include <functional>
#include <thread>
#include <sys/types.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>  // ioctl

#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "comm_interface.h"

class CanInterface : public CommInterface {
 public:
  CanInterface();
  ~CanInterface();
  void Init(
      const std::string &can_name,
      const std::vector<can_filter> &v_filters = std::vector<can_filter>());
  void SetOnlySendMsg();
  void Send(const void *const buf, int n);

 protected:
  virtual void RecvLoop();
  int Receive(char *buf, int buf_size, int &len);

 private:
  int socket_;
  struct ifreq ifr_;
  struct sockaddr_can can_addr;
  int buf_size_;
  char *buf_;
};

#endif
