#ifndef TcpInterface_H
#define TcpInterface_H

#include <iostream>
#include <vector>
#include <functional>
#include <thread>
#include <sys/types.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

#include <sys/socket.h>
#include <sys/ioctl.h>

#include "comm_interface.h"

class TcpClient : public CommInterface {
 public:
  TcpClient();
  ~TcpClient();
  void Init(const std::string &target_ip, int port, int buf_size = 0);
  void Connect();
  void Send(const void *const buf, int n);

 protected:
  virtual void RecvLoop();
  int Receive(char *buf, int buf_size, int &len);

 private:
  int socket_;
  struct sockaddr_in target_addr_;
  int buf_size_;
  char *buf_;
  
  bool connectd_;
  std::thread conn_th_;
};

class TcpServer : public CommInterface {
 public:
  TcpServer();
  ~TcpServer();
  void Init(int port, int buf_size = 0);
  void Send(const void *const buf, int n);

 protected:
  virtual void RecvLoop();
  int Receive(char *buf, int buf_size, int &len);

 private:
  int socket_;
  struct sockaddr_in server_addr_;
  int buf_size_;
  char *buf_;
};

#endif  //
