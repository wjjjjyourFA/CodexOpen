#ifndef CommInterface_H
#define CommInterface_H

#include <iostream>
#include <vector>
#include <functional>
#include <thread>
#include <sys/types.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

class CommInterface {
 public:
  CommInterface();
  ~CommInterface();
  void Start();
  void Stop();
  void Join();
  void Detach();
  void PrintMsg(bool cmd = true);
  void SetResolveFunc(
      const std::function<void(const char *const buf, int n)> &resolve_func);
  virtual void Send(const void *const buf, int n) = 0;

 protected:
  virtual void RecvLoop()                                = 0;
  virtual int Receive(char *buf, int buf_size, int &len) = 0;

  void set_non_blocking(int sockfd);

 protected:
  bool loop_ok_;
  bool print_msg_;
  std::function<void(const char *const buf, int n)> resolve_func_;
  std::thread loop_th_;
};

#endif  // CommInterface_H
