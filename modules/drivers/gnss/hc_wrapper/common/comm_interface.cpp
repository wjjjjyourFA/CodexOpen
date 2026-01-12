#include "comm_interface.h"

CommInterface::CommInterface() {
  loop_ok_      = true;
  print_msg_    = false;
  resolve_func_ = [](const char *const buf, int n) { 
    printf("undefined resolve func \n"); 
  };
}

CommInterface::~CommInterface() { Stop(); }

void CommInterface::Start() {
  loop_ok_ = true;
  loop_th_ = std::thread(&CommInterface::RecvLoop, this);
}

void CommInterface::Join() { loop_th_.join(); }

void CommInterface::Detach() { loop_th_.detach(); }

void CommInterface::Stop() {
  loop_ok_ = false;

  if (loop_th_.joinable()) {
    loop_th_.join();
  }
}

void CommInterface::PrintMsg(bool cmd) { print_msg_ = cmd; }

void CommInterface::SetResolveFunc(
    const std::function<void(const char *const buf, int n)> &resolve_func) {
  resolve_func_ = resolve_func;
}

void CommInterface::set_non_blocking(int sockfd) {
  int flag = fcntl(sockfd, F_GETFL, 0);

  if (flag < 0) {
    printf("fcntl F_GETFL fail");
  }

  if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
    printf("fcntl F_SETFL fail");
  }
}
