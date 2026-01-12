#include "udp_interface.h"

//********************* UDP *****************************
UdpInterface::UdpInterface() { buf_size_ = 0; }

UdpInterface::~UdpInterface() {
  close(socket_);

  if (buf_size_ > 0) {
    delete[] buf_;
  }
}

// if only send msg, set buf_size = 0
void UdpInterface::Init(const std::string &target_ip, int port, int buf_size,
                        bool non_block) {
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);

  target_addr_.sin_addr.s_addr = inet_addr(target_ip.data());
  target_addr_.sin_family      = AF_INET;
  target_addr_.sin_port        = htons(port);

  if (buf_size_ > 0 && buf_size_ != buf_size) {
    delete[] buf_;
  }

  if (buf_size_ != buf_size) {
    buf_size_ = buf_size;

    if (buf_size_ > 0) {
      buf_ = new char[buf_size_];
    }
  }

  non_block_ = non_block;
}

int UdpInterface::Receive(char *buf, int buf_size, int &len) {
  static struct sockaddr_in PeerAddr;
  int addr_size;

  len = recvfrom(socket_, buf, buf_size, 0, (struct sockaddr *)&PeerAddr,
                 (socklen_t *)&addr_size);

  if (buf_size > len) {
    buf[len] = '\0';
  }

  if (print_msg_ && len > 0) {
    printf("received from %s %u :(%d) %s\n", inet_ntoa(PeerAddr.sin_addr),
           ntohs(PeerAddr.sin_port), len, buf);
  }

  return len;
}

void UdpInterface::Send(const void *const buf, int n) {
  sendto(socket_, buf, n, 0, (sockaddr *)&target_addr_, sizeof(target_addr_));
}

void UdpInterface::RecvLoop() {
  if (buf_size_ > 0) {
    int n;

    struct sockaddr_in local_addr;
    local_addr.sin_family      = AF_INET;
    local_addr.sin_port        = target_addr_.sin_port;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int on = 1;
    int ret;
    ret = setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    if (non_block_) {
      set_non_blocking(socket_);
    }

    if (bind(socket_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
      printf("udp socket %d bind", ntohs(local_addr.sin_port));
    }

    while (loop_ok_ && buf_size_ > 0) {
      if (Receive(buf_, buf_size_, n)) {
        resolve_func_(buf_, n);
      }

      usleep(10000);
    }
  }
}