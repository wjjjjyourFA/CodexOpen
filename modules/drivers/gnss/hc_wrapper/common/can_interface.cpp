#include "can_interface.h"

CanInterface::CanInterface() { buf_size_ = 0; }

CanInterface::~CanInterface() {
  close(socket_);

  if (buf_size_ > 0) {
    delete[] buf_;
  }
}

/*
 * @can_name: e.g. can0
 * @v_filters: vector for can filter
 * e.g.
 * filter for only accept msg which id is 0x11
 * can_filter.can_id = 0x11;
 * can_filter.can_mask = CAN_SFF_MASK;
 */
void CanInterface::Init(const std::string &can_name,
                        const std::vector<can_filter> &v_filters) {
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  strcpy(ifr_.ifr_name, can_name.c_str());

  ioctl(socket_, SIOCGIFINDEX, &ifr_);  // 指定 can0 设备

  can_addr.can_family  = AF_CAN;
  can_addr.can_ifindex = ifr_.ifr_ifindex;

  if (!v_filters.empty()) {
    // set can filter
    setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, v_filters.data(),
               v_filters.size() * sizeof(can_filter));
  }

  if (buf_size_ == 0) {
    buf_size_ = sizeof(can_frame);

    buf_ = new char[buf_size_];
  }
}

void CanInterface::SetOnlySendMsg() {
  setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
}

void CanInterface::Send(const void *const buf, int n) {
  write(socket_, buf, n);
}

void CanInterface::RecvLoop() {
  if (buf_size_ > 0) {
    int n;

    if (bind(socket_, (struct sockaddr *)&can_addr, sizeof(can_addr)) < 0) {
      printf("can socket bind error\n");
    }

    while (loop_ok_ && buf_size_ > 0) {
      if (Receive(buf_, buf_size_, n)) {
        resolve_func_(buf_, n);
      }

      usleep(10000);
    }
  }
}

int CanInterface::Receive(char *buf, int buf_size, int &len) {
  len = read(socket_, buf, buf_size);

  if (print_msg_ && len > 0) {
    printf("received (%d) %s\n", len, buf);
  }

  return len;
}
