#include "tcp_interface.h"

//******************* TCP client **************************
TcpClient::TcpClient() {
  connectd_ = false;
  buf_size_ = 0;
}

TcpClient::~TcpClient() {
  close(socket_);

  if (buf_size_ > 0) {
    delete[] buf_;
  }
}

void TcpClient::Init(const std::string &target_ip, int port, int buf_size) {
  socket_ = socket(AF_INET, SOCK_STREAM, 0);

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

  conn_th_ = std::thread(&TcpClient::Connect, this);
}

void TcpClient::Connect() {
  while (connect(socket_, (struct sockaddr *)&target_addr_,
                 sizeof(target_addr_)) < 0) {
    usleep(50000);
  }

  connectd_ = true;
  printf("connect to %s %u\n", inet_ntoa(target_addr_.sin_addr),
         ntohs(target_addr_.sin_port));
}

void TcpClient::Send(const void *const buf, int n) {
  if (connectd_) {
    write(socket_, buf, n);
  }
}

void TcpClient::RecvLoop() {
  if (buf_size_ > 0) {
    int n;

    while (loop_ok_ && buf_size_ > 0) {
      if (!connectd_) {
        if (conn_th_.joinable()) {
          conn_th_.join();
        }
      } else if (Receive(buf_, buf_size_, n)) {
        resolve_func_(buf_, n);
      }

      usleep(10000);
    }
  }
}

int TcpClient::Receive(char *buf, int buf_size, int &len) {
  len = read(socket_, buf, buf_size);

  if (buf_size > len) {
    buf[len] = '\0';
  }

  if (print_msg_ && len > 0) {
    printf("received (%d) %s\n", len, buf);
  }

  return len;
}

//******************* TCP server **************************
TcpServer::TcpServer() { buf_size_ = 0; }

TcpServer::~TcpServer() {
  close(socket_);

  if (buf_size_ > 0) {
    delete[] buf_;
  }
}

void TcpServer::Init(int port, int buf_size) {
  socket_ = socket(AF_INET, SOCK_STREAM, 0);

  server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr_.sin_family      = AF_INET;
  server_addr_.sin_port        = htons(port);

  if (buf_size_ > 0 && buf_size_ != buf_size) {
    delete[] buf_;
  }

  if (buf_size_ != buf_size) {
    buf_size_ = buf_size;

    if (buf_size_ > 0) {
      buf_ = new char[buf_size_];
    }
  }
}

void TcpServer::Send(const void *const buf, int n) { write(socket_, buf, n); }

void TcpServer::RecvLoop() {
  if (buf_size_ > 0) {
    int n;
    int ret;
    int on = 1;
    buf_   = new char[buf_size_];

    ret = setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    if (bind(socket_, (struct sockaddr *)&server_addr_, sizeof(server_addr_)) <
        0) {
      printf("tcp server socket %d bind", ntohs(server_addr_.sin_port));
      return;
    }

    if (listen(socket_, 10) == -1) {
      printf("listen socket error: %s(errno: %d)\n", strerror(errno), errno);
      return;
    }

    struct sockaddr_in PeerAddr;
    socklen_t len = sizeof(PeerAddr);

    if ((socket_ = accept(socket_, (struct sockaddr *)&PeerAddr, &len)) == -1) {
      printf("accept socket error: %s(errno: %d)", strerror(errno), errno);
    } else {
      printf("%s connected\n", inet_ntoa(PeerAddr.sin_addr));
    }

    while (loop_ok_ && buf_size_ > 0) {
      if (Receive(buf_, buf_size_, n) > 0) {
        resolve_func_(buf_, n);
      } else if (n == 0) {
        printf("%s disconnected\n", inet_ntoa(PeerAddr.sin_addr));

        if ((socket_ = accept(socket_, (struct sockaddr *)&PeerAddr, &len)) ==
            -1) {
          printf("accept socket error: %s(errno: %d)", strerror(errno), errno);
        } else {
          printf("%s connected\n", inet_ntoa(PeerAddr.sin_addr));
        }
      }

      usleep(10000);
    }
  }
}

int TcpServer::Receive(char *buf, int buf_size, int &len) {
  len = read(socket_, buf, buf_size);

  if (buf_size > len) {
    buf[len] = '\0';
  }

  if (print_msg_ && len > 0) {
    printf("received (%d) %s\n", len, buf);
  }

  return len;
}