#ifndef SNIFFER_H
#define SNIFFER_H

/* General Use Includes */
#include <getopt.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>

/* Libpcap includes */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/ip_icmp.h>
#include <netinet/tcp.h>
#include <netinet/udp.h>
#include <pcap.h>
#include <sys/socket.h>

#include "data_process.h"

// Reference Blog:
// https://vichargrave.github.io/develop-a-packet-sniffer-with-libpcap/

class Sniffer {
 public:
  Sniffer();
  ~Sniffer();

  static Sniffer* instance_;  // 添加静态成员保存当前实例
  static void bailout(int signo);  // 静态函数供 signal() 调用

  // 业务处理
  void receive_packet_impl(const struct pcap_pkthdr *packethdr, const u_char *packetptr);

  // 静态回调函数，符合pcap_loop回调签名
  static void receive_packet(u_char* args, const struct pcap_pkthdr* header, const u_char* packet) {
    Sniffer* self = reinterpret_cast<Sniffer*>(args);
    self->receive_packet_impl(header, packet);
  }

  void init(int port, int packets, char interface[], const char filter[],
            int capture_live, const char* capture_path);

  bool init_data_process(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                         std::shared_ptr<RuntimeConfig> param);

  int run();

  void cleanup();  // 添加一个清理资源的成员函数

  pcap_t* open_pcap_socket(char* device, const char* bpfstr);
  pcap_t* open_pcap_file(const char* filename);

  DataProcess data_process;

 protected:
  void capture_loop(pcap_t* pd, int packets, pcap_handler func,
                    int capture_live, const char* capture_path);
  void receive_packet(u_char* user, struct pcap_pkthdr* packethdr,
                      u_char* packetptr);

  // void bailout(int signo);  // 静态函数，不能在类内定义

 private:
  pcap_t* pd_;     // pcap file descriptor
  int linkhdrlen;  // Datalink hdr len (so we can skip it)

  int port_ = 0;
  int packets_ = 0;
  char interface_[16] = {0};
  char filter_[256] = {0};
  int capture_live_ = 0;
  char* capture_path_ = NULL;
};

#endif /* SNIFFER_H */
