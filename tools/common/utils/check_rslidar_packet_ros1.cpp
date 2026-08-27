#include <arpa/inet.h>

#include <iostream>
#include <string>

#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "rslidar_sdk-1.5.20/rslidar_packet.hpp"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_rosbag> [topic_name]"
              << std::endl;
    std::cerr << "Example: " << argv[0] << " test.bag /rslidar_packets"
              << std::endl;
    return -1;
  }

  std::string bag_path     = argv[1];
  std::string target_topic = (argc >= 3) ? argv[2] : "";

  rosbag::Bag bag;
  try {
    std::cout << "Opening bag: " << bag_path << std::endl;
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const std::exception& e) {
    std::cerr << "Error opening bag file: " << e.what() << std::endl;
    return -1;
  }

  // 构建 View 查询
  std::vector<std::string> topics;
  if (!target_topic.empty()) {
    topics.push_back(target_topic);
  }

  rosbag::View view;
  if (topics.empty()) {
    view.addQuery(bag);
  } else {
    view.addQuery(bag, rosbag::TopicQuery(topics));
  }

  int64_t total_packets     = 0;
  int64_t total_msop        = 0;
  int64_t total_difop       = 0;
  int64_t drop_count        = 0;
  int64_t frame_begin_count = 0;

  int last_seq = -1;

  for (const rosbag::MessageInstance& m : view) {
    rslidar_msg::RslidarPacket::ConstPtr pkt =
        m.instantiate<rslidar_msg::RslidarPacket>();
    if (!pkt) {
      continue;
    }

    total_packets++;

    // 统计 DIFOP
    if (pkt->is_difop) {
      total_difop++;
      continue;
    }

    total_msop++;

    if (pkt->is_frame_begin) {
      frame_begin_count++;
    }

    // 校验 MSOP 长度（M1/E1 头部至少需要包含 id[4] 和 pkt_seq[2]）
    if (pkt->data.size() < 6) {
      std::cerr << "[Warning] MSOP data size too short: " << pkt->data.size()
                << std::endl;
      continue;
    }

    // 从原始字节中提取 pkt_seq (offset 4, 大端序网络字节)
    uint16_t raw_seq = 0;
    std::memcpy(&raw_seq, &pkt->data[4], sizeof(uint16_t));
    uint16_t seq = ntohs(raw_seq);

    if (last_seq != -1) {
      // 正常情况下序列号单调递增；当达到新一帧时回绕归零（通常是 0 或 1）
      // 如果当前序号不是 (last_seq + 1) 且不是正常帧重置（当前序号远小于上一序号），则判定为异常跳变/丢包
      bool is_sequential  = (seq == last_seq + 1);
      bool is_frame_reset = (seq < last_seq);  // 回绕重置到新一帧

      if (!is_sequential && !is_frame_reset) {
        std::cout << "[Packet Gap] Last seq: " << last_seq
                  << " -> Current seq: " << seq << " (Lost ~"
                  << (seq - last_seq - 1) << " packets)"
                  << " at Rosbag Time: " << m.getTime().toSec() << std::endl;
        drop_count += (seq - last_seq - 1);
      }
    }
    last_seq = seq;
  }

  bag.close();

  // 打印统计报告
  std::cout << "\n================= Summary Report ================="
            << std::endl;
  std::cout << "Total Packets:       " << total_packets << std::endl;
  std::cout << "MSOP Packets:        " << total_msop << std::endl;
  std::cout << "DIFOP Packets:       " << total_difop << std::endl;
  std::cout << "Frame Begin Signals: " << frame_begin_count << std::endl;
  std::cout << "Detected Lost Packets: " << drop_count << std::endl;

  if (total_msop > 0) {
    double loss_rate = (double)drop_count / (total_msop + drop_count) * 100.0;
    std::cout << "Estimated Packet Loss Rate: " << loss_rate << " %"
              << std::endl;
  }
  std::cout << "=================================================="
            << std::endl;

  return 0;
}