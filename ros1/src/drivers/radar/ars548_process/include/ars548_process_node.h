#include <iomanip>
#include <thread>
#include <unordered_map>
#include <string>

#include "ars548_base_node.h"
#include "convert_type.h"
#include "data_process.h"
#include "data_struct.h"
#include "udp_interface.h"

#include "cyber/base/thread_safe_queue.h"

using namespace jojo::cyber::base;
using namespace jojo::drivers;

// ============ 数据结构 ============
struct RadarPacket {
  std::vector<char> data;
  std::string src_ip;
};

class RadarProcessNode {
 public:
  RadarProcessNode();
  ~RadarProcessNode();

  bool Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
            std::shared_ptr<ConfigManager> param);
  void Run();

 protected:
  // for UDP
  std::string group_ip, local_ip;
  int group_port, local_port;

  struct sockaddr_in source_address;  // for recv_addr
  std::shared_ptr<UdpInterface> udp_io;
  std::thread rec_thread;
  char rec_data[40000];
  int rec_length;

  DataProcess data_process;
  void ProcessRadarData(char* data, int len, RadarBaseNode& radar_base_node);
  void ProcessRadarData(int index);

  void receiveThreadSingle();
  void receiveThreadMulti();
  bool ReceivePostProcess(RadarPacket& packet);

 private:
  std::shared_ptr<ConfigManager> param_manager;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  int radar_num = 0;
  std::unordered_map<std::string, int> radar_ips_;
  std::vector<RadarBaseNode> radar_base_node_vector;
  std::unordered_map<int, std::shared_ptr<ThreadSafeQueue<RadarPacket>>> queues_;
  std::vector<std::thread> radar_threads_;
};
