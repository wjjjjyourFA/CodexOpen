#include "ars548_process_node.h"

RadarProcessNode::RadarProcessNode() {
  udp_io = std::make_shared<UdpInterface>();
}

RadarProcessNode::~RadarProcessNode() {
  if (rec_thread.joinable()) {
    rec_thread.join();  // 等待线程完成
  }

  for (auto& th : radar_threads_) {
    if (th.joinable()) th.join();
  }
}

// 这里是面向不同雷达的实例类数据处理
void RadarProcessNode::ProcessRadarData(char* data, int len,
                                        RadarBaseNode& radar_base_node) {
  auto& m = radar_base_node;
  // 匹配相应通信协议中的 UDP Payload
  // because of using UDP
  switch (len) {
    case 9401:
      if (data_process.processObjectListMessage(data, &m.object_list)) {
        if (m.object_list.ObjectList_NumOfObjects > 0)
          m.publishObjectList(m.object_list);
      }
      break;

    case 35336:
      if (data_process.processDetectionListMessage(data, &m.detection_list)) {
        // std::cout << "parser radar data suc " << std::endl;
        if (m.detection_list.List_NumOfDetections)
          m.publishDetectionList(m.detection_list);
      }
      break;

    // case 69:// for 5.48.1
    case 84:  // for 5.48.4
      if (data_process.processRadarStatusMessage(data, &m.radar_status)) {
        // std::cout << "recv radar_status suc " << std::endl;
        m.publishRadarStatus(m.radar_status);
      }
      break;

    default:
      break;
  }
}

// 多线程中，每个雷达的独立处理线程
void RadarProcessNode::ProcessRadarData(int index) {
  auto queue = queues_[index];
  while (ros::ok()) {
    RadarPacket packet;
    queue->WaitDequeue(packet);
    ProcessRadarData(packet.data.data(), packet.data.size(),
                     radar_base_node_vector.at(index));
  }
}

// thread for receive radar data
// 单毫米波用这个, 不适用于多毫米波
void RadarProcessNode::receiveThreadSingle() {
  // 记录上次打印的时间
  ros::Time last_print_time = ros::Time::now();

  while (ros::ok()) {
    // udp_io->receiveFromRadar((struct sockaddr *)&source_address, rec_data, rec_length);
    udp_io->receiveFromRadar((struct sockaddr_in*)&source_address, rec_data,
                             rec_length);
    // 阻塞在这里了！

    // 打印雷达地址，用于连接多个雷达时区分哪一个雷达
    if (rec_length > 0) {
      ProcessRadarData(rec_data, rec_length, radar_base_node_vector.at(0));

      // 每 3 秒打印一次调试信息
      ros::Time new_time = ros::Time::now();
      if ((new_time - last_print_time).toSec() >= 3) {
        std::cout << "================ " << std::endl;
        last_print_time = new_time;  // 更新上次打印的时间
        std::cout << "recv radar data : " << std::fixed << std::setprecision(0)
                  << last_print_time.toSec() * 1000 << std::endl;
        std::cout << "================ " << std::endl;
      }
    }
  }
}

// 多雷达 → 单组播地址 → 单套接字接收 → 多线程处理
// 该线程只负责接收数据，不负责处理数据，所以不会降低传感器的帧率
void RadarProcessNode::receiveThreadMulti() {
  // 记录上次打印的时间
  ros::Time last_print_time = ros::Time::now();

  // recvfrom 会自动阻塞，因此不需要考虑循环频率
  while (ros::ok()) {
    RadarPacket packet;
    if (this->ReceivePostProcess(packet)) {
      // 通过判断不同的IP，分发给不同的线程处理
      auto it_ip = radar_ips_.find(packet.src_ip);  // 查找 src_ip 对应的雷达 id
      if (it_ip != radar_ips_.end()) {
        auto index    = it_ip->second;  // 拿到 int 类型的 id
        auto it_queue = queues_.find(index);  // 用 id 去查队列
        if (it_queue != queues_.end()) {
          it_queue->second->Enqueue(packet);  // push 到对应队列
        } else {
          ROS_INFO("Received data from unknown IP: %s", packet.src_ip.c_str());
        }
      }

      // 每 3 秒打印一次调试信息
      ros::Time new_time = ros::Time::now();
      if ((new_time - last_print_time).toSec() >= 3) {
        std::cout << "================ " << std::endl;
        last_print_time = new_time;  // 更新上次打印的时间
        std::cout << "recv radar data : " << std::fixed << std::setprecision(0)
                  << last_print_time.toSec() * 1000 << std::endl;
        std::cout << "================ " << std::endl;
      }
    }
  }
}

bool RadarProcessNode::ReceivePostProcess(RadarPacket& packet) {
  udp_io->receiveFromRadar((struct sockaddr_in*)&source_address, rec_data,
                           rec_length);

  if (rec_length <= 0) return false;

  packet.data.assign(rec_data, rec_data + rec_length);
  packet.src_ip = inet_ntoa(source_address.sin_addr);

  return true;
}

bool RadarProcessNode::Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                            std::shared_ptr<ConfigManager> param) {
  nh_           = nh;
  private_nh_   = private_nh;
  param_manager = param;

  std::vector<SensorConfig>& radar_configs =
      param_manager->vehicle_model_config.radar_configs;

  if (radar_configs.size() < 1) {
    return false;
  }

  // 初始化多个Radar Process
  for (auto config : radar_configs) {
    auto param          = std::make_shared<RuntimeConfig>();
    param->vehicle_name = param_manager->GetVehicleName();
    param->LoadConfig(config.config_file);
    // std::cout << "vehicle_name: " << param->vehicle_name << std::endl;

    // 以第一个毫米波的参数作为组播参数
    static bool first_radar_param = true;
    if (first_radar_param) {
      this->group_ip   = param->group_ip;
      this->group_port = param->group_port;
      this->local_ip   = param->local_ip;
      this->local_port = param->local_port;

      first_radar_param = false;
    }

    RadarBaseNode radar_base_node;
    radar_base_node.Init(nh_, param);
    radar_base_node.SetUdpIo(udp_io);

    radar_base_node_vector.emplace_back(radar_base_node);

    int index = radar_base_node_vector.size() - 1;
    radar_ips_.emplace(param->radar_ip, index);

    // dds 底层需要注册相关消息，不能启动太快；
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  this->radar_num = radar_base_node_vector.size();

  return true;
}

void RadarProcessNode::Run() {
  // to read radar data
  // 组播服务器监听 42102 端口来接收组播消息。
  // 单播客户端使用 42101 端口发送数据到目标 IP。

  // local_ip = 10.13.1.166
  // udp_io->initUdpMulticastServer("224.0.2.2", 42102, "10.13.1.166");
  udp_io->initUdpMulticastServer(group_ip, group_port, local_ip);

  // to set radar config
  // radar_ip = 10.13.1.113
  // udp_io->initUdpUnicastClient("10.13.1.113", 42101, 42401);
  udp_io->initUdpUnicastClient(local_ip, local_port);

  // 启动接收线程
  if (radar_num == 1) {
    rec_thread = std::thread(&RadarProcessNode::receiveThreadSingle, this);
  } else if (radar_num > 1) {
    rec_thread = std::thread(&RadarProcessNode::receiveThreadMulti, this);

    // 为每个雷达分配一个队列和处理线程
    for (int i = 0; i < radar_num; i++) {
      // unordered_map 在访问一个不存在的 key 时，会自动创建一个默认值插进去
      queues_[i] = std::make_shared<ThreadSafeQueue<RadarPacket>>();
      radar_threads_.emplace_back([this, i]() { this->ProcessRadarData(i); });

      // dds 底层需要注册相关消息，不能启动太快；
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  } else {
    ROS_INFO("No radar found in config file");
  }

  ros::spin();
}

int main(int argc, char** argv) {
  std::string base_path = ros::package::getPath("ars548_process");
  std::string config_file_path_ = base_path + "/../../../../../install/common/vehicle_sensor_config.yaml";

  auto param_manager = std::make_shared<ConfigManager>();
  param_manager->LoadConfig(config_file_path_);
  std::string name = param_manager->GetVehicleName() + "_ars548_process_node";

  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  RadarProcessNode radar_process_node;
  radar_process_node.Init(nh, private_nh, param_manager);
  radar_process_node.Run();

  // 主线程等待退出
  ros::waitForShutdown();

  return 0;
}