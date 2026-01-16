#include "ars430_process_node.h"

RadarProcessNode::RadarProcessNode() {}

RadarProcessNode::~RadarProcessNode() {
  if (rec_thread.joinable()) {
    rec_thread.join();  // 等待线程完成
  }
}

// thread for receive radar data
void RadarProcessNode::receiveThread() {
  ros::Time last_print_time = ros::Time::now();  // 记录上次打印的时间

  // publish radar raw data
  sniffer.run();
  // 阻塞在这里了！

  // ros::spin();
}

void RadarProcessNode::Reset() {}

// void initUnfilteredPublisher(uint8_t id, ros::NodeHandle nh)
bool RadarProcessNode::init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                            std::shared_ptr<RuntimeConfig> param) {
  nh_         = nh;
  private_nh_ = private_nh;
  param_      = param;

  std::string ns    = param_->GetVehicleName();
  std::string topic = "/" + ns + param_->channel_name;

  sniffer.init(param_->port, param_->packets, param_->interface, param_->filter,
               param_->capture_live, param_->capture_path);
  sniffer.init_data_process(nh_, private_nh_, param_);

  return true;
}

void RadarProcessNode::run() {
  // to read radar data
  // 组播服务器监听 42102 端口来接收组播消息。
  // 单播客户端使用 42101 端口发送数据到目标 IP。

  rec_thread = std::thread(&RadarProcessNode::receiveThread, this);

  ros::Rate r(10);

  // 这里是为了配合对雷达进行实时调优，所以需要一直发送数据
  // 目前不需要，所以全部设置为 0
  // /*
  while (ros::ok()) {
    ros::spinOnce();

    // I set all is zero
    this->Reset();

    r.sleep();
  }
  // */
  // ros::spin();
}

extern int opterr;

void printUsage(char buff[]) {
  // clang-format off
  printf("\nUsage: %s [-h] [-i id] [-e interface] [-n packets] [-p port] [-f filter] [-c capture] [-l cpath] \r\n", buff);
  printf("\ti: radar id (int)\r\n");
  printf("\te: ethernet interface (string)\n");
  printf("\tn: number of packets (int)\n");
  printf("\tp: port (int)\n");
  printf("\tf: extra filter string (optional), enclose string in \" \" (string)\n");
  printf("\tc: capture method (int), default 1, LIVE\n");
  printf("\tl: file path to captured data (string), required for OFFLINE capture\n");
  // clang-format on
}

void SingleChannel(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                   std::shared_ptr<RuntimeConfig> param) {
  RadarProcessNode radar_process_node;
  radar_process_node.init(nh, private_nh, param);
  radar_process_node.run();
}

int main(int argc, char** argv) {
  opterr = 0;
  // clang-format off
  char interface[16] = "";  // Which network interface (combination of port & IP) are we reading from?
  char filter[256] = "";    // What simple packet filters are we applying?
  int packets = 0, id = 0, c, port = 31122;  // Radar Default
  int capture_live = 1;     // Based on if we're using live capture or from offline pcap doc
  char capture_path[256] = "";  // File path to pcap doc if using offline capture
  // clang-format on

  std::string name   = "";
  auto param_manager = std::make_shared<ConfigManager>();
  auto param_simple  = std::make_shared<RuntimeConfig>();

  bool manual_conf = false;
  if (argc <= 1) {
    ROS_INFO("Insufficient args\r\n");
    printUsage(argv[0]);

    std::string base_path = ros::package::getPath("ars430_process");
    std::string config_file_path_ = base_path + "/../../../../../install/common/vehicle_sensor_config.yaml";
    ROS_INFO("No command-line args. Using config file: %s",
             config_file_path_.c_str());

    param_manager->LoadConfig(config_file_path_);
    name = param_manager->GetVehicleName() + "_ars430_process_node";
  } else {
    // Get the command line option, if any
    while ((c = getopt(argc, argv, "+hi:p:e:n:c:l:f:")) !=
           -1) {  // Pass remainder of arguments to the sniffer
      switch (c) {
        case 'h':
          printUsage(argv[0]);
          exit(0);
          break;
        case 'i':
          id = atoi(optarg);
          printf("ID: %d\r\n", id);
          break;
        case 'e':
          strcpy(interface, optarg);
          printf("INTERFACE: %s\r\n", optarg);
          break;
        case 'n':
          packets = atoi(optarg);
          break;
        case 'p':
          port = atoi(optarg);
          printf("PORT: %d\r\n", port);
          break;
        case 'c':
          capture_live = atoi(optarg);
          printf("CAPTURE: %d\r\n", capture_live);
          break;
        case 'l':
          strcpy(capture_path, optarg);
          printf("CAPTURE_PATH: %s\r\n", capture_path);
          break;
        case 'f':  // Optional Flag
          strcpy(filter, optarg);
          printf("FILTER: %s\r\n", filter);
          break;
      }
    }

    // param_simple->interface = interface;
    std::strncpy(param_simple->interface, interface,
                 sizeof(param_simple->interface) - 1);
    // 保证以 null 结尾
    param_simple->interface[sizeof(param_simple->interface) - 1] = '\0';
    // param_simple->filter = filter;
    std::strncpy(param_simple->filter, filter,
                 sizeof(param_simple->filter) - 1);
    // 保证以 null 结尾
    // clang-format off
    param_simple->filter[sizeof(param_simple->filter) - 1] = '\0';
    param_simple->packets = packets;
    param_simple->id = id;
    param_simple->port = port;
    param_simple->capture_live = capture_live;
    // clang-format on
    // param_simple->capture_path = capture_path;
    std::strncpy(param_simple->capture_path, capture_path,
                 sizeof(param_simple->capture_path) - 1);
    // 保证以 null 结尾
    param_simple->capture_path[sizeof(param_simple->capture_path) - 1] = '\0';

    /////////////////////////////////////////////////////////////////////
    if (param_simple->id == 0) {
      printf("Bad id or no id given\r\n");
      return 0;
    }

    if (param_simple->capture_live == LIVE) {
      printf("RUNNING LIVE\r\n");
    } else if (param_simple->capture_live == OFFLINE) {
      printf("RUNNING OFFLINE\r\n");
    } else {
      printf("Bad capture option, Definition in parser.h\r\n");
      return 0;
    }
    /////////////////////////////////////////////////////////////////////

    manual_conf = true;
  }

  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::vector<SensorConfig>& radar_configs =
      param_manager->vehicle_model_config.radar_configs;

  if (radar_configs.size() < 1) {
    return -1;
  }

  if (!manual_conf) {
    // 初始化多个Radar Process
    int num = 1;
    std::vector<std::thread> threads_;
    for (auto config : radar_configs) {
      auto param          = std::make_shared<RuntimeConfig>();
      param->vehicle_name = param_manager->GetVehicleName();
      param->LoadConfig(config.config_file);
      // std::cout << "vehicle_name: " << param->vehicle_name << std::endl;

      // 复制当前轮的 num，确保 lambda 捕获的是正确值
      int current_num = num;
      // 启动 单线程/多线程
      threads_.emplace_back([current_num, &nh, &private_nh, param]() {
        SingleChannel(nh, private_nh, param);
      });

      num++;
    }

    // 主线程等待退出
    ros::waitForShutdown();
  } else {
    SingleChannel(nh, private_nh, param_simple);
  }

  return 0;
};