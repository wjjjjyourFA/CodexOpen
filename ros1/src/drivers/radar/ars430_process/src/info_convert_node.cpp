#include "info_convert_node.h"

void InfoConverNode::RawDataCallback(
    const ars430_process::RadarPacket::ConstPtr& msg) {
  uint8_t ret_status      = packet_processor->processRDIMsg(msg);
  std::string status_info = "";

  // Most of our return statuses are of type ERR so make this the default verbosity level
  uint8_t log_verbosity = LOG_ERROR;

  switch (ret_status) {
    case NO_DETECTIONS:
      status_info = "No Detections";
      // No detections isn't really a failure, so just issue a warning so that it gets logged somewhere
      log_verbosity = LOG_WARNING;
      break;
    case BAD_EVENT_ID:
      status_info = "Bad Event ID";
      break;
    case NO_PUB_CLR_FAIL:
      status_info = "No Publisher and Unable to Clear Packets";
      break;
    case NO_PUBLISHER:
      status_info = "No Publisher";
      break;
    case PUBLISH_FAIL:
      status_info = "Publishing failed";
      break;
    case CLEAR_FAIL:
      status_info = "Failed to clear packets";
      break;
    default:
      // Assume default ret_value is success in which case don't log anything
      return;
  }

  if (log_verbosity == LOG_WARNING) {
    ROS_WARN_STREAM(
        "radarCallback warning, unable to execute processRDIMsg. Cause: "
        << status_info);
  } else if (log_verbosity == LOG_ERROR) {
    ROS_ERROR_STREAM(
        "radarCallback ERROR, unable to execute processRDIMsg. Cause: "
        << status_info);
  }
}

void InfoConverNode::publishRadarPacketMsg(
    ars430_process::RadarPacket& radar_packet_msg) {
  return packet_pub_.publish(radar_packet_msg);
}

uint8_t InfoConverNode::pubCallback(
    PacketGroup_t* Packets) {  //call upon the appropriate publish function
  for (uint8_t i = 0; i < Packets->numFarPackets; i++) {
    this->publishRadarPacketMsg(Packets->farPackets[i]);
    ROS_INFO_STREAM("Far packet timestamp: "
                    << std::to_string((Packets->farPackets[i]).time_stamp));
  }

  for (uint8_t i = 0; i < Packets->numNearPackets; i++) {
    this->publishRadarPacketMsg(Packets->nearPackets[i]);
    ROS_INFO_STREAM("Near packet timestamp: "
                    << std::to_string((Packets->nearPackets[i]).time_stamp));
  }

  return SUCCESS;
}

bool InfoConverNode::init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                          std::shared_ptr<RuntimeConfig> param) {
  nh_         = nh;
  private_nh_ = private_nh;
  param_      = param;

  std::string ns    = param_->GetVehicleName();
  std::string topic = "/" + ns + param_->channel_name;

  UnfilteredSub = nh.subscribe(
      topic + "/unfiltered_radar_packet_" + std::to_string(param_->id), 100,
      &InfoConverNode::RawDataCallback, this);

  packet_pub_ = nh_.advertise<ars430_process::RadarPacket>(
      std::string(topic + "filtered_radar_packet_" + std::to_string(param_->id)), 50);

  packet_processor = std::make_shared<PacketProcessor>();
  packet_processor->initializePacketProcessor(param_->id, this);
}

void InfoConverNode::run() { ros::spin(); }

void printUsage(char buff[]) {
  printf("\nUsage: %s [-h] [-n RADARNAME][-y UNFILTEREDTOPIC]\r\n", buff);
  printf(
      "\tn: radar namespace (defaults: radar), should use launch script for "
      "this\r\n");
  printf(
      "\tt: unfiltered topic name if reading (defaults: "
      "unfiltered_radar_packet\r\n");
}

void SingleChannel(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                   std::shared_ptr<RuntimeConfig> param) {
  InfoConverNode info_conver_node;
  info_conver_node.init(nh, private_nh, param);
  std::cout << "Info convert node initialized" << std::endl;
  info_conver_node.run();
}

int main(int argc, char** argv) {
  opterr = 0;
  int id = 0, c;  // Radar Default

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
    name = param_manager->GetVehicleName() + "_ars430_info_convert_node";
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
      }
    }

    param_simple->id = id;

    /////////////////////////////////////////////////////////////////////
    if (param_simple->id == 0) {
      printf("Bad id or no id given\r\n");
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
    return false;
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
}