#include <thread>
#include <signal.h>
#include <string.h>
#include <string>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"

// msgs include
#include "chcnav/hc_sentence.h"
#include "chcnav/int8_array.h"
#include "chcnav/string.h"
#include "device_connector.hpp"
#include "hc_msg_parser.hpp"
#include "serial_common.hpp"
#include "tcp_common.hpp"
#include "udp_common.hpp"
#include "file_common.hpp"

using namespace std;

static void signal_exit(int sigo) {
  exit(0);
  return;
}

void SingleChannel(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
                   std::shared_ptr<RuntimeConfig> param) {
  hc__device_connector *device_connector;

  // for test
  private_nh.setParam("type", param->type);
  private_nh.setParam("rate", param->rate);
  private_nh.setParam("host", param->host);
  private_nh.setParam("port", param->port);

  /*********************************************************
     * params parse
     *********************************************************/
  string type = "";  // 参数配置类型
  if (private_nh.getParam("type", type) == false) {
    ROS_ERROR("chcnav: type is not set.");
    // return -1;
    return;
  }

  int rate = 0;  // 获取处理频率
  private_nh.param<int>("rate", rate, 1000);
  if (0 == rate) ROS_INFO("rate [0], No message pub deal subscribe only");

  if (type.compare("serial") == 0) {
    // 如果参数配置为串口
    string port = "", parity = "";
    int baudrate = -1, databits = -1, stopbits = -1;

    if (private_nh.getParam("port", port) == false) {
      ROS_ERROR("chcnav: port is not set.");
      //   return -1;
      return;
    }
    private_nh.param<int>("baudrate", baudrate, 115200);
    private_nh.param<int>("databits", databits, 8);
    private_nh.param<int>("stopbits", stopbits, 1);
    private_nh.param<std::string>("parity", parity, "None");

    ROS_INFO(
        "serial config port[%s] baudrate[%d] databits[%d] stopbits[%d] "
        "parity[%s] rate [%d]",
        port.c_str(), baudrate, databits, stopbits, parity.c_str(), rate);

    device_connector =
        new serial_common(port, baudrate, databits, stopbits, parity);
  } else if (type.compare("tcp") == 0) {
    string host = "";
    int port    = 0;

    if (private_nh.getParam("host", host) == false) {
      ROS_ERROR("chcnav: host is not set.");
      //   return -1;
      return;
    }

    if (private_nh.getParam("port", port) == false) {
      ROS_ERROR("chcnav: port is not set.");
      //   return -1;
      return;
    }

    ROS_INFO("tcp config host[%s] port[%d]", host.c_str(), port);

    device_connector = new tcp_common(host, port);
  } else if (type.compare("file") == 0) {
    string path = "";
    if (private_nh.getParam("path", path) == false) {
      ROS_ERROR("chcnav: path is not set.");
      //   return -1;
      return;
    }

    ROS_INFO("file path[%s]", path.c_str());

    // 很重要，等待其他节点都启动完毕再解析文件，
    // 不然会造成先解析文件，导致一些在别的节点启动前就被解析出的数据未处理。
    usleep(500 * 1000);
    device_connector = new file_common(path);
  } else if (type.compare("udp") == 0) {
    int port = 0;

    if (private_nh.getParam("port", port) == false) {
      ROS_ERROR("chcnav: port is not set.");
      //   return -1;
      return;
    }

    ROS_INFO("udp config port[%d]", port);

    device_connector = new udp_common(port);
  }

  // 获取节点名
  string node_path = ros::this_node::getName();
  string node_name = node_path.substr(node_path.find_last_of("/") + 1);
  // std::cout << "node_name: " << node_name << std::endl;

  // 初始化信息解析节点
  hc__msg_parser_node parser_node(rate, device_connector, &nh, &private_nh,
                                  node_name, param);

  parser_node.device_connector->connect();

  parser_node.node_start();
}

int main(int argc, char **argv) {
  std::string base_path = ros::package::getPath("chcnav");
  // std::cout << "base_path: " << base_path << std::endl;
  std::string config_file_path_ = base_path + "/../../../../../install/common/vehicle_sensor_config.yaml";
  // std::string config_file_path_ = "/media/jojo/AQiDePan/CodexOpen/install/common/vehicle_sensor_config.yaml";

  // No use it in roslaunch
  // if (argc > 1) {
  //     config_file_path_ = argv[1];
  // }

  auto param_manager = std::make_shared<ConfigManager>();
  param_manager->LoadConfig(config_file_path_);
  std::string name = param_manager->GetVehicleName() + "_hc_msg_parser_node";

  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::vector<SensorConfig> &gnss_configs =
      param_manager->vehicle_model_config.gnss_configs;

  if (gnss_configs.size() < 1) {
    return -1;
  }

  // 初始化多个Gnss Process
  int num = 1;
  std::vector<std::thread> threads_;
  for (auto config : gnss_configs) {
    auto param = std::make_shared<RuntimeConfig>();
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

  signal(SIGTERM, signal_exit);  // signal to eixt
  signal(SIGINT, signal_exit);  // signal to eixt
  signal(SIGKILL, signal_exit);  // signal to eixt

  // 主线程等待退出
  ros::waitForShutdown();

  return 0;
}
