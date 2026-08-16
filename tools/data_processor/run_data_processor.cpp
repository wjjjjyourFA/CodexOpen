#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "modules/common/environment_conf.h"
#if defined(DATA_PROCESSOR_ROS1_LEGACY)
#include "tools/data_processor/ros1_convert_legacy.h"
#elif defined(DATA_PROCESSOR_ROS1)
#include "tools/data_processor/ros1_convert.h"
#elif defined(DATA_PROCESSOR_ROS1_FAST)
#include "tools/data_processor/ros1_convert_fast.h"
#elif defined(DATA_PROCESSOR_ROS2)
#include "tools/data_processor/ros2_convert.h"
#else
#error "Define exactly one DATA_PROCESSOR_ROS1*, or DATA_PROCESSOR_ROS2"
#endif

using namespace std;
using namespace jojo::tools;

int main(int argc, char** argv) {
  try {
    // clang-format off
    std::string name = "DataProcessor";
    std::string config_path = "./../../config/DataProcessor/DataProcessor.ini";
    // std::string config_path = "./../../config/DataProcessor/DataProcessor.json";
    // clang-format on

    // 如果命令行参数提供了自定义配置路径，则使用该路径
    if (argc > 1) {
      config_path = argv[1];
    }

    auto runtime_config = std::make_shared<jojo::tools::RuntimeConfig>();
    runtime_config->set_name(name);
    runtime_config->LoadConfig(config_path);

    std::string if_config_path = "./../../config/DataProcessor/Interface.ini";
    if (argc > 2) {
      if_config_path = argv[2];
    }

    auto interface_config = std::make_shared<jojo::tools::InterfaceConfig>();
    interface_config->set_name(name);
    interface_config->LoadConfig(if_config_path);

#if defined(ENABLE_ROS1)
  ros::init(argc, argv, name);
  ROS_INFO("\033[1;32m----> DataProcessor Started (auto version).\033[0m");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  auto _pRos1Convert = std::make_shared<Ros1Convert>(nh, private_nh);
  if (!_pRos1Convert->Init(runtime_config, interface_config)) {
    throw std::runtime_error("failed to initialize ROS1 data processor");
  }

  int worker_status = 0;
  std::thread worker([&]() {
    try {
      _pRos1Convert->Run();
    } catch (const std::exception& e) {
      std::cerr << "DataProcessor worker failed: " << e.what() << std::endl;
      worker_status = 1;
    }
    ros::shutdown();
  });

  // spin 与 thread 联动，保证了 lidar drvier 触发顺序
  ros::spin();
  if (worker.joinable()) worker.join();
  return worker_status;

#elif defined(ENABLE_ROS2)
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>(name);

  auto _pRos2Convert = std::make_shared<Ros2Convert>(nh);
  if (!_pRos2Convert->Init(runtime_config, interface_config)) {
    throw std::runtime_error("failed to initialize ROS2 data processor");
  }

  int worker_status = 0;
  std::thread worker([&]() {
    try {
      _pRos2Convert->Run();
    } catch (const std::exception& e) {
      std::cerr << "DataProcessor worker failed: " << e.what() << std::endl;
      worker_status = 1;
    }
    rclcpp::shutdown();
  });

  rclcpp::spin(nh);
  if (worker.joinable()) worker.join();
  return worker_status;

#endif
  } catch (const std::exception& e) {
    std::cerr << "DataProcessor startup failed: " << e.what() << std::endl;
    return 1;
  }
}
