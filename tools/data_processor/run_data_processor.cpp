#include <thread>

#include "modules/common/environment_conf.h"
#if defined(ENABLE_ROS1)
#include "tools/data_processor/ros1_convert.h"
#elif defined(ENABLE_ROS2)
#include "tools/data_processor/ros2_convert.h"
#endif

using namespace std;
using namespace jojo::tools;

int main(int argc, char** argv) {
  std::string name = "DataProcessor";
  std::string config_path = "./../../config/DataProcessor/DataProcessor.ini";

  // 如果命令行参数提供了自定义配置路径，则使用该路径
  if (argc > 1) {
    config_path = argv[1];
  }

  auto runtime_config = std::make_shared<RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

#if defined(ENABLE_ROS1)
  ROS_INFO("\033[1;32m----> DataProcessor Started (auto version).\033[0m");

  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  auto _pRos1Convert = std::make_shared<Ros1Convert>();
  _pRos1Convert->Init(nh, private_nh, runtime_config);

  // _pRos1Convert->Run();

  std::thread a(&Ros1Convert::Run, _pRos1Convert);
  a.detach();

  // spin 与 thread 联动，保证了 lidar drvier 触发顺序
  ros::spin();

#elif defined(ENABLE_ROS2)
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>(name);

  auto _pRos2Convert = std::make_shared<Ros2Convert>();
  _pRos2Convert->Init(nh, runtime_config);

  // _pRos2Convert->Run();

  std::thread a(&Ros2Convert::Run, _pRos2Convert);
  a.detach();

  rclcpp::spin(nh);
  rclcpp::shutdown();

#endif

  return 0;
}
