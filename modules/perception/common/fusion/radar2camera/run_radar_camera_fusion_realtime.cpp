#include "modules/common/environment_conf.h"
#if defined(ENABLE_ROS1)
#include "modules/perception/common/fusion/radar2camera/ros1_convert.h"
#elif defined(ENABLE_ROS2)
#include "modules/perception/common/fusion/radar2camera/ros2_convert.h"
#endif

using namespace jojo::perception;

int main(int argc, char** argv) {
  // clang-format off
  std::string name = "RadarImageFuseRealTime";
  std::string cofing_path = "./../../../../config/PerceptionFuse/RadarCamera/RadarCameraFuseRealTime.ini";
  // clang-format on

  auto runtime_config = std::make_shared<RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(cofing_path);

  // clang-format off
  std::string if_config_path = "./../../../../config/PerceptionFuse/RadarCamera/Interface.ini";
  // clang-format on

  auto interface_config = std::make_shared<InterfaceConfig>();
  interface_config->set_name(name);
  interface_config->LoadConfig(if_config_path);

#if defined(ENABLE_ROS1)
  // 这里是线程启动 ros1_convert::run()的示例
  // ros 初始化
  ros::init(argc, argv, runtime_config->get_name());
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  auto _pRos1Convert = std::make_shared<Ros1Convert>(nh, private_nh);
  _pRos1Convert->Init(runtime_config, interface_config);

  // std::thread a(std::bind(&Ros1Convert::Run, _pRos1Convert));
  // a.detach();
  _pRos1Convert->Run();

  // ros::spin();

#elif defined(ENABLE_ROS2)
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>(runtime_config->get_name());

  auto _pRos2Convert = std::make_shared<Ros2Convert>(nh);
  _pRos2Convert->Init(runtime_config, interface_config);

  _pRos2Convert->Run();

  rclcpp::shutdown();

#endif

  return 0;
}
