#include <thread>

#include "modules/common/environment_conf.h"
#if defined(ENABLE_ROS1)
#include "modules/perception/camera_location_estimation/ros1_convert.h"
#elif defined(ENABLE_ROS2)
#include "modules/perception/camera_location_estimation/ros2_convert.h"
#endif

using namespace std;
using namespace jojo::perception;
using namespace jojo::perception::cle;
namespace cle = jojo::perception::cle;

int main(int argc, char** argv) {
  std::string name = "ImageLocation";

  // clang-format off
  std::string config_path = "./../../../config/CameraLocation/CameraLocationEstimation.ini";
  std::string if_config_path = "./../../../config/CameraLocation/Interface.ini";
  // clang-format on

  // 如果命令行参数提供了自定义配置路径，则使用该路径
  if (argc > 1) {
    config_path = argv[1];
  }

  auto runtime_config = std::make_shared<cle::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

  auto interface_config = std::make_shared<cle::InterfaceConfig>();
  interface_config->set_name(name);
  interface_config->LoadConfig(if_config_path);

  if (!runtime_config->valid || !interface_config->valid) {
    std::cerr << "Invalid camera location configuration: "
              << (!runtime_config->valid ? runtime_config->validation_error
                                         : interface_config->validation_error)
              << std::endl;
    return 1;
  }

#if defined(ENABLE_ROS1)
  ROS_INFO("\033[1;32m----> ImageLocation Started .\033[0m");

  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  auto _pRos1Convert = std::make_shared<Ros1Convert>(nh, private_nh);
  if (!_pRos1Convert->Init(runtime_config, interface_config)) return 1;

  // 回调只更新最新数据，融合定位仍由原来的 Run 循环执行。
  std::thread worker(&Ros1Convert::Run, _pRos1Convert);
  ros::spin();
  worker.join();
  _pRos1Convert->Stop();

#elif defined(ENABLE_ROS2)
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>(name);

  auto _pRos2Convert = std::make_shared<Ros2Convert>(nh);
  _pRos2Convert->Init(runtime_config, interface_config);

  // _pRos2Convert->Run();

  std::thread a(&Ros2Convert::Run, _pRos2Convert);
  a.detach();

  rclcpp::spin(nh);
  rclcpp::shutdown();

#endif

  return 0;
}
