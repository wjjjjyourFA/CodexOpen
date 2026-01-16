#include "modules/common/environment_conf.h"
#if defined(ENABLE_ROS1)
#include "modules/perception/common/fusion/lidar2camera/ros1_convert.h"
#elif defined(ENABLE_ROS2)
#include "modules/perception/common/fusion/lidar2camera/ros2_convert.h"
#endif

using namespace jojo::perception;

int main(int argc, char** argv) {
  printf("Lidar Image Fuse...\n");

  std::string cofing_path =
      "./../../../../config/PerceptionFuse/LidarCameraFuseRealTime.ini";
  auto runtime_config = std::make_shared<RuntimeConfig>();
  runtime_config->set_name("LidarImageFuseRealTime");
  runtime_config->LoadConfig(cofing_path);

#if defined(ENABLE_ROS1)
  // 这里是线程启动 ros1_convert::run()的示例
  // ros 初始化
  ros::init(argc, argv, runtime_config->get_name());
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  auto _pRos1Convert = std::make_shared<Ros1Convert>();
  _pRos1Convert->Init(nh, private_nh, runtime_config);

  // std::thread a(std::bind(&Ros1Convert::Run, _pRos1Convert));
  // a.detach();
  _pRos1Convert->Run();

  // ros::spin();

#elif defined(ENABLE_ROS2)
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>(runtime_config->get_name());

  auto _pRos2Convert = std::make_shared<Ros2Convert>();
  _pRos2Convert->Init(nh, runtime_config);

  _pRos2Convert->Run();

  rclcpp::shutdown();

#endif

  return 0;
}
