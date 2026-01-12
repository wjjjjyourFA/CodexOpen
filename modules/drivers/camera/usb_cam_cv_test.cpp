#include <iostream>
#include <string>
#include <thread>

#include "modules/common/environment_conf.h"
#include "modules/drivers/camera/usb_cam_cv.h"
#if defined(ENABLE_ROS1)
#include "modules/drivers/camera/ros1_convert.h"
#elif defined(ENABLE_ROS2)
#include "modules/drivers/camera/ros2_convert.h"
#endif

using namespace jojo::drivers;
// using namespace apollo::drivers::camera;
// using namespace apollo::cyber::common;

int main(int argc, char** argv) {
  // 加载总体配置文件，该配置文件中包含多个相机配置
  // 有多少个就启动多少个实例
  std::string config_file_path_ = 
      "./../../../../common/vehicle_sensor_config.yaml";

  if (argc > 1) {
    config_file_path_ = argv[1];
  }

  auto config_manager = std::make_shared<ConfigManager>();
  config_manager->LoadConfig(config_file_path_);
  std::string name = config_manager->GetVehicleName() + "_CameraDriver";
  config_manager->set_name(name);
  // BaseCam cam;
  // cam.init(camera_config_);
  // cam.open_device();
  // cam.init_device();
  // cam.start_capturing();

#if defined(ENABLE_ROS1)
  // 这里是线程启动 ros1_convert::run()的示例
  // ros 初始化
  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  auto _pRos1Convert = std::make_shared<Ros1Convert>();
  _pRos1Convert->Init(nh, private_nh, config_manager);

  // std::thread a(std::bind(&Ros1Convert::Run, _pRos1Convert));
  // a.detach();
  _pRos1Convert->Run();

  ros::spin();

#elif defined(ENABLE_ROS2)
  // 这里是线程启动 ros2_convert::run()的示例
  // ros2 初始化
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>(name);
  // 全局参数设置
  // nh->declare_parameter<int>("jpeg_quality", config_manager->GetCompressRatio());
  // nh->declare_parameter<std::string>("format", "jpeg");

  auto _pRos2Convert = std::make_shared<Ros2Convert>();
  _pRos2Convert->Init(nh, config_manager);

  // std::thread a(std::bind(&Ros2Convert::Run, _pRos2Convert));
  // a.detach();
  _pRos2Convert->Run();

  // RCLCPP_INFO(nh->get_logger(), "Node running, now you can list parameters");

  rclcpp::spin(nh);
  rclcpp::shutdown();

#endif

  return 0;
}
