#include <gflags/gflags.h>
#include <glog/logging.h>
// #include <QString>

#include "modules/common/environment_conf.h"
#if defined(ENABLE_ROS1)
#include "modules/drivers/gnss/hc_wrapper/ros1_convert.h"
#elif defined(ENABLE_ROS2)
#include "modules/drivers/gnss/hc_wrapper/ros2_convert.h"
#endif

using namespace jojo::drivers;

int main(int argc, char* argv[]) {
  std::string config_file_path_ =
      "./../../../../parameters/vehicle_sensor_config.yaml";
  std::string log_file_path_ = "./../../log/gnss_hc_";

  if (argc > 1) {
    config_file_path_ = argv[1];
  }

  // logger
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::SetLogDestination(google::INFO, log_file_path_.c_str());
  // google::SetLogDestination(google::WARNING, log_file_path_);

  // 设置日志消息除了日志文件之外是否去标准输出
  FLAGS_alsologtostderr = true;
  FLAGS_log_prefix      = false;
  setbuf(stdout, NULL);

  // 从程序运行路径中提取出程序自身的文件名（不含路径），并将其转换为 std::string 类型
  // std::string program_name =
  //     QString::fromLocal8Bit(argv[0]).split('/').back().toStdString();
  // std::cout << program_name << std::endl;

  auto config_manager = std::make_shared<ConfigManager>();
  config_manager->LoadConfig(config_file_path_);
  std::string name = config_manager->GetVehicleName() + "_GnssDriver";
  config_manager->set_name(name);

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
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>(name);

  auto _pRos2Convert = std::make_shared<Ros2Convert>();
  _pRos2Convert->Init(nh, config_manager);

  // std::thread a(std::bind(&Ros2Convert::Run, _pRos2Convert));
  // a.detach();
  _pRos2Convert->Run();

  rclcpp::spin(nh);
  rclcpp::shutdown();

#endif

  return 0;
}
