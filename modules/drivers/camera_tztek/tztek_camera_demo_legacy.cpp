#include <iostream>
#include <string>
#include <thread>

#include "modules/common/environment_conf.h"
#if defined(ENABLE_ROS1)
#include "modules/drivers/camera_tztek/ros1_convert.h"
#elif defined(ENABLE_ROS2)
#include "modules/drivers/camera_tztek/ros2_convert.h"
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

  // 获取线程的底层句柄
  pthread_t nativeHandle = pthread_self();
  // 创建线程属性对象
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  // 设置线程调度策略为RR类型
  pthread_attr_setschedpolicy(&attr, SCHED_RR);
  // 创建线程参数对象
  struct sched_param param;
  param.sched_priority = 90;
  // 设置线程调度参数
  pthread_attr_setschedparam(&attr, &param);
  // 设置线程属性
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  // 修改线程的调度属性
  pthread_setschedparam(nativeHandle, SCHED_RR, &param);
  // 销毁线程属性对象
  pthread_attr_destroy(&attr);

#if defined(ENABLE_ROS1)
  // 这里是线程启动 ros1_convert::run()的示例
  // ros 初始化
  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  // auto _pRos1Convert = std::make_shared<Ros1Convert>();
  // _pRos1Convert->Init(nh, private_nh, config_manager);

  // // std::thread a(std::bind(&Ros1Convert::Run, _pRos1Convert));
  // // a.detach();
  // _pRos1Convert->Run();

  // 存储启用的相机通道号列表
  std::vector<int> channum{2, 8, 9, 10, 12, 13, 14};
  int cam_num = channum.size();  // 获取启用的相机数量
  CCameraMgr* cam[cam_num];  // 相机管理器数组，每个元素对应一个通道

  // 初始化每个相机管理器
  for (int i = 0; i < cam_num; i++) {
    int pipeid = channum[i];  // 当前通道ID
    uint32_t video, weight, height, fps, format;
    video  = pipeid;
    weight = 1920;
    height = 1536;
    fps    = 30;
    // format = "YUYV";
    format = 1;

    // 日志输出相机参数（通道号、分辨率、帧率、格式等）
    debug_crit(
        "i :%d,pipeid:%d video:%d weight:%d height:%d fps:%d,format:%s\n", i,
        pipeid, video, weight, height, fps, DateTypeMap[format]);

    // 创建相机管理器实例并初始化
    cam[i] = new CCameraMgr(pipeid, video, weight, height, fps, format);
    cam[i]->Init();  // 初始化相机（创建句柄、设置回调等）
  }

  // 启动所有相机的数据采集线程
  for (int i = 0; i < cam_num; i++) {
    cam[i]->Start();  // 调用相机驱动开始采集图像
  }

  ros::spin();
  ros::shutdown();

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
