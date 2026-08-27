#include "modules/drivers/camera_tztek/ros1_convert.h"

Ros1Convert::Ros1Convert() {}

Ros1Convert::~Ros1Convert() {
  for (auto& t : threads_) {
    if (t.joinable()) t.join();
  }
}

bool Ros1Convert::Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                       std::shared_ptr<drivers::ConfigManager> param) {
  node = nh;
  // private_node = private_nh;
  param_ = param;

  ns = param_->GetVehicleName();

  std::vector<SensorConfig>& camera_configs =
      param_->vehicle_model_config.camera_configs;

  if (camera_configs.size() < 1) {
    return false;
  }

  // 初始化每个相机管理器
  int num = 1;
  driver_vector.reserve(camera_configs.size());
  for (auto& config : camera_configs) {
    driver_vector.emplace_back();
    auto& driver = driver_vector.back();

    // load config.config_file
    std::string config_file_path_ = config.config_file;

    driver.conf = std::make_shared<Config>();
    if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                                 driver.conf.get())) {
      return false;
    }
    // std::cout << "========>: " << std::endl;
    // AINFO << "UsbCam config: " << driver.conf->DebugString();
    // std::cout << std::endl;

    // 当前通道ID 硬件接口 or 数据流管线ID
    int pipeid;
    // video 使用到的设备末尾数字： /dev/video ==> 0 1 2 3 5 8
    uint32_t video, weight, height, fps, format;

    std::string dev = driver.conf->camera_dev();  // "/dev/video0"

    pipeid = GetVideoIndex(dev);  // "0"
    video  = pipeid;
    weight = driver.conf->width();
    height = driver.conf->height();
    fps    = driver.conf->frame_rate();
    std::cout << "weight: " << weight << " height: " << height
              << " fps: " << fps << std::endl;

    // 装换为大写
    std::string fmt = ToUpper(driver.conf->pixel_format());
    // 将字符串转换为对应的格式
    auto it = DateTypeMapValue.find(fmt);
    if (it != DateTypeMapValue.end()) {
      format = it->second;
    } else {
      // 处理未知格式
      AERROR << "Unknown format: " << fmt;
    }
    std::cout << "format: " << format << std::endl;

    // 创建相机管理器实例并初始化
    // driver.camera_device = new CCameraMgr();
    driver.camera_device = std::make_shared<CCameraMgr>(pipeid, video, weight,
                                                        height, fps, format);

    // init ros pub
    std::string topic = "/" + ns + driver.conf->channel_name();
    // std::cout<<"topic: "<< topic <<std::endl;

    // 避免 相机线程先触发 callback 但 publisher 还没初始化
    std::string ctopic = topic + "/compressed";
    // driver.topic = ctopic;
    driver.camera_device->SetTopic(ctopic);

    driver.camera_device->Init();
    // driver.camera_device->DebugInfo();
    driver.index = num++;
  }

  return true;
}

void Ros1Convert::Run() {
  if (driver_vector.size() == 1) {
    // SingleChannel(0);
    MultiChannel();
  } else if (driver_vector.size() > 1) {
    MultiChannel();
  } else {
    return;
  }
}

void Ros1Convert::SingleChannel(int index) {
  auto& driver_i = driver_vector.at(index);
  ros::Rate loop_rate(driver_i.conf->frame_rate());

  driver_i.camera_device->Start();

  // 检查标志以确定是否退出
  while (ros::ok()) {
    sleep(10);  // 休眠10秒（降低CPU占用）

    loop_rate.sleep();
  }

  driver_i.camera_device->Stop();

  std::string param_name = driver_i.topic + "/compressed/jpeg_quality";
  // std::cout << param_name << std::endl;
  if (node.hasParam(param_name)) {
    node.deleteParam(param_name);
    ROS_INFO("Deleted Compress Param: %s", param_name.c_str());
  }
}

void Ros1Convert::MultiChannel() {
  /* way 1
  for (size_t i = 0; i < driver_vector.size(); ++i) {
    std::thread([this, i]() {
      SingleChannel(i);  // 多线程启动每个通道
    }).detach();
  }
  */

  // way 2
  for (size_t i = 0; i < driver_vector.size(); ++i) {
    threads_.emplace_back([this, i]() { SingleChannel(i); });
  }

  // 主线程等待退出
  ros::waitForShutdown();
}
