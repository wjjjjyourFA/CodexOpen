#include "modules/drivers/camera/ros2_convert.h"

Ros2Convert::Ros2Convert() {}

Ros2Convert::~Ros2Convert() {
  for (auto& t : threads_) {
    if (t.joinable()) t.join();
  }
}

// 这里指的是CV格式的图像
void Ros2Convert::image_pub(SensorConfig& config, DrvierWrapper& driver) {
  rclcpp::Time cur_time = node->get_clock()->now();
  cv::Mat tmp_image = driver.raw_image->image;

  cv::Mat output_image = tmp_image;
  // sensor_msgs::msg::Image::SharedPtr ros2_msg;
  if (config.width != tmp_image.cols) {
    cv::Mat resize_image = driver.raw_image_for_resize;
    cv::resize(tmp_image, resize_image, resize_image.size());
    output_image = resize_image;
  }

  std_msgs::msg::Header header;
  header.stamp = cur_time;
  header.frame_id = driver.conf->frame_id();
  driver.ros2_msg = cv_bridge::CvImage(header, "bgr8", output_image).toImageMsg();

  // uint64_t msg_time = ros2_msg->header.stamp.toSec() * 1000;
  // std::cout << "msg_time " << driver.index << " : " << msg_time << std::endl;
  driver.pub->publish(driver.ros2_msg);
}

// 这里指的是YUV格式的图像
void Ros2Convert::image_compressed_pub() {}

bool Ros2Convert::Init(std::shared_ptr<rclcpp::Node> nh,
                       std::shared_ptr<drivers::ConfigManager> param) {

  // 只有 rclcpp::Node 自身管理的参数才会被 ros2 param list 看到
  node = nh;
  // private_node = private_nh;
  param_ = param;

  ns = param_->GetVehicleName();

  std::vector<SensorConfig>& camera_configs =
      param_->vehicle_model_config.camera_configs;

  if (camera_configs.size() < 1) {
    return false;
  }

  // 在头文件中就已经创建 Node 实例
  it = std::make_shared<image_transport::ImageTransport>(node);

  // 初始化多个相机 Capture
  int num = 1;
  for (auto config : camera_configs) {
    DrvierWrapper driver;

    // load config.config_file
    std::string config_file_path_ = config.config_file;
    // std::cout<<"config_file_path_: "<< config.config_file <<std::endl;

    driver.conf = std::make_shared<Config>();
    if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                                 driver.conf.get())) {
      return false;
    }
    // std::cout << "========>: " << std::endl;
    // AINFO << "UsbCam config: " << driver.conf->DebugString();
    // std::cout << std::endl;

    // init camera_device
    driver.camera_device = std::make_shared<camera::UsbCamCv>();
    driver.camera_device->init(driver.conf);
    driver.camera_device->SimpleDebug();
    driver.index = num++;

    // init ros pub
    std::string topic = "/" + ns + driver.conf->channel_name();
    // std::cout<<"topic: "<< topic <<std::endl;
    std::string param_topic = GetTopicToParamName(topic);

    // !!! shared_ptr 共享的 node 设置参数
    // 声明参数
    // 在 ROS2 参数系统里，参数名字不能带斜杠 / 开头
    node->declare_parameter<int>(param_topic + ".jpeg_quality", param_->GetCompressRatio());
    node->declare_parameter<std::string>(param_topic + ".format", "jpeg");

    // 不用 shared_ptr 管理它
    driver.pub = it->advertise(topic, 1);

    // RCLCPP_INFO(node->get_logger(), "Publisher Created: %s, JPEG quality: %d",
    //             topic.c_str(),
    //             node->get_parameter("jpeg_quality").as_int());

    driver.topic = topic;

    // clang-format off
    camera::CameraImagePtr raw_image_ = std::make_shared<camera::CameraImage>();
    raw_image_->width                  = driver.conf->width();
    raw_image_->height                 = driver.conf->height();
    raw_image_->bytes_per_pixel        = driver.conf->bytes_per_pixel();
    raw_image_->yuv_image.create(raw_image_->height, raw_image_->width, CV_8UC2);
    raw_image_->image.create(raw_image_->height, raw_image_->width, CV_8UC3);
    driver.raw_image = raw_image_;
    // clang-format on

    // Resize
    if (config.width != raw_image_->width ||
        config.height != raw_image_->height) {
      // 预分配内存
      driver.raw_image_for_resize =
          cv::Mat(config.height, config.width, CV_8UC3);

      driver.resize_enabled = true;
    }

    driver_vector.push_back(driver);
  }

  return true;
}

void Ros2Convert::Run() {
  if (driver_vector.size() == 1) {
    // SingleChannel(0);
    MultiChannel();
  } else if (driver_vector.size() > 1) {
    MultiChannel();
  } else {
    return;
  }
}

void Ros2Convert::SingleChannel(int index) {
  SensorConfig config_i = param_->vehicle_model_config.camera_configs.at(index);

  auto driver_i = driver_vector.at(index);
  rclcpp::Rate loop_rate(driver_i.conf->frame_rate());

  int error_count = 0;
  while (rclcpp::ok()) {
    if (!driver_i.camera_device->wait_for_device()) {
      if (error_count > 3) {
        std::cerr << " -------->> " << driver_i.conf->camera_dev()
                  << " set error " << std::endl;
        break;
      }
      error_count++;
      // from task.h
      // sleep for next check
      // apollo::cyber::SleepFor(std::chrono::milliseconds(device_wait_));
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      continue;
    }

    if (!driver_i.camera_device->poll(driver_i.raw_image)) {
      AERROR << "camera device poll failed";
      // continue;
    } else {
      image_pub(config_i, driver_i);
    }

    loop_rate.sleep();

    // DEBUG
    // cv::imshow("test", driver_vector.at(0).raw_image->image);
    // cv::waitKey(1);
  }
}

void Ros2Convert::MultiChannel() {
  /*
  for (size_t i = 0; i < driver_vector.size(); ++i) {
    std::thread([this, i]() {
      SingleChannel(i);  // 多线程启动每个通道
    }).detach();
  }
  */

  for (size_t i = 0; i < driver_vector.size(); ++i) {
    threads_.emplace_back([this, i]() { SingleChannel(i); });
  }

  // humble has; foxy no; 主线程等待退出
  // rclcpp::wait_for_shutdown();
}
