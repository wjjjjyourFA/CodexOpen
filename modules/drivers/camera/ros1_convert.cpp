#include "modules/drivers/camera/ros1_convert.h"

/* sensor_msgs::CompressedImage cv_to_ros(cv::Mat image) {
  // 手动将cv图像格式转换为ROS压缩图像格式
  // 效率极低，并且无法在rviz中显示，不建议使用

  // boost::timer t;
  // double clock = std::clock();

  static std::string encoding = "bgr8";
  static int bitDepth         = enc::bitDepth(encoding);
  static int numChannels      = enc::numChannels(encoding);

  sensor_msgs::CompressedImage compressed;
  compressed.header = std_msgs::Header();
  compressed.format = "bgr8";

  std::vector<int> params;
  params.resize(9, 0);
  params[0] = IMWRITE_JPEG_QUALITY;
  params[1] = 90;
  params[2] = IMWRITE_JPEG_PROGRESSIVE;
  params[3] = 0;
  params[4] = IMWRITE_JPEG_OPTIMIZE;
  params[5] = 0;
  params[6] = IMWRITE_JPEG_RST_INTERVAL;
  params[7] = 0;
  compressed.format += "; jpeg compressed ";

  if ((bitDepth == 8) || (bitDepth == 16)) {
    // Target image format
    std::string targetFormat;
    if (enc::isColor(encoding)) {
      // convert color images to BGR8 format
      targetFormat = "bgr8";
      compressed.format += targetFormat;
    }
    cv::imencode(".jpg", image, compressed.data, params);
  }

  // double clock_ = std::clock() - clock;
  // std::cout << clock_ << std::endl;
  // std::cout << t.elapsed() << std::endl;
  return compressed;
} */

Ros1Convert::Ros1Convert() {}

Ros1Convert::~Ros1Convert() {
  for (auto& t : threads_) {
    if (t.joinable()) t.join();
  }
}

// 这里指的是CV格式的图像
void Ros1Convert::image_pub(SensorConfig& config, DrvierWrapper& driver) {
  ros::Time cur_time = ros::Time::now();
  cv::Mat tmp_image  = driver.raw_image->image;

  cv::Mat output_image = tmp_image;
  // sensor_msgs::ImagePtr ros1_msg;
  if (driver.resize_enabled) {
    cv::Mat resize_image = driver.raw_image_for_resize;
    cv::resize(tmp_image, resize_image, resize_image.size());
    output_image = resize_image;
  }

  driver.ros1_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_image).toImageMsg();
  driver.ros1_msg->header.seq++;
  driver.ros1_msg->header.stamp    = cur_time;
  driver.ros1_msg->header.frame_id = driver.conf->frame_id();

  // uint64_t msg_time = ros1_msg->header.stamp.toSec() * 1000;
  // std::cout << "msg_time " << driver.index << " : " << msg_time << std::endl;
  driver.pub.publish(driver.ros1_msg);
}

// 这里指的是YUV格式的图像
void Ros1Convert::image_compressed_pub() {}

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
    driver.camera_device->DebugInfo();
    driver.index = num++;

    // init ros pub
    std::string topic = "/" + ns + driver.conf->channel_name();
    // std::cout<<"topic: "<< topic <<std::endl;

    // 设置压缩比（必须在 advertise 之前设置）
    // node.setParam(...) 设置的参数是全局可见的，it 在用的时候也会读到
    node.setParam(topic + "/compressed/jpeg_quality",
                  param_->GetCompressRatio());
    node.setParam(topic + "/compressed/format", "jpeg");
    // image_transport + compressed_image_transport 插件是通过 参数服务器 来配置的。
    // 参数是挂在 话题名下面的，比如：/camera/image/compressed/jpeg_quality

    // 不用 shared_ptr 管理 publisher
    driver.pub = it->advertise(topic, 1);

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
  SensorConfig config_i = param_->vehicle_model_config.camera_configs.at(index);

  auto driver_i = driver_vector.at(index);
  ros::Rate loop_rate(driver_i.conf->frame_rate());

  int error_count = 0;
  while (ros::ok()) {
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

  std::string param_name = driver_i.topic + "/compressed/jpeg_quality";
  // std::cout << param_name << std::endl;
  if (node.hasParam(param_name)) {
    node.deleteParam(param_name);
    ROS_INFO("Deleted Compress Param: %s", param_name.c_str());
  }
}

void Ros1Convert::MultiChannel() {
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

  // 主线程等待退出
  ros::waitForShutdown();
}
