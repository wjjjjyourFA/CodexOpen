#include "modules/drivers/camera/ros1_convert.h"

Ros1Convert::Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  node = nh;
  // private_node = private_nh;
}

Ros1Convert::~Ros1Convert() {
  for (auto& t : threads_) {
    if (t.joinable()) t.join();
  }
}

// 这里指的是CV格式的图像
void Ros1Convert::image_pub(DriverWrapper& driver) {
#ifndef ONLY_COMPRESSED_IMAGE
  ros::Time cur_time = ros::Time::now();
  auto& tmp_image    = driver.raw_image->image;

  cv::Mat output_image = tmp_image;
  // sensor_msgs::ImagePtr ros1_msg;
  if (driver.resize_enabled) {
    auto& resize_image = driver.raw_image_for_resize;
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
#endif
}

// 这里指的是sensor_msgs::CompressedImage格式的图像
void Ros1Convert::image_compressed_pub(DriverWrapper& driver,
                                       const uint8_t* pData, int nDatalen) {
#if defined(ONLY_COMPRESSED_IMAGE)
  // 检查数据有效性
  if (pData == nullptr || nDatalen <= 0) {
    // ROS_WARN("Invalid image data: null pointer or zero length");
    return;
  }

  ros::Time cur_time = ros::Time::now();
  // cv::Mat tmp_image  = driver.raw_image->image;

  // 创建压缩图像消息对象
  // auto ros1_msg = boost::make_shared<sensor_msgs::CompressedImage>();
  // auto& ros1_msg = driver.ros1_msg;
  auto& ros1_msg = driver.pool[driver.idx];
  // 数据填充，传递图像数据
  // 会复制一次 JPEG
  // ros1_msg->data.assign(pData, pData + nDatalen);
  // 预分配数据空间（精确匹配输入数据长度）
  // ros1_msg->data.reserve(driver.MAX_JPEG_SIZE);
  ros1_msg->data.resize(nDatalen);
  // 也会复制一次 JPEG
  std::memcpy(ros1_msg->data.data(), pData, nDatalen);
  // 指定编码格式（固定为JPEG格式，确保ROS1客户端兼容性）
  ros1_msg->format = "jpeg";

  // ros1_msg->header.seq++;
  ros1_msg->header.stamp    = cur_time;
  ros1_msg->header.frame_id = driver.conf->frame_id();

  // uint64_t msg_time = ros1_msg->header.stamp.toSec() * 1000;
  // std::cout << "msg_time " << driver.index << " : " << msg_time << std::endl;
  driver.pub.publish(ros1_msg);

  driver.idx = (driver.idx + 1) % driver.BSIZE;
#endif
}

bool Ros1Convert::Init(std::shared_ptr<drivers::ConfigManager> param) {
  param_ = param;

  ns = param_->GetVehicleName();

  std::vector<SensorConfig>& camera_configs =
      param_->vehicle_model_config.camera_configs;

  if (camera_configs.size() < 1) {
    return false;
  }

#ifndef ONLY_COMPRESSED_IMAGE
  // 在头文件中就已经创建 Node 实例
  it = std::make_shared<image_transport::ImageTransport>(node);
#endif

  // 初始化多个相机 Capture
  int num = 1;
  driver_vector.reserve(camera_configs.size());
  for (auto& config : camera_configs) {
    driver_vector.emplace_back();
    auto& driver = driver_vector.back();

    // load config.config_file
    std::string config_file_path_ = config.config_file;
    // std::cout << "config_file_path_: " << config.config_file << std::endl;

    driver.conf = std::make_shared<Config>();
    if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                                 driver.conf.get())) {
      return false;
    }
    // std::cout << "========>: " << std::endl;
    // AINFO << "UsbCam config: " << driver.conf->DebugString();
    // std::cout << std::endl;

    // init camera_device
#if defined(ONLY_COMPRESSED_IMAGE)
    driver.camera_device = std::make_shared<camera::UsbCamCvJpeg>();
#else
    driver.camera_device = std::make_shared<camera::UsbCamCv>();
#endif
    driver.camera_device->init(driver.conf);
    driver.camera_device->DebugInfo();
    driver.index = num++;

    // init ros pub
    std::string topic = "/" + ns + driver.conf->channel_name();
    // std::cout<<"topic: "<< topic <<std::endl;

#ifndef ONLY_COMPRESSED_IMAGE
    // 设置压缩比（必须在 advertise 之前设置）
    // node.setParam(...) 设置的参数是全局可见的，it 在用的时候也会读到
    node.setParam(topic + "/compressed/jpeg_quality",
                  param_->GetCompressRatio());
    node.setParam(topic + "/compressed/format", "jpeg");
    // image_transport + compressed_image_transport 插件是通过 参数服务器 来配置的。
    // 参数是挂在 话题名下面的，比如：/camera/image/compressed/jpeg_quality

    driver.topic = topic;
    // 不用 shared_ptr 管理 publisher
    driver.pub = it->advertise(topic, 1);
#else
    // 避免 相机线程先触发 callback 但 publisher 还没初始化
    std::string ctopic = topic + "/compressed";
    driver.topic       = ctopic;

    driver.pub = node.advertise<sensor_msgs::CompressedImage>(ctopic, 1);

    auto drv = &driver;
    // DrvWrapper* drv = &driver;

    driver.camera_device->SetMJPEGCallback(
        [this, drv](const uint8_t* data, int len) {
          this->image_compressed_pub(*drv, data, len);
        });
#endif

    // clang-format off
    camera::CameraImagePtr raw_image_ = std::make_shared<camera::CameraImage>();
    driver.raw_image = raw_image_;
    raw_image_->width = driver.conf->width();
    raw_image_->height = driver.conf->height();
    raw_image_->bytes_per_pixel = driver.conf->bytes_per_pixel();
#ifndef ONLY_COMPRESSED_IMAGE
    raw_image_->yuv_image.create(raw_image_->height, raw_image_->width, CV_8UC2);
    raw_image_->image.create(raw_image_->height, raw_image_->width, CV_8UC3);
    // clang-format on
    /*
    std::cout << "raw_image_ rows=" << raw_image_->image.rows
              << " cols=" << raw_image_->image.cols
              << " type=" << raw_image_->image.type()
              << std::endl;
    std::cout << "CV_8UC3 = " << CV_8UC3 << std::endl;
    */

    // Resize
    if (config.width != raw_image_->width ||
        config.height != raw_image_->height) {
      // 预分配内存
      driver.raw_image_for_resize =
          cv::Mat(config.height, config.width, CV_8UC3);

      driver.resize_enabled = true;
    }
#else
    driver.MAX_JPEG_SIZE = driver.conf->width() * driver.conf->height();
    // std::cout << "MAX_JPEG_SIZE: " << driver.MAX_JPEG_SIZE << std::endl;
    // driver.ros1_msg = boost::make_shared<sensor_msgs::CompressedImage>();
    // driver.ros1_msg->data.reserve(driver.MAX_JPEG_SIZE);
    driver.Init();
#endif
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
      // std::this_thread::sleep_for(std::chrono::microseconds(100));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    if (!driver_i.camera_device->poll(driver_i.raw_image)) {
      AERROR << "camera device poll failed";
      // continue;
    }
#ifndef ONLY_COMPRESSED_IMAGE
    else {
      image_pub(driver_i);
    }
#endif

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
