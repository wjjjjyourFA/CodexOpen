#include "modules/perception/common/fusion/radar2camera/ros1_convert.h"

Ros1Convert::Ros1Convert() {
  radar_params  = std::make_shared<cfg::SensorExtrinsics>();
  camera_params = std::make_shared<camera::CameraParams>();
  fusion        = std::make_shared<fusion::RadarCameraFusion>();
}

Ros1Convert::~Ros1Convert() {}

void Ros1Convert::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    std::lock_guard<std::mutex> lock(mutex_);  // 自动加解锁
    uint64_t msg_time = msg->header.stamp.toSec() * 1000;

    // recvImg = (cv_bridge::toCvShare(msg, "bgr8")->image).clone();
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    recvImg = cv_ptr->image;  // 浅拷贝

    image_recv_ = true;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void Ros1Convert::ImageCompressedCallback(
    const sensor_msgs::CompressedImageConstPtr& msg) {
  try {
    std::lock_guard<std::mutex> lock(mutex_);  // 自动加解锁
    // uint64_t msg_time = int64((int64)msg->header.stamp.sec * 1000 +
    //                     (int64)msg->header.stamp.nsec * 1e-6);
    uint64_t msg_time = msg->header.stamp.toSec() * 1000;

    cv_bridge::CvImagePtr cv_ptr_compressed =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    recvImg = cv_ptr_compressed->image;

    image_recv_ = true;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from compressed image to 'bgr8'.");
    return;
  }
}

void Ros1Convert::PointCloud2Callback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint64_t msg_time = msg->header.stamp.toSec() * 1000;
  // std::cout << "radar time :" << msg_time << std::endl;

  // 把msg消息指针转化为点云指针
  pcl::fromROSMsg(*msg, *raw_cloud_ptr);

  point_recv_ = true;
}

void Ros1Convert::PointCloudCallback(
    const sensor_msgs::PointCloudConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint64_t msg_time = msg->header.stamp.toSec() * 1000;
  // std::cout << "radar time :" << msg_time << std::endl;

  // PointCloud transfer PointCloud2 XYZ
  sensor_msgs::PointCloud2 cloud2_ptr;
  sensor_msgs::convertPointCloudToPointCloud2(*msg, cloud2_ptr);

  /* debug
  for (const auto& field : cloud2_ptr.fields) {
    ROS_INFO("Field: %s", field.name.c_str());
  }
  */

  pcl::fromROSMsg(cloud2_ptr, *raw_cloud_ptr);

  point_recv_ = true;
}

bool Ros1Convert::Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                       std::shared_ptr<perception::RuntimeConfig> param) {
  node   = nh;
  param_ = param;

  if (!param_->b_compressed) {
    image_sub = node.subscribe<sensor_msgs::Image>(
        param_->image_topic, 1,
        std::bind(&Ros1Convert::ImageCallback, this, std::placeholders::_1));
  } else {
    image_sub = node.subscribe<sensor_msgs::CompressedImage>(
        param_->image_topic, 1,
        std::bind(&Ros1Convert::ImageCompressedCallback, this,
                  std::placeholders::_1));
  }

  if (param_->b_pointcloud2) {
    cloud_sub = node.subscribe<sensor_msgs::PointCloud2>(
        param_->radar_topic, 1,
        std::bind(&Ros1Convert::PointCloud2Callback, this,
                  std::placeholders::_1));
  } else {
    std::cout << "radar is using pointcloud " << std::endl;
    cloud_sub = node.subscribe<sensor_msgs::PointCloud>(
        param_->radar_topic, 1,
        std::bind(&Ros1Convert::PointCloudCallback, this,
                  std::placeholders::_1));
  }

  if (param_->b_undistort) {
    camera_undistort = std::make_shared<camera::UndistortionHandler>();
  }

  InitTransfParams();

  fusion->set_params("Radar", param_->dist_threshold);

  if (!raw_cloud_ptr) {
    // raw_cloud_ptr = boost::make_shared<CloudT>();  // 推荐写法
    raw_cloud_ptr = CloudT::Ptr(new CloudT());
  }

  if (!dst_cloud_ptr) {
    dst_cloud_ptr = CloudT::Ptr(new CloudT());
  }

  return true;
}

void Ros1Convert::Run() {
  ros::Rate loop_rate(param_->rate);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();

    if (!image_recv_ || !point_recv_) {
      continue;
    }

    cv::Mat& img = recvImg;

    static bool first_flag = false;
    if (!first_flag) {
      auto matrix = camera_params->GetMatrixVector();
      Eigen::VectorXf params(17);
      params = cfg::IntrinsicParamsToVector(
          matrix.at(0)->camera_matrix->intrinsic_matrix,
          matrix.at(0)->camera_matrix->distortion_params);

      fusion->SetProjectionMatrix(matrix.at(0)->projection_matrix);

      if (param_->b_undistort) {
        camera_undistort->InitParams(img.cols, img.rows, params);
        camera_undistort->Init("camera");
      }

      first_flag = true;
    }

    static bool first_create = false;
    if (!first_create) {
      dst_img.create(img.size(), img.type());  // 初始化一次
      first_create = true;
    }

    if (param_->b_undistort) {
      camera_undistort->Handle(img, &dst_img);
    }

    // 20250611 calib is mm
    // 20250612 I set calib readparam ==> mm2m
    /*
    pcl::PointCloud<pcl::PointXYZI>::Ptr dst_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& pt : raw_cloud_ptr->points) {
      PointT new_pt;
      new_pt.x         = pt.x * 1000.0f;
      new_pt.y         = pt.y * 1000.0f;
      new_pt.z         = pt.z * 1000.0f;
      new_pt.intensity = pt.intensity;  // 保留原始强度

      dst_cloud_ptr->points.push_back(new_pt);
    }
    */
    dst_cloud_ptr = raw_cloud_ptr;

    show2d_lidar_data(dst_cloud_ptr, 0, 1);

    fusion->SetRadarPointCloud(dst_cloud_ptr);
    fusion->SetCameraImage(dst_img);
    fusion->fuse(1, false);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fused_point_cloud_color;
    cv::Mat fused_image;
    fusion->GetFusedImage(fused_image);
    fusion->GetFusedPointCloudColor(fused_point_cloud_color);

    static bool first_run = true;
    if (first_run) {
      cv::namedWindow("fused_image", cv::WINDOW_GUI_NORMAL);
      cv::resizeWindow("fused_image", 512, 256);  // 只在第一次运行时设置
      first_run = false;
    }
    cv::imshow("fused_image", fused_image);
    cv::waitKey(1);
  }
}

void Ros1Convert::InitTransfParams() {
  camera_params->LoadFromFile(param_->camera_calib_file_path /*kk.ini*/);
  radar_params->LoadFromFile(param_->radar_calib_file_path /*radar.ini*/);

  // radar => lidar => image
  auto camera_matrix = camera_params->GetMatrixVector();
  auto radar_matrix  = radar_params->GetMatrixVector();

  Eigen::Matrix4f extrinsic_matrix = camera_matrix.at(0)->extrinsic_matrix *
                                     radar_matrix.at(0)->extrinsic_matrix;
  // std::cout << "radar extrinsic_matrix: \n" << extrinsic_matrix << std::endl;

  // update to camera_matrix
  camera_matrix.at(0)->extrinsic_matrix = extrinsic_matrix;
  camera_matrix.at(0)->UpdataProjectionMatrix();
}
