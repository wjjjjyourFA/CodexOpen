#include "modules/perception/camera_location_estimation/ros1_convert.h"

Ros1Convert::Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  node = nh;

  camera_params = std::make_shared<camera::CameraParams>();
  image_locator = std::make_shared<cle::CameraLocationEstimation>();
  fusion        = std::make_shared<fusion::LidarCameraFusion>();
}

Ros1Convert::~Ros1Convert() {}

void Ros1Convert::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    std::lock_guard<std::mutex> lock(mutex_);  // 自动加解锁
    msg_time = msg->header.stamp.toSec() * 1000;

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
    // msg_time = int64((int64)msg->header.stamp.sec * 1000 +
    //                  (int64)msg->header.stamp.nsec * 1e-6);
    msg_time = msg->header.stamp.toSec() * 1000;

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
  // std::cout << "lidar time :" << msg_time << std::endl;

  // 把msg消息指针转化为点云指针
#if defined(VELODYNE)
  pcl::fromROSMsg(*msg, *vd_cloud_ptr);
  // VdToPcl(vd_cloud_ptr, raw_cloud_ptr, false);
  VdToPcl(vd_cloud_ptr, raw_cloud_ptr, true);
  // std::cout << "vd_cloud_ptr convert suc " << std::endl;
#else
  pcl::fromROSMsg(*msg, *rs_cloud_ptr);
  RsToPcl(rs_cloud_ptr, raw_cloud_ptr);
#endif

  point_recv_ = true;
}

void Ros1Convert::PointCloudCallback(
    const sensor_msgs::PointCloudConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  uint64_t msg_time = msg->header.stamp.toSec() * 1000;
  // std::cout << "lidar time :" << msg_time << std::endl;

  // PointCloud transfer PointCloud2 XYZ
  sensor_msgs::PointCloud2 cloud2_ptr;
  sensor_msgs::convertPointCloudToPointCloud2(*msg, cloud2_ptr);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(cloud2_ptr, *cloud_xyz_ptr);

  ConvertXYZtoXYZI(cloud_xyz_ptr, raw_cloud_ptr);

  point_recv_ = true;
}

bool Ros1Convert::Init(std::shared_ptr<cle::RuntimeConfig> rparam,
                       std::shared_ptr<cle::InterfaceConfig> iparam) {
  rparam_ = rparam;
  iparam_ = iparam;

  if (!iparam_->b_compressed) {
    image_sub = node.subscribe<sensor_msgs::Image>(
        iparam_->image_topic, 1,
        std::bind(&Ros1Convert::ImageCallback, this, std::placeholders::_1));
  } else {
    image_sub = node.subscribe<sensor_msgs::CompressedImage>(
        iparam_->image_topic, 1,
        std::bind(&Ros1Convert::ImageCompressedCallback, this,
                  std::placeholders::_1));
  }

  if (rparam_->b_undistort) {
    camera_undistort = std::make_shared<camera::UndistortionHandler>();
  }

  cloud_sub = node.subscribe<sensor_msgs::PointCloud2>(
      iparam_->lidar_topic, 1,
      std::bind(&Ros1Convert::PointCloud2Callback, this,
                std::placeholders::_1));

  image_locator->Init(rparam_->engine_file);
  image_locator->Start();

  camera_params->LoadFromFile(rparam_->calib_file_path /*kk.ini*/);

  fusion->set_params("Lidar", rparam_->dist_threshold);

#if defined(RSLIDAR_OLD)
  rs_cloud_ptr = pcl::make_shared<pcl::PointCloud<robosense_ros::PointII>>();
#elif defined(RSLIDAR_NEW)
  rs_cloud_ptr = pcl::make_shared<pcl::PointCloud<robosense_ros::PointIF>>();
#elif defined(VELODYNE)
  vd_cloud_ptr = pcl::make_shared<pcl::PointCloud<velodyne_ros::PointXYZIR>>();
#else
  rs_cloud_ptr = pcl::make_shared<pcl::PointCloud<robosense_ros::Point>>();
#endif

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
  cv::Mat img;
  CloudT::Ptr cloud;

  bool first_flag = false;

  ros::Rate loop_rate(iparam_->rate);

  image_locator->Start();

  while (ros::ok()) {
    ros::spinOnce();

    if (!image_recv_ || !point_recv_) {
      continue;
    }

    // 串行可以不用拷贝
    mutex_.lock();
    img   = recvImg;
    cloud = raw_cloud_ptr;
    mutex_.unlock();

    if (!first_flag) {
      auto matrix = camera_params->GetMatrixVector();
      Eigen::VectorXf params(17);
      params = cfg::IntrinsicParamsToVector(
          matrix.at(0)->camera_matrix->intrinsic_matrix,
          matrix.at(0)->camera_matrix->distortion_params);

      fusion->SetProjectionMatrix(matrix.at(0)->projection_matrix);
      image_locator->SetProjectionMatrix(matrix.at(0)->projection_matrix);

      if (rparam_->b_undistort) {
        camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
        camera_undistort->InitParams(img.cols, img.rows, params);
        camera_undistort->Init("camera");
      }

      first_flag = true;
    }

    cv::Mat dst_img = img.clone();
    if (rparam_->b_undistort) {
      camera_undistort->Handle(img, &dst_img);
    }

    CloudT::Ptr dst_cloud_ptr = cloud;
    fusion->SetLidarPointCloud(dst_cloud_ptr);
    fusion->SetCameraImage(dst_img);
    fusion->fuse(2, true, false);

    cv::Mat fused_image;
    fusion->GetFusedImage(fused_image);

    if (image_locator->isInited()) {
      cv::Mat show_image = dst_img;
      image_locator->DetectionAndLocation(dst_img, fused_image, show_image);
    }

    cv::namedWindow("image_loc", cv::WINDOW_GUI_NORMAL);
    cv::imshow("image_loc", dst_img);
    // cv::resizeWindow("image_loc", 512, 256);
    cv::resizeWindow("image_loc", 1024, 768);
    cv::waitKey(1);

    loop_rate.sleep();
  }
}
