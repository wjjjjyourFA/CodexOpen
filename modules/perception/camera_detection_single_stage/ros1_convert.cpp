#include "modules/perception/camera_detection_single_stage/ros1_convert.h"

Ros1Convert::Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  node = nh;

  camera_params  = std::make_shared<camera::CameraParams>();
  image_detector = std::make_shared<cdss::YoloObstacleDetector>();
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

bool Ros1Convert::Init(std::shared_ptr<cdss::RuntimeConfig> rparam,
                       std::shared_ptr<cdss::InterfaceConfig> iparam) {
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

  camera_params->LoadFromFile(rparam_->calib_file_path /*kk.ini*/);

  image_detector->Init(std::string(rparam_->engine_file));

  return true;
}

void Ros1Convert::Run() {
  cv::Mat img;

  bool first_flag = !rparam_->b_undistort;

  ros::Rate loop_rate(iparam_->rate);

  image_detector->Start();

  while (ros::ok()) {
    ros::spinOnce();

    if (!image_recv_) {
      continue;
    }

    // 串行可以不用拷贝
    mutex_.lock();
    // img = recvImg.clone();
    img = recvImg;
    mutex_.unlock();

    if (img.empty()) {
      loop_rate.sleep();
      continue;
    }

    if (!first_flag) {
      auto matrix = camera_params->GetMatrixVector();
      if (matrix.empty() || !camera_undistort) {
        ROS_ERROR("Invalid camera calibration or undistortion handler.");
        return;
      }

      Eigen::VectorXf params(17);
      params = cfg::IntrinsicParamsToVector(
          matrix.front()->camera_matrix->intrinsic_matrix,
          matrix.front()->camera_matrix->distortion_params);

      camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
      camera_undistort->InitParams(img.cols, img.rows, params);
      camera_undistort->Init("camera");

      first_flag = true;
    }

    cv::Mat dst_img;
    if (rparam_->b_undistort) {
      dst_img = img.clone();
      camera_undistort->Handle(img, &dst_img);
    } else {
      dst_img = img;
    }

    std::vector<jojo::perception::base::Object> detections;
    if (image_detector->isInited()) {
      image_detector->YOLO(dst_img, detections, dst_img.cols, dst_img.rows,
                           true);
    }

    bool show = false;
    if (show) {
      cv::namedWindow("image_det", cv::WINDOW_GUI_NORMAL);
      cv::imshow("image_det", dst_img);
      cv::resizeWindow("image_det", 512, 256);
      cv::waitKey(1);
    }

    loop_rate.sleep();
  }
}
