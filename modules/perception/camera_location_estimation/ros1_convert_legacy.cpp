#include "modules/perception/camera_location_estimation/ros1_convert_legacy.h"

#include <utility>

Ros1Convert::Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : camera_params_(std::make_shared<camera::CameraParams>()),
      fusion_(std::make_shared<fusion::LidarCameraFusion>()),
      node_(nh) {
  (void)private_nh;
}

Ros1Convert::~Ros1Convert() { Stop(); }

void Ros1Convert::ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    const cv_bridge::CvImagePtr image =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (stopping_ || image->image.empty()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_ = image->image;
    has_image_    = true;
  } catch (const cv_bridge::Exception&) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    return;
  }
}

void Ros1Convert::ImageCompressedCallback(
    const sensor_msgs::CompressedImageConstPtr& msg) {
  try {
    const cv_bridge::CvImagePtr image =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (stopping_ || image->image.empty()) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_ = image->image;
    has_image_    = true;
  } catch (const cv_bridge::Exception&) {
    ROS_ERROR("Could not convert compressed image to 'bgr8'.");
    return;
  }
}

void Ros1Convert::PointCloud2Callback(
    const sensor_msgs::PointCloud2ConstPtr& msg) {
  CloudT::Ptr cloud(new CloudT());
#if defined(VELODYNE)
  auto source = pcl::make_shared<pcl::PointCloud<velodyne_ros::PointXYZIR>>();
  pcl::fromROSMsg(*msg, *source);
  VdToPcl(source, cloud, true);
#elif defined(RSLIDAR_OLD)
  auto source = pcl::make_shared<pcl::PointCloud<robosense_ros::PointII>>();
  pcl::fromROSMsg(*msg, *source);
  RsToPcl(source, cloud);
#elif defined(RSLIDAR_NEW)
  auto source = pcl::make_shared<pcl::PointCloud<robosense_ros::PointIF>>();
  pcl::fromROSMsg(*msg, *source);
  RsToPcl(source, cloud);
#else
  auto source = pcl::make_shared<pcl::PointCloud<robosense_ros::Point>>();
  pcl::fromROSMsg(*msg, *source);
  RsToPcl(source, cloud);
#endif
  if (stopping_ || !cloud || cloud->empty()) return;
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_cloud_ = std::move(cloud);
  has_cloud_    = true;
}

void Ros1Convert::PointCloudCallback(
    const sensor_msgs::PointCloudConstPtr& msg) {
  sensor_msgs::PointCloud2 cloud2;
  sensor_msgs::convertPointCloudToPointCloud2(*msg, cloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud2, *xyz);
  CloudT::Ptr cloud(new CloudT());
  ConvertXYZtoXYZI(xyz, cloud);
  if (stopping_ || !cloud || cloud->empty()) return;
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_cloud_ = std::move(cloud);
  has_cloud_    = true;
}

bool Ros1Convert::Init(std::shared_ptr<cle::RuntimeConfig> rparam,
                       std::shared_ptr<cle::InterfaceConfig> iparam) {
  if (!rparam || !iparam) return false;

  std::string error;
  if (!rparam->Validate(&error) || !iparam->Validate(&error)) {
    ROS_ERROR("Invalid camera location configuration: %s", error.c_str());
    return false;
  }
  rparam_ = std::move(rparam);
  iparam_ = std::move(iparam);

  if (!camera_params_->LoadFromFile(rparam_->calib_file_path)) return false;
  const auto matrices = camera_params_->GetMatrixVector();
  if (matrices.empty() || !matrices.front()) return false;
  const Eigen::Matrix4f projection = matrices.front()->projection_matrix;
  if (!fusion_->SetProjectionMatrix(projection) ||
      !fusion_->set_params("Lidar", rparam_->dist_threshold))
    return false;

  image_locator_.reset(
      new cle::CameraLocationEstimation(rparam_->inference_mode));
  if (!image_locator_->Initialize(rparam_->engine_file, rparam_->location,
                                  &error) ||
      !image_locator_->SetProjectionMatrix(projection)) {
    ROS_ERROR("Camera location initialization failed: %s", error.c_str());
    return false;
  }
  image_locator_->Start();

  if (rparam_->b_undistort)
    camera_undistort_ = std::make_shared<camera::UndistortionHandler>();
  undistortion_ready_ = !rparam_->b_undistort;
  stopping_           = false;

  if (iparam_->b_compressed) {
    image_sub_ = node_.subscribe<sensor_msgs::CompressedImage>(
        iparam_->image_topic, 1, &Ros1Convert::ImageCompressedCallback, this);
  } else {
    image_sub_ = node_.subscribe<sensor_msgs::Image>(
        iparam_->image_topic, 1, &Ros1Convert::ImageCallback, this);
  }
  cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>(
      iparam_->lidar_topic, 1, &Ros1Convert::PointCloud2Callback, this);
  return true;
}

void Ros1Convert::Stop() {
  if (stopping_.exchange(true)) return;

  image_sub_.shutdown();
  cloud_sub_.shutdown();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_.release();
    latest_cloud_.reset();
    has_image_ = false;
    has_cloud_ = false;
  }

  if (image_locator_) image_locator_->Stop();
}

void Ros1Convert::Run() {
  if (!iparam_) return;

  ros::Rate loop_rate(iparam_->rate);
  while (ros::ok() && !stopping_) {
    cv::Mat image;
    CloudT::Ptr cloud;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (has_image_ && has_cloud_) {
        // 保留原来的 Run 逻辑：使用回调中保存的最新图像和最新点云。
        image = latest_image_;
        cloud = latest_cloud_;
      }
    }

    if (!image.empty() && cloud && !cloud->empty()) Process(image, cloud);
    loop_rate.sleep();
  }
}

void Ros1Convert::Process(cv::Mat image, const CloudT::Ptr& cloud) {
  if (stopping_ || image.empty() || !cloud || cloud->empty() ||
      !image_locator_ || !image_locator_->isInited()) {
    return;
  }

  if (!undistortion_ready_) {
    const auto matrices = camera_params_->GetMatrixVector();
    if (matrices.empty() || !matrices.front() ||
        !matrices.front()->camera_matrix) {
      ROS_ERROR_THROTTLE(1.0, "Camera calibration matrix is unavailable.");
      return;
    }
    Eigen::VectorXf params = cfg::IntrinsicParamsToVector(
        matrices.front()->camera_matrix->intrinsic_matrix,
        matrices.front()->camera_matrix->distortion_params);
    camera_undistort_->InitModel(camera::CameraDistortionModel::Brown);
    camera_undistort_->InitParams(image.cols, image.rows, params);
    camera_undistort_->Init("camera");
    undistortion_ready_ = true;
  }

  if (rparam_->b_undistort) {
    cv::Mat corrected;
    camera_undistort_->Handle(image, &corrected);
    if (corrected.empty()) return;
    image = std::move(corrected);
  }

  if (!fusion_->SetLidarPointCloud(cloud) || !fusion_->SetCameraImage(image) ||
      !fusion_->fuse(2, true, false)) {
    return;
  }

  cv::Mat mask;
  if (!fusion_->GetFusedImage(mask)) return;

  cle::LocationEstimateResult result;
  if (!image_locator_->Estimate(image, mask, &result)) {
    ROS_WARN_THROTTLE(1.0, "Camera location failed: %s", result.error.c_str());
    return;
  }

  cv::Mat visualization = image.clone();
  image_locator_->Visualize(visualization, result);
  cv::namedWindow("image_loc", cv::WINDOW_GUI_NORMAL);
  cv::imshow("image_loc", visualization);
  cv::resizeWindow("image_loc", 1024, 768);
  cv::waitKey(1);
}
