#include "modules/perception/camera_location_estimation/ros1_convert.h"

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
    if (stopping_ || !image || image->image.empty()) return;

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_ = image->image;
    has_image_    = true;
    ++image_seq_;
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8': %s.",
              msg->encoding.c_str(), e.what());
  }
}

void Ros1Convert::ImageCompressedCallback(
    const sensor_msgs::CompressedImageConstPtr& msg) {
  try {
    const cv_bridge::CvImagePtr image =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (stopping_ || !image || image->image.empty()) return;

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_ = image->image;
    has_image_    = true;
    ++image_seq_;
  } catch (const cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert compressed image to 'bgr8': %s", e.what());
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
  ++cloud_seq_;
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
  ++cloud_seq_;
}

void Ros1Convert::GposePreprocessing(const globalpose_msgtype& msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_location_ = msg;
  has_global_pose_ = true;
  ++pose_seq_;
}

void Ros1Convert::LposePreprocessing(const localpose_msgtype& msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  local_location_ = msg;
  has_local_pose_ = true;
  ++pose_seq_;
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
      !fusion_->set_params("Lidar", rparam_->dist_threshold)) {
    return false;
  }

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

  // clang-format off
  object_location_projector_.reset(new jojo::localization::common::Frame2dTransform());
  // object_location_projector_.reset(new jojo::localization::common::GlobalLocationProjector());
  // clang-format on

  double transf_offset_X = 0, transf_offset_Y = 0, transf_offset_Theta = 0;
  object_location_projector_->SetSensorInBody(
      transf_offset_X, transf_offset_Y, transf_offset_Theta * M_PI / 180.0);

  // fusion_ 从此只在这个线程中使用，图像推理在 Run 线程中执行。
  projection_thread_ = std::thread(&Ros1Convert::ProjectionLoop, this);
  return true;
}

void Ros1Convert::Stop() {
  if (stopping_.exchange(true)) return;

  image_sub_.shutdown();
  cloud_sub_.shutdown();

  projection_cv_.notify_all();
  if (projection_thread_.joinable()) projection_thread_.join();

  if (image_locator_) image_locator_->Stop();

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_image_.release();
    latest_cloud_.reset();
    has_image_ = false;
    has_cloud_ = false;
  }

  {
    std::lock_guard<std::mutex> lock(projection_mutex_);
    projection_task_.reset();
    projection_result_.reset();
    projection_result_ready_ = false;
  }
}

void Ros1Convert::Run() {
  if (!iparam_) return;

  ros::Rate loop_rate(iparam_->rate);
  while (ros::ok() && !stopping_) {
    FrameData frame;
    bool ready = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      // 不用消息时间戳：每个新图像只配当前缓存中的最新点云。
      if (has_image_ && has_cloud_ && image_seq_ != consumed_image_seq_ &&
          cloud_seq_ != consumed_cloud_seq_) {
        frame.frame_id        = ++frame_seq_;
        frame.image           = latest_image_;
        frame.cloud           = latest_cloud_;
        frame.global_location = global_location_;
        frame.local_location  = local_location_;
        frame.has_global_pose = has_global_pose_;
        frame.has_local_pose  = has_local_pose_;

        consumed_image_seq_ = image_seq_;
        consumed_cloud_seq_ = cloud_seq_;
        ready               = true;
      }
    }

    if (ready) ProcessFrame(std::move(frame));
    loop_rate.sleep();
  }
}

void Ros1Convert::Process(cv::Mat image, const CloudT::Ptr& cloud) {
  FrameData frame;
  frame.frame_id = ++frame_seq_;
  frame.image    = std::move(image);
  frame.cloud    = cloud;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    frame.global_location = global_location_;
    frame.local_location  = local_location_;
    frame.has_global_pose = has_global_pose_;
    frame.has_local_pose  = has_local_pose_;
  }

  ProcessFrame(std::move(frame));
}

void Ros1Convert::SubmitProjection(ProjectionTask task) {
  std::lock_guard<std::mutex> lock(projection_mutex_);
  projection_task_ = std::move(task);
  projection_result_.reset();
  projection_result_ready_ = false;
  projection_cv_.notify_one();
}

bool Ros1Convert::WaitProjection(std::uint64_t frame_id,
                                 ProjectionResult* result) {
  if (!result) return false;

  std::unique_lock<std::mutex> lock(projection_mutex_);
  projection_cv_.wait(lock, [this, frame_id] {
    return stopping_ || (projection_result_ready_ && projection_result_ &&
                         projection_result_->frame_id == frame_id);
  });

  if (stopping_ || !projection_result_ ||
      projection_result_->frame_id != frame_id) {
    return false;
  }

  *result = std::move(*projection_result_);
  projection_result_.reset();
  projection_result_ready_ = false;
  return true;
}

void Ros1Convert::ProjectionLoop() {
  while (!stopping_) {
    ProjectionTask task;

    {
      std::unique_lock<std::mutex> lock(projection_mutex_);
      projection_cv_.wait(
          lock, [this] { return stopping_ || projection_task_.has_value(); });

      if (stopping_) break;
      task = std::move(*projection_task_);
      projection_task_.reset();
    }

    ProjectionResult result;
    result.frame_id = task.frame_id;

    if (task.image.empty() || !task.cloud || task.cloud->empty()) {
      ROS_WARN_THROTTLE(1.0, "Invalid projection task.");
    } else if (fusion_->SetLidarPointCloud(task.cloud) &&
               fusion_->SetCameraImage(task.image) &&
               fusion_->fuse(2, true, false)) {
      cv::Mat mask;
      if (fusion_->GetFusedImage(mask) && !mask.empty()) {
        // fusion 内部的 mask 会在下一次 fuse 时被覆盖，不能直接共享其内存。
        result.mask    = mask.clone();
        result.success = true;
      }
    }

    {
      std::lock_guard<std::mutex> lock(projection_mutex_);
      projection_result_       = std::move(result);
      projection_result_ready_ = true;
    }
    projection_cv_.notify_all();
  }
}

void Ros1Convert::ProcessFrame(FrameData frame) {
  if (stopping_ || frame.image.empty() || !frame.cloud ||
      frame.cloud->empty() || !image_locator_ || !image_locator_->isInited()) {
    return;
  }

  cv::Mat image = std::move(frame.image);

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

  ProjectionTask task;
  task.frame_id = frame.frame_id;
  task.image    = image;
  task.cloud    = frame.cloud;
  SubmitProjection(std::move(task));

  // 与点云投影线程并行执行图像推理。
  cle::LocationEstimateResult result;
  const bool detect_success = image_locator_->Detect(image, &result);
  if (!detect_success) {
    ROS_WARN_THROTTLE(1.0, "Camera detection failed: %s", result.error.c_str());
  }

  // 即使 Detect 失败也要回收本帧投影结果，避免下一帧提交任务时，覆盖仍在处理中的任务。
  ProjectionResult projection;
  if (!WaitProjection(frame.frame_id, &projection)) return;
  if (!detect_success || !projection.success) {
    ROS_WARN_THROTTLE(1.0, "Point cloud projection failed.");
    return;
  }

  if (!image_locator_->Locate(projection.mask, &result)) {
    ROS_WARN_THROTTLE(1.0, "Camera location failed: %s", result.error.c_str());
    return;
  }

  cv::Mat visualization = image.clone();
  image_locator_->Visualize(visualization, result);
  cv::namedWindow("image_loc", cv::WINDOW_GUI_NORMAL);
  cv::imshow("image_loc", visualization);
  cv::resizeWindow("image_loc", 1024, 768);
  cv::waitKey(1);

  // 使用组成这一帧时保存的位姿快照，不重新读取回调中的最新位姿。
  const double dr_x     = frame.local_location.dr_x;
  const double dr_y     = frame.local_location.dr_y;
  const double dr_theta = frame.local_location.dr_heading * M_PI / 180.0;
  object_location_projector_->SetBodyInOdom(dr_x, dr_y, dr_theta);

  for (const auto& object : result.objects) {
    if (!object.located) continue;

    const auto& supplement = object.obj.camera_supplement;
    double localpose[2]    = {0.0, 0.0};
    object_location_projector_->SensorPose2Odom(supplement.local_center.x(),
                                                supplement.local_center.y(),
                                                localpose[0], localpose[1]);

    // TODO: 发布 global_result。
  }
}
