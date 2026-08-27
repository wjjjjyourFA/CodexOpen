#ifndef CAMERA_LOCATION_ESTIMATION_ROS1_CONVERT_H
#define CAMERA_LOCATION_ESTIMATION_ROS1_CONVERT_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include <ros/ros.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "modules/common/math/math_utils_extra.h"
#include "modules/localization/common/transform/frame2d_transform.hpp"
// #include "modules/localization/common/transform/object_location_projector.h"
#include "modules/perception/camera_location_estimation/camera_location_estimation.h"
#include "modules/perception/camera_location_estimation/config/interface_config.h"
#include "modules/perception/camera_location_estimation/config/runtime_config.h"
#include "modules/perception/common/camera/common/undistortion_handler_cv.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "modules/perception/common/lidar/convert/robosense.h"
#include "modules/perception/common/lidar/convert/velodyne.h"
#include "self_state/GlobalPose.h"
#include "self_state/LocalPose.h"

namespace cfg    = jojo::perception::config;
namespace camera = jojo::perception::camera;
namespace fusion = jojo::perception::fusion;
namespace cle    = jojo::perception::cle;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

typedef self_state::GlobalPose globalpose_msgtype;
typedef self_state::LocalPose localpose_msgtype;

// 当有上游 数据预处理 模块时，这里的 FrameData 可以直接使用上游的 FrameData::Ptr
struct FrameData {
  std::uint64_t frame_id = 0U;
  cv::Mat image;
  CloudT::Ptr cloud;
  globalpose_msgtype global_location;
  localpose_msgtype local_location;
  bool has_global_pose = false;
  bool has_local_pose  = false;
};

struct ProjectionTask {
  std::uint64_t frame_id = 0U;
  cv::Mat image;
  CloudT::Ptr cloud;
};

struct ProjectionResult {
  std::uint64_t frame_id = 0U;
  cv::Mat mask;
  bool success = false;
};

class Ros1Convert {
 public:
  Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~Ros1Convert();

  bool Init(std::shared_ptr<cle::RuntimeConfig> param,
            std::shared_ptr<cle::InterfaceConfig> interface);
  void Run();
  void Stop();

 protected:
  void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void ImageCompressedCallback(const sensor_msgs::CompressedImageConstPtr& msg);

  void PointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void PointCloudCallback(const sensor_msgs::PointCloudConstPtr& msg);

  void GposePreprocessing(const globalpose_msgtype& msg);
  void LposePreprocessing(const localpose_msgtype& msg);

  std::shared_ptr<camera::CameraParams> camera_params_;
  std::shared_ptr<camera::UndistortionHandler> camera_undistort_;
  std::shared_ptr<fusion::LidarCameraFusion> fusion_;

 private:
  // 保留原有 Process 函数名；新流程通过 ProcessFrame 携带同一帧的位姿快照。
  void Process(cv::Mat image, const CloudT::Ptr& cloud);
  void ProcessFrame(FrameData frame);

  void ProjectionLoop();
  void SubmitProjection(ProjectionTask task);
  bool WaitProjection(std::uint64_t frame_id, ProjectionResult* result);

  std::shared_ptr<cle::RuntimeConfig> rparam_;
  std::shared_ptr<cle::InterfaceConfig> iparam_;

  std::unique_ptr<cle::CameraLocationEstimation> image_locator_;
  // clang-format off
  std::shared_ptr<jojo::localization::common::Frame2dTransform> object_location_projector_;
  // std::shared_ptr<jojo::localization::common::GlobalLocationProjector> object_location_projector_;
  // clang-format on

  ros::NodeHandle node_;
  ros::Subscriber image_sub_;
  ros::Subscriber cloud_sub_;

  // 回调只更新最新数据；Run 组成一帧时复制图像并保存位姿快照，不使用消息时间同步。
  std::mutex data_mutex_;
  cv::Mat latest_image_;
  CloudT::Ptr latest_cloud_;
  globalpose_msgtype global_location_;
  localpose_msgtype local_location_;

  std::uint64_t image_seq_          = 0U;
  std::uint64_t cloud_seq_          = 0U;
  std::uint64_t pose_seq_           = 0U;
  std::uint64_t consumed_image_seq_ = 0U;
  std::uint64_t consumed_cloud_seq_ = 0U;
  std::uint64_t frame_seq_          = 0U;

  bool has_image_       = false;
  bool has_cloud_       = false;
  bool has_global_pose_ = false;
  bool has_local_pose_  = false;

  // fusion_ 只允许 ProjectionLoop 使用，避免跨线程复用其内部工作状态。
  std::mutex projection_mutex_;
  std::condition_variable projection_cv_;
  std::optional<ProjectionTask> projection_task_;
  std::optional<ProjectionResult> projection_result_;
  bool projection_result_ready_ = false;
  std::thread projection_thread_;

  bool undistortion_ready_ = true;
  std::atomic_bool stopping_{false};
};

#endif
