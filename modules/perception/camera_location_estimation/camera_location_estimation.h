#ifndef CAMERA_LOCATION_ESTIMATION_H
#define CAMERA_LOCATION_ESTIMATION_H

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "modules/perception/camera_detection_single_stage/detector/yolo_obstacle_detector.h"
#include "modules/perception/camera_location_estimation/common.h"
#include "modules/perception/camera_tracking/camera_tracking.h"
#include "modules/perception/common/algorithm/image_processing/util/utils.h"
#include "modules/perception/common/algorithm/point_cloud_processing/cluster_postprocess.h"
#include "modules/perception/common/base/box3d_extra.h"
#include "modules/perception/common/base/segment.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "modules/perception/common/lidar/cluster/object_cluster.h"
#include "modules/perception/tools/opencv/cv_colors.h"

namespace jojo {
namespace perception {
namespace cle {

/*
出于融合定位的考虑，所有用于测距的点云，应当转换到统一的主雷达坐标系；
基于该主雷达坐标系，输出定位结果；
*/
class CameraLocationEstimation {
 public:
  // clang-format off
  explicit CameraLocationEstimation(InferenceMode mode = InferenceMode::kDetection);
  explicit CameraLocationEstimation(unsigned int mode);
  virtual ~CameraLocationEstimation();
  // clang-format on

  // clang-format off
  bool Initialize(const std::string& engine_file,
      const ImageLocationHyperparams& config = ImageLocationHyperparams(),
      std::string* error = nullptr/**/);
  // clang-format on
  void Init(const std::string& engine_file);  // 兼容旧调用
  bool isInited() const { return initialized_; }

  void Start();
  void Stop();

  void Run();

  // 新接口：推理/定位与 GUI 解耦，并返回可供下游使用的结构化结果。
  bool Estimate(cv::Mat& image, const cv::Mat& projection_mask,
                LocationEstimateResult* result);
  void Visualize(cv::Mat& image, const LocationEstimateResult& result);

  // 兼容旧接口。
  void DetectionAndLocation(cv::Mat& image, cv::Mat& projection_mask,
                            cv::Mat& show_image, bool show = false);
  void TrackingAndLocation(cv::Mat& image, cv::Mat& projection_mask,
                           cv::Mat& show_image, bool show = false);

  bool SetProjectionMatrix(const Eigen::Matrix4f& projection_matrix);

 protected:
  bool InitEngine(const std::string& engine_file);
  bool InitCluster(const cv::Size& mask_size);

  bool BoxTypeNeed(const base::ObjectType& type) const;

  void FrameObjectGetObjRec(std::vector<base::Object>& detections,
                            std::vector<FrameObject>* frame_boxes,
                            const cv::Size& image_size);

  std::size_t GetCloudAndCluster(std::vector<FrameObject>* frame_boxes,
                                 const cv::Mat& mask);

  void Cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
               std::shared_ptr<base::Segment>* result);

 private:
  std::shared_ptr<jojo::perception::cdss::YoloObstacleDetector> image_detector;
  std::shared_ptr<jojo::perception::ct::CameraTracking> image_tracking;
  std::shared_ptr<jojo::perception::lidar::ObjectCluster> object_cluster;

  ImageLocationHyperparams hps_;
  InferenceMode mode_ = InferenceMode::kDetection;
  std::atomic_bool initialized_{false};
  std::atomic_bool isRunning_{false};

  // 缓冲变量
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud;
  Eigen::Matrix<float, 3, 4> projection_matrix_;
  bool projection_ready_ = false;
  cv::Size cluster_mask_size_;

  void DrawLocateCube(cv::Mat& frame,
                      const std::vector<FrameObject>& frame_boxes);
};

}  // namespace cle
}  // namespace perception
}  // namespace jojo

#endif
