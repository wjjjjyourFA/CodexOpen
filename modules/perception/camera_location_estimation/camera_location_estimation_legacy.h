#ifndef CAMERA_LOCATION_ESTIMATION_H
#define CAMERA_LOCATION_ESTIMATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// #include "modules/common/math/math_utils_extra.h"
#include "modules/perception/camera_detection_single_stage/detector/yolo_obstacle_detector.h"
#include "modules/perception/camera_location_estimation/common.h"
// 投影定位线程，可以是该模块的子线程，也可以是独立的线程
#include "modules/perception/common/algorithm/image_processing/util/utils.h"
#include "modules/perception/common/algorithm/point_cloud_processing/cluster_postprocess.h"
#include "modules/perception/common/base/box3d_extra.h"
#include "modules/perception/common/base/segment.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
// #include "modules/perception/common/fusion/radar2camera/radar_camera_fusion.h"
#include "modules/perception/common/lidar/cluster/object_cluster.h"
#include "modules/perception/tools/common/show_data_2d.h"

namespace jojo {
namespace perception {
namespace cle {

// 目标定位的 距离估计的 结果框
// legacy
// struct FrameObject : public jojo::perception::base::Object {

/*
出于融合定位的考虑，所有用于测距的点云，应当转换到 相机坐标系；
输出结果，基于相机坐标系；
*/
class CameraLocationEstimation {
 public:
  CameraLocationEstimation(uint mode = 1);
  virtual ~CameraLocationEstimation();

  void Init(const std::string& engine_file);
  bool isInited() const { return initialized_; };

  void Start();
  void Stop();

  void Run();

  // 需要的并不是投影线程，而是投影点云的MASK。
  void DetectionAndLocation(cv::Mat& cimage, cv::Mat& projection_mask,
                            cv::Mat& show_image, bool show = false);

  void SetProjectionMatrix(const Eigen::Matrix4f& projection_matrix);

 protected:
  bool InitEngine(const std::string& engine_file);
  void InitCluster();

  bool BoxTypeNeed(const jojo::perception::base::ObjectType& type);

  void FrameObjectGetObjRec(std::vector<base::Object>& detections,
                            std::vector<FrameObject>& frame_boxes,
                            cv::Mat& image, bool show = false);

  void GetCloudAndCluster(std::vector<FrameObject>& frame_boxes, cv::Mat& mask,
                          cv::Mat& image);

  void Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr,
               std::shared_ptr<base::Segment>& result);

 private:
  std::shared_ptr<jojo::perception::cdss::YoloObstacleDetector> image_detector;
  std::shared_ptr<jojo::perception::lidar::ObjectCluster> object_cluster;

  ImageLocationHyperparams hps_;
  uint mode = 1;

  std::atomic_bool initialized_{false}, isRunning_{false};

  // 缓冲变量
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;  // from mask
  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud;
  Eigen::Matrix<float, 3, 4> projection_matrix_;
  // /* debug
  pcl::PointCloud<pcl::PointXYZI>::Ptr clustered;
  // */

  std::vector<std::shared_ptr<jojo::perception::base::Segment>> segs;

  void DrawLocateCube(cv::Mat& frame, std::vector<FrameObject>& frame_boxes);
};

}  // namespace cle
}  // namespace perception
}  // namespace jojo

#endif
