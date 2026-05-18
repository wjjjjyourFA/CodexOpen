#ifndef IMAGE_LOCATION_H
#define IMAGE_LOCATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// #include "modules/common/math/math_utils_extra.h"
#include "modules/perception/camera_detection_single_stage/detector/yolo_obstacle_detector.h"
// 投影定位线程，可以是该模块的子线程，也可以是独立的线程
#include "modules/perception/common/base/box3d_extra.h"
#include "modules/perception/common/base/segment.h"
#include "modules/perception/common/algorithm/image_processing/util/utils.h"
#include "modules/perception/common/algorithm/point_cloud_processing/cluster_postprocess.h"
#include "modules/perception/common/lidar/cluster/object_cluster.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
// #include "modules/perception/common/fusion/radar2camera/radar_camera_fusion.h"
#include "modules/perception/tools/common/show_data_2d.h"

namespace jojo {
namespace perception {
namespace cle {

// 目标定位的 距离估计的 结果框
// legacy
// struct FrameObject : public jojo::perception::base::Object {
struct FrameObject {
  FrameObject() = default;

  // 原始检测（完整保留）
  base::Object obj;
  // 必须共享或很大对象
  // std::shared_ptr<const base::Object> obj = nullptr;

  // 推理框 box2d
  cv::Rect srcRec;
  // 中心修正 / 估计框 box2d <== 推理框
  cv::Rect cenRec;

  // 目标 三维框 box3d <== 估计框 推算得到
  // ==> parent.camera_supplement.box3d;

  // 该 center 中心源自相机使用点云，因此是很不准确的，只能作为参考
  // Eigen::Vector3f local_center = Eigen::Vector3f(0, 0, 0);
  // ==> parent.camera_supplement.local_center
  // float dist = local_center.squaredNorm();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// 中间结构体；用于初始化每一个相机的中间数据；
class ImageLocationData {
 public:
  ImageLocationData() {};
  virtual ~ImageLocationData() = default;

  // 只维护一张 mask
  cv::Mat projection_mask;

  // 多个测距器相关参数  lidar + radar
  // projection_matrix;
};

struct ImageLocationHyperparams {
  // 缩小 30%
  float scale = 0.7f;

  size_t RoiLimit = 50 * 50;

  // cluster
  // width height length
  int imageSize[3] = {1920, 1080, 1};
  float eps        = 0.5;
  int minPts       = 5;

  int pixel_threshold = 15;
};

/*
出于融合定位的考虑，所有用于测距的点云，应当转换到 相机坐标系；
输出结果，基于相机坐标系；
*/
class CameraLocationEstimation {
 public:
  CameraLocationEstimation(uint mode = 1);
  virtual ~CameraLocationEstimation();

  void Init(const std::string& engine_file);
  void InitCluster();
  bool isInited() const { return initialized_; };

  void Start();
  void Stop();

  void Run();

  // 需要的并不是投影线程，而是投影点云的MASK。
  void DetectionAndLocation(cv::Mat& cimage, cv::Mat& projection_mask,
                            cv::Mat& show_image, bool show = false);

  void SetProjectionMatrix(const Eigen::Matrix4f& projection_matrix);

 protected:
  std::shared_ptr<jojo::perception::cdss::YoloObstacleDetector> image_detector;
  bool InitEngine(const std::string& engine_file);

  bool BoxTypeNeed(const jojo::perception::base::ObjectType& type);
  void FrameObjectGetObjRec(
      std::vector<jojo::perception::base::Object>& detections,
      std::vector<FrameObject>& frame_boxes, cv::Mat& image, bool show = false);

  std::shared_ptr<jojo::perception::lidar::ObjectCluster> object_cluster;
  void GetCloudAndCluster(std::vector<FrameObject>& frame_boxes, cv::Mat& mask,
                          cv::Mat& image);
  void Cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr,
               std::shared_ptr<base::Segment>& result);

  std::atomic_bool initialized_{false}, isRunning_{false};

 private:
  ImageLocationHyperparams hps_;
  uint mode = 1;

  // 缓冲变量
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;  // from mask
  pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud;
  // /* debug
  pcl::PointCloud<pcl::PointXYZI>::Ptr clustered;
  // */

  std::vector<std::shared_ptr<jojo::perception::base::Segment>> segs;

  Eigen::Matrix<float, 3, 4> projection_matrix_;
  void DrawLocateCube(cv::Mat& frame, std::vector<FrameObject>& frame_boxes);
};

}  // namespace cle
}  // namespace perception
}  // namespace jojo

#endif
