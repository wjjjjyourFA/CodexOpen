#pragma once

#include <omp.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "modules/perception/tools/opencv/colors.hpp"
// #include "modules/dreamview/map_center_view/config/runtime_config.h"
#include "modules/dreamview/map_center_view/config/static_config.h"

namespace jojo {
namespace dreamview {

class MapCenterView {
 public:
  MapCenterView();
  virtual ~MapCenterView() {};

  void Init(std::shared_ptr<jojo::dreamview::StaticConfig> sparam);

  void InitViewer();
  void InitKDTree();

  void LoadInitMap(const std::string& map_path);

  void SetInitMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map);
  void SetMapCenter(const Eigen::Vector3d& center);
  void SetPoseCenter(const Eigen::Vector3d& p_center);

  void ShowFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame,
                 const Eigen::Matrix4f& pose);

  void ShowFrameROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame,
                    const Eigen::Matrix4f& pose);

  // void ShowPose(pcl::visualization::PCLVisualizer::Ptr& vis, &pose,
  //               pcl::PointCloud<pcl::PointXYZ>::Ptr map,
  //               const string& window_name);

  // void ShowFrame(pcl::visualization::PCLVisualizer::Ptr& vis,
  //                pcl::PointCloud<pcl::PointXYZ>::Ptr frame,
  //                pcl::PointCloud<pcl::PointXYZ>::Ptr map,
  //                const string& window_name);

  void UpdateMapROI(const Eigen::Matrix4f& pose);
  void UpdateMapROIKdTree(const Eigen::Matrix4f& pose);

 protected:
  pcl::visualization::PCLVisualizer::Ptr vis_ = NULL;
  bool vis_inited_                            = false;

  // 动态中心裁剪
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_roi;
  pcl::CropBox<pcl::PointXYZI> crop_box;
  float roi_radius = 120.0f;

  // KD-tree（只建一次）
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  bool kdtree_built = false;

  bool NeedUpdateROI(const Eigen::Vector3f& center);

  void BridView(const Eigen::Matrix4f& pose);

 private:
  std::shared_ptr<jojo::dreamview::StaticConfig> sparam_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ = nullptr;
  Eigen::Vector3d map_center;
  int dyn_map_radius_ = 100;

  Eigen::Vector3d pose_center = Eigen::Vector3d::Zero();

  // 缓存
  pcl::PointCloud<pcl::PointXYZI>::Ptr frame_world = nullptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud = nullptr;
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>::Ptr
      intensity_handler = nullptr;

  // color
  float inv_dist = 1.0;

  // 滑窗更新
  // 上一次构建 ROI 的中心
  Eigen::Vector3f last_center     = Eigen::Vector3f::Zero();
  Eigen::Vector3f last_cam_center = Eigen::Vector3f::Zero();
  bool has_last_center            = false;
  // 触发更新阈值（10m）
  float roi_update_dist = 30.0f;
};

}  // namespace dreamview
}  // namespace jojo
