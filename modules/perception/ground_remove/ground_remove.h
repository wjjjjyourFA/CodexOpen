#pragma once

#include <vector>
#include <cmath>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

#include "modules/perception/ground_remove/config/runtime_config.h"
#include "modules/perception/tools/opencv/colors.hpp"

#include "modules/perception/similarity_map/CGridMap/CLocalWindowMap.h"
#include "modules/perception/ground_remove/ObstacleCell.h"

#include "modules/perception/ground_remove/ground_segmentation/ground_segmentation_legacy.h"
// #include "modules/perception/ground_remove/ground_segmentation/ground_segmentation.h"

namespace jojo {
namespace perception {

struct GroundRemoveHyperparams {
  // 标准栅格图
  // 单位 m
  float map_resolution = 0.2;

  int32_t map_rows = 512;
  int32_t map_cols = 512;

  int32_t half_rows;
  int32_t half_cols;

  int height_z = 10;
};

/* pipeline
1. frame downsampling
2. frame transform 
3. ground remove
*/
class GroundRemove {
 public:
  GroundRemove();
  virtual ~GroundRemove();

  void SetGravityLidarExtrinsicMatrix(const Eigen::Matrix4f& extrinsic_matrix);

  void Init(std::shared_ptr<jojo::perception::RuntimeConfig> rparam);

  void InitViewer();

  void Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame,
           const Eigen::Matrix4f& in_pose);

  void build_occupancy(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame);

  void ShowObstacleGridMap();
  void ShowColorGridMapRotation(const Eigen::Matrix4f& pose);

  void VisColorCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

 protected:
  std::shared_ptr<jojo::perception::RuntimeConfig> rparam_;

  GroundRemoveHyperparams hps_;

  std::shared_ptr<jojo::perception::GroundSegmentation> ground_segmentation;

  // 标准栅格图，用于障碍物图输出
  std::shared_ptr<CLocalWindowMap<ObstacleCell>> obstacle_grid_map;
  cv::Mat show_mat;

  // 障碍物地图 ==> 障碍物点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud_;

  // 用于将 非水平安装的lidar数据 转换到 水平系
  Eigen::Matrix4d gravity_lidar_ext = Eigen::Matrix4d::Identity();

 private:
  pcl::visualization::PCLVisualizer::Ptr vis_ = nullptr;
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>::Ptr
      intensity_handler = nullptr;

  bool vis_inited_ = false;
};

}  // namespace perception
}  // namespace jojo
