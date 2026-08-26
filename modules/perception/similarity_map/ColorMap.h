/**
 * @file ColorMap.h
 * @author Bokai 
 * @brief 
 * @version 0.1
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __COLOR_POINT_H__
#define __COLOR_POINT_H__

#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "modules/perception/similarity_map/CGridMap/CLocalWindowMap.h"
#include "modules/perception/similarity_map/ColorCell.h"
#include "modules/perception/similarity_map/config/runtime_config.h"
#include "toolz/data_loader/group_convert.h"

namespace jojo {
namespace perception {

struct ColorMapHyperparams {
  // 单位 m
  float map_resolution = 0.2;

  int32_t map_rows = 512;
  int32_t map_cols = 512;

  int32_t half_rows;
  int32_t half_cols;
};

// 实际生成的是 frame 级别的 map；而不是全局累积的 map
class ColorMap {
 public:
  ColorMap();
  virtual ~ColorMap();

  void Init(std::shared_ptr<jojo::perception::RuntimeConfig> rparam);

  void InitViewer();

  void Run(std::shared_ptr<const jojo::tools::MeasureGroupDataSet> Measures);

  void UpdateColorGridMapEgoBev(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                const Eigen::Matrix4f& pose);
  void UpdateColorGridMapGlobalBev(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                   const Eigen::Matrix4f& pose);

  void ShowColorGridMap();

  void ShowColorGridMapYawAligned(const Eigen::Matrix4f& pose);

  void VisColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

 protected:
  std::shared_ptr<jojo::perception::RuntimeConfig> rparam_;

  ColorMapHyperparams hps_;

  std::shared_ptr<CLocalWindowMap<ColorCell>> color_grid_map;
  cv::Mat show_mat;

  pcl::CropBox<pcl::PointXYZRGB> boxFilter;
  void PreProcessEgoBev(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  void PreProcessGlobalBev(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           const Eigen::Matrix4f& pose);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_cloud;

  std::shared_ptr<jojo::perception::fusion::LidarCameraFusion> fusion;

 private:
  pcl::visualization::PCLVisualizer::Ptr vis_ = nullptr;

  bool vis_inited_ = false;
};

}  // namespace perception
}  // namespace jojo

#endif
