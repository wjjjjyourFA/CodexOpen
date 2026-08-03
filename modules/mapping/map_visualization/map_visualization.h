#pragma once

#include <omp.h>

#include <algorithm>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <pcl/io/pcd_io.h>

#include "modules/mapping/map_representation/common.h"
#include "modules/mapping/map_representation/load_map_2d.h"
#include "modules/mapping/map_representation/terrain_map.h"
#include "modules/mapping/map_visualization/config/runtime_config.h"
#include "modules/mapping/map_visualization/config/static_config.h"
#include "modules/mapping/map_visualization/post_process.h"

using PointRGBA = pcl::PointXYZRGBA;

struct MapVisualizationHyperparams {
  // 地图分辨率（m）
  float map_resolution = 0.2;

  // 前后扫描范围（m）
  double half_length = 1.0;
  // 左右搜索最大距离（防止无限延伸）
  double max_search_dist = 30.0;

  // frame point cloud range
  double search_radius = 100.0;
};

class MapVisualization {
 public:
  MapVisualization();
  virtual ~MapVisualization();

  void Init(std::shared_ptr<jojo::mapping::RuntimeConfig> rparam,
            std::shared_ptr<jojo::mapping::StaticConfig> sparam);

  void SetDataFolder();

  void InitViewer();

  void LoadRawMap(const std::string& map_path);
  void SetRawMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map);

  void SetPoseCenter(const Eigen::Vector3d& p_center);

  void LoadMapLabel();
  void FillUnknownGroundByHeight();

  void GenerateSequencePassableArea(const Eigen::Matrix4f& in_pose);

  void FillHoleMap();
  void SaveLabelTerrainMat();

  void Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_frame,
           const Eigen::Matrix4f& in_pose);

  bool GetPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  void VisLabelCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_raw,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_label);

 protected:
  std::shared_ptr<jojo::mapping::RuntimeConfig> rparam_;
  std::shared_ptr<jojo::mapping::StaticConfig> sparam_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ = nullptr;
  Eigen::Vector3d map_center;

  Eigen::Vector3d pose_center = Eigen::Vector3d::Zero();

  cv::Mat label_terrain_mat;  // CV_32SC1

  std::string prefix;
  std::string postfix;
  std::string xml_path, gray_path, color_path;

  pcl::PointCloud<pcl::PointXYZI>::Ptr frame = nullptr;

  pcl::visualization::PCLVisualizer::Ptr vis_ = NULL;
  bool vis_inited_                            = false;

 private:
  MapVisualizationHyperparams hps_;
  float p_line_width;

  std::string terrain_map_dir;
  // 底图，存储的栅格点云
  std::shared_ptr<TerrainMap> terrain_map;

 public:
  void DebugShow();
};
