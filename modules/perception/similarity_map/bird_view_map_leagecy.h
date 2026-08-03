#ifndef BIRD_VIEW_MAP_H
#define BIRD_VIEW_MAP_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "modules/perception/lidar_local_mapping/local_mapping_base.h"
#include "modules/perception/similarity_map/config/runtime_config.h"
#include "toolz/data_loader/group_convert.h"

using namespace std;

namespace jojo {
namespace perception {

struct BirdViewMapHyperparams : public LocalMappingHyperparams {
  // 单位 m
  float map_resolution = 0.2;

  int32_t map_rows = 512;
  int32_t map_cols = 512;

  int32_t half_rows;
  int32_t half_cols;
};

/* 实际上在做极短范围内，无 distance 过滤的 Lidar Local Mapping
*/
class BirdViewMap : public LocalMappingBase<pcl::PointXYZRGB> {
 public:
  BirdViewMap();
  ~BirdViewMap();

  void Init(std::shared_ptr<jojo::perception::RuntimeConfig> rparam);

  void InitViewer();

  void Run(std::shared_ptr<const jojo::tools::MeasureGroupDataSet> Measures);

  void VisColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

 protected:
  void GenerateBirdView();

 protected:
  std::shared_ptr<jojo::perception::RuntimeConfig> rparam_;

  BirdViewMapHyperparams hps_;

  std::shared_ptr<jojo::perception::fusion::LidarCameraFusion> fusion;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_color;

  cv::Mat global_bev;

 private:
  pcl::visualization::PCLVisualizer::Ptr vis_ = nullptr;

  bool vis_inited_ = false;
};

}  // namespace perception
}  // namespace jojo

#endif  // BIRD_VIEW_MAP_H
