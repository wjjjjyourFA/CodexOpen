#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "modules/perception/ground_remove/ground_segmentation/GridElement.h"
#include "modules/perception/ground_remove/config/runtime_config.h"

namespace jojo {
namespace perception {

struct GroundSegmentationHyperparams {
  // 单位 m
  float hanging_z;
  float mean_z_thresh;
  float delta_z_thresh;
  // float std_z_thresh;

  // 非标栅格图
  // 180, 360, 720
  // 要想过滤的比较干净，360 是比较好的选择
  int segment_num = 360;
  int bin_num;  // 75 + 50 + 40 = 165
  int near_bin_num   = 75;
  int middle_bin_num = 50;
  int far_bin_num    = 40;

  float far_resolution    = 0.6;
  float middle_resolution = 0.4;
  float near_resolution   = 0.3;

  void Init() {
    bin_num         = near_bin_num + middle_bin_num + far_bin_num;
    near_distance   = near_bin_num * near_resolution;
    middle_distance = near_distance + middle_bin_num * middle_resolution;
    far_distance    = middle_distance + far_bin_num * far_resolution;

    far_distance_2    = far_distance * far_distance;
    middle_distance_2 = middle_distance * middle_distance;
    near_distance_2   = near_distance * near_distance;
  }

  float far_distance;
  float middle_distance;
  float near_distance;

  float far_distance_2;
  float middle_distance_2;
  float near_distance_2;
};

class GroundSegmentation {
 public:
  GroundSegmentation();
  virtual ~GroundSegmentation();

  void Init(std::shared_ptr<jojo::perception::RuntimeConfig> rparam);
  void InitGridTable();

  void Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame);

  const std::vector<GridElement>& GetPolarGridMap();

 protected:
  std::shared_ptr<jojo::perception::RuntimeConfig> rparam_;

  GroundSegmentationHyperparams hps_;

  // 非标准栅格图，用于地面提取
  std::vector<GridElement> polar_grid_;

 private:
  // 存 GridCell 地址，用于增量清除
  std::vector<GridElement*> active_grids_;
};

}  // namespace perception
}  // namespace jojo
