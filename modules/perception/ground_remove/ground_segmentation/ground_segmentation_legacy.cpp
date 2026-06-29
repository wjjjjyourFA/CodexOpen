#include "modules/perception/ground_remove/ground_segmentation/ground_segmentation_legacy.h"

namespace jojo {
namespace perception {

GroundSegmentation::GroundSegmentation() {}

GroundSegmentation::~GroundSegmentation() {}

void GroundSegmentation::Init(
    std::shared_ptr<jojo::perception::RuntimeConfig> rparam) {
  rparam_ = rparam;

  // 如果默认 ref_height = 0，那么代表 地面高度是 0 左右；
  // 对于非地面坐标系点云来说，需要 ref_height + mean_z_thresh ==> 波动 车轮半高
  hps_.mean_z_thresh     = rparam->ref_height + rparam->mean_z_thresh;
  hps_.delta_z_thresh    = rparam->delta_z_thresh;
  hps_.far_resolution    = rparam->far_resolution;
  hps_.middle_resolution = rparam->middle_resolution;
  hps_.near_resolution   = rparam->near_resolution;
  hps_.Init();
  // std::cout << "hps_.mean_z_thresh: " << hps_.mean_z_thresh << std::endl;
  // std::cout << "hps_.delta_z_thresh: " << hps_.delta_z_thresh << std::endl;

  this->InitGridTable();
}

void GroundSegmentation::InitGridTable() {
  // polar_grid_.reserve(hps_.segment_num * hps_.bin_num);
  polar_grid_.resize(hps_.segment_num * hps_.bin_num);
  active_grids_.reserve(hps_.segment_num * hps_.bin_num);
}

// !! 此代码要求 输入的 frame 是车体坐标系，严格意义是对应的车辆中心（非地面0高）
void GroundSegmentation::Run(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame) {
  // frame 单位 m
  constexpr float PI2 = 2.f * M_PI;

  // std::cout << "ground remove legacy" << std::endl;
  // std::cout << "frame size: " << frame->points.size() << std::endl;

  // TODO：这是一个单帧系统？
  // 初始化 构建极坐标栅格地图
  /* way 1 全量清除
  // polar_grid_.clear();
  for (auto& grid : polar_grid_) {
    grid.Reset();

  }
  */
  // /* way 2 只清空上一帧已经使用的栅格，减少初始化时间
  for (auto* grid : active_grids_) {
    grid->Reset();
  }
  // */
  active_grids_.clear();

  // 1. 栅格化
  for (size_t i = 0; i < frame->points.size(); i++) {
    const auto& pt = frame->points[i];

    const float x = pt.x;
    const float y = pt.y;
    const float z = pt.z;

    // hanging 简单处理悬挂物
    if (z > hps_.hanging_z) continue;

    // 粗筛
    float r2 = x * x + y * y;
    if (r2 < 1e-6f) continue;
    if (r2 >= hps_.far_distance_2) continue;

    // 角度扇区
    float angle = std::atan2(y, x);
    if (angle < 0.f) angle += PI2;
    // 角度离散化（Angular Quantization）
    int segment_idx = std::floor(angle * hps_.segment_num / PI2);
    // std::cout << "segment_idx: " << segment_idx << std::endl;

    // 距离分区
    int bin_idx = 0;
    // 获得距离idx
    float r1 = std::sqrt(r2);
    // 分段非均匀 bin
    if (r2 >= hps_.middle_distance_2) {
      bin_idx = std::floor((r1 - hps_.middle_distance) / hps_.far_resolution) +
                hps_.near_bin_num + hps_.middle_bin_num;
    } else if (r2 >= hps_.near_distance_2) {
      bin_idx = std::floor((r1 - hps_.near_distance) / hps_.middle_resolution) +
                hps_.near_bin_num;
    } else {
      bin_idx = std::floor(r1 / hps_.near_resolution);
    }
    // std::cout << "bin_idx: " << bin_idx << std::endl;

    if (bin_idx < 0 || bin_idx >= hps_.bin_num) continue;
    segment_idx = std::min(segment_idx, hps_.segment_num - 1);

    // 一维表
    int grid_idx = segment_idx * hps_.bin_num + bin_idx;

    auto& grid = polar_grid_[grid_idx];
    // 记录已使用的栅格
    if (!grid.active) {
      active_grids_.push_back(&grid);
      grid.active = true;
    }
    grid.AddPoint(i, z, r1);

    /* debug
    if (polar_grid_[grid_idx].point_num > 3){
      std::cout << "grid_idx: " << grid_idx << " point_num: " 
                << polar_grid_[grid_idx].point_num << std::endl;
    }
    */
  }

  // 2. classify ground
  for (auto& grid : polar_grid_) {
    if (grid.z_points.size() < 3) {
      // grid 中的点数太少，无法计算统计量
      grid.active = false;
      continue;
    }
    // std::cout << "grid.point_num: " << grid.point_num << std::endl;

    grid.ComputeStatistic();

    // /* 悬空判别条件
    if (grid.mean_z > hps_.hanging_z) {
      // TODO：是否标记为悬挂物
      grid.is_ground = false;
      continue;
    }
    // */

    // 地面判别条件 mean_z_thresh ==> 车轮半高
    if ((grid.mean_z < hps_.mean_z_thresh) &&
        (grid.delta_z < hps_.delta_z_thresh)
        /*&& (grid.std_z < hps_.std_z_thresh)*/) {
      grid.is_ground = true;
      // std::cout << "pt is ground " << std::endl;
      // TODO：通过索引，获取 一堆一堆的 地面点
    } else {
      grid.is_ground = false;
      // TODO：是否添加到障碍物点云中
    }
  }
}

const std::vector<GridElement>& GroundSegmentation::GetPolarGridMap() {
  return polar_grid_;
}

}  // namespace perception
}  // namespace jojo
