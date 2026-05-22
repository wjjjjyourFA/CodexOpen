#ifndef CRESIDUAL_GRID_MAP_H
#define CRESIDUAL_GRID_MAP_H

#include "modules/perception/similarity_map/CGridMap/CGridMap.h"

// !! 只支持 平移 residual
/*
连续世界坐标
        ↓
吸附到 grid
        ↓
记录剩余误差(residual)
*/
template <typename CellType>
class CResidualGridMap : public CGridMap<CellType> {
 public:
  CResidualGridMap(int32_t rows = 512, int32_t cols = 512,
                   double resolution = 0.2);
  virtual ~CResidualGridMap() = default;

  CResidualGridMap(const CResidualGridMap&)            = delete;
  CResidualGridMap& operator=(const CResidualGridMap&) = delete;

  // 重置 以 map_center 拓展的 mat patch，并设置默认值
  void ResetMap(double residual_x = 0.0, double residual_y = 0.0);

  void SetResidual(double residual_x, double residual_y);
  void SetResidualByPose(double pose_x, double pose_y);

  virtual CellType* GetValueFromXY(float local_x, float local_y);
  virtual CellType* GetValueFromXY(float local_x, float local_y, int32_t& r,
                                   int32_t& c);

  virtual CellType* GetValueFromRC(int32_t r, int32_t c);
  virtual CellType* GetValueFromRC(int32_t r, int32_t c, float& local_x,
                                   float& local_y);

 protected:
  double resolution_;  // map resolution_ in meters
  // 地图滚动残差 default is zero
  double residual_x_ = 0.0;
  double residual_y_ = 0.0;
};

template <typename CellType>
CResidualGridMap<CellType>::CResidualGridMap(int32_t rows, int32_t cols,
                                             double resolution)
    : CGridMap<CellType>(rows, cols), resolution_(resolution) {
  assert(resolution > 0.0);
}

template <typename CellType>
void CResidualGridMap<CellType>::ResetMap(double residual_x,
                                          double residual_y) {
  this->Clear();
  residual_x_ = residual_x;
  residual_y_ = residual_y;
}

template <typename CellType>
void CResidualGridMap<CellType>::SetResidual(double residual_x,
                                             double residual_y) {
  residual_x_ = residual_x;
  residual_y_ = residual_y;
}

template <typename CellType>
void CResidualGridMap<CellType>::SetResidualByPose(double pose_x,
                                                   double pose_y) {
  // 建立：map grid 坐标系 和 连续世界坐标系 之间的偏移关系。
  // t_pose_mapgrid
  // residual = grid_origin - pose
  // 表示 grid 坐标系原点相对 pose 的偏移：pose 在 grid 左边多少，上边多少
  residual_x_ = std::floor(pose_x / resolution_) * resolution_ - pose_x;
  residual_y_ = std::floor(pose_y / resolution_) * resolution_ - pose_y;
}

template <typename CellType>
CellType* CResidualGridMap<CellType>::GetValueFromXY(float local_x,
                                                     float local_y) {
  // (0,0,0) 处的 cur_frame 坐标系
  int32_t r, c;
  return GetValueFromXY(local_x, local_y, r, c);
}

// GridMap index 采用 image-style 坐标系（top-left origin）
// world mapping 以 grid center 为参考点进行 offset projection
template <typename CellType>
CellType* CResidualGridMap<CellType>::GetValueFromXY(float local_x,
                                                     float local_y, int32_t& r,
                                                     int32_t& c) {
  // 把连续世界坐标，对齐到当前 grid map 的真实原点，并返回对应的栅格
  // ==> 把 local_x = 0 放到 grid center
  // 这里实际变换的是 local 系，不影响 grid 索引（0,rows 0,cols）
  // local 的 Y 轴，被翻转成图像坐标系
  // c = this->half_cols_ + std::floor((local_x - residual_x_) / resolution_);
  // r = this->half_rows_ - 1 - std::floor((local_y - residual_y_) / resolution_);
  c = this->half_cols_ + std::floor(local_x / resolution_);
  r = this->half_rows_ - 1 - std::floor(local_y / resolution_);
  // std::cout << "c: " << c << " r: " << r << std::endl;

  return this->AtRC(r, c);
}

template <typename CellType>
CellType* CResidualGridMap<CellType>::GetValueFromRC(int32_t r, int32_t c) {
  float local_x, local_y;
  return GetValueFromRC(r, c, local_x, local_y);
}

template <typename CellType>
CellType* CResidualGridMap<CellType>::GetValueFromRC(int32_t r, int32_t c,
                                                     float& local_x,
                                                     float& local_y) {
  CellType* ptr = this->AtRC(r, c);
  if (!ptr) {
    local_x = local_y = 0;
    return nullptr;
  }

  // !! 亚像素精度是不是在这里才正在发挥作用，用来计算物理坐标
  local_x = (c - this->half_cols_ + 0.5) * resolution_ + residual_x_;
  local_y = (this->half_rows_ - 1 - r + 0.5) * resolution_ + residual_y_;

  return ptr;
}

#endif
