// The following code is adapted from the Stanford Self Driving Car Code.
#ifndef CLOCAL_WINDOW_MAP_H
#define CLOCAL_WINDOW_MAP_H

#include "modules/perception/similarity_map/CGridMap/CRollingGridMap.h"

// !! 添加 rolling sliding window
/* Ring-Buffer Rolling Grid Map
  Layer:
    CGridMap
        ↓
    CResidualGridMap
        ↓
    CRollingGridMap
        ↓
    CLocalWindowMap

  This class implements:
    world(x,y)
        ↓
    global logical grid(rc)
        ↓
    local logical window(rc)
        ↓
    physical ring-buffer(rc)
*/
template <typename CellType>
class CLocalWindowMap : public CRollingGridMap<CellType> {
 public:
  CLocalWindowMap(int32_t rows = 512, int32_t cols = 512,
                  double resolution = 0.2);
  virtual ~CLocalWindowMap() = default;

  CLocalWindowMap(const CLocalWindowMap&)            = delete;
  CLocalWindowMap& operator=(const CLocalWindowMap&) = delete;

  // recenter rolling window 让地图窗口跟随新的中心位置
  bool ReCenterByPose(double x, double y);

  inline double center_x() const { return center_x_; }
  inline double center_y() const { return center_y_; }

  CellType* GetValueFromWorldXY(double world_x, double world_y);
  CellType* GetValueFromWorldXY(double world_x, double world_y, int32_t& r,
                                int32_t& c);

  CellType* GetWorldXYFromRC(int32_t r, int32_t c);
  CellType* GetWorldXYFromRC(int32_t r, int32_t c, double& world_x,
                             double& world_y);

 protected:
  // +X -> east, +Y -> north
  // array_rc 修改的是内存映射，map_rc 修改的是逻辑地图窗口
  // ==> 当前窗口正中心在 global grid 中的位置
  int32_t map_anchor_r_ = 0;
  int32_t map_anchor_c_ = 0;

  double center_x_ = 0.0;
  double center_y_ = 0.0;

 private:
  /* Coordinate system
    World:
      +X : east/right
      +Y : north/up
    Grid / Array:
      +col : right
      +row : down
    Therefore:
      east  -> col++
      west  -> col--
      north -> row--
      south -> row++
  */
  void MoveWindowEast();
  void MoveWindowWest();
  void MoveWindowNorth();
  void MoveWindowSouth();
};

template <typename CellType>
CLocalWindowMap<CellType>::CLocalWindowMap(int32_t rows, int32_t cols,
                                           double resolution)
    : CRollingGridMap<CellType>(rows, cols, resolution) {}

template <typename CellType>
bool CLocalWindowMap<CellType>::ReCenterByPose(double pose_x, double pose_y) {
  center_x_ = pose_x;
  center_y_ = pose_y;

  // synchronize metric residual
  // !! 这里只是设置残差
  this->SetResidualByPose(center_x_, center_y_);

  // 1. 当前 pose 点对应的世界 grid 坐标
  // clang-format off
  // !! 这里是数学坐标上进行操作，并没有和实际物理坐标对应 ==> 并不知道向上移动是 rows++ 还是 rows--
  // 通过 MoveWindowNorth() 反推，这里要将 pose_y 转换为 world_grid_r 取反
  // int32_t world_grid_c = std::floor((pose_x - this->residual_x_) / this->resolution_);
  // int32_t world_grid_r = -std::floor((pose_y - this->residual_y_) / this->resolution_);
  int32_t world_grid_c = std::floor(pose_x / this->resolution_);
  int32_t world_grid_r = -std::floor(pose_y / this->resolution_);
  // std::cout << "world_grid_c: " << world_grid_c << " world_grid_r: " << world_grid_r << std::endl;
  // clang-format on

  // 3. 检测窗口是否需要移动
  int32_t dc = world_grid_c - map_anchor_c_;
  int32_t dr = world_grid_r - map_anchor_r_;

  // no movement
  if (dr == 0 && dc == 0) return false;

  // too far -> full reset
  if (std::abs(dr) >= this->rows_ || std::abs(dc) >= this->cols_) {
    this->ResetMap();

    this->array_c0_ = 0;
    this->array_r0_ = 0;

    map_anchor_c_ = world_grid_c;
    map_anchor_r_ = world_grid_r;
  } 
  // 这里进行窗口的滚动，才是真正在物理坐标上移动，即数学坐标和物理坐标对应
  else {
    // north/south rolling
    if (dr < 0) {
      for (int32_t i = 0; i < -dr; i++) MoveWindowNorth();
    } else if (dr > 0) {
      for (int32_t i = 0; i < dr; i++) MoveWindowSouth();
    }
    // east/west rolling
    if (dc > 0) {
      for (int32_t i = 0; i < dc; i++) MoveWindowEast();
    } else if (dc < 0) {
      for (int32_t i = 0; i < -dc; i++) MoveWindowWest();
    }
  }

  // std::cout << " map_anchor_c_: " << map_anchor_c_ << "map_anchor_r_: " << map_anchor_r_ << std::endl;
  return true;
}

template <typename CellType>
void CLocalWindowMap<CellType>::MoveWindowEast() {
  map_anchor_c_++;
  this->ShiftPhysicalColumnPositive();
}

template <typename CellType>
void CLocalWindowMap<CellType>::MoveWindowWest() {
  map_anchor_c_--;
  this->ShiftPhysicalColumnNegative();
}

template <typename CellType>
void CLocalWindowMap<CellType>::MoveWindowNorth() {
  map_anchor_r_--;
  this->ShiftPhysicalRowNegative();
}

template <typename CellType>
void CLocalWindowMap<CellType>::MoveWindowSouth() {
  map_anchor_r_++;
  this->ShiftPhysicalRowPositive();
}

template <typename CellType>
CellType* CLocalWindowMap<CellType>::GetValueFromWorldXY(double world_x,
                                                         double world_y) {
  int32_t r, c;
  return GetValueFromWorldXY(world_x, world_y, r, c);
}

template <typename CellType>
CellType* CLocalWindowMap<CellType>::GetValueFromWorldXY(double world_x,
                                                         double world_y,
                                                         int32_t& r,
                                                         int32_t& c) {
  // world ==> global logical
  // clang-format off
  // int32_t global_c = this->half_cols_ + std::floor((world_x - this->residual_x_) / this->resolution_);
  // int32_t global_r = this->half_rows_ - 1 - std::floor((world_y - this->residual_y_) / this->resolution_);
  int32_t global_c = this->half_cols_ + std::floor(world_x / this->resolution_);
  int32_t global_r = this->half_rows_ - 1 - std::floor(world_y / this->resolution_);
  // clang-format on
  // global logical ==> local logical: [0, rows_), [0, cols_)
  r = global_r - map_anchor_r_;
  c = global_c - map_anchor_c_;
  // std::cout << "c: " << c << " r: " << r << std::endl;

  // local logical ==> physical ring-buffer
  return this->AtRC(r, c);
}

template <typename CellType>
CellType* CLocalWindowMap<CellType>::GetWorldXYFromRC(int32_t r, int32_t c) {
  double world_x, world_y;
  return GetWorldXYFromRC(r, c, world_x, world_y);
}

template <typename CellType>
CellType* CLocalWindowMap<CellType>::GetWorldXYFromRC(int32_t r, int32_t c,
                                                      double& world_x,
                                                      double& world_y) {
  CellType* ptr = this->AtRC(r, c);
  if (!ptr) {
    world_x = world_y = 0.0;
    return nullptr;
  }

  // local logical ==> global logical
  int32_t global_r = r + map_anchor_r_;
  int32_t global_c = c + map_anchor_c_;

  // global logical ==> world(cell center)
  // clang-format off
  world_x = (global_c - this->half_cols_ + 0.5) * this->resolution_ + this->residual_x_;
  world_y = (this->half_rows_ - 1 - global_r + 0.5) * this->resolution_ + this->residual_y_;
  // clang-format on

  return ptr;
}

#endif
