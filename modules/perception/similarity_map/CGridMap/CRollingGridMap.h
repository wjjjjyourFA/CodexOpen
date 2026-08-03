// The following code is adapted from the Stanford Self Driving Car Code.
#ifndef CROLLING_GRID_MAP_H
#define CROLLING_GRID_MAP_H

#include "modules/perception/similarity_map/CGridMap/CResidualGridMap.h"

// !! 添加 rolling sliding window
/* Ring-Buffer Rolling Grid Map
  Layer:
    CGridMap
        ↓
    CResidualGridMap
        ↓
    CRollingGridMap

  Responsibilities:
  - ResidualGridMap:
      local(x,y) <-> logical(r,c)

  - RollingGridMap:
      logical(r,c) <-> physical(r,c)

  This class implements:
    - rolling window
    - ring-buffer storage reuse
    - O(1) rolling shift
*/
template <typename CellType>
class CRollingGridMap : public CResidualGridMap<CellType> {
 public:
  CRollingGridMap(int32_t rows = 512, int32_t cols = 512,
                  double resolution = 0.2);
  virtual ~CRollingGridMap() = default;

  CRollingGridMap(const CRollingGridMap&)            = delete;
  CRollingGridMap& operator=(const CRollingGridMap&) = delete;

  // logical rc -> physical rc
  CellType* AtRC(int32_t r, int32_t c) override;
  const CellType* AtRC(int32_t r, int32_t c) const override;

  /* rolling logical grid
    dr > 0: shift south
    dr < 0: shift north
    dc > 0: shift east
    dc < 0: shift west
  */
  void Shift(int32_t dr, int32_t dc);

 protected:
  // physical array offset (ring buffer)
  // logical(0,0) mapped to: physical(array_r0_, array_c0_)
  // 当前逻辑地图左上角在真实数组里的位置 ==> 滚动更新到内存数组的哪个地方了
  int32_t array_r0_ = 0;  // 从 up 往 down
  int32_t array_c0_ = 0;  // 从 left 往 right

  // 图像坐标 grid_map: +X --> right, +Y --> down
  // 世界坐标:
  //   +X: east -> right --> c++ , west -> left --> c--,
  //   +Y: north -> up --> r-- , south -> down --> r++
  void ShiftPhysicalColumnPositive();
  void ShiftPhysicalColumnNegative();
  void ShiftPhysicalRowNegative();
  void ShiftPhysicalRowPositive();

 private:
  // 周期边界映射（wrap around）把任意整数 x 映射到：[0, max) 范围内
  // change x to a value lies in [0, max). Equivalent to (x%max) ??
  static int32_t WrapAround(int32_t x, int32_t max) {
    assert(max > 0);

    /* way 1 while 太慢
    if (x >= max) {
      while (x >= max) x -= max;
    } else if (x < 0) {
      while (x < 0) x += max;
    }
    return x;
    */

    // way 2 增加处理负数的情况
    return ((x % max) + max) % max;
  }
};

template <typename CellType>
CRollingGridMap<CellType>::CRollingGridMap(int32_t rows, int32_t cols,
                                           double resolution)
    : CResidualGridMap<CellType>(rows, cols, resolution) {}

template <typename CellType>
CellType* CRollingGridMap<CellType>::AtRC(int32_t r, int32_t c) {
  // std::cout << "CRollingGridMap::AtRC" << std::endl;

  if (r < 0 || r >= this->rows_ || c < 0 || c >= this->cols_) {
    return nullptr;
  }

  // 通过这一步，得到真正的内存位置
  int32_t rr = WrapAround(r + array_r0_, this->rows_);
  int32_t cc = WrapAround(c + array_c0_, this->cols_);

  return &this->cell_[rr * this->cols_ + cc];
}

template <typename CellType>
const CellType* CRollingGridMap<CellType>::AtRC(int32_t r, int32_t c) const {
  if (r < 0 || r >= this->rows_ || c < 0 || c >= this->cols_) {
    return nullptr;
  }

  int32_t rr = WrapAround(r + array_r0_, this->rows_);
  int32_t cc = WrapAround(c + array_c0_, this->cols_);

  return &this->cell_[rr * this->cols_ + cc];
}

template <typename CellType>
void CRollingGridMap<CellType>::Shift(int32_t dr, int32_t dc) {
  // 如果一次更新对应的距离太远，那么整个图应该重新更新，而不是一行一行的滚动更新过去
  if (std::abs(dr) >= this->rows_ || std::abs(dc) >= this->cols_) {
    this->Clear();
    array_r0_ = 0;
    array_c0_ = 0;
    return;
  }

  // rows
  if (dr < 0) {  // logical row-- --> north/up
    for (int32_t i = 0; i < -dr; ++i) {
      // 模板类中 的 this->func() 表明使用 父类函数
      ShiftPhysicalRowPositive();
    }
  } else if (dr > 0) {  // logical row++ -->south/down
    for (int32_t i = 0; i < dr; ++i) {
      ShiftPhysicalRowNegative();
    }
  }

  // cols
  if (dc > 0) {  // logical col++ --> east/right
    for (int32_t i = 0; i < dc; ++i) {
      ShiftPhysicalColumnPositive();
    }
  } else if (dc < 0) {  // logical col-- --> west/left
    for (int32_t i = 0; i < -dc; ++i) {
      ShiftPhysicalColumnNegative();
    }
  }
}

template <typename CellType>
void CRollingGridMap<CellType>::ShiftPhysicalColumnPositive() {
  // recycle left column
  int32_t& recycle_c = array_c0_;

  // 遍历所有行，把这一列的值全部重置为默认值
  for (int32_t r = 0; r < this->rows_; r++) {
    CellType* cell = &this->cell_[r * this->cols_ + recycle_c];
    // memcpy(cell, &default_value_, sizeof(CellType));
    *cell = this->default_value_;
  }

  // move left boundary to right
  // way 1
  // array_c0_ = WrapAround(array_c0_ + 1, this->cols_);
  // way 2
  array_c0_++;
  if (array_c0_ == this->cols_) array_c0_ = 0;
}

template <typename CellType>
void CRollingGridMap<CellType>::ShiftPhysicalColumnNegative() {
  // new left boundary
  // way 1
  // int32_t new_array_c0 = WrapAround(array_c0_ - 1, this->cols_);
  // way 2
  int32_t new_array_c0 = array_c0_ - 1;
  if (new_array_c0 < 0) {
    new_array_c0 = this->cols_ - 1;
  }

  for (int32_t r = 0; r < this->rows_; r++) {
    CellType* cell = &this->cell_[r * this->cols_ + new_array_c0];
    // memcpy(cell, &default_value_, sizeof(CellType));
    *cell = this->default_value_;
  }

  array_c0_ = new_array_c0;
}

template <typename CellType>
void CRollingGridMap<CellType>::ShiftPhysicalRowNegative() {
  int32_t new_array_r0 = array_r0_ - 1;
  if (new_array_r0 < 0) {
    new_array_r0 = this->rows_ - 1;
  }

  for (int32_t c = 0; c < this->cols_; c++) {
    CellType* cell = &this->cell_[new_array_r0 * this->cols_ + c];
    // memcpy(cell, &default_value_, sizeof(CellType));
    *cell = this->default_value_;
  }

  array_r0_ = new_array_r0;
}

template <typename CellType>
void CRollingGridMap<CellType>::ShiftPhysicalRowPositive() {
  for (int32_t c = 0; c < this->cols_; c++) {
    CellType* cell = &this->cell_[array_r0_ * this->cols_ + c];
    // memcpy(cell, &default_value_, sizeof(CellType));
    *cell = this->default_value_;
  }

  array_r0_++;
  if (array_r0_ == this->rows_) array_r0_ = 0;
}

#endif
