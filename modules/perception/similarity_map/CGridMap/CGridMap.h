#ifndef CGRID_MAP_H
#define CGRID_MAP_H

#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdint>

// 定义了一种栅格图基类，用于存储地图信息，for Axis-Aligned Rolling Grid Map
// NOT equal to TerrainMap : modules/mapping/map_representation/terrain_map.h
// 行优先存储：一整行连续，然后下一行
template <typename CellType>
class CGridMap {
 public:
  CGridMap(int32_t rows = 512, int32_t cols = 512);
  virtual ~CGridMap();

  CGridMap(const CGridMap&)            = delete;
  CGridMap& operator=(const CGridMap&) = delete;

  virtual CellType* AtRC(int32_t r, int32_t c);
  virtual const CellType* AtRC(int32_t r, int32_t c) const;

  void Fill(const CellType& value);
  void Clear();

 protected:
  int32_t rows_, cols_;  // size of grid
  int32_t half_rows_, half_cols_;

  CellType* cell_;  // actual map data
  // TODO：裸指针 替换 vector
  // std::vector<CellType> cell_;

  CellType default_value_{};  // default value for new cells
};

template <typename CellType>
CGridMap<CellType>::CGridMap(int32_t rows, int32_t cols) {
  rows_ = rows;
  cols_ = cols;
  // 该结构只能用于 偶数 大小的 grid map
  assert(rows_ % 2 == 0);
  assert(cols_ % 2 == 0);

  half_rows_ = rows_ / 2;
  half_cols_ = cols_ / 2;

  // 内部数组，用于存储栅格图中的每个栅格的值
  cell_ = new CellType[rows_ * cols_];

  this->Clear();
}

template <typename CellType>
CGridMap<CellType>::~CGridMap() {
  delete[] cell_;
}

template <typename CellType>
CellType* CGridMap<CellType>::AtRC(int32_t r, int32_t c) {
  if (r < 0 || r >= rows_ || c < 0 || c >= cols_) {
    return nullptr;
  }

  // index = row * width + col ==> 行优先存储（row-major layout）
  return &cell_[r * cols_ + c];
}

template <typename CellType>
const CellType* CGridMap<CellType>::AtRC(int32_t r, int32_t c) const {
  if (r < 0 || r >= rows_ || c < 0 || c >= cols_) {
    return nullptr;
  }

  return &cell_[r * cols_ + c];
}

template <typename CellType>
void CGridMap<CellType>::Fill(const CellType& value) {
  for (int32_t i = 0; i < rows_ * cols_; ++i) {
    // 仅限 trivially copyable 类型，如 int, float, double, char 等
    // memcpy(&cell_[i], &value, sizeof(CellType));
    // TODO：彻底删 memcpy
    cell_[i] = value;
  }
}

template <typename CellType>
void CGridMap<CellType>::Clear() {
  this->Fill(default_value_);
}

#endif
