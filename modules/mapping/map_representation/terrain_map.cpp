#include "modules/mapping/map_representation/terrain_map.h"

namespace transform = jojo::common::transform;

TerrainMap::TerrainMap() {}

TerrainMap::~TerrainMap() {}

void TerrainMap::Init(const std::string& root,
                      std::shared_ptr<jojo::mapping::StaticConfig> sparam) {
  if (sparam) {
    sparam_ = sparam;
    // m ==> cm
    hps_.map_min_x      = sparam_->map_min_x * 100;
    hps_.map_min_y      = sparam_->map_min_y * 100;
    hps_.map_resolution = sparam_->map_resolution * 100;
  }
  this->LoadInitMap(root);
  std::cout << "TerrainMap Init Done!" << std::endl;
  std::cout << "map_min_x: " << hps_.map_min_x << std::endl;
  std::cout << "map_min_y: " << hps_.map_min_y << std::endl;
  std::cout << "map_resolution: " << hps_.map_resolution << std::endl;
}

void TerrainMap::SetInpaintedFlag(bool flag) { b_use_inpainted = flag; }

void TerrainMap::LoadInitMap(const std::string& root) {
  if (b_use_inpainted) {
    map_path = root + std::string("/inpainted_terrain_height.bin");
  } else {
    map_path = root + std::string("/terrain_height.bin");
  }

  ReadMat(map_path, this->data);
  if (data.empty()) {
    std::cerr << "[LoadLabel] terrain_map is empty!" << std::endl;
    return;
  }

  /* debug show
  cv::Mat vis_resized;
  cv::resize(data, vis_resized, cv::Size(1920, 1080), 0, 0, cv::INTER_NEAREST);
  cv::imshow("TerrainMap", vis_resized);
  cv::waitKey(0);
  */
}

void TerrainMap::SetInitMap(cv::Mat& map) { this->data = map; }

void TerrainMap::QueryMapFromXY(double x_m, double y_m, int radius,
                                float* in_data) {
  int32_t center_row, center_col;
  if (!WorldToGrid(x_m, y_m, center_row, center_col)) {
    return;
  }

  // radius 是 cv::mat 的像素半径
  for (int r = -radius; r <= radius; r++) {
    for (int c = -radius; c <= radius; c++) {
      if (r + center_row >= 0 && r + center_row < data.rows &&
          c + center_col >= 0 && c + center_col < data.cols) {
        in_data[(r + radius) * (2 * radius + 1) + c + radius] =
            data.at<double>(r + center_row, c + center_col);
      } else {
        in_data[(r + radius) * (2 * radius + 1) + c + radius] = INVALID_VALUE;
      }
    }
  }
}

double TerrainMap::GetValueFromXY(double x_m, double y_m) {
  // “世界坐标 → 栅格地图索引 → 查询高度值”
  // 根据输入的 (x, y) 世界坐标，查询对应栅格地图中的地形高度（height / elevation）
  int r, c;
  return GetValueFromXY(x_m, y_m, r, c);
}

double TerrainMap::GetValueFromXY(double x_m, double y_m, int32_t& r,
                                  int32_t& c) {
  if (!WorldToGrid(x_m, y_m, r, c)) {
    return INVALID_VALUE;
  }
  // std::cout << "value: " << data.at<double>(r, c) << std::endl;
  return data.at<double>(r, c);
}

double TerrainMap::GetValueFromRC(int32_t r, int32_t c) {
  double x_m, y_m;
  return GetValueFromRC(r, c, x_m, y_m);
}

double TerrainMap::GetValueFromRC(int32_t r, int32_t c, double& x_m,
                                  double& y_m) {
  // 从栅格地图坐标 (row, col) 反算出世界坐标 (x, y)，并读取该栅格的值

  // 边界检查
  if (!GridToWorld(r, c, x_m, y_m)) {
    x_m = y_m = 0.0;
    return double(INVALID_VALUE);
  }

  return data.at<double>(r, c);
}

bool TerrainMap::WorldToGrid(double x_m, double y_m, int32_t& row,
                             int32_t& col) {
  // 单位换算
  double x_cm = x_m * 100.0;
  double y_cm = y_m * 100.0;

  // int t = std::floor((y_cm - hps_.map_min_y) / hps_.map_resolution);
  // std::cout << " " << t << std::endl;

  // 栅格分离
  col = static_cast<int>(
      std::floor((x_cm - hps_.map_min_x) / hps_.map_resolution));
  // 变换到 cv::mat 坐标系
  row = static_cast<int>(
      data.rows - 1 -
      std::floor((y_cm - hps_.map_min_y) / hps_.map_resolution));
  // std::cout << "row: " << row << " col: " << col << std::endl;

  if (row >= 0 && row < data.rows && col >= 0 && col < data.cols) {
    return true;
  }
  return false;
}

bool TerrainMap::GridToWorld(int32_t row, int32_t col, double& x_m,
                             double& y_m) {
  // rc -> xy（与 Query 完全对称）
  // 地图内部单位是：厘米（cm）==> 输出：米（m）

  if (row < 0 || row >= data.rows || col < 0 || col >= data.cols) {
    x_m = y_m = 0.0;
    return false;
  }

  /*/ way 1 用 cell 左下角
  x_m = (hps_.map_min_x + col * hps_.map_resolution) * 0.01;
  y_m = (hps_.map_min_y + (data.rows - 1 - row) * hps_.map_resolution) * 0.01;
  */
  // way 2 用 cell 中心（推荐）
  x_m = (hps_.map_min_x + (col + 0.5) * hps_.map_resolution) * 0.01;
  y_m = (hps_.map_min_y + (data.rows - 1 - row + 0.5) * hps_.map_resolution) *
        0.01;

  return true;
}

void TerrainMap::GetTerrainMapPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pc) {
  pc->clear();

  const int rows = data.rows;
  const int cols = data.cols;

  // 预分配（最大容量）
  pc->points.reserve(rows * cols);

  // 预计算（避免重复计算）
  // cm → m
  const double res      = hps_.map_resolution * 0.01;
  const double origin_x = hps_.map_min_x * 0.01;
  const double origin_y = hps_.map_min_y * 0.01;
  for (int r = 0; r < rows; ++r) {
    // 预计算这一行的 y（关键优化点）
    double y_m = origin_y + (rows - 1 - r + 0.5) * res;

    const double* row_ptr = data.ptr<double>(r);

    for (int c = 0; c < cols; ++c) {
      double z_m = row_ptr[c];
      if (z_m == INVALID_VALUE) continue;

      // 直接算 x（避免函数调用）
      double x_m = origin_x + (c + 0.5) * res;

      // 如果 z 是 cm，这里要转
      // double z_m = z * 0.01;

      // z_m
      pc->points.emplace_back(static_cast<float>(x_m), static_cast<float>(y_m),
                              static_cast<float>(z_m));
    }
  }

  pc->width    = pc->points.size();
  pc->height   = 1;
  pc->is_dense = false;
}

void TerrainMap::SetPatchUpdateRadius(int radius) {
  patch_radius  = radius;
  use_step_mode = true;
}

void TerrainMap::GetTerrainMapPatch(double* xyzrpy,
                                    std::shared_ptr<TerrainMapPatch> output) {
  // 此方式生成的 patch 是 严格 x，y 对齐的 map 的，即没有依据车头方向旋转
  output->pose.pos.x() = xyzrpy[0];
  output->pose.pos.y() = xyzrpy[1];
  output->pose.pos.z() = xyzrpy[2];

  double roll  = xyzrpy[3];
  double pitch = xyzrpy[4];
  double yaw   = xyzrpy[5];

  // clang-format off
  Eigen::Matrix3d R = transform::YPR2RotationZYX(Eigen::Matrix<double, 3, 1>(yaw, pitch, roll));
  transform::RotationToQuaternion(R, output->pose.rot);
  // clang-format on

  // 配置 栅格图 patch header 数据；这里应该是在外部配置好，此处只负责读取；
  // assert(output->resolution == 0.2);  // 单位是 m

  const int patch_rows = output->rows;
  const int patch_cols = output->cols;

  const double res_m       = hps_.map_resolution * 0.01;  // cm → m
  const double map_min_x_m = hps_.map_min_x * 0.01;
  const double map_min_y_m = hps_.map_min_y * 0.01;

  // 以当前位姿为中心，计算 patch 覆盖范围
  double center_x = xyzrpy[0];
  double center_y = xyzrpy[1];

  // 3. 对齐到 grid map ==> terrain_map
  const double center_cell_x = std::floor(center_x / res_m) * res_m;
  const double center_cell_y = std::floor(center_y / res_m) * res_m;

  // patch 覆盖范围（固定），每个 frame 取到图像大小是一致的，但是更新范围可以不一样
  const double half_width  = patch_cols * res_m * 0.5;
  const double half_height = patch_rows * res_m * 0.5;

  const double patch_min_x = center_cell_x - half_width;
  const double patch_min_y = center_cell_y - half_height;

  // 4. 转 global index
  int base_c = static_cast<int>((patch_min_x - map_min_x_m) / res_m);
  int base_r =
      data.rows - 1 - static_cast<int>((patch_min_y - map_min_y_m) / res_m);

  /* 世界坐标（y ↑）：
    ↑ y
    |
    |      (patch)
    |
  ---+----------------
  */
  /* 图像坐标（r ↓）：
  0  ----------------
    |
    |
    |
  rows-1
  */
  base_r -= patch_rows - 1;

  // 要更新的区域范围
  if (use_step_mode) {
    // radius：单位 m
    const int update_half_w = static_cast<int>(patch_radius / res_m);
    const int update_half_h = static_cast<int>(patch_radius / res_m);

    const int center_r = patch_rows / 2;
    const int center_c = patch_cols / 2;

    const int r_min = std::max(0, center_r - update_half_h);
    const int r_max = std::min(patch_rows, center_r + update_half_h + 1);
    const int c_min = std::max(0, center_c - update_half_w);
    const int c_max = std::min(patch_cols, center_c + update_half_w + 1);

    // 更新patch 以中心点指定半径范围
    for (int r = r_min; r < r_max; ++r) {
      int src_r = base_r + r;
      if (src_r < 0 || src_r >= data.rows) continue;

      const double* row_ptr = data.ptr<double>(src_r);

      for (int c = c_min; c < c_max; ++c) {
        int src_c = base_c + c;

        if (src_c >= 0 && src_c < data.cols) {
          output->At(r, c) = static_cast<float>(row_ptr[src_c]);
        } else {
          // 越界 → 可选：保持旧值 or 标 invalid
          output->At(r, c) = INVALID_VALUE;
        }
      }
    }
  } else {
    // 更新整个patch
    for (int r = 0; r < patch_rows; ++r) {
      int src_r = base_r + r;
      if (src_r < 0 || src_r >= data.rows) continue;

      const double* row_ptr = data.ptr<double>(src_r);

      for (int c = 0; c < patch_cols; ++c) {
        int src_c = base_c + c;

        if (src_c >= 0 && src_c < data.cols) {
          output->At(r, c) = static_cast<float>(row_ptr[src_c]);
        } else {
          // continue;
          output->At(r, c) = INVALID_VALUE;
          // output->At(r, c) = 0;
        }
      }
    }
  }
}
