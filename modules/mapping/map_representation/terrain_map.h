#ifndef TERRAIN_MAP_H_
#define TERRAIN_MAP_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/search/kdtree.h>
#include <pcl/register_point_struct.h>
#include <pcl/registration/transforms.h>
#include "pcl/visualization/pcl_visualizer.h"

#include "opencv2/opencv.hpp"

#include "modules/common_struct/basic_msgs/Pose6D.h"
#include "modules/common/transform/geometry/rotation_conversions.h"

#include "modules/mapping/map_representation/common.h"
#include "modules/mapping/map_representation/load_map_2d.h"
#include "modules/mapping/map_visualization/config/static_config.h"

// current frame in grid map
struct TerrainMapPatch {
  /* 使用 se3_pose 表示地图中心的位姿
  double map_center_x;
  double map_center_y;
  double map_center_z;
  double roll;
  double pitch;
  double yaw;
  */
  jojo::common_struct::SE3Pose pose;

  // default: height
  // float data[250000];

  // default：512 x 512
  int rows = 512;
  int cols = 512;
  // 默认单位 m
  float resolution = 0.2f;
  // size = rows * cols
  std::vector<float> data;

  void Resize(int r, int c, float init_value = 0.0f) {
    rows = r;
    cols = c;
    // 先 row，后 col
    data.assign(rows * cols, init_value);
  }

  int Index(int r, int c) const {
    // “行优先（row-major）存储”
    return r * cols + c;
  }

  float& At(int r, int c) {
    assert(r >= 0 && r < rows && c >= 0 && c < cols);
    return data[Index(r, c)];
  }

  const float& At(int r, int c) const {
    // assert(r >= 0 && r < rows && c >= 0 && c < cols);
    return data[Index(r, c)];
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct TerrainMapHyperparams {
  int64_t map_min_x, map_min_y;  // cm
  uint32_t map_resolution = 20;  // 20cm ==> 0.2m
};

// 内部是 cm，接口输出是 m
class TerrainMap {
 public:
  TerrainMap();
  virtual ~TerrainMap();

  void Init(const std::string& root,
            std::shared_ptr<jojo::mapping::StaticConfig> sparam = nullptr);
  void SetInpaintedFlag(bool flag);

  void LoadInitMap(const std::string& root);
  void SetInitMap(cv::Mat& map);

  void QueryMapFromXY(double x_m, double y_m, int radius, float* in_data);

  double GetValueFromXY(double x_m, double y_m);
  double GetValueFromXY(double x_m, double y_m, int32_t& r, int32_t& c);

  double GetValueFromRC(int32_t r, int32_t c);
  double GetValueFromRC(int32_t r, int32_t c, double& x_m, double& y_m);

  void GetTerrainMapPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc);

  void SetPatchUpdateRadius(int radius);
  void GetTerrainMapPatch(double* xyzrpy,
                          std::shared_ptr<TerrainMapPatch> ouptut);

 protected:
  bool WorldToGrid(double x_m, double y_m, int32_t& row, int32_t& col);
  bool GridToWorld(int32_t row, int32_t col, double& x_m, double& y_m);

 protected:
  std::shared_ptr<jojo::mapping::StaticConfig> sparam_;

  std::string map_path;
  // 数据底图，单位：m；如果存放的是高度，单位也是 m；
  cv::Mat data;

  pcl::visualization::PCLVisualizer::Ptr vis_ = NULL;

 private:
  TerrainMapHyperparams hps_;

  bool b_use_inpainted = false;
  bool use_step_mode   = false;
  int patch_radius;
};

#endif
