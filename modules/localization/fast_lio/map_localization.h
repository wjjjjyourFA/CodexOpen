#pragma once

#include <omp.h>

#define PCL_NO_PRECOMPILE
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

#include "modules/perception/tools/opencv/cv_colors.h"

#include "modules/localization/fast_lio/lidar_odometry.h"
#include "modules/localization/fast_lio/config/runtime_config.h"
#include "modules/localization/fast_lio/config/static_config.h"

namespace fastlio {

enum class MapState { INSIDE, OUTSIDE };

class MapLocalization : public LidarOdometry {
 public:
  MapLocalization();
  ~MapLocalization();

  void LoadInitMap(const std::string& map_path);

  void SetInitMap(const PointCloudXYZI::Ptr& map);
  void SetMapCenter(const Eigen::Vector3d& center);

  void SetInitPose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot);

  void Init(std::shared_ptr<jojo::localization::RuntimeConfig> rparam,
            std::shared_ptr<jojo::localization::StaticConfig> sparam);

  void UpdateKfState(state_ikfom& state_point);

  // 同时引用基类同名函数
  using LidarOdometry::pointBodyToWorld;
  // 派生类中出现同名函数 → 会隐藏基类所有同名函数，除非使用 using 声明
  void pointBodyToWorld(PointType const* const pi, PointType* const po,
                        state_ikfom& state_point);

  void lasermap_fov_segment() override;

  void InitDynMap();
  void map_incremental() override;

  void h_share_model(
      state_ikfom& s,
      esekfom::dyn_share_datastruct<double>& ekfom_data) override;

  void run_localization_legacy(MeasureGroup& Measures);

  void run_localization(MeasureGroup& Measures);

  bool Optimization(PointCloudXYZI::Ptr frame);

  // void save_result(bool b_save_pcd = false) override;

  void Show(bool b_pause) override;

  void GetWholeMap(PointCloudXYZI::Ptr& cloud_map);

 protected:
  // 匹配结果
  Eigen::Matrix4f Tr_delta;
  void NDT(pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu,
           pcl::PointCloud<pcl::PointXYZ>::Ptr map_imu);
  void ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr frame_imu,
           pcl::PointCloud<pcl::PointXYZ>::Ptr map_imu);

  void GetAutoInitPose(state_ikfom& init_state, PointCloudXYZI::Ptr frame,
                       PointCloudXYZI::Ptr map);

  bool CheckOutsideGlobalMap();
  bool enable_incremental_mapping_ = false;
  double IsFarFromMap(const PointType& pt);
  int far_point_num_ = 5;

  // 在 ikdtree 的基础上，动态维护，可删除
  KD_TREE<PointType> ikdtree_dyn;

 private:
  // LidarOdometry::param_;  // ==> rparam_
  std::shared_ptr<jojo::localization::StaticConfig> sparam_;

  PointCloudXYZI::Ptr map_ = nullptr;
  Eigen::Vector3d map_center;
  int dyn_map_radius_ = 100;

  bool loc_inited = false;
  // for filter useless .pcd
  int loc_init_count = 0;

  state_ikfom init_state;
  // 强制认为，初始位置在地图内
  MapState map_state_             = MapState::INSIDE;
  int outside_counter             = 0;
  int inside_counter              = 0;
  const double th_global_to_local = 6.0;  // GLOBAL → LOCAL
  const double th_local_to_global = 4.0;  // LOCAL → GLOBAL
  const int stable_frame_num      = 10;  // 连续帧阈值

 private:
  void ShowInitResult(pcl::visualization::PCLVisualizer::Ptr vis,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr frame,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr map,
                      const string& window_name);

  void ShowMatchResultDual(pcl::visualization::PCLVisualizer::Ptr vis,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_first,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_second,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map,
                           const string& window_name);
};

}  // namespace fastlio
