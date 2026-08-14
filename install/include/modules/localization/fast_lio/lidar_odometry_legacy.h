#pragma once

#include <omp.h>

#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#define PCL_NO_PRECOMPILE
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <yaml-cpp/yaml.h>

#include "cyber/common/file.h"
#include "modules/common/transform/geometry/rotation_conversions.h"
#include "modules/localization/fast_lio/config/runtime_config.h"
#include "modules/localization/fast_lio/include/imu_processing_legacy.hpp"
#include "modules/perception/common/algorithm/point_cloud_processing/ikd-Tree/ikd_Tree.h"

using namespace std;

namespace fastlio {

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

class LidarOdometry {
 public:
  LidarOdometry();
  virtual ~LidarOdometry();

  void SetExtrinsicMatrix(const Eigen::Matrix4f& extrinsic_matrix);
  void SetDataFolder();
  void Close();

  virtual void Init(std::shared_ptr<jojo::localization::RuntimeConfig> rparam);
         
  void pointBodyToWorld(PointType const* const pi, PointType* const po);

  void points_cache_collect();

  void lasermap_fov_segment();

  // bool sync_packages(MeasureGroup& meas);

  void map_incremental();

  void run_odometry(MeasureGroup& Measures);

  void save_result(bool b_save_pcd = false);

  void Show(bool b_pause = false);

 protected:
  std::vector<PointVector> Nearest_Points;

  PointCloudXYZI::Ptr feats_undistort;  // 去畸变后的点云，lidar 坐标系
  PointCloudXYZI::Ptr feats_undistort_filtered;  // 抽样采样后的点云
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr normvec;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr traj_cloud;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterMap;

  KD_TREE<PointType> ikdtree;

  V3D Lidar_T_wrt_IMU = V3D::Zero();
  M3D Lidar_R_wrt_IMU = M3D::Identity();

  /*** EKF inputs and output ***/
  MeasureGroup Measures;
  // esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
  esekfom::esekf kf;
  state_ikfom state_point;

  std::shared_ptr<ImuProcess> p_imu;

  bool pose_inited = false;
  // for filter useless .pcd
  int pose_init_count = 0;
  OdomData o_pose;
  std::ofstream ofs_pose;

  std::string prefix;
  std::string postfix;
  std::string path_lidar;
  std::string path_pose;

 private:
  std::shared_ptr<jojo::localization::RuntimeConfig> rparam_;

  int lidar_type = 1;
  // float LASER_POINT_COV = 0.001;
  int point_filter_num = 5;

  double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
  double lidar_end_time = 0, first_lidar_time = 0.0;

  bool flg_first_scan = true, flg_EKF_inited;

  // lasermap_fov_segment
  float DET_RANGE           = 100.0f;
  const float MOV_THRESHOLD = 1.5f;
  double cube_len           = 1000;
  std::vector<BoxPointType> cub_needrm;

  V3D pos_lid;

  // 雷达坐标系下的局部地图
  BoxPointType LocalMap_Points;
  bool Localmap_Initialized = false;

  // map_incremental
  int add_point_size = 0;

  float filter_size_surf_min = 0.5, filter_size_map_min = 0.5;
  int feats_down_size = 0;

  // update_iterated_dyn_share_modified
  bool extrinsic_est_en = false;

  int NUM_MAX_ITERATIONS = 5;

  /*** Time Log Variables ***/
  double kdtree_delete_time = 0.0;
  int kdtree_delete_counter = 0;
  /**************************/

  pcl::visualization::PCLVisualizer::Ptr vis = NULL;
  
  bool trajectory_added_to_viewer_    = false;
};

}  // namespace fastlio
