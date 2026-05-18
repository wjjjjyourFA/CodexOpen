#ifndef IMU_PROCESSING_HPP
#define IMU_PROCESSING_HPP

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>

// #include <so3_math.h>

#include <Eigen/Eigen>

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

// #include <common_lib.h>
// #include "use-ikfom.hpp"
// #include "preprocess.h"
#include "modules/localization/fast_lio/include/common_lib.h"
#include "modules/localization/fast_lio/ieskf/use-ikfom.hpp"
#include "modules/localization/fast_lio/ieskf/esekfom.hpp"
#include "modules/localization/fast_lio/include/preprocess.h"

/*
 * 这个hpp主要包含：
 * IMU数据预处理：IMU初始化，IMU正向传播，反向传播补偿运动失真   
 */

namespace fastlio {
/// *************Preconfiguration
#define MAX_INI_COUNT (10)  // 最大迭代次数
// 判断点的时间先后顺序(注意 intensity 中存储的是时间戳)
inline const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};
// inline const bool time_list(PointType &x, PointType &y) {return (x.intensity < y.intensity);};
// inline const bool time_list(PointType &x, PointType &y) {return (x.timestamp < y.timestamp);};

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const ImuData &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_whole_param(const V3D &transl, const M3D &rot, 
                       const V3D &gyr, const V3D &acc, 
                       const V3D &gyr_bias, const V3D &acc_bias);
  Eigen::Matrix<double, 12, 12> Q;    // 噪声协方差矩阵  对应论文式(8)中的 Q
  // void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);
  void Process(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;
  V3D cov_acc;             // 加速度协方差
  V3D cov_gyr;             // 角速度协方差
  V3D cov_acc_scale;       // 外部传入的 初始加速度协方差
  V3D cov_gyr_scale;       // 外部传入的 初始角速度协方差
  V3D cov_bias_gyr;        // 角速度bias的协方差
  V3D cov_bias_acc;        // 加速度bias的协方差
  double first_lidar_time; // 当前帧第一个点云时间
  int lidar_type;

 private:
  // void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  // void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);
  void IMU_init(const MeasureGroup &meas, esekfom::esekf &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;        // 当前帧点云未去畸变
  ImuData last_imu_;                      // 上一帧imu
  std::deque<ImuData> v_imu_;
  std::vector<Pose6D> IMUpose;            // 存储imu位姿(反向传播用)
  std::vector<M3D> v_rot_pcl_;
  // WRT with respect to（相对于）
  M3D Lidar_R_wrt_IMU;                    // lidar到IMU的旋转外参
  V3D Lidar_T_wrt_IMU;                    // lidar到IMU的平移外参
  V3D mean_acc;                           // 加速度均值,用于计算方差
  V3D mean_gyr;                           // 角速度均值，用于计算方差
  V3D angvel_last;                        // 上一帧角速度
  V3D acc_s_last;                         // 上一帧加速度
  double start_timestamp_;                // 开始时间戳
  double last_lidar_end_time_;            // 上一帧结束时间戳
  int init_iter_num   = 1;                // 初始化迭代次数
  bool b_first_frame_ = true;             // 是否是第一帧
  bool imu_need_init_ = true;             // 是否需要初始化imu
};
  
}
#endif