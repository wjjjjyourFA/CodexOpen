#ifndef IMUPROCESS_H
#define IMUPROCESS_H

//#include <math.h>
//#include <cmath>
//#include <condition_variable>
//#include <eigen_conversions/eigen_msg.h>




#include "global_define.h"

#include "common_lib.h"
#include "so3_math.h"
#include "use-ikfom.hpp"


#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "use-ikfom.hpp"


#define MAX_INI_COUNT (10)
inline const bool time_list(PointType &x, PointType &y)  {
    return (x.curvature < y.curvature);
}

class ImuProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void set_extrinsic(const V3D &transl, const M3D &rot);
    void set_extrinsic(const V3D &transl);
    void set_extrinsic(const MD(4,4) &T);
    void set_gyr_cov(const V3D &scaler);
    void set_acc_cov(const V3D &scaler);
    void set_gyr_bias_cov(const V3D &b_g);
    void set_acc_bias_cov(const V3D &b_a);
    Eigen::Matrix<double, 12, 12> Q;
    bool Process(const MeasureGroup &meas,
                 esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state,
                 PointCloudXYZI::Ptr pcl_un_);

    ofstream fout_imu;
    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;
    double first_lidar_time;
    double  gyr_cov, acc_cov, b_gyr_cov, b_acc_cov;
    vector<double> extrinT;                 // 平移外参
    vector<double> extrinR;                 // 旋转外参
    M3D Lidar_R_wrt_IMU;
    V3D Lidar_T_wrt_IMU;

private:
    void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
    void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

    PointCloudXYZI::Ptr cur_pcl_un_;
    ImuDataConstPtr last_imu_;
    deque<ImuDataConstPtr> v_imu_;
    vector<Pose6D> IMUpose;
    vector<M3D>    v_rot_pcl_;

    V3D mean_acc;
    V3D mean_gyr;
    V3D angvel_last;
    V3D acc_s_last;
    double start_timestamp_;
    double last_lidar_end_time_;
    int    init_iter_num = 1;
    bool   b_first_frame_ = true;
    bool   imu_need_init_ = true;
};


#endif // IMUPROCESS_H
