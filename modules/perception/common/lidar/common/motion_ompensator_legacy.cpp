#include "modules/perception/common/lidar/common/motion_ompensator.h"

namespace jojo {
namespace perception {
namespace lidar {
namespace cstruct = jojo::common_struct;

void MotionCompensator::SetExtrinsicMatrix(
    const Eigen::Matrix4f& extrinsic_matrix) {
  extrinsic_matrix_      = extrinsic_matrix;
  extrinsic_matrix_inv_  = extrinsic_matrix.inverse();
  extrinsic_matrix_d     = extrinsic_matrix_.cast<double>();
  extrinsic_matrix_inv_d = extrinsic_matrix_inv_.cast<double>();
}

void MotionCompensator::Init() {
  // ori_cloud_.reset(new PointCloudXYZI);
  // ori_cloud_->points.reserve(230400);
}

// lower_bound 复杂度：O(NlogM)
void MotionCompensator::UndistortPointCloudByImu(
    PointCloudXYZI::Ptr cloud, const std::deque<cstruct::ImuData>& imu_data,
    uint64_t cloud_start_time, uint64_t cloud_end_time,
    double deskew_time_ratio, bool visualize) {
  // 简化版 rotational deskew，没有 姿态积分 + 坐标变换 (fast-lio)

  if (imu_data.empty() || cloud->empty()) return;

  if (visualize) {
    // /* 复制一份原始点云用于可视化
    // 浅拷贝
    // ori_cloud_ = cloud;
    // 深拷贝
    ori_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>(*cloud));
    // std::cout << "cloud size: " << cloud->points.size() << std::endl;
    // */
  }

  // point time is ms
  // cloud_start_time 是全局时间 1734319152416ms ，不是 0ms
  double scan_duration = cloud_end_time - cloud_start_time;
  // std::cout << "scan_duration size: " << scan_duration << std::endl;
  // 整帧点云最终对齐到的时间点
  // 通常使用 deskew_time_ratio = 0.5，即对齐到中间时刻
  // double ref_time = cloud_start_time + deskew_time_ratio * scan_duration;
  // 相对时间
  double ref_time = deskew_time_ratio * scan_duration;
  // std::cout << std::fixed << std::setprecision(9) << "ref_time: " << ref_time
  //           << std::endl;

  // 已经重建完成的点云，根据IMU数据对点云进行补偿
  // 一行（row） 对应 LiDAR 的一条激光线（ring）
  // 一列（col） 对应 水平扫描的角度步进（azimuth）
  for (size_t idx = 0; idx < cloud->points.size(); ++idx) {
    auto& point = cloud->points[idx];

    // 正确计算rel_time：从 0 线性插值到 1
    // 输入的点云已经经过结构化重排，前 1800 个点是 ring 1；
    // size_t row_idx = idx / total_cols;  // 第几行
    size_t col_idx = idx % total_cols;  // 第几列
    // 旋转扫描式雷达的当前列在整个列周期的比例，代表了这一时刻相对扫描开始时刻的 offset ：0~1
    double rel_time = static_cast<double>(col_idx) / (total_cols - 1);
    // 相对时间
    double point_time = rel_time * scan_duration;
    // 真实时间 寻找 IMU 数据
    double point_time_rel = cloud_start_time + point_time;
    // std::cout << "point_time: " << point_time << std::endl;

    // 在 IMU 数据中查找对应时间段的最近两帧
    // 找到 时间大于等于当前点的第一个 IMU 样本。
    auto it = std::lower_bound(imu_data.begin(), imu_data.end(), point_time_rel,
                               [](const cstruct::ImuData& imu, double time) {
                                 return imu.time < time;
                               });

    if (it == imu_data.end() || it == imu_data.begin()) continue;

    double ratio;
    // 取前一个 (imu1) 和当前 (imu2) 两个 IMU 数据做线性插值，得出当前时刻的角速度
    auto& imu1 = *(it - 1);
    auto& imu2 = *it;
    double idt = imu2.time - imu1.time;
    if (idt < 1e-6) {
      ratio = 0.0;
    } else {
      double t = (point_time_rel - imu1.time) / idt;
      // 防止越界
      // ratio = std::clamp(t, 0.0, 1.0);
      ratio = apollo::common::math::Clamp(t, 0.0, 1.0);
    }
    // std::cout << "imu1: " << imu1.time << " imu2: " << imu2.time << std::endl;
    // std::cout << "ratio: " << ratio << std::endl;

    // 角速度插值：
    Eigen::Vector3d gyro1(imu1.gyro.x, imu1.gyro.y, imu1.gyro.z);
    Eigen::Vector3d gyro2(imu2.gyro.x, imu2.gyro.y, imu2.gyro.z);
    // omega 的单位是 rad/s（通常 IMU 输出角速度单位是弧度每秒）
    Eigen::Vector3d omega = gyro1 + ratio * (gyro2 - gyro1);
    if (omega.norm() < 1e-8) {
      std::cout << "warning, ignore desk!!!" << std::endl;
      continue;
    }

    // 用这个插值得到的角速度，代表当前 point 运动到 ref_time 的恒定角速度
    // 小角度近似，补偿旋转，注意没有 平移补偿
    // double dt = point_time - cloud_start_time;
    // Eigen::AngleAxisd rot(dt * omega.norm(), omega.normalized());
    // imu.time is ms ==> need s ==> omega
    double dt = (point_time - ref_time) / 1000.0;
    // !! 通过放大系数，可以观察到补偿效果
    // double dt = (point_time - ref_time) / 100.0;
    // std::cout << "dt: " << dt << std::endl;
    Eigen::AngleAxisd rot(dt * omega.norm(), omega.normalized());
    // Eigen::Vector3d p(point.x, point.y, point.z);
    // p = rot.inverse().toRotationMatrix() * p;

    // 这里是角速度引起的杆臂速度，并不是平移补偿
    // !! 平移补偿需要积分，这里整个过程都没有积分
    Eigen::Vector4f p_l_h(point.x, point.y, point.z, 1.0f);
    // LiDAR → IMU
    Eigen::Vector4f p_i_h = extrinsic_matrix_ * p_l_h;
    // IMU 坐标系下做补偿
    Eigen::Vector3f p_i = p_i_h.head<3>();
    // 旋转补偿
    Eigen::Vector3f p_i_rot =
        rot.inverse().toRotationMatrix().cast<float>() * p_i;
    // 杆臂补偿
    Eigen::Vector3f t_li  = extrinsic_matrix_.block<3, 1>(0, 3);
    Eigen::Vector3f lever = omega.cast<float>().cross(t_li) * dt;
    p_i_rot               = p_i_rot - lever;

    // 回到 LiDAR 坐标系
    Eigen::Vector4f p_i_h_undistort;
    p_i_h_undistort.head<3>() = p_i_rot;
    p_i_h_undistort[3]        = 1.0f;

    // IMU → LiDAR
    Eigen::Vector4f p_l_undistort_h = extrinsic_matrix_inv_ * p_i_h_undistort;

    // point.x = p_l_undistort_h.x();
    // point.y = p_l_undistort_h.y();
    // point.z = p_l_undistort_h.z();
    point.x = p_l_undistort_h[0];
    point.y = p_l_undistort_h[1];
    point.z = p_l_undistort_h[2];
  }

  // 可视化前后对比
  if (visualize) {
    // clang-format off
    if (viewer_ == NULL) {
      viewer_.reset(new pcl::visualization::PCLVisualizer("Undistortion Compare"));
    }

    viewer_->removeAllPointClouds();
    viewer_->setBackgroundColor(0, 0, 0);

    // 绿色 显示原始点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> o_color(ori_cloud_, 0, 255, 0);
    viewer_->addPointCloud<pcl::PointXYZI>(ori_cloud_, o_color, "original");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original");

    // 红色 显示处理过的点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> u_color(cloud, 255, 0, 0);
    viewer_->addPointCloud<pcl::PointXYZI>(cloud, u_color, "undistorted");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "undistorted");

    viewer_->addCoordinateSystem(1.0);
    viewer_->spinOnce(10);
    // viewer_->spin();
    // clang-format on
  }
}

bool MotionCompensator::InterpolatePose(
    const std::deque<cstruct::OdomData>& poses, double timestamp,
    Eigen::Matrix4d& pose_out) {
  if (poses.size() < 2 || timestamp < poses.front().time ||
      timestamp > poses.back().time)
    return false;

  // 二分查找姿态时间区间
  auto it = std::lower_bound(
      poses.begin(), poses.end(), timestamp,
      [](const cstruct::OdomData& p, double time) { return p.time < time; });

  if (it == poses.begin() || it == poses.end()) return false;

  // 插值得到 timestamp 的 pose
  double ratio;  // 插值因子 ∈ (0, 1)
  auto& pose1 = *(it - 1);
  auto& pose2 = *it;
  double dt   = pose2.time - pose1.time;
  if (dt < 1e-6) {
    ratio = 0.0;
  } else {
    double t = (timestamp - pose1.time) / dt;
    // 防止越界
    // ratio = std::clamp(t, 0.0, 1.0);
    ratio = apollo::common::math::Clamp(t, 0.0, 1.0);
  }

  // 平移 线性插值
  Eigen::Vector3d p1(pose1.position.x, pose1.position.y, pose1.position.z);
  Eigen::Vector3d p2(pose2.position.x, pose2.position.y, pose2.position.z);
  Eigen::Vector3d pos = p1 + ratio * (p2 - p1);
  // 旋转（处理双解 + 归一化）
  Eigen::Quaterniond r1 = RpyToQuat(pose1.orientation);
  Eigen::Quaterniond r2 = RpyToQuat(pose2.orientation);
  r1.normalize();
  r2.normalize();
  if (r1.dot(r2) < 0.0) {
    r2.coeffs() *= -1.0;
  }
  // SLERP插值
  Eigen::Quaterniond rot = r1.slerp(ratio, r2);
  rot.normalize();

  // 构造矩阵
  pose_out.setIdentity();
  pose_out.block<3, 1>(0, 3) = pos;
  pose_out.block<3, 3>(0, 0) = rot.toRotationMatrix();

  return true;
}

void MotionCompensator::UndistortPointCloudByOdom(
    PointCloudXYZI::Ptr cloud, const std::deque<cstruct::OdomData>& pose_data,
    uint64_t cloud_start_time, uint64_t cloud_end_time,
    double deskew_time_ratio, bool visualize) {
  // std::cout << "using undistort" << std::endl;

  if (cloud->empty() || pose_data.size() < 2) return;

  double scan_duration = cloud_end_time - cloud_start_time;
  // 整帧点云最终对齐到的时间点
  double ref_time = cloud_start_time + deskew_time_ratio * scan_duration;

  Eigen::Matrix4d pose_ref;
  if (!InterpolatePose(pose_data, ref_time, pose_ref)) return;

  for (size_t idx = 0; idx < cloud->points.size(); ++idx) {
    auto& point = cloud->points[idx];

    // 正确计算rel_time：从0线性到1
    // intensity ==> timestamp
    double rel_time   = point.intensity / 100.0;  // 0~1
    double point_time = cloud_start_time + rel_time * scan_duration;

    Eigen::Matrix4d pose_point;
    if (!InterpolatePose(pose_data, point_time, pose_point)) return;

    // 点齐次坐标
    Eigen::Vector4d pt(point.x, point.y, point.z, 1.0);

    // 逆变换到参考帧坐标系
    Eigen::Vector4d deskewed_pt = extrinsic_matrix_inv_d * pose_ref.inverse() *
                                  pose_point * extrinsic_matrix_d * pt;

    point.x = deskewed_pt.x();
    point.y = deskewed_pt.y();
    point.z = deskewed_pt.z();
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace jojo