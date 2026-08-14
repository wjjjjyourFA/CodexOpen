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

// 双指针顺序推进：O(N+M)
void MotionCompensator::UndistortPointCloudByImu(
    PointCloudXYZI::Ptr cloud, const std::deque<cstruct::ImuData>& imu_data,
    uint64_t cloud_start_time, uint64_t cloud_end_time,
    double deskew_time_ratio, bool visualize) {
  // 简化版 rotational deskew，没有 姿态积分 + 坐标变换 (fast-lio)

  if (!cloud || cloud->empty() || imu_data.size() < 2 || total_rows == 0 ||
      total_cols < 2 || cloud_end_time < cloud_start_time ||
      deskew_time_ratio < 0.0 || deskew_time_ratio > 1.0) {
    return;
  }

  if (visualize) {
    ori_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>(*cloud));
  }

  // point time is ms
  // cloud_start_time 是全局时间 1734319152416ms ，不是 0ms
  double scan_duration = cloud_end_time - cloud_start_time;
  // 整帧点云最终对齐到的时间点
  // 通常使用 deskew_time_ratio = 0.5，即对齐到中间时刻
  double ref_time = cloud_start_time + deskew_time_ratio * scan_duration;

  // 外参平移提前取出来，避免每次循环都计算
  const Eigen::Vector3f t_li = extrinsic_matrix_.block<3, 1>(0, 3);

  size_t imu_idx = 1;
  for (size_t col = 0; col < total_cols; ++col) {
    // 当前列对应时间
    double rel_time   = static_cast<double>(col) / (total_cols - 1);
    double point_time = cloud_start_time + rel_time * scan_duration;

    // 双指针推进，找到 IMU 数据中时间 最接近 小于等于 当前点时间的 IMU 样本
    while (imu_idx < imu_data.size() && imu_data[imu_idx].time < point_time) {
      ++imu_idx;
    }

    if (imu_idx >= imu_data.size()) break;

    double ratio = 0.0;
    auto& imu1   = imu_data[imu_idx - 1];
    auto& imu2   = imu_data[imu_idx];
    double idt   = imu2.time - imu1.time;
    if (idt < 1e-6) {
      ratio = 0.0;
    } else {
      double t = (point_time - imu1.time) / idt;
      // 防止越界
      // ratio = std::clamp(t, 0.0, 1.0);
      ratio = apollo::common::math::Clamp(t, 0.0, 1.0);
    }

    // 角速度插值：
    Eigen::Vector3d gyro1(imu1.gyro.x, imu1.gyro.y, imu1.gyro.z);
    Eigen::Vector3d gyro2(imu2.gyro.x, imu2.gyro.y, imu2.gyro.z);
    // omega 的单位是 rad/s（通常 IMU 输出角速度单位是弧度每秒）
    Eigen::Vector3d omega = gyro1 + ratio * (gyro2 - gyro1);
    if (omega.norm() < 1e-8) {
      std::cout << "warning, ignore desk!!!" << std::endl;
      continue;
    }

    // 当前列到参考时刻的时间差
    double dt = (point_time - ref_time) / 1000.0;
    Eigen::AngleAxisd rot(dt * omega.norm(), omega.normalized());
    Eigen::Matrix3f R = rot.inverse().toRotationMatrix().cast<float>();
    // 杆臂补偿
    Eigen::Vector3f lever = omega.cast<float>().cross(t_li) * dt;

    // 当前 imu 区间，计算同一列的所有点
    for (size_t row = 0; row < total_rows; ++row) {
      size_t idx = row * total_cols + col;
      if (idx >= cloud->points.size()) break;

      auto& point = cloud->points[idx];

      // 这里是角速度引起的杆臂速度，并不是平移补偿
      // !! 平移补偿需要积分，这里整个过程都没有积分
      Eigen::Vector4f p_l_h(point.x, point.y, point.z, 1.0f);
      // LiDAR → IMU
      Eigen::Vector4f p_i_h = extrinsic_matrix_ * p_l_h;
      // IMU 坐标系下做补偿
      Eigen::Vector3f p_i = p_i_h.head<3>();
      // 旋转补偿
      Eigen::Vector3f p_i_rot = R * p_i;
      // 杆臂补偿
      p_i_rot = p_i_rot - lever;

      // 回到 LiDAR 坐标系
      Eigen::Vector4f p_i_h_undistort;
      p_i_h_undistort.head<3>() = p_i_rot;
      p_i_h_undistort[3]        = 1.0f;

      // IMU → LiDAR
      Eigen::Vector4f p_l_undistort_h = extrinsic_matrix_inv_ * p_i_h_undistort;

      point.x = p_l_undistort_h[0];
      point.y = p_l_undistort_h[1];
      point.z = p_l_undistort_h[2];
    }
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

bool MotionCompensator::InterpolatePoseBase(double timestamp,
                                            const cstruct::OdomData& pose1,
                                            const cstruct::OdomData& pose2,
                                            Eigen::Matrix4d& pose_out) {
  // TODO：添加 pose_out 的防御性代码，默认值由 pose1 生成
  // 插值得到 timestamp 的 pose
  double ratio = 0.0;  // 插值因子 ∈ (0, 1)
  double dt    = pose2.time - pose1.time;
  if (dt > 1e-6) {
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
  // 保证最短路径
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

  auto& pose1 = *(it - 1);
  auto& pose2 = *it;

  if (!InterpolatePoseBase(timestamp, pose1, pose2, pose_out)) return false;

  return true;
}

void MotionCompensator::UndistortPointCloudByOdom(
    PointCloudXYZI::Ptr cloud, const std::deque<cstruct::OdomData>& pose_data,
    uint64_t cloud_start_time, uint64_t cloud_end_time,
    double deskew_time_ratio, bool visualize) {
  // std::cout << "using undistort" << std::endl;

  if (!cloud || cloud->empty() || pose_data.size() < 2 || total_rows == 0 ||
      total_cols < 2 || cloud_end_time < cloud_start_time ||
      deskew_time_ratio < 0.0 || deskew_time_ratio > 1.0) {
    return;
  }

  double scan_duration = cloud_end_time - cloud_start_time;
  // 整帧点云最终对齐到的时间点
  double ref_time = cloud_start_time + deskew_time_ratio * scan_duration;

  Eigen::Matrix4d pose_ref;
  if (!InterpolatePose(pose_data, ref_time, pose_ref)) return;

  // 提前求逆
  const Eigen::Matrix4d pose_ref_inv = pose_ref.inverse();

  size_t pose_idx = 1;
  for (size_t col = 0; col < total_cols; ++col) {
    double rel_time   = static_cast<double>(col) / (total_cols - 1);
    double point_time = cloud_start_time + rel_time * scan_duration;

    // clang-format off
    while (pose_idx < pose_data.size() && pose_data[pose_idx].time < point_time) {
      ++pose_idx;
    }
    // clang-format on

    if (pose_idx >= pose_data.size()) break;

    auto& pose1 = pose_data[pose_idx - 1];
    auto& pose2 = pose_data[pose_idx];

    Eigen::Matrix4d pose_point;
    if (!InterpolatePoseBase(point_time, pose1, pose2, pose_point)) {
      continue;
    }

    Eigen::Matrix4d relative_pose = pose_ref_inv * pose_point;
    Eigen::Matrix4d lidar_relative =
        extrinsic_matrix_inv_d * relative_pose * extrinsic_matrix_d;

    // 同一列所有点
    for (size_t row = 0; row < total_rows; ++row) {
      size_t idx = row * total_cols + col;
      if (idx >= cloud->points.size()) break;

      auto& point = cloud->points[idx];

      Eigen::Vector4d pt(point.x, point.y, point.z, 1.0);

      Eigen::Vector4d deskewed_pt = lidar_relative * pt;

      point.x = deskewed_pt.x();
      point.y = deskewed_pt.y();
      point.z = deskewed_pt.z();
    }
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace jojo
