#pragma once

#include "modules/localization/fast_lio/include/common_lib.h"
#include "modules/perception/common/config/vehicle_config.h"
#include "tools/data_loader/group_convert.h"

void ConvertMeasureGroup(
    const std::shared_ptr<const jojo::tools::MeasureGroup> group,
    fastlio::MeasureGroup& t,
    std::shared_ptr<jojo::perception::config::VehicleConfig> vp) {
  // 1. 传递点云
  // t.lidar = group->lidar.data;
  t.lidar.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
  const auto& src = group->lidar.data;
  // /* way 1
  t.lidar->points.reserve(src->points.size());
  for (const auto& pt : src->points) {
    // 过滤无效点
    if (pt.x == 0 && pt.y == 0 && pt.z == 0) {
      continue;
    }

    // remove those lidar points that hit on the vehicle itself!
    /* way 1
      if (!(pt.x < vp.RONI_min_x || pt.x > vp.RONI_max_x ||
            pt.y < vp.RONI_min_y || pt.y > vp.RONI_max_y ||
            pt.z < vp.RONI_min_z || pt.z > vp.RONI_max_z)) {
        continue;
      }
    */
    // way 2
    // 语义更清晰：“在 box 内不保留”
    // 分支更集中（更容易被编译器优化）
    if (pt.x >= vp->RONI_min_x && pt.x <= vp->RONI_max_x &&
        pt.y >= vp->RONI_min_y && pt.y <= vp->RONI_max_y &&
        pt.z >= vp->RONI_min_z && pt.z <= vp->RONI_max_z) {
      continue;
    }

    pcl::PointXYZINormal p;
    // 逐字段赋值
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    // 如果原始点没有 intensity，可以自己赋值
    p.intensity = pt.intensity;
    p.normal_x  = 0;
    p.normal_y  = 0;
    p.normal_z  = 0;
    p.curvature = pt.timestamp;

    t.lidar->points.emplace_back(std::move(p));
  }
  t.lidar->width    = t.lidar->points.size();
  t.lidar->height   = 1;
  t.lidar->is_dense = true;
  // */
  // 时间 秒
  t.lidar_end_time = group->lidar.time / 1000.0;
  t.lidar_beg_time = group->lidar.start_time / 1000.0;
  // std::cout << "lidar_beg_time: " << t.lidar_beg_time << std::endl;
  // std::cout << "lidar_end_time: " << t.lidar_end_time << std::endl;

  // 2. 传递IMU
  // t.imu = group->imu_vec;
  t.imu.clear();
  // std::deque 没有 reserve()，也不需要
  // t.imu.reserve(group->imu_vec.size());
  // int c = 0;
  for (const auto& src : group->imu_vec) {
    fastlio::ImuData dst;
    // 时间 秒
    dst.timestamp = static_cast<double>(src.time / 1000.0);
    // 加速度
    dst.linear_acceleration.x() = static_cast<double>(src.acc.x);
    dst.linear_acceleration.y() = static_cast<double>(src.acc.y);
    dst.linear_acceleration.z() = static_cast<double>(src.acc.z);
    // 陀螺仪
    dst.angular_velocity.x() = static_cast<double>(src.gyro.x);
    dst.angular_velocity.y() = static_cast<double>(src.gyro.y);
    dst.angular_velocity.z() = static_cast<double>(src.gyro.z);

    t.imu.push_back(dst);

    /* debug
    c++;
    if (c == group->imu_vec.size()) {
      std::cout << std::fixed << std::setprecision(13);
      std::cout << "dst: " << dst.timestamp << std::endl;
      std::cout << " " << dst.linear_acceleration.x() << " "
                << dst.linear_acceleration.y() << " "
                << dst.linear_acceleration.z() << std::endl;
      std::cout << " " << dst.angular_velocity.x() << " "
                << dst.angular_velocity.y() << " " << dst.angular_velocity.z()
                << std::endl;
    }
    */
  }

  // TODO：print imu first and last
}