#include "modules/perception/common/lidar/convert/livox.h"

#ifdef _OPENMP
#include <omp.h>

#include <atomic>
#endif

#include <iomanip>

// #ifdef _OPENMP
// // TODO

// #else

bool LvToPcl(pcl::PointCloud<livox_ros::PointXYZIRT>::Ptr point_lv_,
             pcl::PointCloud<pcl::PointXYZIRT>::Ptr point_pcl_) {
  if (!point_lv_ || !point_pcl_) return false;
  size_t point_num = point_lv_->size();
  if (point_num == 0) return false;

  double time_start = std::numeric_limits<double>::max();
  double time_end   = std::numeric_limits<double>::lowest();

  point_pcl_->clear();

  //固态雷达没有结构化的意义，每帧点云是不一样数量的
  point_pcl_->points.reserve(point_num);
  for (size_t i = 0; i < point_num; i++) {
    const auto& pt = point_lv_->points[i];

    bool invalid = !IsFinitePoint(pt) ||
                   (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
                   (pt.intensity == 0);
    if (invalid) continue;

    pcl::PointXYZIRT p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    // --- 自动兼容 ---
    // p.intensity = pt.reflectivity;
    p.intensity = pt.intensity;
    p.ring      = pt.line;
    // system time: ns ==> ms
    // p.timestamp = static_cast<double>(pt.offset_time / 1000000.0);
    // p.timestamp = static_cast<double>(pt.timestamp / 1000000.0);
    p.timestamp = static_cast<double>(pt.timestamp * 1e-6);
    point_pcl_->emplace_back(p);

    // std::cout << pt.timestamp << std::endl;
    // std::cout << p.timestamp << std::endl;

    if (p.timestamp < time_start) time_start = p.timestamp;
    if (p.timestamp > time_end) time_end = p.timestamp;
  }

  // std::cout << std::fixed << std::setprecision(0);
  // std::cout << "livox time start: " << time_start << " time end: " << time_end << std::endl;

  // 归一化时间戳到 0~100 帧内时间戳
  const double time_diff = time_end - time_start;
  if (time_diff > 1e-9) {
    const double inv_diff = 100.0 / time_diff;
    for (auto& p : point_pcl_->points) {
      // p.timestamp = static_cast<float>((p.timestamp - time_start) * inv_diff);
      p.timestamp = (p.timestamp - time_start) * inv_diff;
    }
  }

  point_pcl_->width    = point_pcl_->size();
  point_pcl_->height   = 1;
  point_pcl_->is_dense = true;

  return true;
}

// #endif
