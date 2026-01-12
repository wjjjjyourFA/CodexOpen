#include "modules/perception/common/lidar/convert/velodyne.h"

#ifdef _OPENMP
#include <omp.h>
#include <atomic>
#endif

// #ifdef _OPENMP
// // TODO

// #else

bool VdToPcl(pcl::PointCloud<velodyne_ros::PointXYZIR>::Ptr point_vd_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_, bool structured) {
  size_t point_num = point_vd_->size();
  if (point_num == 0) return false;

  point_pcl_->clear();
  if (structured) {
    /* way 1
    int max_ring = 0;
    for (auto& pt : point_vd_->points)
      if (pt.ring > max_ring) max_ring = pt.ring;
    */
    int max_ring = 32;

    size_t width_per_ring = point_num / max_ring;
    // std::cout << "width_per_ring: " << width_per_ring << std::endl;

    // point_pcl_->resize(point_num);
    point_pcl_->resize(width_per_ring * max_ring);
    point_pcl_->header = point_vd_->header;
    point_pcl_->width  = width_per_ring;
    point_pcl_->height = max_ring;
    // point_pcl_->is_dense = point_vd_->is_dense;
    point_pcl_->is_dense = false;

    std::vector<size_t> ring_count(max_ring + 1, 0);

    // 初始化每个点为 0 或 NaN
    for (auto& pt : point_pcl_->points) {
      pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
      pt.intensity       = 0;
    }

    for (const auto& pt : point_vd_->points) {
      auto& ring = pt.ring;
      if (ring >= max_ring) {
        std::cerr << "ring index out of range: " << ring << std::endl;
        return false;
      }

      size_t idx = ring * width_per_ring + ring_count[ring]++;
      if (idx >= point_pcl_->points.size()) continue;  // 安全检查
      auto& dst = point_pcl_->points[idx];

      dst.x = pt.x;
      dst.y = pt.y;
      dst.z = pt.z;
      // --- 自动兼容 float ---
      dst.intensity = static_cast<float>(pt.intensity);
    }
  } else {
    // std::cout << "width: " << point_vd_->width << std::endl;
    // std::cout << "height: " << point_vd_->height << std::endl;

    point_pcl_->reserve(point_num);
    for (size_t i = 0; i < point_num; i++) {
      const auto& pt = point_vd_->points[i];

      bool invalid = !pcl::isFinite(pt) ||
                     (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
                     (pt.intensity == 0);
      if (invalid) continue;

      pcl::PointXYZI p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      // --- 自动兼容 float ---
      p.intensity = static_cast<float>(pt.intensity);
      point_pcl_->emplace_back(p);
    }

    point_pcl_->width    = point_pcl_->size();
    point_pcl_->height   = 1;
    point_pcl_->is_dense = true;
  }

  return true;
}

bool VdToPcl(pcl::PointCloud<velodyne_ros::PointXYZIRT>::Ptr point_vd_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_, bool structured) {
  size_t point_num = point_vd_->size();
  if (point_num == 0) return false;

  point_pcl_->clear();
  if (structured) {
    int max_ring = 32;

    size_t width_per_ring = point_num / max_ring;
    // std::cout << "width_per_ring: " << width_per_ring << std::endl;

    // point_pcl_->resize(point_num);
    point_pcl_->resize(width_per_ring * max_ring);
    point_pcl_->header = point_vd_->header;
    point_pcl_->width  = width_per_ring;
    point_pcl_->height = max_ring;
    // point_pcl_->is_dense = point_vd_->is_dense;
    point_pcl_->is_dense = false;

    std::vector<size_t> ring_count(max_ring + 1, 0);

    // 初始化每个点为 0 或 NaN
    for (auto& pt : point_pcl_->points) {
      pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
      pt.intensity       = 0;
    }

    for (const auto& pt : point_vd_->points) {
      auto& ring = pt.ring;
      if (ring >= max_ring) {
        std::cerr << "ring index out of range: " << ring << std::endl;
        return false;
      }

      size_t idx = ring * width_per_ring + ring_count[ring]++;
      if (idx >= point_pcl_->points.size()) continue;  // 安全检查
      auto& dst = point_pcl_->points[idx];

      dst.x = pt.x;
      dst.y = pt.y;
      dst.z = pt.z;
      // --- 自动兼容 float ---
      dst.intensity = static_cast<float>(pt.intensity);
    }
  } else {
    point_pcl_->reserve(point_num);
    for (size_t i = 0; i < point_num; i++) {
      const auto& pt = point_vd_->points[i];

      bool invalid = !pcl::isFinite(pt) ||
                     (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
                     (pt.intensity == 0);
      if (invalid) continue;

      pcl::PointXYZI p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      // --- 自动兼容 float ---
      p.intensity = static_cast<float>(pt.intensity);
      point_pcl_->emplace_back(p);
    }

    point_pcl_->width    = point_pcl_->size();
    point_pcl_->height   = 1;
    point_pcl_->is_dense = true;
  }

  return true;
}

// #endif

bool VdToPcl(pcl::PointCloud<velodyne_ros::PointXYZIRT>::Ptr point_vd_,
             pcl::PointCloud<pcl::PointXYZIRT>::Ptr point_pcl_) {
  // TODO
}