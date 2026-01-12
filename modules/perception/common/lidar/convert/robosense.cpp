#include "modules/perception/common/lidar/convert/robosense.h"

// 只要你的构建系统启用了 OpenMP，宏 _OPENMP 就会定义
// Qt/CMake
#ifdef _OPENMP
#include <omp.h>
#include <atomic>
#endif

#include <vector>

#ifdef _OPENMP

bool RsToPcl(pcl::PointCloud<robosense_ros::PointII>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_, bool structured) {
  // std::cout << "omp enabled" << std::endl;

  size_t point_num = point_rs_->size();
  if (point_num == 0) return false;

  point_pcl_->clear();
  if (structured) {
    // --- 保留结构 ---
    point_pcl_->resize(point_num);
    point_pcl_->width    = point_rs_->width;
    point_pcl_->height   = point_rs_->height;
    point_pcl_->is_dense = false;
  } else {
    // --- 非结构化点云：预留容量，但不 resize ---
    point_pcl_->points.reserve(point_num);
    point_pcl_->width = point_pcl_->height = 1;
    // point_pcl_->is_dense = point_rs_->is_dense;
    point_pcl_->is_dense = true;
  }
  point_pcl_->header = point_rs_->header;

  int n_threads = 4;
  // int n_threads = omp_get_max_threads();
  omp_set_num_threads(n_threads);

#pragma omp parallel
  {
    // 每个线程自己的局部 buffer
    std::vector<pcl::PointXYZI> local_buffer;
    if (!structured) local_buffer.reserve(point_num / omp_get_num_threads());

    if (structured) {
      // --- 结构化点云 ring-major 排列 ---
#pragma omp for collapse(2) schedule(static)
      for (int j = 0; j < static_cast<int>(point_rs_->width); j++) {
        for (int i = 0; i < static_cast<int>(point_rs_->height); i++) {
          const auto& pt = point_rs_->points[j * point_rs_->height + i];
          int dst_idx    = pt.ring * point_rs_->width + j;  // ring-major 索引
          auto& dst      = point_pcl_->points[dst_idx];

          // if (!pcl::isFinite(pt) ||
          //     (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
          //     (pt.intensity == 0)) {
          //   dst.x = dst.y = dst.z = std::numeric_limits<float>::quiet_NaN();
          //   dst.intensity         = 0;
          // } else {
            dst.x         = pt.x;
            dst.y         = pt.y;
            dst.z         = pt.z;
            dst.intensity = static_cast<float>(pt.intensity);
          // }
        }
      }
    } else {
      // --- 非结构化点云原序列 ---
#pragma omp for schedule(static)
      for (size_t i = 0; i < point_num; ++i) {
        const auto& pt = (*point_rs_)[i];
        if (!pcl::isFinite(pt) || (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
            (pt.intensity == 0))
          continue;

        pcl::PointXYZI new_pt;
        new_pt.x         = pt.x;
        new_pt.y         = pt.y;
        new_pt.z         = pt.z;
        new_pt.intensity = static_cast<float>(pt.intensity);
        local_buffer.emplace_back(new_pt);
      }

#pragma omp critical
      {
        point_pcl_->points.insert(point_pcl_->points.end(),
                                  local_buffer.begin(), local_buffer.end());
      }
    }
  }  // end omp parallel

  // 非结构化模式：重新设置 width
  if (!structured) {
    point_pcl_->width  = point_pcl_->points.size();
    point_pcl_->height = 1;
  }

  return true;
}

bool RsToPcl(pcl::PointCloud<robosense_ros::PointIF>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_, bool structured) {
  size_t point_num = point_rs_->size();
  if (point_num == 0) return false;

  point_pcl_->clear();
  if (structured) {
    point_pcl_->resize(point_num);
    point_pcl_->width    = point_rs_->width;
    point_pcl_->height   = point_rs_->height;
    point_pcl_->is_dense = false;
  } else {
    point_pcl_->points.reserve(point_num);
    point_pcl_->width = point_pcl_->height = 1;
    point_pcl_->is_dense                   = true;
  }
  point_pcl_->header = point_rs_->header;

  int n_threads = 4;
  omp_set_num_threads(n_threads);
#pragma omp parallel
  {
    // 每个线程自己的局部 buffer
    std::vector<pcl::PointXYZI> local_buffer;
    if (!structured) local_buffer.reserve(point_num / omp_get_num_threads());

    if (structured) {
      // --- 结构化点云 ring-major 排列 ---
#pragma omp for collapse(2) schedule(static)
      for (int j = 0; j < static_cast<int>(point_rs_->width); j++) {
        for (int i = 0; i < static_cast<int>(point_rs_->height); i++) {
          const auto& pt = point_rs_->points[j * point_rs_->height + i];
          int dst_idx    = pt.ring * point_rs_->width + j;  // ring-major 索引
          auto& dst      = point_pcl_->points[dst_idx];

          // if (!pcl::isFinite(pt) ||
          //     (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
          //     (pt.intensity == 0)) {
          //   dst.x = dst.y = dst.z = std::numeric_limits<float>::quiet_NaN();
          //   dst.intensity         = 0;
          // } else {
            dst.x         = pt.x;
            dst.y         = pt.y;
            dst.z         = pt.z;
            dst.intensity = static_cast<float>(pt.intensity);
          // }
        }
      }
    } else {
      // --- 非结构化点云原序列 ---
#pragma omp for schedule(static)
      for (size_t i = 0; i < point_num; ++i) {
        const auto& pt = (*point_rs_)[i];
        if (!pcl::isFinite(pt) || (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
            (pt.intensity == 0))
          continue;

        pcl::PointXYZI new_pt;
        new_pt.x         = pt.x;
        new_pt.y         = pt.y;
        new_pt.z         = pt.z;
        new_pt.intensity = static_cast<float>(pt.intensity);
        local_buffer.emplace_back(new_pt);
      }

#pragma omp critical
      {
        point_pcl_->points.insert(point_pcl_->points.end(),
                                  local_buffer.begin(), local_buffer.end());
      }
    }
  }  // end omp parallel

  // 非结构化模式：重新设置 width
  if (!structured) {
    point_pcl_->width  = point_pcl_->points.size();
    point_pcl_->height = 1;
  }

  return true;
}

#else

bool RsToPcl(pcl::PointCloud<robosense_ros::PointII>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_, bool structured) {
  // std::cout << "omp not enabled" << std::endl;

  size_t point_num = point_rs_->size();
  if (point_num == 0) return false;

  /* debug
  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored->resize(point_num);
  colored->width = point_rs_->width;
  colored->height = point_rs_->height;
  colored->is_dense = false;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored2(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored2->resize(point_num);
  colored2->width = point_rs_->width;
  colored2->height = point_rs_->height;
  colored2->is_dense = false;
  // height == max ring
  int max_ring = 128;
  // std::cout << "max_ring: " << max_ring << std::endl;
  // std::cout << "height: " << point_rs_->height << std::endl;
  std::vector<std::tuple<int,int,int>> colors(max_ring + 1);
  for (int r = 0; r <= max_ring; r++) {
    auto c = GetColor(r);
    colors[r] = {int(c[2]), int(c[1]), int(c[0])};
  }
  // clang-format on
  */

  point_pcl_->clear();
  if (structured) {
    point_pcl_->resize(point_num);
    // 保留结构，适合 SLAM / 地面分割 / 特征提取 / 图像投影
    // 与直接过滤相比，速度差异在毫秒级，不重要。但数据正确性非常重要。
    point_pcl_->header = point_rs_->header;
    point_pcl_->width  = point_rs_->width;
    point_pcl_->height = point_rs_->height;
    // 有 NaN 必须设为 false
    // point_pcl_->is_dense = point_rs_->is_dense;
    point_pcl_->is_dense = false;

    /* way 1 
    for (size_t i = 0; i < point_num; i++) {
      const auto& pt = point_rs_->points[i];
      auto& dst      = point_pcl_->points[i];

      bool invalid = !pcl::isFinite(pt) ||
                     (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
                     (pt.intensity == 0);

      if (invalid) {
        // 写 NaN
        dst.x = dst.y = dst.z = std::numeric_limits<float>::quiet_NaN();
        dst.intensity         = 0;
        continue;
      }

      // 有效点
      dst.x = pt.x;
      dst.y = pt.y;
      dst.z = pt.z;
      // --- 自动兼容 uint8_t ---
      dst.intensity = static_cast<float>(pt.intensity);
    }
    */
    // way 2
    for (int j = 0; j < point_rs_->width; j++) {
      for (int i = 0; i < point_rs_->height; i++) {
        const auto& pt      = point_rs_->points[j * point_rs_->height + i];
        uint16_t ring_index = pt.ring;
        // 重排后的点云 point_pcl_ 实际上是 row-major：每行对应一个 ring，每列对应原始点云的 width
        auto& dst = point_pcl_->points[ring_index * point_rs_->width + j];

        dst.x = pt.x;
        dst.y = pt.y;
        dst.z = pt.z;
        // --- 自动兼容 uint8_t ---
        dst.intensity = static_cast<float>(pt.intensity);

        /* debug
        auto& q = colored->points[ring_index * point_rs_->width + j];
        q.x = pt.x;
        q.y = pt.y;
        q.z = pt.z;
        auto [R, G, B] = colors[ring_index];
        q.r = static_cast<uint8_t>(R);
        q.g = static_cast<uint8_t>(G);
        q.b = static_cast<uint8_t>(B);
        */
      }
    }
    /* debug
    for (int j = 0; j < point_pcl_->width; j++) {
      for (int i = 0; i < point_pcl_->height; i++) {
        const auto& pt = point_pcl_->points[i * point_pcl_->width + j];

        auto& q2 = colored2->points[i * point_pcl_->width + j];
        q2.x = pt.x;
        q2.y = pt.y;
        q2.z = pt.z;
        auto [R2, G2, B2] = colors[i];
        q2.r = static_cast<uint8_t>(R2);
        q2.g = static_cast<uint8_t>(G2);
        q2.b = static_cast<uint8_t>(B2);
      }
    }
    */
  } else {
    // 非结构化模式：只保留有效点，紧凑存储
    point_pcl_->reserve(point_num);  // 预分配空间
    for (size_t i = 0; i < point_num; i++) {
      const auto& pt = point_rs_->points[i];

      bool invalid = !pcl::isFinite(pt) ||
                     (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
                     (pt.intensity == 0);
      if (invalid) continue;

      pcl::PointXYZI p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      // --- 自动兼容 uint8_t ---
      p.intensity = static_cast<float>(pt.intensity);
      point_pcl_->emplace_back(p);
    }

    // 非结构化点云宽高设置
    point_pcl_->width    = point_pcl_->size();
    point_pcl_->height   = 1;
    point_pcl_->is_dense = true;  // 已经去掉 NaN
  }

  // clang-format off
  /* debug ==== 可视化 ====
  static pcl::visualization::PCLVisualizer::Ptr viewer;
  static pcl::visualization::PCLVisualizer::Ptr viewer2;
  if (!viewer) {
    // 第一次创建
    viewer.reset(new pcl::visualization::PCLVisualizer("Colored Cloud"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud(colored, "colored_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "colored_cloud");
  } else {
    // 更新点云数据
    viewer->updatePointCloud(colored, "colored_cloud");
  }
  if (!viewer2) {
    viewer2.reset(new pcl::visualization::PCLVisualizer("Colored Cloud2"));
    viewer2->setBackgroundColor(0, 0, 0);
    viewer2->addPointCloud(colored2, "colored_cloud2");
    viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "colored_cloud2");
  } else {
    viewer2->updatePointCloud(colored2, "colored_cloud2");
  }

  // 刷新显示（可以放在循环里调用）
  viewer->spinOnce(10);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  viewer2->spinOnce(10);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  */
  // clang-format on

  return true;
}

bool RsToPcl(pcl::PointCloud<robosense_ros::PointIF>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_, bool structured) {
  size_t point_num = point_rs_->size();
  if (point_num == 0) return false;

  point_pcl_->clear();
  if (structured) {
    point_pcl_->resize(point_num);
    point_pcl_->header = point_rs_->header;
    point_pcl_->width  = point_rs_->width;
    point_pcl_->height = point_rs_->height;
    // point_pcl_->is_dense = point_rs_->is_dense;
    point_pcl_->is_dense = false;

    for (int j = 0; j < point_rs_->width; j++) {
      for (int i = 0; i < point_rs_->height; i++) {
        const auto& pt      = point_rs_->points[j * point_rs_->height + i];
        uint16_t ring_index = pt.ring;
        auto& dst = point_pcl_->points[ring_index * point_rs_->width + j];

        dst.x = pt.x;
        dst.y = pt.y;
        dst.z = pt.z;
        // --- 自动兼容 float ---
        dst.intensity = static_cast<float>(pt.intensity);
      }
    }
  } else {
    point_pcl_->reserve(point_num);
    for (size_t i = 0; i < point_num; i++) {
      const auto& pt = point_rs_->points[i];

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

#endif

bool RsToPcl(pcl::PointCloud<robosense_ros::Point>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_) {
  size_t point_num = point_rs_->size();
  if (point_num == 0) return false;

  point_pcl_->clear();
  point_pcl_->resize(point_num);
  point_pcl_->header = point_rs_->header;
  point_pcl_->width  = point_rs_->width;
  point_pcl_->height = point_rs_->height;
  // point_pcl_->is_dense = point_rs_->is_dense;
  point_pcl_->is_dense = false;

  for (size_t i = 0; i < point_num; i++) {
    const auto& pt = point_rs_->points[i];
    auto& dst      = point_pcl_->points[i];

    bool invalid =
        !pcl::isFinite(pt) || (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0);

    if (invalid) {
      // 写 NaN
      dst.x = dst.y = dst.z = std::numeric_limits<float>::quiet_NaN();
      dst.intensity         = 0;
      continue;
    }

    // 有效点
    dst.x = pt.x;
    dst.y = pt.y;
    dst.z = pt.z;
    // --- 自动兼容 ---
    dst.intensity = 0;
  }

  return true;
}

bool RsToPcl(pcl::PointCloud<robosense_ros::PointII>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZIRT>::Ptr point_pcl_) {
  // std::cout << "omp not enabled" << std::endl;

  size_t point_num = point_rs_->size();
  if (point_num == 0) return false;

  point_pcl_->clear();
  point_pcl_->resize(point_num);
  point_pcl_->header = point_rs_->header;
  point_pcl_->width  = point_rs_->width;
  point_pcl_->height = point_rs_->height;
  // point_pcl_->is_dense = point_rs_->is_dense;
  point_pcl_->is_dense = false;

  auto min_point_it = std::min_element(
      point_rs_->points.begin(), point_rs_->points.end(),
      [](const robosense_ros::PointII& a, const robosense_ros::PointII& b) {
        return a.timestamp < b.timestamp;
      });

  auto max_point_it = std::max_element(
      point_rs_->points.begin(), point_rs_->points.end(),
      [](const robosense_ros::PointII& a, const robosense_ros::PointII& b) {
        return a.timestamp < b.timestamp;
      });

  double& time_start = min_point_it->timestamp;
  double& time_end   = max_point_it->timestamp;
  double time_diff   = time_end - time_start;

  for (size_t i = 0; i < point_num; i++) {
    const auto& pt = point_rs_->points[i];
    auto& dst      = point_pcl_->points[i];

    bool invalid = !pcl::isFinite(pt) ||
                   (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
                   (pt.intensity == 0);

    if (invalid) {
      // 写 NaN
      dst.x = dst.y = dst.z = std::numeric_limits<float>::quiet_NaN();
      dst.intensity         = 0;
      dst.timestamp         = 0;
      continue;
    }

    // 有效点
    dst.x = pt.x;
    dst.y = pt.y;
    dst.z = pt.z;
    // --- 自动兼容 ---
    dst.intensity = pt.intensity;
    // 0~100
    dst.timestamp =
        static_cast<float>((pt.timestamp - time_start) / time_diff * 100.0);
  }

  return true;
}
