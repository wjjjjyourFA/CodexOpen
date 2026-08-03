#include "modules/perception/common/lidar/convert/robosense.h"

// 只要你的构建系统启用了 OpenMP，宏 _OPENMP 就会定义
// Qt/CMake
#ifdef _OPENMP
#include <omp.h>

#include <atomic>
#endif

#include <vector>

/* row-major 的“行排列”（标准 SLAM 形式）
 * 行 = ring（激光线）
 * 列 = azimuth（水平扫描）
 */

/* 注意这里只是进行了 行 ==> ring 的排列
 * 并不代表 height 0 ==> ring 0 || height 2 ==> ring 2
 * ring 的线束重排，需要依据雷达配置，通过 SortMap 操作
 */
#ifdef _OPENMP

bool RsToPcl(pcl::PointCloud<robosense_ros::PointII>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_, bool structured) {
  // std::cout << "omp enabled" << std::endl;

  size_t point_num = point_rs_->size();
  if (point_num == 0) return false;

  point_pcl_->clear();
  if (structured) {
    // --- 保留结构 ---
    point_pcl_->points.resize(point_num);
    point_pcl_->width    = point_rs_->width;
    point_pcl_->height   = point_rs_->height;
    point_pcl_->is_dense = false;
    // std::cout << "height: " << point_pcl_->height << std::endl;
    // std::cout << "width: " << point_pcl_->width << std::endl;
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
          if (pt.ring < 0 || pt.ring >= point_pcl_->height) continue;
          int dst_idx = pt.ring * point_rs_->width + j;  // ring-major 索引
          auto& dst   = point_pcl_->points[dst_idx];

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
    point_pcl_->points.resize(point_num);
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
          if (pt.ring < 0 || pt.ring >= point_pcl_->height) continue;
          int dst_idx = pt.ring * point_rs_->width + j;  // ring-major 索引
          auto& dst   = point_pcl_->points[dst_idx];

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
  colored->points.reserve(point_num);
  colored->width = point_rs_->width;
  colored->height = point_rs_->height;
  colored->is_dense = false;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored2(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored2->points.reserve(point_num);
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
    point_pcl_->points.resize(point_num);
    // 保留结构，适合 SLAM / 地面分割 / 特征提取 / 图像投影
    // 与直接过滤相比，速度差异在毫秒级，不重要。但数据正确性非常重要。
    point_pcl_->header = point_rs_->header;
    point_pcl_->width  = point_rs_->width;
    point_pcl_->height = point_rs_->height;
    // 有 NaN 必须设为 false
    // point_pcl_->is_dense = point_rs_->is_dense;
    point_pcl_->is_dense = false;
    // std::cout << "height: " << point_pcl_->height << std::endl;
    // std::cout << "width: " << point_pcl_->width << std::endl;

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
    for (int j = 0; j < point_rs_->width; j++) {  // 1800
      for (int i = 0; i < point_rs_->height; i++) {  // 128
        const auto& pt = point_rs_->points[j * point_rs_->height + i];
        if (pt.ring < 0 || pt.ring >= point_pcl_->height) continue;
        uint16_t ring_index = pt.ring;
        // 重排后的点云 point_pcl_ 实际上是 row-major：每行对应一个 ring，每列对应原始点云的 width
        auto& dst = point_pcl_->points[ring_index * point_rs_->width + j];

        dst.x = pt.x;
        dst.y = pt.y;
        dst.z = pt.z;
        // --- 自动兼容 uint8_t ---
        dst.intensity = static_cast<float>(pt.intensity);

        /* debug
        // 显示 ring == 2 的点云
        if (ring_index != 2) {
          continue;
        }
        pcl::PointXYZRGB q;
        q.x = pt.x;
        q.y = pt.y;
        q.z = pt.z;
        auto [R, G, B] = colors[ring_index];
        q.r = static_cast<uint8_t>(R);
        q.g = static_cast<uint8_t>(G);
        q.b = static_cast<uint8_t>(B);
        colored->points.push_back(q);
        */
      }
    }

    /* way 3
    auto& Map128 = GetMap128();
    for (int j = 0; j < point_rs_->width; j++) {
      for (int i = 0; i < point_rs_->height; i++) {
        // sort the lidar by increasing elevation angle
        point_pcl_->points[Map128.laser_sort[j] * point_rs_->width + i] =
            point_rs_->points[i * point_rs_->height + j];
      }
    }
    */

    /* way 4
    for (size_t k = 0; k < point_rs_->points.size(); k++) {
      const auto& pt = point_rs_->points[k];

      // -------- 安全检查 --------
      if (!pcl::isFinite(pt)) continue;
      if (pt.ring < 0 || pt.ring >= point_pcl_->height) continue;

      // -------- 计算 azimuth index --------
      float angle = atan2(pt.y, pt.x);  // -pi ~ pi
      int col =
          static_cast<int>((angle + M_PI) / (2 * M_PI) * point_pcl_->width);

      if (col < 0) col = 0;
      if (col >= point_pcl_->width) col = point_pcl_->width - 1;

      int row = pt.ring;
      int idx = row * point_pcl_->width + col;

      auto& dst = point_pcl_->points[idx];

      // -------- 冲突处理（保留最近点）--------
      float new_range = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;

      if (!pcl::isFinite(dst)) {
        dst.x = pt.x;
        dst.y = pt.y;
        dst.z = pt.z;
        // --- 自动兼容 uint8_t ---
        dst.intensity = static_cast<float>(pt.intensity);
      } else {
        float old_range = dst.x * dst.x + dst.y * dst.y + dst.z * dst.z;
        if (new_range < old_range) {
          dst.x = pt.x;
          dst.y = pt.y;
          dst.z = pt.z;
          // --- 自动兼容 uint8_t ---
          dst.intensity = static_cast<float>(pt.intensity);
        }
      }
    }
    */

    /* debug
    for (int j = 0; j < point_pcl_->width; j++) {
      for (int i = 0; i < point_pcl_->height; i++) {
        const auto& pt = point_pcl_->points[i * point_pcl_->width + j];

        if (i != 2) {
          continue;
        }
        pcl::PointXYZRGB q2;
        q2.x = pt.x;
        q2.y = pt.y;
        q2.z = pt.z;
        auto [R2, G2, B2] = colors[i];
        q2.r = static_cast<uint8_t>(R2);
        q2.g = static_cast<uint8_t>(G2);
        q2.b = static_cast<uint8_t>(B2);
        colored2->points.push_back(q2);
      }
    }
    */
  } else {
    // 非结构化模式：只保留有效点，紧凑存储
    point_pcl_->points.reserve(point_num);  // 预分配空间
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
  static pcl::visualization::PCLVisualizer::Ptr viewer1;
  static pcl::visualization::PCLVisualizer::Ptr viewer2;
  if (!viewer1) {
    // 第一次创建
    viewer1.reset(new pcl::visualization::PCLVisualizer("Colored Cloud"));
    viewer1->setBackgroundColor(0, 0, 0);
    viewer1->addPointCloud(colored, "colored_cloud");
    viewer1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "colored_cloud");
  } else {
    // 更新点云数据
    viewer1->updatePointCloud(colored, "colored_cloud");
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
  viewer1->spinOnce(10);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  viewer2->spinOnce(10);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  cv::waitKey(0);
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
    point_pcl_->points.resize(point_num);
    point_pcl_->header = point_rs_->header;
    point_pcl_->width  = point_rs_->width;
    point_pcl_->height = point_rs_->height;
    // point_pcl_->is_dense = point_rs_->is_dense;
    point_pcl_->is_dense = false;

    for (int j = 0; j < point_rs_->width; j++) {
      for (int i = 0; i < point_rs_->height; i++) {
        const auto& pt = point_rs_->points[j * point_rs_->height + i];
        if (pt.ring < 0 || pt.ring >= point_pcl_->height) continue;
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
    point_pcl_->points.reserve(point_num);
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
  point_pcl_->points.resize(point_num);
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

bool RsToPcl(pcl::PointCloud<robosense_ros::PointII>::Ptr point_in_,
             pcl::PointCloud<pcl::PointXYZIRT>::Ptr point_out_,
             bool structured) {
  // std::cout << "omp not enabled" << std::endl;

  size_t point_num = point_in_->size();
  if (point_num == 0) return false;

  /* way 1
  auto min_point_it = std::min_element(
      point_in_->points.begin(), point_in_->points.end(),
      [](const robosense_ros::PointII& a, const robosense_ros::PointII& b) {
        return a.timestamp < b.timestamp;
      });

  auto max_point_it = std::max_element(
      point_in_->points.begin(), point_in_->points.end(),
      [](const robosense_ros::PointII& a, const robosense_ros::PointII& b) {
        return a.timestamp < b.timestamp;
      });

  double& time_start = min_point_it->timestamp;
  double& time_end   = max_point_it->timestamp;
  double time_diff   = time_end - time_start;
  */

  double time_start = std::numeric_limits<double>::max();
  double time_end   = std::numeric_limits<double>::lowest();

  point_out_->clear();
  if (structured) {
    // 获取 sortMap（128线映射表），仅在 structured 模式使用
    // GetMap128().laser_sort[physical_ring] => sorted_row_index
    const auto& smap = GetMap128();

    point_out_->points.resize(point_num);
    point_out_->header = point_in_->header;
    point_out_->width  = point_in_->width;
    point_out_->height = point_in_->height;
    // point_out_->is_dense = point_in_->is_dense;
    point_out_->is_dense = false;

    /* 用 NaN 初始化，保证未填位置可识别
    for (auto& p : point_out_->points) {
      p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
      p.intensity      = 0.0f;
      p.ring           = 0;
      p.timestamp      = 0.0f;
    }
    */

    for (int j = 0; j < point_in_->width; j++) {  // 1800
      for (int i = 0; i < point_in_->height; i++) {  // 128
        const auto& pt = point_in_->points[j * point_in_->height + i];
        // ring 越界保护
        if (pt.ring < 0 || pt.ring >= point_out_->height) continue;
        // uint16_t ring_index = pt.ring;
        // sortMap 重映射：物理 ring → 仰角升序的行索引
        uint16_t ring_index = smap.laser_sort[pt.ring];
        auto& dst = point_out_->points[ring_index * point_in_->width + j];

        // 如果 xyz 有效且不全为 0，才赋值
        // bool has_valid_xyz = std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);
        bool has_valid_xyz = std::isfinite(pt.x);
        // bool has_valid_xyz = !(pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0);

        // 先清零（包括 padding）
        // memset(&dst, 0, sizeof(dst));

        if (has_valid_xyz) {
          dst.x = pt.x;
          dst.y = pt.y;
          dst.z = pt.z;
          // --- 自动兼容 uint8_t ---
          dst.intensity = static_cast<float>(pt.intensity);
          // 写排序后行号
          dst.ring      = ring_index;
          dst.timestamp = static_cast<double>(pt.timestamp);
        } else {
          // 无效点：一次性清零（推荐）
          dst = {};  // 等价 memset，但只发生在无效点
        }

        if (pt.timestamp < time_start) time_start = pt.timestamp;
        if (pt.timestamp > time_end) time_end = pt.timestamp;
      }
    }

    const double time_diff = time_end - time_start;
    /*
    std::cout << std::fixed << std::setprecision(9)
              << "time_start: " << time_start << " time_end: " << time_end
              << " time_diff: " << time_diff << std::endl;
    */

    if (time_diff > 1e-9) {
      const double inv_diff = 100.0 / time_diff;
      for (auto& p : point_out_->points) {
        // 真实点的 时间戳 一定大于 0
        if (p.timestamp > 0) {
          // p.timestamp = static_cast<float>((p.timestamp - time_start) * inv_diff);
          p.timestamp = (p.timestamp - time_start) * inv_diff;
        }
      }
    }
  } else {
    point_out_->points.reserve(point_num);
    for (size_t i = 0; i < point_num; i++) {
      const auto& pt = point_in_->points[i];

      bool invalid = !pcl::isFinite(pt) ||
                     (pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0) ||
                     (pt.intensity == 0);
      if (invalid) continue;

      // 有效点
      pcl::PointXYZIRT p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      // --- 自动兼容 ---
      p.intensity = pt.intensity;
      p.ring      = pt.ring;
      // 先写原始值
      p.timestamp = static_cast<float>(pt.timestamp);
      point_out_->emplace_back(p);

      // 追踪 min / max
      if (pt.timestamp < time_start) time_start = pt.timestamp;
      if (pt.timestamp > time_end) time_end = pt.timestamp;
    }

    // 归一化时间戳到 0~100 帧内时间戳
    const double time_diff = time_end - time_start;
    if (time_diff > 1e-9) {
      const double inv_diff = 100.0 / time_diff;
      for (auto& p : point_out_->points) {
        // p.timestamp = static_cast<float>((p.timestamp - time_start) * inv_diff);
        p.timestamp = (p.timestamp - time_start) * inv_diff;
      }
    }

    point_out_->width    = point_out_->size();
    point_out_->height   = 1;
    point_out_->is_dense = true;
  }

  return true;
}
