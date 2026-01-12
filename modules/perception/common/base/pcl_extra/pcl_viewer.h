#ifndef PCL_VIEWER_H_
#define PCL_VIEWER_H_

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "modules/perception/common/base/opencv_extra/cv_colors.h"
#include "modules/perception/common/lidar/convert/robosense.h"

inline void viewerShow(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& colored,
    pcl::visualization::PCLVisualizer::Ptr* viewer = nullptr) {
  // 如果外部没有传 viewer，则使用内部的静态 viewer
  static pcl::visualization::PCLVisualizer::Ptr internal_viewer = nullptr;
  if (viewer == nullptr) {
    viewer = &internal_viewer;
  }

  // 解引用获得实际 viewer
  auto& v = *viewer;

  // 若为空，自动创建
  if (!v) {
    v.reset(new pcl::visualization::PCLVisualizer("Colored Cloud"));
    v->setBackgroundColor(0, 0, 0);
  }

  // ---- 创建可视化器 ----
  // clang-format off
  if (!v->updatePointCloud(colored, "colored_cloud")) {
    v->addPointCloud(colored, "colored_cloud");
  }
  v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "colored_cloud");

  // ---- 主循环 ----
  while (!v->wasStopped()) {
    v->spinOnce(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // clang-format on
}

template <typename PointT>
void show_pointcloud_ring(const pcl::PointCloud<PointT>& cloud) {
  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored->points.reserve(cloud.points.size());
  // clang-format on

  // 找最大 ring 数
  static int max_ring = -1;  // 记录是否已经初始化
  if (max_ring < 0) {
    max_ring = 0;
    for (auto& p : cloud.points) {
      if (p.ring > max_ring) max_ring = p.ring;
    }
    // max_ring = 128;
    max_ring += 1;
    std::cout << "max_ring initialized = " << max_ring << std::endl;
  }

  // ---- 颜色生成 ----
  std::vector<std::tuple<int, int, int>> ring_colors(max_ring + 1);
  for (int r = 0; r <= max_ring; r++) {
    auto c = GetColor(r);
    // Scalar 是 BGR，需要转成 RGB
    ring_colors[r] = {int(c[2]), int(c[1]), int(c[0])};
  }

  // ---- 遍历一次点云，把点丢到对应 ring ----
  for (const auto& p : cloud.points) {
    if (p.ring > max_ring) continue;

    pcl::PointXYZRGB pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;

    if (p.ring != 5) {
      continue;
    }

    auto [R, G, B] = ring_colors[p.ring];
    pt.r           = R;
    pt.g           = G;
    pt.b           = B;

    colored->points.push_back(pt);
  }
  colored->width    = colored->points.size();
  colored->height   = 1;
  colored->is_dense = false;

  viewerShow(colored);
}

template <typename PointT>
void show_pointcloud_height(const pcl::PointCloud<PointT>& cloud) {
  // 必须是 organized 点云
  if (cloud.height <= 1) {
    std::cerr << "[show_pointcloud_height] cloud.height <= 1, not organized. "
                 "Cannot color by height index.\n";
    return;
  }

  const int width  = static_cast<int>(cloud.width);
  const int height = static_cast<int>(cloud.height);

  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored->points.reserve(cloud.points.size());
  // clang-format on

  // ---- height 最大值静态初始化（只做一次） ----
  static int max_height = -1;
  if (max_height < 0) {
    max_height = height;
    std::cout << "[show_pointcloud_height] max_height initialized = "
              << max_height << std::endl;
  }

  // ---- 预生成 height 行的颜色表 ----
  std::vector<std::tuple<int, int, int>> height_colors(max_height + 1);
  for (int r = 0; r <= max_height; ++r) {
    cv::Scalar c = GetColor(r);  
    // 返回 BGR  转成 RGB
    height_colors[r] = {int(c[2]), int(c[1]), int(c[0])};
  }

  // ---- 双重遍历（row major） ----
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      // 这里用 行优先索引（row major） 是错误的，一维数组的展开不一样
      // int idx       = row * width + col;
      int idx       = col * height + row;
      const auto& p = cloud.points[idx];

      // 如果点含有无效值也可以选择写 NaN 或跳过（此处直接写）
      pcl::PointXYZRGB q;
      q.x = p.x;
      q.y = p.y;
      q.z = p.z;

      int r = row;
      if (r > max_height) r = max_height;

      if (row != 5) {
        continue;
      }

      auto [R, G, B] = height_colors[r];
      q.r = static_cast<uint8_t>(R);
      q.g = static_cast<uint8_t>(G);
      q.b = static_cast<uint8_t>(B);

      colored->points.push_back(q);
    }
  }
  colored->width    = colored->points.size();
  colored->height   = 1;
  colored->is_dense = false;

  viewerShow(colored);
}

enum class ColorMode { RING, AZIMUTH, RANGE, TIMESTAMP };

template <typename PointT>
void show_pointcloud_strategy(const pcl::PointCloud<PointT>& cloud,
                              ColorMode mode = ColorMode::RANGE) {
  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored->points.resize(cloud.points.size());
  // clang-format on

  // ==== 预处理：max ring ====
  static int max_ring = -1;

  // ==== 预处理：timestamp 范围 ====
  double tmin = 1e9, tmax = -1e9;

  if (mode == ColorMode::RING) {
    if (max_ring < 0) {
      max_ring = 0;
      for (auto& p : cloud.points) {
        if (p.ring > max_ring) max_ring = p.ring;
      }
      // max_ring = 128;
      std::cout << "max_ring initialized = " << max_ring << std::endl;
    }
  } else if (mode == ColorMode::AZIMUTH) {
  } else if (mode == ColorMode::RANGE) {
  } else if (mode == ColorMode::TIMESTAMP) {
    if (mode == ColorMode::TIMESTAMP) {
      for (auto& p : cloud.points) {
        tmin = std::min(tmin, (double)p.timestamp);
        tmax = std::max(tmax, (double)p.timestamp);
      }
    }
  } else {
    // TODO: 根据强度进行着色
  }

  // ==== 一次遍历点云 ====
  for (size_t i = 0; i < cloud.points.size(); i++) {
    const auto& p = cloud.points[i];
    auto& c       = colored->points[i];

    c.x = p.x;
    c.y = p.y;
    c.z = p.z;

    uint8_t R = 128, G = 128, B = 128;

    // -------------------------------------------------
    //  1) 按 ring 显示
    // -------------------------------------------------
    if (mode == ColorMode::RING) {
      auto c = GetColor(p.ring);
      R      = c[2];
      G      = c[1];
      B      = c[0];
    }

    // -------------------------------------------------
    //  2) 按 azimuth 水平角显示
    // -------------------------------------------------
    else if (mode == ColorMode::AZIMUTH) {
      float azimuth = atan2(p.y, p.x);  // -pi ~ pi
      float ratio   = (azimuth + M_PI) / (2 * M_PI);  // 0~1
      R             = int(255 * ratio);
      G             = int(255 * (1 - ratio));
      B             = 255 - R;
    }

    // -------------------------------------------------
    //  3) 按距离 RANGE heatmap
    // -------------------------------------------------
    else if (mode == ColorMode::RANGE) {
      float range = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      float ratio = std::min(range / 100.0f, 1.0f);  // 100米封顶

      R = int(255 * ratio);
      G = int(255 * (1 - ratio));
      B = 128;
    }

    // -------------------------------------------------
    //  4) 按 timestamp 显示扫描顺序
    // -------------------------------------------------
    else if (mode == ColorMode::TIMESTAMP) {
      double ratio = (p.timestamp - tmin) / (tmax - tmin + 1e-9);
      R            = int(255 * ratio);
      G            = 50;
      B            = int(255 * (1 - ratio));
    }

    // 赋值
    c.r = R;
    c.g = G;
    c.b = B;
  }
  colored->width    = colored->points.size();
  colored->height   = 1;
  colored->is_dense = false;

  viewerShow(colored);
}

#endif
