#ifndef PERCEPTION_TOOLS_PCL_VIEWER_H
#define PERCEPTION_TOOLS_PCL_VIEWER_H

#include <thread>

#define PCL_NO_PRECOMPILE
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
// pcl > 1.13
// #include <pcl/common/properties.h>  // getFieldIndex
#include <pcl/register_point_struct.h>

#include "modules/perception/tools/opencv/cv_colors.h"
#include "modules/perception/tools/pcl/show_pcd_head.h"
// #include "modules/perception/common/lidar/convert/robosense.h"
#include "modules/perception/tools/pcl/point_types.h"
#include "modules/perception/tools/pcl/viewer_runner.h"

void SpinViewer(pcl::visualization::PCLVisualizer::Ptr viewer);

template <typename PointT>
void ViewerShowCloud(
    const typename pcl::PointCloud<PointT>::Ptr& cloud,
    /*可外部创建传入*/ pcl::visualization::PCLVisualizer::Ptr* viewer = nullptr,
    const std::string& name = "cloud") {
  if (!cloud || cloud->empty()) {
    return;
  }

  // 未传 viewer 时，本次阻塞调用使用局部 viewer，不跨调用共享 GUI 状态。
  pcl::visualization::PCLVisualizer::Ptr internal_viewer;
  if (viewer == nullptr) {
    viewer = &internal_viewer;
  }

  // 解引用获得实际 viewer
  auto& v = *viewer;

  // 若为空，自动创建
  if (!v) {
    v.reset(new pcl::visualization::PCLVisualizer("Viewer"));
    v->setBackgroundColor(0, 0, 0);
    v->addCoordinateSystem(1.0);
  }

  // ---- 创建可视化器 ----
  // clang-format off
  if (!v->updatePointCloud(cloud, name)) {
    v->addPointCloud(cloud, name);
  }
  v->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
  // clang-format on

  // ---- 主循环 ----
  /*
  while (!v->wasStopped()) {
    v->spinOnce(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  */
  SpinViewer(v);
}

template <typename PointT>
void show_pointcloud_ring(const pcl::PointCloud<PointT>& cloud,
                          int target_ring = -1) {
  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored->points.reserve(cloud.points.size());
  // clang-format on

  // ---- 遍历一次点云，把点丢到对应 ring ----
  for (const auto& p : cloud.points) {
    if (!pcl::isFinite(p)) continue;

    pcl::PointXYZRGB pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;

    // 用 ring 上色
    int r = p.ring;
    if (target_ring != -1) {
      if (r != target_ring) {
        continue;
      }
    }
    pt.r = (r * 50) % 255;
    pt.g = (r * 80) % 255;
    pt.b = (r * 120) % 255;

    colored->points.push_back(pt);
  }
  colored->width    = colored->points.size();
  colored->height   = 1;
  colored->is_dense = false;

  ViewerShowCloud<pcl::PointXYZRGB>(colored);
}

template <typename PointT>
void show_pointcloud_height(const pcl::PointCloud<PointT>& cloud,
                            int target_height = -1) {
  // 必须是 organized 点云
  if (cloud.height <= 1) {
    std::cerr << "[show_pointcloud_height] cloud.height <= 1, not organized. "
                 "Cannot color by height index.\n";
    return;
  }

  const int width  = static_cast<int>(cloud.width);
  const int height = static_cast<int>(cloud.height);
  const auto expected_size =
      static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (cloud.points.size() < expected_size) {
    std::cerr
        << "[show_pointcloud_height] inconsistent organized cloud size.\n";
    return;
  }

  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored->points.reserve(cloud.points.size());
  // clang-format on

  // ---- 双重遍历（row major） ----
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      // 这里用 行优先索引（row major），一维数组的展开不一样
      int idx = row * width + col;
      // int idx = col * height + row;
      const auto& p = cloud.points[idx];

      // 如果点含有无效值也可以选择写 NaN 或跳过（此处直接写）
      pcl::PointXYZRGB pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;

      // 用 height 上色
      int r = row;
      if (target_height != -1) {
        if (r != target_height) {
          continue;
        }
      }
      pt.r = (r * 50) % 255;
      pt.g = (r * 80) % 255;
      pt.b = (r * 120) % 255;

      colored->points.push_back(pt);
    }
  }
  colored->width    = colored->points.size();
  colored->height   = 1;
  colored->is_dense = false;

  ViewerShowCloud<pcl::PointXYZRGB>(colored);
}

template <typename PointT>
void show_pointcloud_num(const pcl::PointCloud<PointT>& cloud,
                         int target_num = -1) {
  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored->points.reserve(cloud.points.size());
  // clang-format on

  int count = 0;
  for (const auto& p : cloud.points) {
    if (target_num >= 0 && count >= target_num) {
      break;
    }

    // 用 ring 上色
    int r = 1;

    pcl::PointXYZRGB pt;
    pt.x = p.x;
    pt.y = p.y;
    pt.z = p.z;
    pt.r = (r * 50) % 255;
    pt.g = (r * 80) % 255;
    pt.b = (r * 120) % 255;

    colored->points.push_back(pt);
    ++count;
  }
  colored->width    = colored->points.size();
  colored->height   = 1;
  colored->is_dense = false;

  ViewerShowCloud<pcl::PointXYZRGB>(colored);
}

enum class ColorMode { RING, AZIMUTH, RANGE, TIMESTAMP };

template <typename PointT>
void show_pointcloud_strategy(const pcl::PointCloud<PointT>& cloud,
                              ColorMode mode = ColorMode::RANGE) {
  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
  colored->points.resize(cloud.points.size());
  // clang-format on

  // ==== 预处理：timestamp 范围 ====
  double tmin = 1e9, tmax = -1e9;

  if (mode == ColorMode::RING) {
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

  ViewerShowCloud<pcl::PointXYZRGB>(colored);
}

template <typename PointT>
void show_pointcloud_timestamp(
    const typename pcl::PointCloud<PointT>::Ptr& cloud) {
  if (!cloud || cloud->empty()) {
    return;
  }

  /* pcl > 1.13
  auto idx = pcl::getFieldIndex<PointT>(cloud, "timestamp");
  if (!idx) {
    std::cerr << "PointT does not have field named 'timestamp'!" << std::endl;
    return;
  }
  */

  // 创建 Viewer
  // clang-format off
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
  // v.reset(new pcl::visualization::PCLVisualizer("Viewer"));
  // clang-format on

  // 使用 timestamp 字段自定义颜色
  // 会把字段值映射到 蓝→红渐变，默认会自动归一化
  // clang-format off
  // only support int / float / uint8_t / uint16_t
  // pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloud, "timestamp");
  // viewer->addPointCloud(cloud, color_handler, "cloud");

  // 使用 lambda 或手动 copy 点云 intensity 或 rgb
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
  tmp->points.resize(cloud->points.size());

  for (size_t i = 0; i < cloud->points.size(); i++) {
      tmp->points[i].x = cloud->points[i].x;
      tmp->points[i].y = cloud->points[i].y;
      tmp->points[i].z = cloud->points[i].z;
      // 用 intensity 显示
      tmp->points[i].intensity = static_cast<float>(cloud->points[i].timestamp); 
  }
  viewer->addPointCloud(tmp, pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>(tmp, "intensity"), "cloud");
  // clang-format on

  // 运行 Viewer
  viewer->spin();
}

#endif
