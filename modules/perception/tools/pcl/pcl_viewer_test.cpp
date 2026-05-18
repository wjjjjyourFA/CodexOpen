#include <thread>
#include <chrono>
#include <iostream>
#include <algorithm>

#include <pcl/io/pcd_io.h>

#include "modules/perception/tools/pcl/point_types.h"
#include "modules/perception/tools/pcl/pcl_viewer.h"

// 将 timestamp 映射到 RGB 蓝→红渐变
void TimestampToRGB(double ts, double ts_min, double ts_max, float& r, float& g,
                    float& b) {
  double t = (ts - ts_min) / (ts_max - ts_min);
  // 限制范围
  t = std::min(std::max(t, 0.0), 1.0);

  // 简单线性渐变：蓝 → 红
  r = static_cast<float>(t);
  g = 0.0f;
  b = static_cast<float>(1.0 - t);
}

void play_pointcloud_timestamp_animation(
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud,
    double dt = 0.05 /*秒每帧*/, const std::string& name = "cloud") {
  if (!cloud || cloud->empty()) return;

  float t_min = cloud->points.front().curvature;
  float t_max = cloud->points.front().curvature;
  for (auto& p : cloud->points) {
    t_min = std::min(t_min, p.curvature);
    t_max = std::max(t_max, p.curvature);
  }
  std::cout << "Timestamp range: " << t_min << " ~ " << t_max << std::endl;

  // clang-format off
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Timestamp Animation"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  frame_cloud->points.reserve(cloud->points.size());
  // clang-format on

  std::vector<bool> colored(cloud->points.size(), false);

  double t_step = 1.0;  // 每帧显示 timestamp 步长
  for (double t = t_min; t <= t_max; t += t_step) {
    frame_cloud->clear();
    bool any_new   = false;
    bool once_flag = true;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      // 已经上色的点跳过
      // if (colored[i]) continue;

      auto& p = cloud->points[i];
      // auto& pt = frame_cloud->points[i];

      // 判断是否在当前时间区间 [t, t+t_step)
      if (p.curvature >= t && p.curvature < t + t_step) {
        if (once_flag) {
          std::cout << "t: " << p.curvature << std::endl;
          once_flag = false;
        }

        pcl::PointXYZRGB pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;

        float r, g, b;
        TimestampToRGB(cloud->points[i].curvature, t_min, t_max, r, g, b);
        pt.r = static_cast<uint8_t>(r * 255);
        pt.g = static_cast<uint8_t>(g * 255);
        pt.b = static_cast<uint8_t>(b * 255);

        frame_cloud->points.push_back(pt);

        colored[i] = true;  // 标记：此点已上色
        any_new    = true;
      }
    }

    if (any_new) {
      // viewer->updatePointCloud(frame_cloud, name);
      viewer->removeAllPointClouds();
      viewer->addPointCloud(frame_cloud, name);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    }

    viewer->spinOnce(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));

    if (viewer->wasStopped()) break;
  }

  // 保持 Viewer 打开
  while (!viewer->wasStopped()) {
    viewer->spinOnce(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void play_pointcloud_timestamp_animation(
    pcl::PointCloud<pcl::PointXYZIRT>::Ptr cloud, double dt = 0.05 /*秒每帧*/,
    const std::string& name = "cloud") {
  if (!cloud || cloud->empty()) return;

  double t_min = cloud->points.front().timestamp;
  double t_max = cloud->points.front().timestamp;
  for (auto& p : cloud->points) {
    t_min = std::min(t_min, p.timestamp);
    t_max = std::max(t_max, p.timestamp);
  }
  std::cout << "Timestamp range: " << t_min << " ~ " << t_max << std::endl;

  // clang-format off
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Timestamp Animation"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addCoordinateSystem(1.0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  frame_cloud->points.reserve(cloud->points.size());

  /* 初始全部灰色
  frame_cloud->points.resize(cloud->points.size());
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    auto& p  = cloud->points[i];
    auto& pt = frame_cloud->points[i];
    pt.x     = p.x;
    pt.y     = p.y;
    pt.z     = p.z;
    pt.r     = 30;
    pt.g     = 30;
    pt.b     = 30;
  }

  viewer->addPointCloud(frame_cloud, name);
  // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
  */
  // clang-format on

  // 记录每个点是否已经上色，上色后不再重复赋值
  std::vector<bool> colored(cloud->points.size(), false);

  // 动画循环
  double t_step = 1.0;  // 每帧显示 timestamp 步长
  for (double t = t_min; t <= t_max; t += t_step) {
    frame_cloud->clear();
    bool any_new   = false;
    bool once_flag = true;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      // 已经上色的点跳过
      // if (colored[i]) continue;

      auto& p = cloud->points[i];
      // auto& pt = frame_cloud->points[i];

      // if (p.timestamp <= t) {
      // 判断是否在当前时间区间 [t, t+t_step)
      if (p.timestamp >= t && p.timestamp < t + t_step) {
        if (once_flag) {
          std::cout << "t: " << p.timestamp << std::endl;
          once_flag = false;
        }

        pcl::PointXYZRGB pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = p.z;

        // ★ 只在点第一次"出现"时赋色，之后永远不再改变
        // /* way 1
        float r, g, b;
        TimestampToRGB(cloud->points[i].timestamp, t_min, t_max, r, g, b);
        pt.r = static_cast<uint8_t>(r * 255);
        pt.g = static_cast<uint8_t>(g * 255);
        pt.b = static_cast<uint8_t>(b * 255);
        // */
        /* way 2
        int r = p.ring;
        pt.r  = static_cast<uint8_t>((r * 50) % 255);
        pt.g  = static_cast<uint8_t>((r * 80) % 255);
        pt.b  = static_cast<uint8_t>((r * 120) % 255);
        */
        frame_cloud->points.push_back(pt);

        colored[i] = true;  // 标记：此点已上色
        any_new    = true;
      }
    }

    if (any_new) {
      // viewer->updatePointCloud(frame_cloud, name);
      viewer->removeAllPointClouds();
      viewer->addPointCloud(frame_cloud, name);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
    }

    viewer->spinOnce(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));

    if (viewer->wasStopped()) break;
  }

  // 保持 Viewer 打开
  while (!viewer->wasStopped()) {
    viewer->spinOnce(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

int main() {
  std::string pcd_file = "path/1769563898747.pcd";

  // clang-format off
  pcl::PointCloud<pcl::PointXYZIRT>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZIRT>);
  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
  // clang-format on

  int ret = pcl::io::loadPCDFile(pcd_file, *cloud);
  if (ret == -1) {
    std::cerr << "[Warning] Failed to load PCD file: " << std::endl;
  }

  // show_pointcloud_timestamp<pcl::PointXYZIRT>(cloud);
  play_pointcloud_timestamp_animation(cloud, 0.02);  // 0.02 ==> 50 FPS 模拟扫描

  return 0;
}