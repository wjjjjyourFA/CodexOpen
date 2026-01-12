#ifndef __ROBOSENSE_H__
#define __ROBOSENSE_H__

#include <bits/stdc++.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "modules/perception/common/base/opencv_extra/cv_colors.h"
#include "modules/perception/common/base/pcl_extra/point_types.h"

/* 保留点云的结构，将 intensity 类型改为 float，点云类型改为 pcl::PointXYZI */
// RoboSense / 速腾 Lidar 的 ring 字段 = 真实物理通道编号（从上到下顺序）

// clang-format off
/**** struct of the rslidar_sdk cloud ****/
namespace robosense_ros {
// 定义 Robosense 点云类型 Point
// 带有 XYZ、Intensity、Ring 和 Timestamp 的点云类型

// intensity 用 uint8_t
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  // std::uint16_t ring = 0;
  // double timestamp = 0.0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// intensity 用 uint8_t
struct EIGEN_ALIGN16 PointII {
  PCL_ADD_POINT4D;
  std::uint8_t intensity = 0;
  std::uint16_t ring = 0;
  double timestamp = 0.0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// intensity 用 float
struct EIGEN_ALIGN16 PointIF {
  PCL_ADD_POINT4D;
  float intensity = 0.0;
  std::uint16_t ring = 0;
  double timestamp = 0.0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace robosense_ros

// namespace robosense_ros 注册pcl数据类型
POINT_CLOUD_REGISTER_POINT_STRUCT(
    robosense_ros::Point,
    (float, x, x)(float, y, y)(float, z, z))
    // (std::uint16_t, ring, ring)
    // (double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    robosense_ros::PointII,
    (float, x, x)(float, y, y)(float, z, z)
    (std::uint8_t, intensity, intensity)(std::uint16_t, ring, ring)
    (double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    robosense_ros::PointIF,
    (float, x, x)(float, y, y)(float, z, z)
    (float, intensity, intensity)(std::uint16_t, ring, ring)
    (double, timestamp, timestamp))
// clang-format on

// convert robosense pointcloud to pcl pointcloud
bool RsToPcl(pcl::PointCloud<robosense_ros::Point>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_);

bool RsToPcl(pcl::PointCloud<robosense_ros::PointII>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_,
             bool structured = true);

bool RsToPcl(pcl::PointCloud<robosense_ros::PointIF>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZI>::Ptr point_pcl_,
             bool structured = true);

// 数据集构建
bool RsToPcl(pcl::PointCloud<robosense_ros::PointIF>::Ptr point_rs_,
             pcl::PointCloud<pcl::PointXYZIRT>::Ptr point_pcl_);

#endif
