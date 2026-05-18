#ifndef SHOW_DATA_3D_H
#define SHOW_DATA_3D_H

#include <iostream>

#include <Eigen/Dense>

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 会引发 OpenCV 和 VTK 的 detail 错误
// VTK 8.x/9.x 修复
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

// #include <opencv2/opencv.hpp>

#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/box3d_extra.h"

namespace base = jojo::perception::base;

void show3d_lidar_data(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                       const std::string &name);

void show3d_lidar_data(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                       const std::string &viewer_name);

void show3d_lidar_data_shared(
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const std::string &cloud_name);

void show3d_lidar_data_realtime(
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
    const std::string &cloud_name);

void show3d_box3d_shared(
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr,
    const base::Point3DF (&vertex_p)[8], const std::string &box_name);

#endif  // SHOW_DATA_3D_H
