#ifndef SHOW_DATA_3D_H
#define SHOW_DATA_3D_H

#include <iostream>

#include <Eigen/Dense>

// #include <boost/make_shared.hpp>
// #include <boost/shared_ptr.hpp>
// #include <boost/thread/thread.hpp>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 会引发 OpenCV 和 VTK 的 detail 错误
// VTK 8.x/9.x 修复
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

// #include <opencv2/opencv.hpp>

#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/box3d_extra.h"

void show3d_lidar_data(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                       const std::string &name);

void show3d_lidar_data(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                       const std::string &viewer_name);

void show3d_lidar_data_shared(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const std::string &cloud_name);

void show3d_lidar_data_realtime(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
    const std::string &cloud_name);

void show3d_box3d_shared(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr,
    const jojo::perception::base::Point3DF (&vertex_p)[8], 
    const std::string &box_name);

#endif  // SHOW_DATA_3D_H
