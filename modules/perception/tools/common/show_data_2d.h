#ifndef SHOW_DATA_2D_H
#define SHOW_DATA_2D_H

#include <iostream>
#include <string>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>

#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/pcl_extra/point_types.h"

namespace base = jojo::perception::base;

// clang-format off
void show2d_lidar_data(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                       const uint& idx = 0, const uint& mode = 0,
                       const std::string& name = "lidar",
                       cv::Mat* ext_img = nullptr);

void show2d_lidar_data(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const uint& idx = 0, const uint& mode = 0,
                       const std::string& name = "lidar",
                       cv::Mat* ext_img = nullptr);

void show2d_lidar_bev(const base::Point3DF p[8] /*vertex*/,
                      const Eigen::Vector3f& center,
                      cv::Scalar color = cv::Scalar(0, 97, 0),
                      cv::Mat* ext_img = nullptr);
// clang-format on

// void show2d_lidar_data(const base::Frame* frame);

// for depth image color
void show2d_camera_data(const cv::Mat& image_float, const int max_depth);

void show2d_camera_data(const cv::Mat& image, const uint& idx = 0,
                        const std::string& name = "camera");

// void show2d_radar_data(const base::Frame* frame);

// void show2d_segmentation_data(const base::Frame* frame);

// void show2d_tracking_data(const base::Frame* frame);

// void show2d_detection_data(const base::Frame* frame);

// void show2d_perception_data(const base::Frame* frame);

// Mat 必须是连续内存
void show_cv_splits_cloud(std::vector<cv::Mat>& splits);

int display_image_pause(const std::string& window_title, const cv::Mat& image);

#endif  // SHOW_DATA_2D_H