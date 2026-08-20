#ifndef SHOW_DATA_2D_H
#define SHOW_DATA_2D_H

#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "modules/perception/common/base/point.h"
#include "modules/perception/tools/common/bev_projector.h"
#include "modules/perception/tools/opencv/common.h"
#include "modules/perception/tools/pcl/common.h"

// clang-format off
void show2d_lidar_data(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                       const uint& idx = 0, const uint& mode = 0,
                       const std::string& name = "lidar",
                       cv::Mat* ext_img = nullptr);

void show2d_lidar_data(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const uint& idx = 0, const uint& mode = 0,
                       const std::string& name = "lidar",
                       cv::Mat* ext_img = nullptr);

template <typename PointT>
void show2d_lidar_data_normal(const typename pcl::PointCloud<PointT>::Ptr& cloud, 
                              const uint& idx = 0, const uint& mode = 0,
                              const std::string& name = "lidar",
                              cv::Mat* ext_img = nullptr) {
  if (!cloud || cloud->empty()) return;

  int width  = 1024;
  int height = 768;
  // static int width  = 1920;
  // static int height = 1080;

  static constexpr float resolution = 100 / 20.0f;

  cv::Mat LidarImage;
  if (ext_img != nullptr) {
    if (!jojo::perception::tools::IsValidBgrImage(*ext_img)) {
      std::cerr << "ext_img must be a non-empty CV_8UC3 image" << std::endl;
      return;
    }
    LidarImage = *ext_img;
    width      = LidarImage.cols;
    height     = LidarImage.rows;
  } else {
    LidarImage = cv::Mat::zeros(height, width, CV_8UC3);
  }
  const int width_half  = width / 2;
  const int height_half = height / 2;
  const jojo::perception::tools::BevProjector projector(
      jojo::perception::tools::BevRenderConfig{width, height, resolution});

  for (size_t i = 0; i < cloud->size(); i++) {
    jojo::perception::tools::BevPixel pixel;
    if (projector.Project(cloud->points[i].x, cloud->points[i].y, &pixel)) {
      cv::Vec3b color(0, 97, static_cast<uchar>(cloud->points[i].z));

      if (mode == 0) {
        // LidarImage.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 97, 255);
        LidarImage.at<cv::Vec3b>(pixel.y, pixel.x) = color;
      } else if (mode == 1) {
        cv::circle(LidarImage, cv::Point(pixel.x, pixel.y), 1, color, -1);
      }
    }
  }

  cv::line(LidarImage, cv::Point(0, height_half), cv::Point(width, height_half),
           cv::Scalar(125, 125, 125));
  cv::line(LidarImage, cv::Point(width_half, 0), cv::Point(width_half, height),
           cv::Scalar(125, 125, 125));

  std::string win_name = name + "_" + std::to_string(idx);
  cv::namedWindow(win_name, cv::WINDOW_NORMAL);
  cv::resizeWindow(win_name, width, height);
  cv::imshow(win_name, LidarImage);
  cv::waitKey(1);
}

void show2d_lidar_bev(const jojo::perception::base::Point3DF p[8] /*vertex*/,
                      const Eigen::Vector3f& center,
                      cv::Scalar color = cv::Scalar(0, 97, 0),
                      cv::Mat* ext_img = nullptr);
// clang-format on

// void show2d_lidar_data(const jojo::perception::base::Frame* frame);

// for depth image color
void show2d_camera_data(const cv::Mat& image_float, const int max_depth);

void show2d_camera_data(const cv::Mat& image, const uint& idx = 0,
                        const std::string& name = "camera");

// void show2d_radar_data(const jojo::perception::base::Frame* frame);

// void show2d_segmentation_data(const jojo::perception::base::Frame* frame);

// void show2d_tracking_data(const jojo::perception::base::Frame* frame);

// void show2d_detection_data(const jojo::perception::base::Frame* frame);

// void show2d_perception_data(const jojo::perception::base::Frame* frame);

// Mat 必须是连续内存
void show_cv_splits_cloud(std::vector<cv::Mat>& splits);

int display_image_pause(const std::string& window_title, const cv::Mat& image);

#endif  // SHOW_DATA_2D_H
