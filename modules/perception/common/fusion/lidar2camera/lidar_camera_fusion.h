#ifndef LIDAR_CAMERA_FUSION_H
#define LIDAR_CAMERA_FUSION_H

#include <mutex>

#include <Eigen/Dense>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>

#include "modules/perception/common/base/pcl_extra/pcl_eigen.h"
#include "modules/perception/common/base/opencv_extra/colors.hpp"
#include "modules/perception/common/base/opencv_extra/cv_colors.h"
// #include "modules/perception/tools/common/show_data_3d.h"

namespace jojo {
namespace perception {
namespace fusion {

class LidarCameraFusion {
 public:
  LidarCameraFusion(/* args */);
  virtual ~LidarCameraFusion();

  void set_params(const std::string& name = "", int dist_threshold = 100);

  void SetProjectionMatrix(const Eigen::Matrix4f& projection_matrix);
  void SetProjectionMatrix(const Eigen::Matrix<float, 3, 4>& projection_matrix);

  void SetLidarPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void SetLidarPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  // undistort_image
  void SetCameraImage(const cv::Mat& image);

  bool GetFusedPointCloudColor(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_color);
  bool GetFusedImage(cv::Mat& image);

  // 所有的调用都通过 fuse 函数
  void fuse(int mode = 1, bool is_mask = true, bool color = false);
  void show_lidar_color_cloud();
  void show_image_proj();

 protected:
  // 默认不输出彩色点云
  void project_lidar_to_camera(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const Eigen::Matrix<float, 3, 4>& projection_matrix, const cv::Mat& image,
      cv::Mat& mask, bool color = false);

  void project_lidar_to_camera_fast(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const Eigen::Matrix<float, 3, 4>& projection_matrix, const cv::Mat& image,
      cv::Mat& mask, bool color = false);

  void project_lidar_to_camera_fast_impl(
      const Eigen::Matrix<float, 4, Eigen::Dynamic>& points,
      const Eigen::Matrix<float, 3, 4>& projection_matrix, const cv::Mat& image,
      cv::Mat& mask, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_color,
      bool color = false);

  // m
  int dist_ = 100;
  float inv_dist;
  std::string name_ = "Lidar";

 protected:  // <== private
  // input
  Eigen::Matrix<float, 3, 4> projection_matrix_;
  // cloud 来源于外部输入；
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  cv::Mat image_, mask_;
  // output
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_;

  mutable std::mutex data_mutex_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace jojo

#endif  // LIDAR_CAMERA_FUSION_H
