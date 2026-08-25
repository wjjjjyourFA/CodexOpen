#ifndef LIDAR_CAMERA_FUSION_H
#define LIDAR_CAMERA_FUSION_H

#include <mutex>

#include <Eigen/Dense>

// 禁用（deprecated / 易炸）
// #include <pcl/io/io.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/tools/opencv/colors.hpp"
#include "modules/perception/tools/opencv/cv_colors.h"
#include "modules/perception/tools/pcl/pcl_eigen.h"
// #include "modules/perception/tools/common/show_data_3d.h"

namespace jojo {
namespace perception {
namespace fusion {

class LidarCameraFusion {
 public:
  LidarCameraFusion(/* args */);
  virtual ~LidarCameraFusion();

  bool set_params(const std::string& name = "", int dist_threshold = 100);

  // clang-format off
  bool SetProjectionMatrix(const Eigen::Matrix4f& projection_matrix);
  bool SetProjectionMatrix(const Eigen::Matrix<float, 3, 4>& projection_matrix);
  bool SetL2CMatrix(std::shared_ptr<jojo::perception::camera::Lidar2CameraMatrix> l2c_matrix);
  // clang-format on

  bool SetLidarPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  bool SetLidarPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  // undistort_image
  bool SetCameraImage(const cv::Mat& image);

  // clang-format off
  bool GetFusedPointCloudColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_color);
  bool GetFusedImage(cv::Mat& image);
  // clang-format on

  // 所有的调用都通过 fuse 函数
  bool fuse(int mode = 1, bool is_mask = true, bool color = false);
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

  // Thread-safe variant when each caller provides an independent projection workspace. 
  // The points matrix is read-only and can be shared by all camera projection tasks.
  void project_lidar_to_camera_fast_impl(
      const Eigen::Matrix<float, 4, Eigen::Dynamic>& points,
      const Eigen::Matrix<float, 3, 4>& projection_matrix, const cv::Mat& image,
      cv::Mat& mask, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_color,
      Eigen::Matrix<float, 3, Eigen::Dynamic>& projected_workspace,
      bool color = false);

  void project_lidar_to_camera_raw(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const Eigen::Matrix<float, 3, 4>& extrinsic_matrix, const cv::Mat& image,
      cv::Mat& mask, bool color = false);

  // m
  int dist_ = 100;
  float inv_dist_;
  std::string name_ = "Lidar";

 protected:  // <== private
  // input
  Eigen::Matrix<float, 3, 4> projection_matrix_;
  bool projection_ready_ = false;
  // lidar to camera matrix, include:
  // intrinsic_matrix, distortion_params, extrinsic_matrix
  std::shared_ptr<jojo::perception::camera::Lidar2CameraMatrix> l2c_matrix_;
  // cloud 来源于外部输入；
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  cv::Mat image_, mask_;
  // output
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color_;

  // Reused by the fast projection path while data_mutex_ is held.
  Eigen::Matrix<float, 4, Eigen::Dynamic> points_workspace_;
  Eigen::Matrix<float, 3, Eigen::Dynamic> projected_workspace_;

  mutable std::mutex data_mutex_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace jojo

#endif  // LIDAR_CAMERA_FUSION_H
