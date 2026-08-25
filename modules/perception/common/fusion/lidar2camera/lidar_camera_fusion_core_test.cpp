#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"

int main() {
  using jojo::perception::fusion::LidarCameraFusion;
  LidarCameraFusion fusion;
  if (fusion.fuse()) return 1;
  if (fusion.set_params("test", 0)) return 2;

  Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();
  projection(0, 0) = 1.0f;
  projection(1, 1) = 1.0f;
  projection(2, 2) = 1.0f;
  projection(3, 3) = 1.0f;
  if (!fusion.SetProjectionMatrix(projection)) return 3;

  auto input = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  input->emplace_back(1.0f, 1.0f, 1.0f);
  cv::Mat image(4, 4, CV_8UC3, cv::Scalar(0, 0, 0));
  image.at<cv::Vec3b>(1, 1) = cv::Vec3b(10, 20, 30);
  if (!fusion.SetLidarPointCloud(input) || !fusion.SetCameraImage(image)) return 4;

  (*input)[0].x = 100.0f;
  image.at<cv::Vec3b>(1, 1) = cv::Vec3b(0, 0, 0);
  if (!fusion.fuse(2, true, true)) return 5;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr first;
  if (!fusion.GetFusedPointCloudColor(first) || first->size() != 1) return 6;
  if ((*first)[0].x != 1.0f || (*first)[0].b != 10 || (*first)[0].r != 30) return 7;

  (*first)[0].x = 50.0f;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr second;
  if (!fusion.GetFusedPointCloudColor(second) || (*second)[0].x != 1.0f) return 8;

  return 0;
}
