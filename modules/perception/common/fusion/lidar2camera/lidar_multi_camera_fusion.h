#ifndef LIDAR_MULTI_CAMERA_FUSION_H
#define LIDAR_MULTI_CAMERA_FUSION_H

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "cyber/base/thread_pool_legacy.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"

namespace jojo {
namespace perception {
namespace fusion {

class LidarMultiCameraFusion : public LidarCameraFusion {
 public:
  LidarMultiCameraFusion()           = default;
  ~LidarMultiCameraFusion() override = default;

  bool Init(const std::shared_ptr<jojo::cyber::base::ThreadPool>& thread_pool =
                nullptr);

  void Start();
  void Stop();
  void Run(bool is_mask = true);

  // CameraParams 与图像数组按相同下标一一对应。
  void SetCameraParams(
      std::shared_ptr<jojo::perception::camera::CameraParams> camera_params);

  // 传入的是 undistort_images
  void SetCameraImageVector(
      const std::vector<std::shared_ptr<cv::Mat>>& image_v);

  bool GetFusedImageVector(std::vector<std::shared_ptr<cv::Mat>>& image_v);

 private:
  using PointsMatrix     = Eigen::Matrix<float, 4, Eigen::Dynamic>;
  using ProjectionMatrix = Eigen::Matrix<float, 3, 4>;

  bool ValidateBatchInput(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const std::shared_ptr<jojo::perception::camera::CameraParams>&
          camera_params,
      const std::vector<std::shared_ptr<cv::Mat>>& image_v) const;

  void ProjectBatch(const PointsMatrix& points,
                    const std::vector<ProjectionMatrix>& projection_matrices,
                    const std::vector<std::shared_ptr<cv::Mat>>& image_v,
                    std::vector<std::shared_ptr<cv::Mat>>& output_v);

  // 公共快速投影函数输出的是每个像素对应的点云 XYZ；这里仅负责把该结果
  // 渲染成可显示的彩色投影图，不重复计算点云到图像的投影关系。
  size_t DrawProjectionResult(const cv::Mat& projection,
                              cv::Mat& output) const;

  std::atomic_bool is_running_{false};

  // 这是一个全新的参数值，不再是单相机自己和多雷达的参数
  // 而是单雷达到多相机的参数
  std::shared_ptr<jojo::perception::camera::CameraParams> camera_params_;
  std::vector<std::shared_ptr<cv::Mat>> image_v_;
  std::vector<std::shared_ptr<cv::Mat>> mask_v_;

  std::shared_ptr<jojo::cyber::base::ThreadPool> thread_pool_;

  // 同一实例一次只执行一个 batch，输入和输出仍由父类 data_mutex_ 保护。
  std::mutex run_mutex_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace jojo

#endif  // LIDAR_MULTI_CAMERA_FUSION_H
