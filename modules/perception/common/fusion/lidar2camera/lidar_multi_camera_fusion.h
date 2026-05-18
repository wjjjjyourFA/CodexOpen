#ifndef LIDAR_CAMERA_FUSION_THREAD_H
#define LIDAR_CAMERA_FUSION_THREAD_H

#include <thread>
#include <functional>

// #include "cyber/base/thread_pool.h"
#include "cyber/base/thread_pool_legacy.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "modules/perception/common/camera/params/camera_params.h"

namespace jojo {
namespace perception {
namespace fusion {

class LidarMultiCameraFusion : public LidarCameraFusion {
 public:
  LidarMultiCameraFusion();
  virtual ~LidarMultiCameraFusion() = default;

  void Init(const std::shared_ptr<jojo::cyber::base::ThreadPool> &thread_pool = nullptr);

  void Start();
  void Stop();

  void Run(bool is_mask = true);

  // 可能涉及到外部对点云和图像冻结坐标的乘算，因此使用值传递，避免异步过程中悬空
  void SetCameraParams(std::shared_ptr<camera::CameraParams> camera_params);

  // undistort_images
  void SetCameraImageVector(
      const std::vector<std::shared_ptr<cv::Mat>> &image_v);

  bool GetFusedImageVector(std::vector<std::shared_ptr<cv::Mat>> &image_v);

 protected:
  void project_lidar_to_camera_fast(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
      const std::shared_ptr<camera::CameraParams> &camera_params,
      const std::vector<std::shared_ptr<cv::Mat>> &image_v,
      std::vector<std::shared_ptr<cv::Mat>> &mask_v);

 private:
  std::atomic_bool isRunning_{false};

  // 需要外部传入 已经读取好了的 参数类
  std::shared_ptr<camera::CameraParams> camera_params_;

  std::vector<std::shared_ptr<cv::Mat>> image_v_, mask_v_;

  // 使用智能指针管理 ThreadPool；默认由外部传入，全局唯一；
  std::shared_ptr<jojo::cyber::base::ThreadPool> thread_pool_{nullptr};
  std::mutex cloud_mutex_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace jojo

#endif