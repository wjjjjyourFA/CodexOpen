#ifndef CAMERA_LOCATION_ESTIMATION_COMMON_H
#define CAMERA_LOCATION_ESTIMATION_COMMON_H

#include <cmath>
#include <string>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "modules/perception/common/base/object.h"

namespace jojo {
namespace perception {
namespace cle {

bool Fail(const std::string& message, std::string* error);

// 将相机坐标系中的三维点投影到图像平面。
// 投影深度必须为正且大于 epsilon，所有输入和结果都必须是有限值。
bool ProjectPoint(const Eigen::Matrix<float, 3, 4>& projection_matrix,
                  const Eigen::Vector3f& point, float epsilon,
                  cv::Point* image_point);

enum class InferenceMode { kDetection = 1, kTracking = 2 };

bool ParseInferenceMode(int value, InferenceMode* mode);
const char* InferenceModeName(InferenceMode mode);

struct ImageLocationHyperparams {
  // 缩小 30%
  float scale = 0.7f;  // roi_scale

  // for legacy
  size_t RoiLimit  = 50 * 50;
  int imageSize[3] = {1920, 1080, 1};

  float eps  = 0.5f;  // cluster_epsilon
  int minPts = 5;  // cluster_min_points

  int pixel_threshold = 15;  // minimum_roi_points

  float projection_epsilon = 1.0e-5f;

  bool Validate(std::string* error = nullptr) const;
};

// 目标定位的 距离估计的 结果框
struct FrameObject {
  FrameObject() = default;

  // 原始检测（完整保留）
  base::Object obj;
  // 必须共享或很大对象
  // std::shared_ptr<const base::Object> obj = nullptr;

  // 推理框 box2d
  cv::Rect srcRec;
  // 中心修正 / 估计框 box2d <== 推理框
  cv::Rect cenRec;

  // 目标 三维框 box3d <== 估计框 推算得到
  // ==> parent.camera_supplement.box3d;

  // 该 center 中心源自相机使用点云，因此是很不准确的，只能作为参考
  // Eigen::Vector3f local_center = Eigen::Vector3f(0, 0, 0);
  // ==> parent.camera_supplement.local_center
  // float dist = local_center.squaredNorm();

  bool located = false;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// 中间结构体；用于初始化每一个相机的中间数据；
class ImageLocationData {
 public:
  ImageLocationData() {};
  virtual ~ImageLocationData() = default;

  // 只维护一张 mask
  cv::Mat projection_mask;

  // 多个测距器相关参数  lidar + radar
  // projection_matrix;
};

struct LocationEstimateResult {
  std::vector<FrameObject> objects;
  std::size_t inference_count = 0U;
  std::size_t accepted_count  = 0U;
  std::size_t located_count   = 0U;
  std::string error;

  bool ok() const { return error.empty(); }
  void Clear();
};

}  // namespace cle
}  // namespace perception
}  // namespace jojo
#endif
