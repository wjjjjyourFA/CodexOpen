#include "modules/perception/camera_location_estimation/common.h"

namespace jojo {
namespace perception {
namespace cle {

bool Fail(const std::string& message, std::string* error) {
  if (error) *error = message;
  return false;
}

bool ImageLocationHyperparams::Validate(std::string* error) const {
  if (scale <= 0.0f || scale > 1.0f)
    return Fail("roi_scale must be in (0, 1]", error);
  if (eps <= 0.0f) return Fail("cluster_epsilon must be positive", error);
  if (minPts <= 0 || pixel_threshold <= 0)
    return Fail("cluster point thresholds must be positive", error);
  if (projection_epsilon <= 0.0f)
    return Fail("projection_epsilon must be positive", error);
  if (error) error->clear();
  return true;
}

bool ParseInferenceMode(int value, InferenceMode* mode) {
  if (!mode) return false;
  if (value == static_cast<int>(InferenceMode::kDetection)) {
    *mode = InferenceMode::kDetection;
    return true;
  }
  if (value == static_cast<int>(InferenceMode::kTracking)) {
    *mode = InferenceMode::kTracking;
    return true;
  }
  return false;
}

const char* InferenceModeName(InferenceMode mode) {
  switch (mode) {
    case InferenceMode::kDetection:
      return "detection";
    case InferenceMode::kTracking:
      return "tracking";
  }
  return "invalid";
}

bool ProjectPoint(const Eigen::Matrix<float, 3, 4>& projection_matrix,
                  const Eigen::Vector3f& point, float epsilon,
                  cv::Point* image_point) {
  if (!image_point || !point.allFinite()) return false;

  const Eigen::Vector3f uv =
      projection_matrix *
      Eigen::Vector4f(point.x(), point.y(), point.z(), 1.0f);
  if (!uv.allFinite() || uv.z() <= epsilon) return false;

  const float x = uv.x() / uv.z();
  const float y = uv.y() / uv.z();
  if (!std::isfinite(x) || !std::isfinite(y)) return false;

  *image_point = cv::Point(static_cast<int>(x), static_cast<int>(y));
  return true;
}

void LocationEstimateResult::Clear() {
  objects.clear();
  inference_count = 0U;
  accepted_count  = 0U;
  located_count   = 0U;
  error.clear();
}

}  // namespace cle
}  // namespace perception
}  // namespace jojo
