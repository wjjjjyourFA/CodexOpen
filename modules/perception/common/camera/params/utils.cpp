#include "modules/perception/common/camera/params/utils.h"

namespace jojo {
namespace perception {
namespace camera {

void ApplyResolutionScale(CameraInfo& src, CameraInfo& dst,
                          CameraMatrix& matrix) {
  if (dst.width == 0 || dst.height == 0) {
    std::cerr << "CameraInfo dst width or height is 0" << std::endl;
    return;
  }

  float width_coeff  = static_cast<float>(dst.width) / src.width;
  float height_coeff = static_cast<float>(dst.height) / src.height;

  auto& intrinsic_matrix = matrix.intrinsic_matrix;
  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  intrinsic_matrix(0, 0) *= width_coeff;  // fx
  intrinsic_matrix(0, 2) *= width_coeff;  // cx
  intrinsic_matrix(1, 1) *= height_coeff;  // fy
  intrinsic_matrix(1, 2) *= height_coeff;  // cy

  dst.width  = static_cast<int>(std::round(src.width * width_coeff));
  dst.height = static_cast<int>(std::round(src.height * height_coeff));
}

void ApplyResolutionScale(CameraInfo& src, CameraInfo& dst,
                          std::shared_ptr<CameraMatrix> matrix) {
  if (!matrix) return;
  ApplyResolutionScale(src, dst, *matrix);
}

void ApplyFovScale(CameraMatrix& matrix, float fov_scale) {
  if (fov_scale <= 0.0f || fov_scale == 1.0f) return;  // 不修改

  auto& intrinsic_matrix = matrix.intrinsic_matrix;

  // 焦距（fx, fy）与视场角（FOV）是反比关系
  // FOV变大 → 焦距变小
  double inv = 1.0 / fov_scale;

  // 缩放焦距（改变 FoV）
  intrinsic_matrix(0, 0) *= inv;  // fx
  intrinsic_matrix(1, 1) *= inv;  // fy
}

void ApplyFovScale(std::shared_ptr<CameraMatrix> matrix, float fov_scale) {
  if (!matrix) return;
  ApplyFovScale(*matrix, fov_scale);
}

void ApplyShift(CameraMatrix& matrix, const cv::Point2f& shift) {
  auto& intrinsic_matrix = matrix.intrinsic_matrix;

  // 修改光心位置
  intrinsic_matrix(0, 2) += shift.x;  // cx
  intrinsic_matrix(1, 2) += shift.y;  // cy
}

void ApplyShift(std::shared_ptr<CameraMatrix> matrix,
                const cv::Point2f& shift) {
  if (!matrix) return;
  ApplyShift(*matrix, shift);
}

void ApplyRoiCrop(CameraInfo& dst, CameraMatrix& matrix, const cv::Rect& roi) {
  auto& intrinsic_matrix = matrix.intrinsic_matrix;

  // 在 OpenCV 中，cv::Rect roi(x, y, width, height) 定义的是：
  // x：ROI 左上角在原图中的 水平偏移
  // y：ROI 左上角在原图中的 垂直偏移

  // 主点偏移
  intrinsic_matrix(0, 2) -= roi.x;
  intrinsic_matrix(1, 2) -= roi.y;

  // 焦距不变
  // intrinsic_matrix(0,0) = fx;
  // intrinsic_matrix(1,1) = fy;

  // 更新图像尺寸
  dst.width  = roi.width;
  dst.height = roi.height;
}

void ApplyRoiCrop(CameraInfo& dst, std::shared_ptr<CameraMatrix> matrix,
                  const cv::Rect& roi) {
  if (!matrix) return;
  ApplyRoiCrop(dst, *matrix, roi);
}

void ApplyScaleProjectionMatrix(CameraInfo& src, CameraInfo& dst,
                                CameraMatrix& camera_matrix,
                                Lidar2CameraMatrix& lidar2camera_matrix,
                                const float fov_scale, const cv::Point2f& shift,
                                const cv::Rect* roi) {
  auto& intrinsic_matrix  = camera_matrix.intrinsic_matrix;
  auto& projection_matrix = lidar2camera_matrix.projection_matrix;

  // 解算 外参矩阵
  // Eigen::Matrix4f extrinsic_matrix =
  //     ComputeExtrinsicMatrix(intrinsic_matrix, projection_matrix);

  // 使用 已知外参矩阵
  auto& extrinsic_matrix = lidar2camera_matrix.extrinsic_matrix;

  // 更新 resize 内参矩阵
  ApplyResolutionScale(src, dst, camera_matrix);

  ApplyFovScale(camera_matrix, fov_scale);

  ApplyShift(camera_matrix, shift);

  ApplyRoiCrop(dst, camera_matrix, *roi);

  // 更新 resize 投影矩阵
  // intrinsic_matrix 是引用，所以不用重新赋值
  projection_matrix.block<3, 4>(0, 0) =
      intrinsic_matrix * extrinsic_matrix.block<3, 4>(0, 0);
}

void ApplyScaleProjectionMatrix(
    CameraInfo& src, CameraInfo& dst,
    std::shared_ptr<CameraMatrix> camera_matrix,
    std::shared_ptr<Lidar2CameraMatrix> lidar2camera_matrix,
    const float fov_scale, const cv::Point2f& shift, const cv::Rect* roi) {
  if (!camera_matrix || !lidar2camera_matrix) return;
  ApplyScaleProjectionMatrix(src, dst, *camera_matrix, *lidar2camera_matrix,
                             fov_scale, shift, roi);
}

}  // namespace camera
}  // namespace perception
}  // namespace jojo
