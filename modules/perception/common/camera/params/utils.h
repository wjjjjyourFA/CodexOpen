#ifndef CAMERA_PARAMS_UTILS_H
#define CAMERA_PARAMS_UTILS_H

#pragma once

#include <opencv2/opencv.hpp>

#include "modules/perception/common/camera/common/camera_info.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/config/utils.h"

namespace jojo {
namespace perception {
namespace camera {

/* 顺序：Resize → FOV Scale → Crop
*/

// 根据图像分辨率比例缩放内参（仅像素坐标变化）
void ApplyResolutionScale(CameraInfo& src, CameraInfo& dst,
                          CameraMatrix& matrix);

void ApplyResolutionScale(CameraInfo& src, CameraInfo& dst,
                          std::shared_ptr<CameraMatrix> matrix);

// 根据FOV缩放调整焦距（反比关系）
void ApplyFovScale(CameraMatrix& matrix, float fov_scale);

void ApplyFovScale(std::shared_ptr<CameraMatrix> matrix, float fov_scale);

// 应用shift变换（光心平移）
void ApplyShift(CameraMatrix& matrix, const cv::Point2f& shift);

void ApplyShift(std::shared_ptr<CameraMatrix> matrix, const cv::Point2f& shift);

// 裁剪只改变坐标系原点，不改变焦距。
void ApplyRoiCrop(CameraInfo& dst, CameraMatrix& matrix, const cv::Rect& roi);

void ApplyRoiCrop(CameraInfo& dst, std::shared_ptr<CameraMatrix> matrix,
                  const cv::Rect& roi);

void ApplyScaleProjectionMatrix(CameraInfo& src, CameraInfo& dst,
                                CameraMatrix& camera_matrix,
                                Lidar2CameraMatrix& lidar2camera_matrix,
                                const float fov_scale    = 1.0f,
                                const cv::Point2f& shift = cv::Point2f(0, 0),
                                const cv::Rect* roi      = nullptr);

void ApplyScaleProjectionMatrix(
    CameraInfo& src, CameraInfo& dst,
    std::shared_ptr<CameraMatrix> camera_matrix,
    std::shared_ptr<Lidar2CameraMatrix> lidar2camera_matrix,
    const float fov_scale = 1.0f, const cv::Point2f& shift = cv::Point2f(0, 0),
    const cv::Rect* roi = nullptr);

}  // namespace camera
}  // namespace perception
}  // namespace jojo

#endif  //