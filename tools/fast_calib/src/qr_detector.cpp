/*
 * ArUco target detection adapted from hku-mars/FAST-Calib (GPLv2).
 */

#include "calibration_internal.h"

#include <cmath>
#include <sstream>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace jojo {
namespace tools {
namespace fast_calib {

QrDetector::QrDetector(const Params& params) : params_(params) {
  camera_matrix_ =
      (cv::Mat_<float>(3, 3) << params.fx, 0.0, params.cx, 0.0, params.fy,
       params.cy, 0.0, 0.0, 1.0);
  distortion_coefficients_ =
      (cv::Mat_<float>(1, 5) << params.k1, params.k2, params.p1, params.p2,
       0.0);
  dictionary_ =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
}

cv::Point2f QrDetector::ProjectPoint(const cv::Point3f& point) const {
  const std::vector<cv::Point3f> input{point};
  std::vector<cv::Point2f> projected(1);
  cv::projectPoints(input, cv::Mat::zeros(3, 1, CV_64FC1),
                    cv::Mat::zeros(3, 1, CV_64FC1), camera_matrix_,
                    distortion_coefficients_, projected);
  return projected[0];
}

bool QrDetector::Detect(const cv::Mat& image,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr centers,
                        std::string* error) {
  centers->clear();
  if (image.empty()) {
    if (error != nullptr) {
      *error = "input image is empty";
    }
    return false;
  }
  image.copyTo(annotated_image_);

  // Marker positions are ordered 0/1/2/3 around the target, while their
  // ArUco IDs are 1/2/4/3 respectively.
  std::vector<std::vector<cv::Point3f>> board_corners(4);
  std::vector<cv::Point3f> board_circle_centers;
  board_circle_centers.reserve(4);
  const float marker_half_width =
      static_cast<float>(params_.delta_width_qr_center);
  const float marker_half_height =
      static_cast<float>(params_.delta_height_qr_center);
  const float circle_half_width =
      static_cast<float>(params_.delta_width_circles / 2.0);
  const float circle_half_height =
      static_cast<float>(params_.delta_height_circles / 2.0);
  for (int marker = 0; marker < 4; ++marker) {
    const int horizontal_sign = marker % 3 == 0 ? -1 : 1;
    const int vertical_sign = marker < 2 ? 1 : -1;
    const float marker_x = horizontal_sign * marker_half_width;
    const float marker_y = vertical_sign * marker_half_height;
    board_circle_centers.emplace_back(horizontal_sign * circle_half_width,
                                      vertical_sign * circle_half_height,
                                      0.0F);
    for (int corner = 0; corner < 4; ++corner) {
      const int corner_x_sign = corner % 3 == 0 ? -1 : 1;
      const int corner_y_sign = corner < 2 ? 1 : -1;
      board_corners[static_cast<std::size_t>(marker)].emplace_back(
          marker_x + corner_x_sign * params_.marker_size / 2.0,
          marker_y + corner_y_sign * params_.marker_size / 2.0, 0.0F);
    }
  }

  const std::vector<int> board_ids{1, 2, 4, 3};
  const cv::Ptr<cv::aruco::Board> board =
      cv::aruco::Board::create(board_corners, dictionary_, board_ids);
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters =
      cv::aruco::DetectorParameters::create();
#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
  detector_parameters->doCornerRefinement = true;
#else
  detector_parameters->cornerRefinementMethod =
      cv::aruco::CORNER_REFINE_SUBPIX;
#endif

  std::vector<int> detected_ids;
  std::vector<std::vector<cv::Point2f>> detected_corners;
  cv::aruco::detectMarkers(image, dictionary_, detected_corners, detected_ids,
                           detector_parameters);
  if (!detected_ids.empty()) {
    cv::aruco::drawDetectedMarkers(annotated_image_, detected_corners,
                                   detected_ids);
  }
  if (detected_ids.size() <
          static_cast<std::size_t>(params_.min_detected_markers) ||
      detected_ids.size() > kTargetCircleCount) {
    if (error != nullptr) {
      std::ostringstream stream;
      stream << detected_ids.size() << " marker(s) found; expected at least "
             << params_.min_detected_markers << " and no more than "
             << kTargetCircleCount;
      *error = stream.str();
    }
    return false;
  }

  cv::Vec3d rotation_vector(0.0, 0.0, 0.0);
  cv::Vec3d translation_vector(0.0, 0.0, 0.0);
  std::vector<cv::Vec3d> marker_rotations;
  std::vector<cv::Vec3d> marker_translations;
  cv::Vec3f rotation_sine(0.0F, 0.0F, 0.0F);
  cv::Vec3f rotation_cosine(0.0F, 0.0F, 0.0F);
  cv::aruco::estimatePoseSingleMarkers(
      detected_corners, params_.marker_size, camera_matrix_,
      distortion_coefficients_, marker_rotations, marker_translations);
  for (std::size_t index = 0; index < detected_ids.size(); ++index) {
    cv::aruco::drawAxis(annotated_image_, camera_matrix_,
                        distortion_coefficients_, marker_rotations[index],
                        marker_translations[index], 0.1);
    translation_vector += marker_translations[index];
    for (int axis = 0; axis < 3; ++axis) {
      rotation_sine[axis] +=
          static_cast<float>(std::sin(marker_rotations[index][axis]));
      rotation_cosine[axis] +=
          static_cast<float>(std::cos(marker_rotations[index][axis]));
    }
  }
  translation_vector /= static_cast<double>(detected_ids.size());
  rotation_sine /= static_cast<float>(detected_ids.size());
  rotation_cosine /= static_cast<float>(detected_ids.size());
  for (int axis = 0; axis < 3; ++axis) {
    rotation_vector[axis] =
        std::atan2(rotation_sine[axis], rotation_cosine[axis]);
  }

#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
  const int valid = cv::aruco::estimatePoseBoard(
      detected_corners, detected_ids, board, camera_matrix_,
      distortion_coefficients_, rotation_vector, translation_vector);
#else
  const int valid = cv::aruco::estimatePoseBoard(
      detected_corners, detected_ids, board, camera_matrix_,
      distortion_coefficients_, rotation_vector, translation_vector, true);
#endif
  if (valid <= 0) {
    if (error != nullptr) {
      *error = "ArUco board pose estimation failed";
    }
    return false;
  }
  cv::aruco::drawAxis(annotated_image_, camera_matrix_,
                      distortion_coefficients_, rotation_vector,
                      translation_vector, 0.2);

  cv::Mat rotation(3, 3, CV_32F);
  cv::Rodrigues(rotation_vector, rotation);
  if (rotation.type() != CV_32F) {
    rotation.convertTo(rotation, CV_32F);
  }
  cv::Mat translation = cv::Mat::zeros(3, 1, CV_32F);
  translation.at<float>(0) = static_cast<float>(translation_vector[0]);
  translation.at<float>(1) = static_cast<float>(translation_vector[1]);
  translation.at<float>(2) = static_cast<float>(translation_vector[2]);
  cv::Mat board_transform = cv::Mat::eye(3, 4, CV_32F);
  rotation.copyTo(board_transform.rowRange(0, 3).colRange(0, 3));
  translation.copyTo(board_transform.rowRange(0, 3).col(3));

  pcl::PointCloud<pcl::PointXYZ>::Ptr candidates(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& board_center : board_circle_centers) {
    cv::Mat homogeneous = cv::Mat::zeros(4, 1, CV_32F);
    homogeneous.at<float>(0) = board_center.x;
    homogeneous.at<float>(1) = board_center.y;
    homogeneous.at<float>(2) = board_center.z;
    homogeneous.at<float>(3) = 1.0F;
    const cv::Mat camera_center = board_transform * homogeneous;
    const cv::Point3f center(camera_center.at<float>(0),
                             camera_center.at<float>(1),
                             camera_center.at<float>(2));
    cv::circle(annotated_image_, ProjectPoint(center), 5,
               cv::Scalar(0, 255, 0), -1);
    candidates->push_back(pcl::PointXYZ(center.x, center.y, center.z));
  }

  const auto groups = BuildCombinations(
      static_cast<int>(candidates->size()), kTargetCircleCount);
  int best_group = -1;
  double best_score = -1.0;
  for (std::size_t group_index = 0; group_index < groups.size();
       ++group_index) {
    std::vector<pcl::PointXYZ> candidate_set;
    for (const int candidate_index : groups[group_index]) {
      candidate_set.push_back(
          candidates->points[static_cast<std::size_t>(candidate_index)]);
    }
    const double score = IsTargetRectangle(
                             candidate_set, params_.delta_width_circles,
                             params_.delta_height_circles)
                             ? 1.0
                             : -1.0;
    if (best_score == 1.0 && score == 1.0) {
      if (error != nullptr) {
        *error = "more than one camera candidate set fits target geometry";
      }
      return false;
    }
    if (score > best_score) {
      best_score = score;
      best_group = static_cast<int>(group_index);
    }
  }
  if (best_group < 0) {
    if (error != nullptr) {
      *error = "no camera candidate set matches target geometry";
    }
    return false;
  }

  for (const int candidate_index :
       groups[static_cast<std::size_t>(best_group)]) {
    centers->push_back(
        candidates->points[static_cast<std::size_t>(candidate_index)]);
  }
  for (const auto& center : *centers) {
    cv::circle(annotated_image_,
               ProjectPoint(cv::Point3f(center.x, center.y, center.z)), 2,
               cv::Scalar(255, 0, 255), -1);
  }
  return true;
}

}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo
