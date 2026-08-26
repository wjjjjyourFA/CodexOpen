/*
 * Algorithm code adapted from hku-mars/FAST-Calib (GPLv2).
 */

#include "calibration_internal.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

#include <boost/filesystem.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

namespace jojo {
namespace tools {
namespace fast_calib {
namespace {

std::string OutputFile(const std::string& output_path,
                       const std::string& filename) {
  return (boost::filesystem::path(output_path) / filename).string();
}

double Distance(const pcl::PointXYZ& first, const pcl::PointXYZ& second) {
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

bool EnsureOutputDirectory(const std::string& output_path,
                           std::string* error) {
  if (output_path.empty()) {
    if (error != nullptr) {
      *error = "output_path is empty";
    }
    return false;
  }

  boost::system::error_code status;
  const boost::filesystem::path directory(output_path);
  if (boost::filesystem::exists(directory, status)) {
    if (status || !boost::filesystem::is_directory(directory, status)) {
      if (error != nullptr) {
        *error = "output_path is not a directory: " + output_path;
      }
      return false;
    }
    return true;
  }
  if (!boost::filesystem::create_directories(directory, status) || status) {
    if (error != nullptr) {
      *error = "cannot create output directory " + output_path + ": " +
               status.message();
    }
    return false;
  }
  return true;
}

std::vector<std::vector<int>> BuildCombinations(int count,
                                                int selection_size) {
  std::vector<std::vector<int>> groups;
  if (count < selection_size || selection_size <= 0) {
    return groups;
  }

  std::string bitmask(static_cast<std::size_t>(selection_size), 1);
  bitmask.resize(static_cast<std::size_t>(count), 0);
  do {
    std::vector<int> group;
    for (int index = 0; index < count; ++index) {
      if (bitmask[static_cast<std::size_t>(index)] != 0) {
        group.push_back(index);
      }
    }
    groups.push_back(std::move(group));
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

  std::cout << count << " centers found. Iterating over " << groups.size()
            << " possible sets of candidates" << std::endl;
  return groups;
}

bool SortPatternCenters(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output,
    bool lidar_axes,
    std::string* error) {
  output->clear();
  if (input == nullptr || input->size() != kTargetCircleCount) {
    if (error != nullptr) {
      std::ostringstream stream;
      stream << "Number of " << (lidar_axes ? "lidar" : "camera")
             << " center points is not " << kTargetCircleCount;
      *error = stream.str();
    }
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr working(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (lidar_axes) {
    working->reserve(input->size());
    for (const auto& point : *input) {
      pcl::PointXYZ converted;
      converted.x = -point.y;
      converted.y = -point.z;
      converted.z = point.x;
      working->push_back(converted);
    }
  } else {
    *working = *input;
  }

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*working, centroid);
  std::vector<std::pair<float, int>> projected;
  projected.reserve(working->size());
  for (std::size_t index = 0; index < working->size(); ++index) {
    const auto& point = working->points[index];
    projected.emplace_back(
        std::atan2(point.y - centroid.y(), point.x - centroid.x()),
        static_cast<int>(index));
  }
  std::sort(projected.begin(), projected.end());

  output->resize(kTargetCircleCount);
  for (int index = 0; index < kTargetCircleCount; ++index) {
    output->points[static_cast<std::size_t>(index)] =
        working->points[static_cast<std::size_t>(
            projected[static_cast<std::size_t>(index)].second)];
  }

  const auto& first = output->points[0];
  const auto& second = output->points[1];
  const auto& third = output->points[2];
  const Eigen::Vector3f first_edge(second.x - first.x,
                                  second.y - first.y, 0.0F);
  const Eigen::Vector3f second_edge(third.x - second.x,
                                   third.y - second.y, 0.0F);
  if (first_edge.cross(second_edge).z() > 0.0F) {
    std::swap(output->points[1], output->points[3]);
  }

  if (lidar_axes) {
    for (auto& point : output->points) {
      const float lidar_x = point.z;
      const float lidar_y = -point.x;
      const float lidar_z = -point.y;
      point.x = lidar_x;
      point.y = lidar_y;
      point.z = lidar_z;
    }
  }
  return true;
}

bool IsTargetRectangle(const std::vector<pcl::PointXYZ>& candidates,
                       double target_width,
                       double target_height) {
  if (candidates.size() != kTargetCircleCount || target_width <= 0.0 ||
      target_height <= 0.0) {
    return false;
  }

  pcl::PointXYZ center;
  center.x = center.y = center.z = 0.0F;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& point : candidates) {
    center.x += point.x;
    center.y += point.y;
    center.z += point.z;
    cloud->push_back(point);
  }
  center.x /= static_cast<float>(candidates.size());
  center.y /= static_cast<float>(candidates.size());
  center.z /= static_cast<float>(candidates.size());

  const double target_diagonal =
      std::sqrt(target_width * target_width + target_height * target_height);
  for (const auto& point : candidates) {
    if (std::fabs(Distance(center, point) - target_diagonal / 2.0) /
            (target_diagonal / 2.0) >
        kGeometryTolerance * 2.0) {
      return false;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr sorted(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::string ignored_error;
  if (!SortPatternCenters(cloud, sorted, false, &ignored_error)) {
    return false;
  }

  const double side01 = Distance(sorted->points[0], sorted->points[1]);
  const double side12 = Distance(sorted->points[1], sorted->points[2]);
  const double side23 = Distance(sorted->points[2], sorted->points[3]);
  const double side30 = Distance(sorted->points[3], sorted->points[0]);
  const bool width_first =
      std::fabs(side01 - target_width) / target_width < kGeometryTolerance &&
      std::fabs(side12 - target_height) / target_height < kGeometryTolerance &&
      std::fabs(side23 - target_width) / target_width < kGeometryTolerance &&
      std::fabs(side30 - target_height) / target_height < kGeometryTolerance;
  const bool height_first =
      std::fabs(side01 - target_height) / target_height < kGeometryTolerance &&
      std::fabs(side12 - target_width) / target_width < kGeometryTolerance &&
      std::fabs(side23 - target_height) / target_height < kGeometryTolerance &&
      std::fabs(side30 - target_width) / target_width < kGeometryTolerance;
  if (!width_first && !height_first) {
    return false;
  }

  const double perimeter = side01 + side12 + side23 + side30;
  const double target_perimeter = 2.0 * (target_width + target_height);
  return std::fabs(perimeter - target_perimeter) / target_perimeter <=
         kGeometryTolerance;
}

void AlignPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input,
    pcl::PointCloud<pcl::PointXYZ>::Ptr output,
    const Eigen::Matrix4f& transformation) {
  output->clear();
  output->reserve(input->size());
  for (const auto& point : *input) {
    const Eigen::Vector4f homogeneous(point.x, point.y, point.z, 1.0F);
    const Eigen::Vector4f transformed = transformation * homogeneous;
    output->push_back(pcl::PointXYZ(transformed.x(), transformed.y(),
                                    transformed.z()));
  }
}

double ComputeRmse(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& first,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& second) {
  if (first == nullptr || second == nullptr || first->empty() ||
      first->size() != second->size()) {
    return -1.0;
  }
  double squared_error = 0.0;
  for (std::size_t index = 0; index < first->size(); ++index) {
    const double dx = first->points[index].x - second->points[index].x;
    const double dy = first->points[index].y - second->points[index].y;
    const double dz = first->points[index].z - second->points[index].z;
    squared_error += dx * dx + dy * dy + dz * dz;
  }
  return std::sqrt(squared_error / static_cast<double>(first->size()));
}

void ProjectPointCloudToImage(
    const pcl::PointCloud<PointXYZRing>::ConstPtr& cloud,
    const Eigen::Matrix4f& transformation,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coefficients,
    const cv::Mat& image,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud) {
  colored_cloud->clear();
  colored_cloud->reserve(cloud->size());

  cv::Mat undistorted_image;
  cv::undistort(image, undistorted_image, camera_matrix,
                distortion_coefficients);
  const cv::Mat rotation_vector = cv::Mat::zeros(3, 1, CV_32F);
  const cv::Mat translation_vector = cv::Mat::zeros(3, 1, CV_32F);
  const cv::Mat zero_distortion = cv::Mat::zeros(5, 1, CV_32F);
  std::vector<cv::Point3f> object_points(1);
  std::vector<cv::Point2f> image_points(1);

  for (const auto& point : *cloud) {
    const Eigen::Vector4f homogeneous(point.x, point.y, point.z, 1.0F);
    const Eigen::Vector4f transformed = transformation * homogeneous;
    if (transformed.z() < 0.0F) {
      continue;
    }
    object_points[0] =
        cv::Point3f(transformed.x(), transformed.y(), transformed.z());
    cv::projectPoints(object_points, rotation_vector, translation_vector,
                      camera_matrix, zero_distortion, image_points);
    const int column = static_cast<int>(image_points[0].x);
    const int row = static_cast<int>(image_points[0].y);
    if (column < 0 || column >= undistorted_image.cols || row < 0 ||
        row >= undistorted_image.rows || undistorted_image.channels() < 3) {
      continue;
    }

    const cv::Vec3b color = undistorted_image.at<cv::Vec3b>(row, column);
    pcl::PointXYZRGB colored;
    colored.x = transformed.x();
    colored.y = transformed.y();
    colored.z = transformed.z();
    colored.r = color[2];
    colored.g = color[1];
    colored.b = color[0];
    colored_cloud->push_back(colored);
  }
}

bool SaveTargetHoleCenters(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& lidar_centers,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& camera_centers,
    const std::string& output_path,
    std::string* error) {
  if (lidar_centers->size() != kTargetCircleCount ||
      camera_centers->size() != kTargetCircleCount) {
    if (error != nullptr) {
      *error = "the number of LiDAR or camera centers is not 4";
    }
    return false;
  }
  if (!EnsureOutputDirectory(output_path, error)) {
    return false;
  }

  const std::string record_path =
      OutputFile(output_path, "circle_center_record.txt");
  std::ofstream output(record_path, std::ios::app);
  if (!output.is_open()) {
    if (error != nullptr) {
      *error = "cannot open " + record_path;
    }
    return false;
  }

  const auto now = std::chrono::system_clock::now();
  const std::time_t local_time = std::chrono::system_clock::to_time_t(now);
  output << "time: "
         << std::put_time(std::localtime(&local_time), "%Y-%m-%d %H:%M:%S")
         << '\n';
  output << "lidar_centers:";
  for (const auto& point : *lidar_centers) {
    output << " {" << point.x << ',' << point.y << ',' << point.z << '}';
  }
  output << '\n' << "qr_centers:";
  for (const auto& point : *camera_centers) {
    output << " {" << point.x << ',' << point.y << ',' << point.z << '}';
  }
  output << '\n';
  if (!output.good()) {
    if (error != nullptr) {
      *error = "failed to write " + record_path;
    }
    return false;
  }
  std::cout << "[Record] Saved four pairs of circular hole centers to "
            << record_path << std::endl;
  return true;
}

bool SaveSingleCalibrationResults(const Params& params,
                                  const SingleCalibrationResult& result,
                                  std::string* error) {
  if (result.colored_cloud->empty()) {
    if (error != nullptr) {
      *error = "colored point cloud is empty";
    }
    return false;
  }
  if (!EnsureOutputDirectory(params.output_path, error)) {
    return false;
  }

  const std::string result_path =
      OutputFile(params.output_path, "single_calib_result.txt");
  std::ofstream output(result_path);
  if (!output.is_open()) {
    if (error != nullptr) {
      *error = "cannot open " + result_path;
    }
    return false;
  }
  output << "# FAST-LIVO2 calibration format\n";
  output << "cam_model: Pinhole\n";
  output << "cam_width: " << result.annotated_image.cols << '\n';
  output << "cam_height: " << result.annotated_image.rows << '\n';
  output << "scale: 1.0\n";
  output << "cam_fx: " << params.fx << '\n';
  output << "cam_fy: " << params.fy << '\n';
  output << "cam_cx: " << params.cx << '\n';
  output << "cam_cy: " << params.cy << '\n';
  output << "cam_d0: " << params.k1 << '\n';
  output << "cam_d1: " << params.k2 << '\n';
  output << "cam_d2: " << params.p1 << '\n';
  output << "cam_d3: " << params.p2 << '\n';
  output << "\nRcl: [" << std::fixed << std::setprecision(6);
  output << std::setw(10) << result.transform(0, 0) << ", "
         << std::setw(10) << result.transform(0, 1) << ", "
         << std::setw(10) << result.transform(0, 2) << ",\n";
  output << "      " << std::setw(10) << result.transform(1, 0) << ", "
         << std::setw(10) << result.transform(1, 1) << ", "
         << std::setw(10) << result.transform(1, 2) << ",\n";
  output << "      " << std::setw(10) << result.transform(2, 0) << ", "
         << std::setw(10) << result.transform(2, 1) << ", "
         << std::setw(10) << result.transform(2, 2) << "]\n";
  output << "Pcl: [" << std::setw(10) << result.transform(0, 3) << ", "
         << std::setw(10) << result.transform(1, 3) << ", "
         << std::setw(10) << result.transform(2, 3) << "]\n";
  output.close();

  const std::string cloud_path =
      OutputFile(params.output_path, "colored_cloud.pcd");
  if (pcl::io::savePCDFileASCII(cloud_path, *result.colored_cloud) != 0) {
    if (error != nullptr) {
      *error = "failed to save " + cloud_path;
    }
    return false;
  }
  const std::string image_path =
      OutputFile(params.output_path, "qr_detect.png");
  if (!cv::imwrite(image_path, result.annotated_image)) {
    if (error != nullptr) {
      *error = "failed to save " + image_path;
    }
    return false;
  }

  std::cout << "[Result] Single-scene calibration results saved to "
            << result_path << std::endl;
  std::cout << "[Result] Saved colored point cloud to " << cloud_path
            << std::endl;
  std::cout << "[Result] Saved QR detection image to " << image_path
            << std::endl;
  return true;
}

}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo
