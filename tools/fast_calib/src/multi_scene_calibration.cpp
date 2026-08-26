/*
 * Multi-scene solver adapted from hku-mars/FAST-Calib (GPLv2).
 */

#include "fast_calib/fast_calib.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <Eigen/Dense>

#include "calibration_internal.h"

namespace jojo {
namespace tools {
namespace fast_calib {
namespace {

struct CenterBlock {
  std::string time_line;
  std::vector<Eigen::Vector3d> lidar_points;
  std::vector<Eigen::Vector3d> camera_points;
};

struct RigidResult {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  double rmse = 0.0;
  bool valid = false;
};

bool ParseCentersLine(const std::string& line,
                      std::vector<Eigen::Vector3d>* points) {
  static const std::regex braces("\\{([^\\}]*)\\}");
  points->clear();
  for (std::sregex_iterator entry(line.begin(), line.end(), braces), end;
       entry != end; ++entry) {
    std::string values = (*entry)[1];
    values.erase(
        std::remove_if(values.begin(), values.end(), [](unsigned char value) {
          return std::isspace(value) != 0;
        }),
        values.end());
    std::stringstream parser(values);
    std::string token;
    std::vector<double> xyz;
    while (std::getline(parser, token, ',')) {
      try {
        xyz.push_back(std::stod(token));
      } catch (...) {
        return false;
      }
    }
    if (xyz.size() != 3) {
      return false;
    }
    points->emplace_back(xyz[0], xyz[1], xyz[2]);
  }
  return !points->empty();
}

RigidResult SolveRigidTransformWeighted(
    const std::vector<Eigen::Vector3d>& lidar_points,
    const std::vector<Eigen::Vector3d>& camera_points,
    const std::vector<double>* supplied_weights = nullptr) {
  RigidResult result;
  const std::size_t count = lidar_points.size();
  if (count < 3 || camera_points.size() != count) {
    return result;
  }

  std::vector<double> weights(count, 1.0);
  if (supplied_weights != nullptr && supplied_weights->size() == count) {
    weights = *supplied_weights;
  }
  double weight_sum = 0.0;
  for (const double weight : weights) {
    weight_sum += weight;
  }
  if (weight_sum <= 0.0) {
    return result;
  }

  Eigen::Vector3d lidar_mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d camera_mean = Eigen::Vector3d::Zero();
  for (std::size_t index = 0; index < count; ++index) {
    lidar_mean += weights[index] * lidar_points[index];
    camera_mean += weights[index] * camera_points[index];
  }
  lidar_mean /= weight_sum;
  camera_mean /= weight_sum;

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (std::size_t index = 0; index < count; ++index) {
    covariance += weights[index] *
                  ((lidar_points[index] - lidar_mean) *
                   (camera_points[index] - camera_mean).transpose());
  }
  const Eigen::JacobiSVD<Eigen::Matrix3d> decomposition(
      covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d left = decomposition.matrixU();
  const Eigen::Matrix3d right = decomposition.matrixV();
  Eigen::Matrix3d rotation = right * left.transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) = -1.0;
    rotation = right * correction * left.transpose();
  }
  const Eigen::Vector3d translation =
      camera_mean - rotation * lidar_mean;

  double squared_error = 0.0;
  for (std::size_t index = 0; index < count; ++index) {
    const Eigen::Vector3d residual =
        rotation * lidar_points[index] + translation - camera_points[index];
    squared_error += weights[index] * residual.squaredNorm();
  }
  result.rotation = rotation;
  result.translation = translation;
  result.rmse = std::sqrt(squared_error / weight_sum);
  result.valid = true;
  return result;
}

bool Fail(const std::string& message, std::string* error) {
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

}  // namespace

bool RunMultiSceneCalibration(const std::string& output_path,
                              MultiCalibrationResult* result,
                              std::string* error) {
  if (result == nullptr) {
    return Fail("multi-scene result pointer is null", error);
  }
  if (!EnsureOutputDirectory(output_path, error)) {
    return false;
  }
  const boost::filesystem::path directory(output_path);
  const std::string record_path =
      (directory / "circle_center_record.txt").string();
  const std::string result_path =
      (directory / "multi_calib_result.txt").string();

  std::ifstream input(record_path);
  if (!input.is_open()) {
    return Fail("failed to open center record: " + record_path, error);
  }
  std::vector<std::string> lines;
  for (std::string line; std::getline(input, line);) {
    if (!line.empty()) {
      lines.push_back(std::move(line));
    }
  }
  if (lines.size() < 9) {
    return Fail("center record has fewer than three complete blocks: " +
                    record_path,
                error);
  }

  std::vector<CenterBlock> blocks;
  for (std::size_t index = 0; index + 2 < lines.size(); ++index) {
    if (lines[index].rfind("time:", 0) != 0 ||
        lines[index + 1].find("lidar_centers:") == std::string::npos ||
        lines[index + 2].find("qr_centers:") == std::string::npos) {
      continue;
    }
    CenterBlock block;
    block.time_line = lines[index];
    if (!ParseCentersLine(lines[index + 1], &block.lidar_points) ||
        !ParseCentersLine(lines[index + 2], &block.camera_points) ||
        block.lidar_points.size() != kTargetCircleCount ||
        block.camera_points.size() != kTargetCircleCount) {
      continue;
    }
    blocks.push_back(std::move(block));
    index += 2;
  }
  if (blocks.size() < 3) {
    std::ostringstream message;
    message << "parsed fewer than three complete center blocks (got "
            << blocks.size() << ") from " << record_path;
    return Fail(message.str(), error);
  }

  std::vector<Eigen::Vector3d> lidar_points;
  std::vector<Eigen::Vector3d> camera_points;
  for (std::size_t block_index = blocks.size() - 3;
       block_index < blocks.size(); ++block_index) {
    for (int point_index = 0; point_index < kTargetCircleCount;
         ++point_index) {
      lidar_points.push_back(blocks[block_index].lidar_points[
          static_cast<std::size_t>(point_index)]);
      camera_points.push_back(blocks[block_index].camera_points[
          static_cast<std::size_t>(point_index)]);
    }
  }
  if (lidar_points.size() != 12 || camera_points.size() != 12) {
    return Fail("the last three center blocks did not produce 12 pairs",
                error);
  }

  std::cout << "LiDAR centers:" << std::endl;
  for (std::size_t index = 0; index < lidar_points.size(); ++index) {
    std::cout << "L[" << index << "]: (" << lidar_points[index].x() << ", "
              << lidar_points[index].y() << ", " << lidar_points[index].z()
              << ")" << std::endl;
  }
  std::cout << "QR centers:" << std::endl;
  for (std::size_t index = 0; index < camera_points.size(); ++index) {
    std::cout << "C[" << index << "]: (" << camera_points[index].x()
              << ", " << camera_points[index].y() << ", "
              << camera_points[index].z() << ")" << std::endl;
  }

  const RigidResult rigid =
      SolveRigidTransformWeighted(lidar_points, camera_points);
  if (!rigid.valid) {
    return Fail("weighted rigid-transform solution failed", error);
  }

  MultiCalibrationResult calibration;
  calibration.transform.block<3, 3>(0, 0) = rigid.rotation;
  calibration.transform.block<3, 1>(0, 3) = rigid.translation;
  calibration.rmse = rigid.rmse;
  std::cout << "[Result] RMSE: " << std::fixed << std::setprecision(4)
            << calibration.rmse << " m" << std::endl;
  std::cout << "[Result] Multi-scene calibration: extrinsic parameters "
               "T_cam_lidar =\n"
            << std::fixed << std::setprecision(6) << calibration.transform
            << std::endl;

  std::ofstream output(result_path);
  if (!output.is_open()) {
    return Fail("failed to open multi-scene result: " + result_path, error);
  }
  output << "# FAST-LIVO2 calibration format\n";
  output << std::fixed << std::setprecision(6);
  output << "Rcl: [ " << std::setw(9) << rigid.rotation(0, 0) << ", "
         << std::setw(9) << rigid.rotation(0, 1) << ", " << std::setw(9)
         << rigid.rotation(0, 2) << ",\n"
         << "      " << std::setw(9) << rigid.rotation(1, 0) << ", "
         << std::setw(9) << rigid.rotation(1, 1) << ", " << std::setw(9)
         << rigid.rotation(1, 2) << ",\n"
         << "      " << std::setw(9) << rigid.rotation(2, 0) << ", "
         << std::setw(9) << rigid.rotation(2, 1) << ", " << std::setw(9)
         << rigid.rotation(2, 2) << "]\n";
  output << "Pcl: [ " << std::setw(9) << rigid.translation.x() << ", "
         << std::setw(9) << rigid.translation.y() << ", " << std::setw(9)
         << rigid.translation.z() << "]\n";
  if (!output.good()) {
    return Fail("failed to write multi-scene result: " + result_path, error);
  }

  std::cout << "[Result] Multi-scene calibration results saved to "
            << result_path << std::endl;
  *result = std::move(calibration);
  return true;
}

}  // namespace fast_calib
}  // namespace tools
}  // namespace jojo
