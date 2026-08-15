#include "modules/perception/common/camera/params/camera_params_json.h"

#include <iostream>

namespace jojo {
namespace perception {
namespace camera {
namespace {

using json = nlohmann::json;

std::string IntrinsicPath(const std::string& extrinsic_path,
                          const std::string& child) {
  // 找最后一个路径分隔符（兼容 Linux / Windows）
  const auto pos = extrinsic_path.find_last_of("/\\");
  if (pos == std::string::npos) {
    return "intrinsic/" + child + ".json";
  }
  return extrinsic_path.substr(0, pos) + "/intrinsic/" + child + ".json";
}

}  // namespace

bool CameraParamsJson::LoadFromFileBase(
    const std::string& filename, Eigen::Matrix3f& intrinsic_matrix,
    Eigen::Matrix<float, 8, 1>& distortion_params,
    Eigen::Matrix4f& extrinsic_matrix, Eigen::Matrix4f& projection_matrix) {
  // This reader is dedicated to the dataset file format:
  // filename is the extrinsic json path, e.g.
  // "path/sensor_data/para/lidar_camera_1.json"
  try {
    json extrinsic_root;
    if (!config::ReadJsonFile(filename, &extrinsic_root)) {
      return false;
    }
    if (!extrinsic_root.contains("extrinsic")) {
      std::cerr << "Missing key 'extrinsic' in " << filename << std::endl;
      return false;
    }

    const json& extrinsic = extrinsic_root.at("extrinsic");

    // 1. Read rotation / translation (with unit conversion).
    Eigen::Matrix4f parsed_extrinsic;
    if (!config::ParseExtrinsicMatrix(extrinsic, filename,
                                      &parsed_extrinsic)) {
      return false;
    }

    // 2. Read intrinsic parameters (K / distortion).
    if (!extrinsic.contains("child")) {
      std::cerr << "Missing child sensor name in " << filename << std::endl;
      return false;
    }
    const std::string intrinsic_path =
        IntrinsicPath(filename, extrinsic.at("child").get<std::string>());
    json intrinsic_root;
    if (!config::ReadJsonFile(intrinsic_path, &intrinsic_root)) {
      return false;
    }
    if (!intrinsic_root.contains("intrinsic")) {
      std::cerr << "Missing key 'intrinsic' in " << intrinsic_path << std::endl;
      return false;
    }

    const json& intrinsic = intrinsic_root.at("intrinsic");
    Eigen::Matrix3f parsed_intrinsic;
    if (!intrinsic.contains("K") ||
        !config::ReadMatrix3f(intrinsic.at("K"), &parsed_intrinsic)) {
      std::cerr << "Invalid K format in " << intrinsic_path << std::endl;
      return false;
    }

    // 3. Read distortion coefficients.
    // OpenCV radtan is usually [k1, k2, p1, p2, k3].
    if (!intrinsic.contains("distortion")) {
      std::cerr << "Missing distortion in " << intrinsic_path << std::endl;
      return false;
    }
    const json& distortion = intrinsic.at("distortion");
    if (distortion.value("model", std::string()) != "radtan") {
      std::cerr << "Unsupported distortion model in " << intrinsic_path
                << std::endl;
      return false;
    }
    const json& radial     = distortion.at("k");
    const json& tangential = distortion.at("p");
    if (!radial.is_array() || radial.size() != 3 || !tangential.is_array() ||
        tangential.size() != 2) {
      std::cerr << "Invalid distortion coefficients in " << intrinsic_path
                << std::endl;
      return false;
    }

    Eigen::Matrix<float, 8, 1> parsed_distortion =
        Eigen::Matrix<float, 8, 1>::Zero();
    parsed_distortion(0) = radial.at(0).get<float>();
    parsed_distortion(1) = radial.at(1).get<float>();
    parsed_distortion(2) = tangential.at(0).get<float>();
    parsed_distortion(3) = tangential.at(1).get<float>();
    parsed_distortion(4) = radial.at(2).get<float>();

    // 4. Compute projection matrix P = K * [R | T].
    Eigen::Matrix4f parsed_projection = Eigen::Matrix4f::Identity();
    parsed_projection.block<3, 4>(0, 0) =
        parsed_intrinsic * parsed_extrinsic.block<3, 4>(0, 0);

    intrinsic_matrix  = parsed_intrinsic;
    distortion_params = parsed_distortion;
    extrinsic_matrix  = parsed_extrinsic;
    projection_matrix = parsed_projection;

    return true;
  } catch (const nlohmann::json::exception& error) {
    std::cerr << "Invalid camera calibration json '" << filename
              << "': " << error.what() << std::endl;
    return false;
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace jojo
