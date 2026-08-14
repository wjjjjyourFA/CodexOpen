#include "modules/perception/common/camera/params/camera_params_json.h"

#include <fstream>
#include <iostream>

namespace jojo {
namespace perception {
namespace camera {
namespace {

using json = nlohmann::json;

bool ReadJson(const std::string& filename, json* root) {
  std::ifstream input(filename);
  if (!input) {
    std::cerr << "Cannot open json file: " << filename << std::endl;
    return false;
  }
  input >> *root;
  return true;
}

bool ReadMatrix3f(const json& value, Eigen::Matrix3f* matrix) {
  if (!value.is_array() || value.size() != 3) {
    return false;
  }
  for (int row = 0; row < 3; ++row) {
    if (!value.at(row).is_array() || value.at(row).size() != 3) {
      return false;
    }
    for (int col = 0; col < 3; ++col) {
      (*matrix)(row, col) = value.at(row).at(col).get<float>();
    }
  }
  return true;
}

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
  // 此读取代码专用于 dataset 文件格式
  // filename ==> extrinsic_json_path
  // "path/sensor_data/para/lidar_camera_1.json";
  try {
    json extrinsic_root;
    if (!ReadJson(filename, &extrinsic_root) ||
        !extrinsic_root.contains("extrinsic")) {
      std::cerr << "Missing key 'extrinsic' in " << filename << std::endl;
      return false;
    }

    const json& extrinsic = extrinsic_root.at("extrinsic");

    // 3. 读取 R
    Eigen::Matrix4f parsed_extrinsic = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotation;
    if (!extrinsic.contains("rotation") ||
        !ReadMatrix3f(extrinsic.at("rotation"), &rotation)) {
      std::cerr << "Invalid rotation format in " << filename << std::endl;
      return false;
    }
    parsed_extrinsic.block<3, 3>(0, 0) = rotation;

    // 4. 读取 T
    // 原始单位是 millimeter，这里统一转成 millimeter
    // 如果你后续代码一直用米，可去掉 *1000.0
    if (!extrinsic.contains("translation")) {
      std::cerr << "Missing translation in " << filename << std::endl;
      return false;
    }
    const json& translation = extrinsic.at("translation");
    if (!translation.is_array() || translation.size() != 3) {
      std::cerr << "Invalid translation format in " << filename << std::endl;
      return false;
    }

    const std::string unit = extrinsic.value("unit", "meter");
    float unit_scale       = 1.0f;
    if (unit == "millimeter") {
      unit_scale = 1.0f / 1000.0f;
    } else if (unit != "meter") {
      std::cerr << "Unsupported translation unit: " << unit << std::endl;
      return false;
    }
    for (int i = 0; i < 3; ++i) {
      parsed_extrinsic(i, 3) = translation.at(i).get<float>() * unit_scale;
    }

    // 读取其他参数
    if (!extrinsic.contains("child")) {
      std::cerr << "Missing child sensor name in " << filename << std::endl;
      return false;
    }
    const std::string intrinsic_path =
        IntrinsicPath(filename, extrinsic.at("child").get<std::string>());
    json intrinsic_root;
    if (!ReadJson(intrinsic_path, &intrinsic_root) ||
        !intrinsic_root.contains("intrinsic")) {
      std::cerr << "Missing key 'intrinsic' in " << intrinsic_path << std::endl;
      return false;
    }

    const json& intrinsic = intrinsic_root.at("intrinsic");
    // 1. 读取 K
    Eigen::Matrix3f parsed_intrinsic;
    if (!intrinsic.contains("K") ||
        !ReadMatrix3f(intrinsic.at("K"), &parsed_intrinsic)) {
      std::cerr << "Invalid K format in " << intrinsic_path << std::endl;
      return false;
    }

    // 2. 读取畸变 D
    // OpenCV radtan 一般按 [k1, k2, p1, p2, k3]
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

    // k1 k2 p1 p2 k3
    Eigen::Matrix<float, 8, 1> parsed_distortion =
        Eigen::Matrix<float, 8, 1>::Zero();
    parsed_distortion(0) = radial.at(0).get<float>();
    parsed_distortion(1) = radial.at(1).get<float>();
    parsed_distortion(2) = tangential.at(0).get<float>();
    parsed_distortion(3) = tangential.at(1).get<float>();
    parsed_distortion(4) = radial.at(2).get<float>();

    // 5. 计算投影矩阵 P = K * [R | T]
    /*
    cv::Mat RT;
    cv::hconcat(R, T, RT);
    P = K * RT;
    */
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
