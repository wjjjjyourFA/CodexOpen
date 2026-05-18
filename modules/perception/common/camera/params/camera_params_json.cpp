#include "modules/perception/common/camera/params/camera_params_json.h"

namespace jojo {
namespace perception {
namespace camera {
using json = nlohmann::json;

bool CameraParamsJson::LoadFromFileBase(
    const std::string& filename, Eigen::Matrix3f& intrinsic_matrix,
    Eigen::Matrix<float, 8, 1>& distortion_params,
    Eigen::Matrix4f& extrinsic_matrix, Eigen::Matrix4f& projection_matrix) {
  // 此读取代码专用于 dataset 文件格式
  // filename ==> extrinsic_json_path
  // "path/sensor_data/para/lidar_camera_1.json";
  auto& extrinsic_json_path = filename;

  std::ifstream ifs_extrinsic(extrinsic_json_path);
  if (!ifs_extrinsic.is_open()) {
    std::cerr << "Cannot open extrinsic json file: " << extrinsic_json_path
              << std::endl;
    exit(-1);
  }

  json extrinsic_root;
  ifs_extrinsic >> extrinsic_root;
  if (!extrinsic_root.contains("extrinsic")) {
    std::cerr << "Missing key 'extrinsic' in " << extrinsic_json_path
              << std::endl;
    exit(-1);
  }

  const json& extrinsic = extrinsic_root.at("extrinsic");

  // 3. 读取 R
  const json& rotation_json = extrinsic.at("rotation");
  if (!rotation_json.is_array() || rotation_json.size() != 3) {
    std::cerr << "Invalid rotation format in " << extrinsic_json_path
              << std::endl;
    exit(-1);
  }

  for (int i = 0; i < 3; ++i) {
    if (!rotation_json.at(i).is_array() || rotation_json.at(i).size() != 3) {
      std::cerr << "Invalid rotation format in " << extrinsic_json_path
                << std::endl;
      exit(-1);
    }

    for (int j = 0; j < 3; ++j) {
      extrinsic_matrix(i, j) = rotation_json.at(i).at(j).get<float>();
    }
  }

  // 4. 读取 T
  // 原始单位是 millimeter，这里统一转成 millimeter
  // 如果你后续代码一直用米，可去掉 *1000.0
  const json& translation_json = extrinsic.at("translation");
  if (!translation_json.is_array() || translation_json.size() != 3) {
    std::cerr << "Invalid translation format in " << extrinsic_json_path
              << std::endl;
    exit(-1);
  }

  std::string unit = "meter";
  if (extrinsic.contains("unit")) {
    unit = extrinsic.at("unit").get<std::string>();
  }

  double unit_scale = 1.0;
  if (unit == "millimeter") {
    unit_scale = 1.0 / 1000.0f;
  } else if (unit == "meter") {
    unit_scale = 1.0;
  } else {
    std::cerr << "Unsupported translation unit: " << unit << std::endl;
    exit(-1);
  }

  for (int i = 0; i < 3; ++i) {
    extrinsic_matrix(i, 3) = translation_json.at(i).get<float>() * unit_scale;
  }

  // Set the last row for RT and P matrices
  extrinsic_matrix(3, 0)  = 0;
  extrinsic_matrix(3, 1)  = 0;
  extrinsic_matrix(3, 2)  = 0;
  extrinsic_matrix(3, 3)  = 1;
  projection_matrix(3, 0) = 0;
  projection_matrix(3, 1) = 0;
  projection_matrix(3, 2) = 0;
  projection_matrix(3, 3) = 1;

  // clang-format off
  std::string child = extrinsic.at("child").get<std::string>();
  const std::string& path = extrinsic_json_path;
  // 找最后一个路径分隔符（兼容 Linux / Windows）
  auto pos = path.find_last_of("/\\");
  std::string parent = (pos == std::string::npos) ? "" : path.substr(0, pos);
  const std::string intrinsic_json_path = parent + "/intrinsic/" + child + ".json";
  // clang-format on

  std::ifstream ifs_intrinsic(intrinsic_json_path);
  if (!ifs_intrinsic.is_open()) {
    std::cerr << "Cannot open intrinsic json file: " << intrinsic_json_path
              << std::endl;
    exit(-1);
  }

  json intrinsic_root;
  ifs_intrinsic >> intrinsic_root;
  if (!intrinsic_root.contains("intrinsic")) {
    std::cerr << "Missing key 'intrinsic' in " << intrinsic_json_path
              << std::endl;
    exit(-1);
  }

  const json& intrinsic = intrinsic_root.at("intrinsic");
  // 1. 读取 K
  const json& k_json = intrinsic.at("K");
  if (!k_json.is_array() || k_json.size() != 3) {
    std::cerr << "Invalid K format in " << intrinsic_json_path << std::endl;
    exit(-1);
  }

  for (int i = 0; i < 3; ++i) {
    if (!k_json.at(i).is_array() || k_json.at(i).size() != 3) {
      std::cerr << "Invalid K format in " << intrinsic_json_path << std::endl;
      exit(-1);
    }

    for (int j = 0; j < 3; ++j) {
      intrinsic_matrix(i, j) = k_json.at(i).at(j).get<float>();
    }
  }

  // 2. 读取畸变 D
  // OpenCV radtan 一般按 [k1, k2, p1, p2, k3]
  const json& distortion_json  = intrinsic.at("distortion");
  std::string distortion_model = distortion_json.at("model").get<std::string>();
  if (distortion_model != "radtan") {
    std::cerr << "Unsupported distortion model: " << distortion_model
              << std::endl;
    exit(-1);
  }

  const json& k_dist = distortion_json.at("k");
  if (!k_dist.is_array() || k_dist.size() != 3) {
    std::cerr << "Invalid distortion.k format in " << intrinsic_json_path
              << std::endl;
    exit(-1);
  }

  const json& p_dist = distortion_json.at("p");
  if (!p_dist.is_array() || p_dist.size() != 2) {
    std::cerr << "Invalid distortion.p format in " << intrinsic_json_path
              << std::endl;
    exit(-1);
  }

  distortion_params(0) = k_dist.at(0).get<float>();  // k1
  distortion_params(1) = k_dist.at(1).get<float>();  // k2
  distortion_params(2) = p_dist.at(0).get<float>();  // p1
  distortion_params(3) = p_dist.at(1).get<float>();  // p2
  distortion_params(4) = k_dist.at(2).get<float>();  // k3

  // 5. 计算投影矩阵 P = K * [R | T]
  /*
  cv::Mat RT;
  cv::hconcat(R, T, RT);
  P = K * RT;
  */
  projection_matrix.block<3, 4>(0, 0) =
      intrinsic_matrix * extrinsic_matrix.block<3, 4>(0, 0);
}

}  // namespace camera
}  // namespace perception
}  // namespace jojo