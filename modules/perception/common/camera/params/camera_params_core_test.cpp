#include <chrono>
#include <filesystem>
#include <fstream>

#include "modules/perception/common/camera/params/camera_params_json.h"

int main() {
  const auto suffix =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = std::filesystem::temp_directory_path() /
                    ("perception_common_camera_" + std::to_string(suffix));
  std::filesystem::create_directories(root / "intrinsic");
  const auto extrinsic_file = root / "lidar_camera.json";
  std::ofstream(extrinsic_file)
      << R"({"extrinsic":{"rotation":[[1,0,0],[0,1,0],[0,0,1]],"translation":[1000,0,0],"unit":"millimeter","child":"cam0"}})";
  std::ofstream(root / "intrinsic/cam0.json")
      << R"({"intrinsic":{"K":[[100,0,10],[0,100,20],[0,0,1]],"distortion":{"model":"radtan","k":[0.1,0.2,0.3],"p":[0.01,0.02]}}})";

  jojo::perception::camera::CameraParamsJson params;
  if (!params.LoadFromFile(extrinsic_file.string()) ||
      params.GetMatrixVector().size() != 1) {
    std::filesystem::remove_all(root);
    return 1;
  }
  const auto& matrix = params.GetMatrixVector()[0];
  if (!matrix || std::abs(matrix->extrinsic_matrix(0, 3) - 1.0f) > 1e-6f ||
      std::abs(matrix->projection_matrix(0, 3) - 100.0f) > 1e-4f) {
    std::filesystem::remove_all(root);
    return 2;
  }
  jojo::perception::camera::CameraParamsJson invalid;
  std::ofstream(root / "invalid.json") << "{}";
  if (invalid.LoadFromFile((root / "invalid.json").string())) {
    std::filesystem::remove_all(root);
    return 3;
  }
  std::filesystem::remove_all(root);
  return 0;
}
