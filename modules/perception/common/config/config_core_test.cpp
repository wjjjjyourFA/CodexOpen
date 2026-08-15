#include <chrono>
#include <filesystem>
#include <fstream>

#include "modules/perception/common/config/sensor_extrinsics_json.h"
#include "modules/perception/common/config/vehicle_config.h"

int main() {
  const auto suffix =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = std::filesystem::temp_directory_path() /
                    ("perception_common_config_" + std::to_string(suffix));
  std::filesystem::create_directories(root);

  const auto extrinsic_file = root / "extrinsic.json";
  std::ofstream(extrinsic_file)
      << R"({"extrinsic":{"rotation":[[1,0,0],[0,1,0],[0,0,1]],"translation":[1000,0,0],"unit":"millimeter"}})";
  jojo::perception::config::SensorExtrinsicsJson extrinsics;
  if (!extrinsics.LoadFromFile(extrinsic_file.string()) ||
      extrinsics.GetMatrixVector().size() != 1 ||
      std::abs(extrinsics.GetMatrixVector()[0]->extrinsic_matrix(0, 3) - 1.0f) >
          1e-6f) {
    std::filesystem::remove_all(root);
    return 1;
  }
  if (extrinsics.LoadFromFile((root / "missing.json").string())) {
    std::filesystem::remove_all(root);
    return 2;
  }

  const auto vehicle_file = root / "vehicle.ini";
  std::ofstream(vehicle_file) << "RONI_min_x -1\nRONI_max_x 2\n"
                                 "RONI_min_y -3\nRONI_max_y 4\n"
                                 "RONI_min_z -5\nRONI_max_z 6\n";
  jojo::perception::config::VehicleConfig vehicle;
  if (!vehicle.LoadFromFile(vehicle_file.string()) ||
      vehicle.RONI_min_x != -1 || vehicle.RONI_max_z != 6) {
    std::filesystem::remove_all(root);
    return 3;
  }
  std::filesystem::remove_all(root);
  return 0;
}
