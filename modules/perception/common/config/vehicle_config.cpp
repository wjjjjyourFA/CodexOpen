#include "modules/perception/common/config/vehicle_config.h"

namespace jojo {
namespace perception {
namespace config {

void VehicleConfig::SetLoadPath(const std::string& load_path) {
  LoadPath = load_path;
}

void VehicleConfig::LoadFromFile(const std::string& config_file) {
  std::ifstream fin(config_file);
  if (!fin.is_open()) {
    std::cerr << "Cannot open config file: " << config_file << std::endl;
    exit(-1);
  }

  std::string t_s;
  while (fin >> t_s) {
    if (t_s[0] == '#' || t_s[0] == '/')
      std::getline(fin, t_s);
    else if (t_s == "RONI_min_x")
      fin >> RONI_min_x;
    else if (t_s == "RONI_max_x")
      fin >> RONI_max_x;
    else if (t_s == "RONI_min_y")
      fin >> RONI_min_y;
    else if (t_s == "RONI_max_y")
      fin >> RONI_max_y;
    else if (t_s == "RONI_min_z")
      fin >> RONI_min_z;
    else if (t_s == "RONI_max_z")
      fin >> RONI_max_z;
  }
  fin.close();
}

}  // namespace config
}  // namespace perception
}  // namespace jojo