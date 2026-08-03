#include "modules/perception/common/fusion/lidar2camera/config/runtime_config.h"

namespace jojo {
namespace perception {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  /*
  std::ifstream fin(config_path.c_str());
  if (fin.is_open() != 1) {
    std::cerr << node_name_ << " fail to open params file: " << config_path
              << std::endl;
    abort();
  }

  // clang-format off
  std::string key;
  while(fin >> key) {
    if(key[0] =='#' || key[0] == '/')
        std::getline(fin, key);
    else if(key == "LidarName")            fin >> lidar_name;
    else if(key == "ImageName")            fin >> image_name;
    else if(key == "calib_file_path")  fin >> calib_file_path;
    else if(key == "data_root_path")   fin >> data_root_path;

    else if(key == "b_bin_or_pcd")     fin >> b_bin_or_pcd;
    else if(key == "b_jpg_or_png")     fin >> b_jpg_or_png;
    else if(key == "b_do_undistort")      fin >> b_do_undistort;

    else if(key == "b_lt_none_rt")     fin >> b_lt_none_rt;
  }
  fin.close();
  // clang-format on
  */

  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    b_do_undistort = pt.get<bool>("general.b_do_undistort", 0);
    b_lt_none_rt = pt.get<int>("general.b_lt_none_rt", 1);
    
    dist_threshold = pt.get<int>("general.dist_threshold", 100);

    calib_file_path = pt.get<std::string>("calibration.calib_file_path", "");
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }

}

}  // namespace tools
}  // namespace jojo
