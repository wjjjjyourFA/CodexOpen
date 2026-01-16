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
    else if(key == "b_undistort")      fin >> b_undistort;

    else if(key == "b_lt_none_rt")     fin >> b_lt_none_rt;
  }
  fin.close();
  // clang-format on
  */

  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);
    data_root_path = pt.get<std::string>("general.data_root_path", "");

    b_bin_or_pcd = pt.get<bool>("general.b_bin_or_pcd", 0);
    b_jpg_or_png = pt.get<bool>("general.b_jpg_or_png", 0);
    b_undistort = pt.get<bool>("general.b_undistort", 0);
    b_lt_none_rt = pt.get<int>("general.b_lt_none_rt", 1);
    b_matched = pt.get<bool>("general.b_matched", 0);
    
    dist_threshold = pt.get<int>("general.dist_threshold", 100);

    lidar_name = pt.get<std::string>("topics.lidar_name", "");
    image_name = pt.get<std::string>("topics.image_name", "");

    calib_file_path = pt.get<std::string>("calibration.calib_file_path", "");
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }

  InitPrefixPath();
}

void RuntimeConfig::InitPrefixPath() {
  if (data_root_path == "./../data/PerceptionFuse") {
    if (!b_bin_or_pcd) {
      lidar_file = data_root_path + "/" + lidar_name + ".bin";
    } else {
      lidar_file = data_root_path + "/" + lidar_name + ".pcd";
    }

    if (!b_jpg_or_png) {
      image_file = data_root_path + "/" + image_name + ".jpg";
    } else {
      image_file = data_root_path + "/" + image_name + ".png";
    }
  } else {
    std::string _lidar_file;
    std::string _image_file;

    if (!b_bin_or_pcd) {
      _lidar_file = lidar_name + ".bin";
    } else {
      _lidar_file = lidar_name + ".pcd";
    }

    if (!b_jpg_or_png) {
      _image_file = image_name + ".jpg";
    } else {
      _image_file = image_name + ".png";
    }

    if (b_matched) {
      lidar_file = data_root_path + "/matched/lidar/" + _lidar_file;
      image_file = data_root_path + "/matched/image/" + _image_file;
    } else {
      lidar_file = data_root_path + "/lidar/" + _lidar_file;
      image_file = data_root_path + "/image/" + _image_file;
    }
  }
}

}  // namespace tools
}  // namespace jojo
