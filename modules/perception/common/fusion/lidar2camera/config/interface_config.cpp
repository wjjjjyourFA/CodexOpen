#include "modules/perception/common/fusion/lidar2camera/config/interface_config.h"

namespace jojo {
namespace perception {

void InterfaceConfig::LoadConfig(const std::string& config_path) {
  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    // 离散文件
    data_root_path = pt.get<std::string>("general.data_root_path", "");
    b_matched = pt.get<bool>("general.b_matched", 0);

    b_bin_or_pcd = pt.get<bool>("general.b_bin_or_pcd", 0);
    b_jpg_or_png = pt.get<bool>("general.b_jpg_or_png", 0);

    lidar_name = pt.get<std::string>("topics.lidar_name", "");
    image_name = pt.get<std::string>("topics.image_name", "");

    // 在线话题
    b_compressed = pt.get<bool>("general.b_compressed", true);

    image_topic = pt.get<std::string>("topics.image_topic", "");
    lidar_topic = pt.get<std::string>("topics.lidar_topic", "");

    rate = pt.get<int>("general.rate", 10);

    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }

  if (data_root_path != "") {
    InitPrefixPath();
  }

  // std::cout << "data_root_path: " << data_root_path << std::endl;
}

void InterfaceConfig::InitPrefixPath() {
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

}  // namespace perception
}  // namespace jojo
