#include "modules/tools/image_undistortion/config/runtime_config.h"

namespace jojo {
namespace tools {

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
    else if(key == "LoadResolveFile")       fin >> resolve_file;
    else if(key == "calib_file_path")       fin >> calib_file_path;
  }
  fin.close();
  // clang-format on
  */

  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);
    resolve_file = pt.get<std::string>("general.ResolveFile", "");
    b_matched = pt.get<bool>("general.b_matched", false);

    calib_file_path = pt.get<std::string>("calibration.calib_file_path", "");
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }

  LoadResolveFile(resolve_file);
};

void RuntimeConfig::LoadResolveFile(const std::string& file_path) {
  /*
  std::ifstream fin(file_path.c_str());
  if (fin.is_open() != 1) {
    std::cerr << node_name_ << " fail to open resolve file: " << file_path
              << std::endl;
    abort();
  }

  // clang-format off
  std::string key;
  while (fin >> key) {
    if (key[0] == '#' || key[0] == '/')
      std::getline(fin, key);
    else if (key == "LoadPath")             fin >> data_root_path;
  }
  fin.close();
  // clang-format on
  */

  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(file_path, pt);

    data_root_path = pt.get<std::string>("general.LoadPath", "");
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

void RuntimeConfig::InitPrefixPath() {}

}  // namespace tools
}  // namespace jojo
