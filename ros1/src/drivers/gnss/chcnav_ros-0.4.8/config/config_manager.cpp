#include "config_manager.h"

namespace jojo {
namespace drivers {

std::string remove_filename_from_path(const std::string& path) {
  // 查找最后一个 '/' 或 '\'
  size_t pos = path.find_last_of("/\\");
  if (pos == std::string::npos) {
    // 没有分隔符，说明没有路径，直接返回空
    return "";
  }
  // 返回路径部分（不包含最后的文件名）
  return path.substr(0, pos);
}

void ConfigManager::LoadConfig(const std::string& config_path) {
  // std::cout<< "config_path: " << config_path << std::endl;
  std::string prefix = remove_filename_from_path(config_path);
  // std::cout<< "prefix: " << prefix << std::endl;
  vehicle_model_config.load_configs(config_path, prefix);

  /* useless
  if(vehicle_model_config.gnss_configs.size() > 1) {
    std::cerr << "Warning: only surpport one gps!" << std::endl;
    return;
  }
  */
}

}  // namespace drivers
}  // namespace jojo
