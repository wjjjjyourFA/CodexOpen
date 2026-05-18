#include "modules/dreamview/map_center_view/config/runtime_config.h"

namespace jojo {
namespace dreamview {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);

    // clang-format off
    // 从文件中读取配置项

    // 打印其他参数...
    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace dreamview
}  // namespace jojo