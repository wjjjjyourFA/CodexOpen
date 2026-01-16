#include "modules/tools/sensor_calibration/camera_intrinsic/intrinsic_calib/config/runtime_config.h"

namespace jojo {
namespace tools {

void RuntimeConfig::LoadConfig(const std::string& config_path) {
  try {
    // clang-format off
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_path, pt);
    data_root_path = pt.get<std::string>("general.data_root_path", "");

    grid_size = pt.get<int>("general.grid_size", 5);
    corner_width = pt.get<int>("general.corner_width", 15);
    corner_height = pt.get<int>("general.corner_height", 17);

    IMAGE_MARGIN_PERCENT = pt.get<float>("autopicker.IMAGE_MARGIN_PERCENT", 0.1);
    CHESSBOARD_MIN_AREA_PORTION = pt.get<float>("autopicker.CHESSBOARD_MIN_AREA_PORTION", 0.04);
    CHESSBOARD_MIN_ANGLE = pt.get<float>("autopicker.CHESSBOARD_MIN_ANGLE", 40.0);
    CHESSBOARD_MIN_MOVE_THRESH = pt.get<float>("autopicker.CHESSBOARD_MIN_MOVE_THRESH", 0.08);
    CHESSBOARD_MIN_ANGLE_CHANGE_THRESH = pt.get<float>("autopicker.CHESSBOARD_MIN_ANGLE_CHANGE_THRESH", 5.0);
    CHESSBOARD_MIN_AREA_CHANGE_PARTION_THRESH = pt.get<float>("autopicker.CHESSBOARD_MIN_AREA_CHANGE_PARTION_THRESH", 0.005);
    MAX_SELECTED_SAMPLE_NUM = pt.get<float>("autopicker.MAX_SELECTED_SAMPLE_NUM", 45);
    REGION_MAX_SELECTED_SAMPLE_NUM = pt.get<float>("autopicker.REGION_MAX_SELECTED_SAMPLE_NUM", 15);

    // clang-format on
  } catch (const std::exception& e) {
    std::cerr << "Error reading ini file: " << e.what() << std::endl;
  }
}

}  // namespace tools
}  // namespace jojo
