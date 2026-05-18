#include "modules/localization/fast_lio/config/static_config.h"

namespace jojo {
namespace localization {
using json = nlohmann::json;

void StaticConfig::LoadConfig(const std::string& config_path) {
  try {
    yaml = YAML::LoadFile(config_path);

    NUM_MAX_ITERATIONS = yaml["mapping"]["max_iteration"].as<int>(4);

    map_file_path = yaml["map_file_path"].as<std::string>("");
    // std::cout << "map_file_path: " << map_file_path << std::endl;

    init_method = yaml["init"]["init_method"].as<int>(0);
    this->SwitchMethod(init_method);

    b_init_twice   = yaml["init"]["b_init_twice"].as<bool>(true);
    b_display_init = yaml["init"]["b_display_init"].as<bool>(false);

    filter_size_surf_min = yaml["filter_size_surf"].as<double>(0.5);
    filter_size_map_min  = yaml["filter_size_map"].as<double>(0.5);
    cube_len             = yaml["cube_side_length"].as<double>(200);

    DET_RANGE = yaml["mapping"]["det_range"].as<float>(300.f);
    fov_deg   = yaml["mapping"]["fov_degree"].as<double>(180);
    gyr_cov   = yaml["mapping"]["gyr_cov"].as<double>(0.1);
    acc_cov   = yaml["mapping"]["acc_cov"].as<double>(0.1);
    b_gyr_cov = yaml["mapping"]["b_gyr_cov"].as<double>(0.0001);
    b_acc_cov = yaml["mapping"]["b_acc_cov"].as<double>(0.0001);

    extrinsic_est_en = yaml["mapping"]["extrinsic_est_en"].as<bool>(true);

    lidar_type = yaml["preprocess"]["lidar_type"].as<int>(RS128);

    blind     = yaml["preprocess"]["blind"].as<double>(5);
    N_SCANS   = yaml["preprocess"]["scan_line"].as<int>(16);
    time_unit = yaml["preprocess"]["timestamp_unit"].as<int>(US);
    SCAN_RATE = yaml["preprocess"]["scan_rate"].as<int>(10);

    point_filter_num = yaml["point_filter_num"].as<int>(5);
    feature_enabled  = yaml["feature_extract_enable"].as<bool>(false);

  } catch (const std::exception& e) {
    std::cerr << "Fail to open yaml file: " << e.what() << std::endl;
  }
}

void StaticConfig::SwitchMethod(int method) {
  // 状态清理
  b_chaos_start = false;
  b_hand_start  = false;

  switch (method) {
    case 0:
      b_chaos_start    = true;
      map_center       = Eigen::Vector3d::Zero();
      yaw_offset_euler = yaml["init"]["yaw_offset_euler"].as<double>(0);
      break;
    case 1:
      map_center       = Vec3FromYaml(yaml["init"]["map_center"]);
      yaw_offset_euler = yaml["init"]["yaw_offset_euler"].as<double>(0);
      break;
    case 2:
      b_hand_start   = true;
      hand_init_pose = yaml["init"]["hand_init_pose"].as<vector<double>>();
      if (hand_init_pose.size() != 6) {
        throw std::runtime_error("hand_init_pose must be size 6");
      }
      // yaw_offset_euler = hand_init_pose.at(5);
      break;
    case 3: {
      // read map_info.json
      map_info_file = yaml["init"]["map_info_file"].as<std::string>("map_info");

      // clang-format off
      // 找最后一个路径分隔符（兼容 Linux / Windows）
      auto pos = map_file_path.find_last_of("/\\");
      std::string parent = (pos == std::string::npos) ? "" : map_file_path.substr(0, pos);
      const std::string m_i_path = parent + "/" + map_info_file + ".json";
      // std::cout << "m_i_path: " << m_i_path << std::endl;
      // clang-format on

      this->LoadJson(m_i_path);
      break;
    }
    default:
      break;
  }
}

void StaticConfig::LoadJson(const std::string& config_path) {
  std::ifstream ifs(config_path);
  if (!ifs.is_open()) {
    std::cerr << "Config open failed: " << config_path << std::endl;
    return;
  }

  nlohmann::json j;
  try {
    ifs >> j;
    // 用 JSON 反序列化出来的 一个全新的 StaticConfig 对象，整体赋值覆盖当前对象，重置原有的数据
    // *this = j.get<StaticConfig>();
    // 只改需要的值
    from_json(j, *this);
  } catch (std::exception& e) {
    std::cerr << "JSON parse error: " << e.what() << std::endl;
    return;
  }
}

void from_json(const nlohmann::json& j, StaticConfig& c) {
  /* way 1
  const auto& m = j.at("map_center");
  for (int i = 0; i < 3; ++i) {
    c.map_center(i) = m.at(i).get<double>();
  }
  */
  // way 2
  const auto m = j.at("map_center").get<std::vector<double>>();
  c.map_center = Eigen::Map<const Eigen::Vector3d>(m.data());

  const auto& p = j.at("pose_offset");
  c.pose_offset.clear();
  for (int i = 0; i < 6; ++i) {
    c.pose_offset.push_back(p.at(i).get<double>());
  }
  c.yaw_offset_euler = c.pose_offset.at(5);
}

}  // namespace localization
}  // namespace jojo
