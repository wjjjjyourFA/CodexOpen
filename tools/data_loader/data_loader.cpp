#include "tools/data_loader/data_loader.h"

namespace jojo {
namespace tools {
namespace common = apollo::cyber::common;
namespace camera = jojo::perception::camera;
namespace cfg    = jojo::perception::config;

DataLoader::DataLoader() {}

DataLoader::~DataLoader() {}

bool DataLoader::Init(std::shared_ptr<jojo::tools::RuntimeConfig> rparam,
                      std::shared_ptr<jojo::tools::InterfaceConfig> iparam) {
  if (!rparam || !iparam) {
    std::cerr << "[ERROR] DataLoader requires non-null configuration"
              << std::endl;
    return false;
  }

  rparam_ = rparam;
  iparam_ = iparam;

  if (iparam_->b_camera < 0 || iparam_->b_infra < 0 ||
      iparam_->b_star < 0 || iparam_->b_radar < 0 ||
      iparam_->b_radar4d < 0) {
    std::cerr << "[ERROR] sensor counts must not be negative" << std::endl;
    return false;
  }
  if (rparam_->start_time < 0 || rparam_->end_time < 0 ||
      (rparam_->end_time != 0 && rparam_->start_time > rparam_->end_time)) {
    std::cerr << "[ERROR] invalid playback time range" << std::endl;
    return false;
  }

  if (rparam_->b_do_undistort) {
    if (iparam_->b_camera || iparam_->b_infra || iparam_->b_star) {
      if (rparam_->camera_name.size() <
              static_cast<std::size_t>(iparam_->b_camera) ||
          rparam_->infra_name.size() <
              static_cast<std::size_t>(iparam_->b_infra) ||
          rparam_->star_name.size() <
              static_cast<std::size_t>(iparam_->b_star)) {
        std::cerr << "[ERROR] undistortion device-name count does not match "
                     "sensor count"
                  << std::endl;
        return false;
      }
      // 数据回放时，直接加载已经矫正后的图像
      InitUndistortion();
    }
  }

  if (iparam_->b_imu_data) {
    // TODO：读取IMU的标定矩阵
  }

  return true;
}

void DataLoader::InitUndistortion() {
  camera_params = std::make_shared<camera::CameraParams>();
  camera_params->SetLoadPath(rparam_->calib_file_dir);

  for (int i = 0; i < iparam_->b_camera; i++) {
    camera_params->LoadFromName(rparam_->camera_name.at(i));

    auto camera_undistort = std::make_shared<camera::UndistortionHandler>();
    camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
    undistort_vector.push_back(camera_undistort);
  }

  for (int i = 0; i < iparam_->b_infra; i++) {
    camera_params->LoadFromName(rparam_->infra_name.at(i));

    auto camera_undistort = std::make_shared<camera::UndistortionHandler>();
    camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
    undistort_vector.push_back(camera_undistort);
  }

  for (int i = 0; i < iparam_->b_star; i++) {
    camera_params->LoadFromName(rparam_->star_name.at(i));

    auto camera_undistort = std::make_shared<camera::UndistortionHandler>();
    camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
    undistort_vector.push_back(camera_undistort);
  }

  // clang-format off
  undistort_init.resize(iparam_->b_camera + iparam_->b_infra + iparam_->b_star, false);
  // clang-format on
}

void DataLoader::Start() {
  // std::cout << "DataLoader Start" << std::endl;
  this->LoadDataFolder();
}

std::vector<std::string> DataLoader::SetDataFolderVector(
    const std::string& prefix, const std::string& name, int count) {
  std::vector<std::string> path;

  if (count <= 0) return path;

  /* 如果只有一个相机，就是 camera，否则是 camera_1, camera_2 ...
  if (count == 1) {
    std::string tmp = prefix + "/" + name;
    path.push_back(tmp);
  } else {
    for (int i = 1; i <= count; i++) {
      std::string tmp = prefix + "/" + name + "_" + std::to_string(i);
      path.push_back(tmp);
    }
  }
  */
  // 保持同样的组织格式
  for (int i = 1; i <= count; i++) {
    std::string tmp = prefix + "/" + name + "_" + std::to_string(i);
    common::EnsureDirectory(tmp);
    path.push_back(tmp);
  }

  return path;
}

void DataLoader::LoadDataFolder() {
  this->prefix = rparam_->root_path + "/" + rparam_->file_name;
  std::cout << "data_file : " << this->prefix << std::endl;

  this->postfix = this->prefix + "-O";

  if (rparam_->use_bin_or_pcd == 0) {
    path_lidar = this->prefix + "/" + "lidar";
  } else {
    path_lidar = this->prefix + "/" + "lidar_pcd";
  }
  // common::CreateDir(path_lidar);

  // clang-format off
  if(iparam_->b_undistort) {
    path_camera = SetDataFolderVector(this->postfix, "sensor_data/camera_undistort", iparam_->b_camera);
    path_infra = SetDataFolderVector(this->postfix, "sensor_data/infra_undistort", iparam_->b_infra);
    path_star = SetDataFolderVector(this->postfix, "sensor_data/star_undistort", iparam_->b_star);
  } else {
    path_camera = SetDataFolderVector(this->prefix, "camera", iparam_->b_camera);
    path_infra = SetDataFolderVector(this->prefix, "infra", iparam_->b_infra);
    path_star = SetDataFolderVector(this->prefix, "star", iparam_->b_star);
  }
  // std::cout << "path_camera.size() = " << path_camera.size() << std::endl;
  // std::cout << "path_infra.size() = " << path_infra.size() << std::endl;

  path_global_pose = this->prefix + "/" + "global_pose" + ".txt";
  path_local_pose = this->prefix + "/" + "local_pose" + ".txt";
  path_imu_data = this->prefix + "/" + "imu_data" + ".txt";

  path_radar = this->prefix + "/" + "radar";

  path_radar4d = SetDataFolderVector(this->prefix, "radar4d", iparam_->b_radar4d);
  // clang-format on
}

// 没有直接加载，避免内存爆炸，使用时才加载真正的数据
bool DataLoader::LoadTimeStamp(const std::string& path,
                               const std::string& ts_file /*TimeStampFile*/,
                               DataContainerBase& data_container) {
  const std::string file = path + "/" + ts_file + ".txt";
  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load timestamp file: " << file << std::endl;
    return false;
  }

  std::ifstream input(file);
  if (!input.is_open()) {
    std::cerr << "[ERROR] Failed to open timestamp file: " << file << std::endl;
    return false;
  }

  uint64_t frame_time = 0;
  std::size_t count   = 0;
  while (input >> frame_time) {
    data_container.insert(frame_time, nullptr);
    ++count;
  }
  if (!input.eof()) {
    std::cerr << "[ERROR] Malformed timestamp file: " << file << std::endl;
    return false;
  }

  if (count == 0 || data_container.empty()) {
    std::cerr << "[ERROR] Empty timestamp file: " << file << std::endl;
    return false;
  }

  std::cout << "[INFO] load timestamp file " << file << std::endl;
  return true;
}

bool DataLoader::ExtractTimestamp(const std::string& path,
                                  DataContainerBase& data_container) {
  std::cout << "[INFO] Trying to extract from path: " << path << std::endl;

  if (!fs::exists(path) || !fs::is_directory(path)) {
    std::cerr << "[ERROR] Failed to extract timestamps from path: " << path
              << std::endl;
    return false;
  }

  std::vector<uint64_t> timestamps;
  timestamps.reserve(10000);

  for (const auto& entry : fs::directory_iterator(path)) {
    if (fs::is_regular_file(entry.path())) {  // ✔ 跨 C++14 / C++17 都支持
      std::string filename = entry.path().stem().string();  // 不带后缀
      try {
        uint64_t ts = std::stoull(filename);  // 转换成整数
        timestamps.push_back(ts);
      } catch (const std::exception& e) {
        std::cerr << "无法解析文件名为时间戳: " << filename
                  << "，错误: " << e.what() << std::endl;
      }
    }
  }

  if (timestamps.empty()) {
    std::cerr << "[WARN] no timestamp found" << std::endl;
    return false;
  }

  // 按时间戳排序
  std::sort(timestamps.begin(), timestamps.end());

  for (auto& frame_time : timestamps) {
    data_container.insert(frame_time, nullptr);
  }

  std::cout << "[INFO] extracted " << timestamps.size() << " timestamps"
            << std::endl;

  return true;
}

bool DataLoader::SaveTimeStamp(const std::string& path,
                               const std::string& ts_file,
                               const DataContainerBase& data_container) {
  common::CreateDir(path + "/timestamp");

  const std::string file = path + "/" + ts_file + ".txt";
  if (!common::CreateFile(file)) {
    std::cerr << "[ERROR] cannot create directory " << file << std::endl;
    return false;
  }

  std::ofstream output(file, std::ios::trunc);
  if (!output.is_open()) {
    std::cerr << "[ERROR] cannot write " << file << std::endl;
    return false;
  }

  std::vector<uint64_t> timestamps;
  data_container.GetAllTimeStamp(timestamps);
  for (auto& ts : timestamps) {
    output << ts << '\n';
  }
  if (!output) {
    std::cerr << "[ERROR] failed while writing " << file << std::endl;
    return false;
  }

  std::cout << "[INFO] save timestamp file " << file << std::endl;

  return true;
}

}  // namespace tools
}  // namespace jojo
