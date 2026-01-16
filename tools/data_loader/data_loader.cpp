#include "tools/data_loader/data_loader.h"

namespace jojo {
namespace tools {
namespace common = apollo::cyber::common;
namespace camera = jojo::perception::camera;
namespace cfg = jojo::perception::config;

DataLoader::DataLoader() {}

DataLoader::~DataLoader() {}

void DataLoader::Init(std::shared_ptr<RuntimeConfigOffline> param) {
  param_ = param;

  if (param_->b_camera || param_->b_infra || param_->b_star) {
    // 数据回放时，直接加载已经矫正后的图像
    // InitUndis();
  }
}

void DataLoader::InitUndis() {
  camera_params = std::make_shared<camera::CameraParams>();
  camera_params->SetLoadPath(param_->calib_file_path);

  for (int i = 0; i < param_->b_camera; i++) {
    camera_params->LoadFromName(param_->camera_name.at(i));

    auto camera_undist = std::make_shared<camera::UndistortionHandler>();
    undistort_vector.push_back(camera_undist);
  }

  for (int i = 0; i < param_->b_infra; i++) {
    camera_params->LoadFromName(param_->infra_name.at(i));

    auto camera_undist = std::make_shared<camera::UndistortionHandler>();
    undistort_vector.push_back(camera_undist);
  }

  for (int i = 0; i < param_->b_star; i++) {
    camera_params->LoadFromName(param_->star_name.at(i));

    auto camera_undist = std::make_shared<camera::UndistortionHandler>();
    undistort_vector.push_back(camera_undist);
  }

  // clang-format off
  undistort_init.resize(param_->b_camera + param_->b_infra + param_->b_star, false);
  // clang-format on
}

void DataLoader::Start() {
  sleep(1);

  // std::cout << "DataLoader Start" << std::endl;
  this->LoadDataFolder();
}

std::vector<std::string> DataLoader::SetDataFolderVector(
    const std::string& prefix, const std::string& name, int count) {
  std::vector<std::string> path;

  if (count == 1) {
    std::string tmp = prefix + "/" + name;
    path.push_back(tmp);
  } else {
    for (int i = 1; i <= count; i++) {
      std::string tmp = prefix + "/" + name + "_" + std::to_string(i);
      path.push_back(tmp);
    }
  }

  return path;
}

void DataLoader::LoadDataFolder() {
  this->prefix = param_->root_path + "/" + param_->file_name;
  std::cout << "data_file : " << this->prefix << std::endl;

  if (param_->use_bin_or_pcd == 0) {
    path_lidar = this->prefix + "/" + "lidar";
  } else {
    path_lidar = this->prefix + "/" + "lidar_pcd";
  }

  // clang-format off
  if(param_->b_undistort) {
    path_camera_u = SetDataFolderVector(this->prefix, "undistort_image", param_->b_camera);
    path_infra_u = SetDataFolderVector(this->prefix, "undistort_infra", param_->b_infra);
    path_star_u = SetDataFolderVector(this->prefix, "undistort_star", param_->b_star);
  } else {
    path_camera = SetDataFolderVector(this->prefix, "image", param_->b_camera);
    path_infra = SetDataFolderVector(this->prefix, "infra", param_->b_infra);
    path_star = SetDataFolderVector(this->prefix, "star", param_->b_star);
  }
  // std::cout << "path_camera_u.size() = " << path_camera_u.size() << std::endl;

  path_global_pose = this->prefix + "/" + "global_pose" + ".txt";
  path_local_pose = this->prefix + "/" + "local_pose" + ".txt";
  path_imu_data = this->prefix + "/" + "imu_data" + ".txt";

  path_radar = this->prefix + "/" + "radar";

  path_radar_4d = SetDataFolderVector(this->prefix, "radar_4d", param_->b_radar_4d);
  // clang-format on
}

// 没有直接加载，避免内存爆炸，使用时才加载真正的数据
bool DataLoader::LoadTimeStamp(const std::string& path,
                               const std::string& ts_file /*TimeStampFile*/,
                               DataContainerBase& data_container) {
  uint64_t frame_time;

  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), ts_file.c_str());
  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load timestamp file: " << file << std::endl;
    return false;
  }

  FILE* _fp = fopen(file, "r");
  if (_fp != NULL) {
    while (!feof(_fp)) {
      fscanf(_fp, "%ld\n", &frame_time);
      data_container.insert(frame_time, nullptr);
    }
    fclose(_fp);
    return true;
  }

  return false;
}

bool DataLoader::ExtractTimestamp(const std::string& path,
                                  DataContainerBase& data_container) {
  std::cout << "[INFO] Trying to extract from path: " << path << std::endl;
  std::vector<uint64_t> timestamps;

  if (!fs::exists(path) || !fs::is_directory(path)) {
    std::cerr << "[ERROR] Failed to extract timestamps from path: " << path
              << std::endl;
    return false;
  }

  for (const auto& entry : fs::directory_iterator(path)) {
    if (fs::is_regular_file(entry.path())) {   // ✔ 跨 C++14 / C++17 都支持
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

  // 按时间戳排序
  std::sort(timestamps.begin(), timestamps.end());

  for (auto& frame_time : timestamps) {
    data_container.insert(frame_time, nullptr);
  }

  return true;
}

}  // namespace tools
}  // namespace jojo
