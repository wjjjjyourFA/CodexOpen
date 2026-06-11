#include "tools/data_loader/data_loader.h"

namespace jojo {
namespace tools {
namespace common = apollo::cyber::common;
namespace camera = jojo::perception::camera;
namespace cfg    = jojo::perception::config;

DataLoader::DataLoader() {}

DataLoader::~DataLoader() {}

void DataLoader::Init(std::shared_ptr<RuntimeConfigOffline> param) {
  param_ = param;

  if (param_->b_camera || param_->b_infra || param_->b_star) {
    if (param_->b_do_undistort) {
      // 数据回放时，直接加载已经矫正后的图像
      InitUndistortion();
    }
  }

  if (param_->b_imu_data){
    // TODO：读取IMU的标定矩阵
  }
}

void DataLoader::InitUndistortion() {
  camera_params = std::make_shared<camera::CameraParams>();
  camera_params->SetLoadPath(param_->calib_file_dir);

  for (int i = 0; i < param_->b_camera; i++) {
    camera_params->LoadFromName(param_->camera_name.at(i));

    auto camera_undistort = std::make_shared<camera::UndistortionHandler>();
    camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
    undistort_vector.push_back(camera_undistort);
  }

  for (int i = 0; i < param_->b_infra; i++) {
    camera_params->LoadFromName(param_->infra_name.at(i));

    auto camera_undistort = std::make_shared<camera::UndistortionHandler>();
    camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
    undistort_vector.push_back(camera_undistort);
  }

  for (int i = 0; i < param_->b_star; i++) {
    camera_params->LoadFromName(param_->star_name.at(i));

    auto camera_undistort = std::make_shared<camera::UndistortionHandler>();
    camera_undistort->InitModel(camera::CameraDistortionModel::Brown);
    undistort_vector.push_back(camera_undistort);
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
  this->prefix = param_->root_path + "/" + param_->file_name;
  std::cout << "data_file : " << this->prefix << std::endl;

  this->postfix = this->prefix + "-O";

  if (param_->use_bin_or_pcd == 0) {
    path_lidar = this->prefix + "/" + "lidar";
  } else {
    path_lidar = this->prefix + "/" + "lidar_pcd";
  }
  // common::CreateDir(path_lidar);

  // clang-format off
  if(param_->b_undistort) {
    path_camera = SetDataFolderVector(this->postfix, "sensor_data/camera_undistort", param_->b_camera);
    path_infra = SetDataFolderVector(this->postfix, "sensor_data/infra_undistort", param_->b_infra);
    path_star = SetDataFolderVector(this->postfix, "sensor_data/star_undistort", param_->b_star);
  } else {
    path_camera = SetDataFolderVector(this->prefix, "camera", param_->b_camera);
    path_infra = SetDataFolderVector(this->prefix, "infra", param_->b_infra);
    path_star = SetDataFolderVector(this->prefix, "star", param_->b_star);
  }
  // std::cout << "path_camera.size() = " << path_camera.size() << std::endl;

  path_global_pose = this->prefix + "/" + "global_pose" + ".txt";
  path_local_pose = this->prefix + "/" + "local_pose" + ".txt";
  path_imu_data = this->prefix + "/" + "imu_data" + ".txt";

  path_radar = this->prefix + "/" + "radar";

  path_radar4d = SetDataFolderVector(this->prefix, "radar4d", param_->b_radar4d);
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
      fscanf(_fp, "%lu\n", &frame_time);
      data_container.insert(frame_time, nullptr);
    }
    fclose(_fp);
    std::cout << "[INFO] load timestamp file " << file << std::endl;
    return true;
  }

  return false;
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

  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), ts_file.c_str());
  if (!common::CreateFile(file)) {
    std::cerr << "[ERROR] cannot create directory " << file << std::endl;
    return false;
  }

  FILE* fp = fopen(file, "w");
  if (!fp) {
    std::cerr << "[ERROR] cannot write " << file << std::endl;
    return false;
  }

  std::vector<uint64_t> timestamps;
  data_container.GetAllTimeStamp(timestamps);
  for (auto& ts : timestamps) {
    fprintf(fp, "%lu\n", ts);
  }

  fclose(fp);

  std::cout << "[INFO] save timestamp file " << file << std::endl;

  return true;
}

}  // namespace tools
}  // namespace jojo
