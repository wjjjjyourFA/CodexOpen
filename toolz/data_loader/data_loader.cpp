#include "toolz/data_loader/data_loader.h"

namespace jojo {
namespace tools {
namespace common = apollo::cyber::common;
namespace camera = jojo::perception::camera;
namespace cfg    = jojo::perception::config;

DataLoaderDataSet::DataLoaderDataSet() {}

DataLoaderDataSet::~DataLoaderDataSet() {}

void DataLoaderDataSet::InitUndistortion() {
  camera_params = std::make_shared<camera::CameraParamsJson>();
  camera_params->SetLoadPath(param_->calib_file_dir);

  for (int i = 0; i < param_->b_camera; i++) {
    camera_params->LoadFromName(param_->camera_name.at(i), ".json");

    auto camera_undist = std::make_shared<camera::UndistortionHandler>();
    undistort_vector.push_back(camera_undist);
  }

  for (int i = 0; i < param_->b_infra; i++) {
    camera_params->LoadFromName(param_->infra_name.at(i), ".json");

    auto camera_undist = std::make_shared<camera::UndistortionHandler>();
    undistort_vector.push_back(camera_undist);
  }

  for (int i = 0; i < param_->b_star; i++) {
    camera_params->LoadFromName(param_->star_name.at(i), ".json");

    auto camera_undist = std::make_shared<camera::UndistortionHandler>();
    undistort_vector.push_back(camera_undist);
  }

  // clang-format off
  undistort_init.resize(param_->b_camera + param_->b_infra + param_->b_star, false);
  // clang-format on
}

void DataLoaderDataSet::LoadDataFolder() {
  this->prefix = param_->root_path + "/" + param_->file_name;
  std::cout << "data_file : " << this->prefix << std::endl;

  this->postfix = this->prefix + "-O";

  if (param_->use_bin_or_pcd == 0) {
    path_lidar = this->postfix + "/sensor_data/" + "lidar";
  } else {
    path_lidar = this->postfix + "/sensor_data/" + "lidar_pcd";
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

  path_global_pose = this->postfix + "/sensor_data/" + "pose_global" + ".txt";
  path_local_pose = this->postfix + "/sensor_data/" + "pose_local" + ".txt";

  path_radar = this->postfix + "/" + "radar";

  path_radar4d = SetDataFolderVector(this->postfix, "radar4d", param_->b_radar4d);
  // clang-format on
}

}  // namespace tools
}  // namespace jojo
