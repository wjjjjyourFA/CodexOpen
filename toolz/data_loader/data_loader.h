#ifndef DATA_LOADER_DATASET_H
#define DATA_LOADER_DATASET_H

#pragma once

#include "modules/perception/common/camera/params/camera_params_json.h"
#include "tools/data_loader/data_loader.h"

namespace jojo {
namespace tools {

class DataLoaderDataSet : public DataLoader {
 public:
  DataLoaderDataSet();
  virtual ~DataLoaderDataSet();

  void InitUndistortion() override;

 protected:
  void LoadDataFolder() override;

 private:
  // clang-format off
  std::shared_ptr<jojo::perception::camera::CameraParamsJson> camera_params;
  // std::vector<std::shared_ptr<jojo::perception::camera::CameraParamsJson>> camera_params_vector;
  // clang-format on

 private:
  // 简易版本，不使用模板特化等功能
  // std::vector<DataContainer<uint64_t /*sensor_msgs::Image*/>> dc_camera;
  // std::vector<DataContainer<uint64_t /*sensor_msgs::Image*/>> dc_infra;
  // std::vector<DataContainer<uint64_t /*sensor_msgs::Image*/>> dc_star;
};

}  // namespace tools
}  // namespace jojo

#endif  // DATA_LOADER_HH
