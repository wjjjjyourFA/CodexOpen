#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/config/config_file_base.h"
#include "modules/common/math/math_utils_extra.h"
#include "modules/perception/common/camera/common/undistortion_handler_cv.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/fusion/lidar2camera/config/interface_config.h"
#include "modules/perception/common/fusion/lidar2camera/config/runtime_config.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_multi_camera_fusion.h"
#include "modules/perception/tools/common/show_data_2d.h"

using namespace jojo::common::math;
using namespace jojo::perception;
using namespace jojo::perception::camera;
using namespace jojo::perception::fusion;
namespace cfg = jojo::perception::config;

namespace {

constexpr size_t kTestCameraCount = 3;

bool LoadSingleFrameImage(
    const std::shared_ptr<RuntimeConfig>& runtime_config,
    const std::shared_ptr<InterfaceConfig>& interface_config,
    const std::shared_ptr<CameraParams>& single_camera_params, cv::Mat* image) {
  if (!runtime_config || !interface_config || !single_camera_params || !image) {
    return false;
  }

  const cv::Mat raw_image = cv::imread(interface_config->image_file);
  if (raw_image.empty()) {
    std::cerr << "Failed to load image: " << interface_config->image_file
              << std::endl;
    return false;
  }

  *image = raw_image.clone();
  if (!runtime_config->b_do_undistort) {
    return true;
  }

  const auto& matrix_v = single_camera_params->GetMatrixVector();
  if (matrix_v.empty() || !matrix_v.front() ||
      !matrix_v.front()->camera_matrix) {
    return false;
  }

  Eigen::VectorXf params = cfg::IntrinsicParamsToVector(
      matrix_v.front()->camera_matrix->intrinsic_matrix,
      matrix_v.front()->camera_matrix->distortion_params);

  auto camera_undistort = std::make_shared<UndistortionHandlerCv>();
  camera_undistort->InitModel(CameraDistortionModel::Brown);
  camera_undistort->InitParams(raw_image.cols, raw_image.rows, params);
  camera_undistort->Init("camera");
  camera_undistort->Handle(raw_image, image);

  return !image->empty();
}

bool LoadSingleFrameCloud(
    const std::shared_ptr<InterfaceConfig>& interface_config,
    pcl::PointCloud<pcl::PointXYZI>::Ptr* cloud) {
  if (!interface_config || !cloud) {
    return false;
  }

  auto loaded_cloud =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (interface_config->b_bin_or_pcd == 0) {
    FILE* file = std::fopen(interface_config->lidar_file.c_str(), "rb");
    if (!file) {
      std::cerr << "Failed to load lidar file: " << interface_config->lidar_file
                << std::endl;
      return false;
    }

    int data[4] = {0, 0, 0, 0};
    while (std::fread(data, sizeof(int), 4, file) == 4) {
      pcl::PointXYZI point;
      // 原始点云单位为 cm，当前投影矩阵使用 m。
      point.x = static_cast<float>(data[0]) / 100.0F;
      point.y = static_cast<float>(data[1]) / 100.0F;
      point.z = static_cast<float>(data[2]) / 100.0F;

      point.intensity = static_cast<float>(data[3]);
      loaded_cloud->push_back(point);
    }
    std::fclose(file);
  } else if (pcl::io::loadPCDFile<pcl::PointXYZI>(interface_config->lidar_file,
                                                  *loaded_cloud) != 0) {
    std::cerr << "Failed to load PCD file: " << interface_config->lidar_file
              << std::endl;
    return false;
  }

  if (loaded_cloud->empty()) {
    return false;
  }

  std::vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(*loaded_cloud, *loaded_cloud, valid_indices);

  *cloud = std::move(loaded_cloud);

  return !(*cloud)->empty();
}

std::shared_ptr<CameraParams> MakeSameProjectionParams(
    const std::shared_ptr<CameraParams>& single_camera_params,
    size_t camera_count) {
  if (!single_camera_params || camera_count == 0) {
    return nullptr;
  }

  const auto& source_matrix_v = single_camera_params->GetMatrixVector();
  if (source_matrix_v.empty() || !source_matrix_v.front()) {
    return nullptr;
  }

  auto multi_camera_params = std::make_shared<CameraParams>();
  if (!multi_camera_params->InitMatrixVector(static_cast<int>(camera_count))) {
    return nullptr;
  }

  // 本测试的三个相机使用完全相同的内参、外参和投影矩阵。
  const auto& target_matrix_v = multi_camera_params->GetMatrixVector();
  for (size_t i = 0; i < camera_count; ++i) {
    *target_matrix_v[i] = *source_matrix_v.front();
  }
  std::cout << "size: " << target_matrix_v.size() << std::endl;

  return multi_camera_params;
}

void ShowBatchResult(
    const std::vector<std::shared_ptr<cv::Mat>>& fused_image_v) {
  for (size_t i = 0; i < fused_image_v.size(); ++i) {
    if (!fused_image_v[i] || fused_image_v[i]->empty()) {
      continue;
    }

    const std::string window_name = "fused_image_" + std::to_string(i);
    cv::namedWindow(window_name, cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(window_name, 512, 256);
    cv::imshow(window_name, *fused_image_v[i]);
  }
  cv::waitKey(0);
}

}  // namespace

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  // clang-format off
  const std::string name = "LidarMultiCameraFuse";
  const std::string config_path = "./../../../../config/PerceptionFuse/LidarCamera/LidarCameraFuse.ini";
  // clang-format on

  auto runtime_config = std::make_shared<RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);
  if (runtime_config->calib_file_path.empty()) {
    std::cerr << "Invalid runtime config: " << config_path << std::endl;
    return 1;
  }

  // clang-format off
  const std::string interface_config_path = "./../../../../config/PerceptionFuse/LidarCamera/InterfaceTest.ini";
  // clang-format on

  auto interface_config = std::make_shared<InterfaceConfig>();
  interface_config->set_name(name);
  interface_config->LoadConfig(interface_config_path);
  if (interface_config->image_file.empty() ||
      interface_config->lidar_file.empty()) {
    std::cerr << "Invalid interface config: " << interface_config_path
              << std::endl;
    return 1;
  }

  // 单帧测试：只读取一帧图像和一帧点云。
  auto single_camera_params = std::make_shared<CameraParams>();
  if (!single_camera_params->LoadFromFile(runtime_config->calib_file_path)) {
    return 1;
  }

  cv::Mat single_frame_image;
  if (!LoadSingleFrameImage(runtime_config, interface_config,
                            single_camera_params, &single_frame_image)) {
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr single_frame_cloud;
  if (!LoadSingleFrameCloud(interface_config, &single_frame_cloud)) {
    return 1;
  }

  if (runtime_config->b_lt_none_rt != 1) {
    pcl::transformPointCloud(*single_frame_cloud, *single_frame_cloud,
                             GetTransMatrix(runtime_config->b_lt_none_rt));
  }

  std::cout << "single frame image size: " << single_frame_image.size()
            << std::endl;
  std::cout << "single frame cloud size: " << single_frame_cloud->size()
            << std::endl;

  // show2d_lidar_data(single_frame_cloud);

  // Multi-camera 测试：同一幅图像 push 三次，三个相机使用同一份投影参数。
  auto multi_camera_params =
      MakeSameProjectionParams(single_camera_params, kTestCameraCount);
  if (!multi_camera_params) {
    return 1;
  }

  auto shared_image = std::make_shared<cv::Mat>(single_frame_image);
  std::vector<std::shared_ptr<cv::Mat>> image_v;
  image_v.reserve(kTestCameraCount);
  for (size_t i = 0; i < kTestCameraCount; ++i) {
    image_v.push_back(shared_image);
  }

  auto fusion = std::make_shared<LidarMultiCameraFusion>();
  if (!fusion->Init()) {
    return 1;
  }

  fusion->SetCameraParams(multi_camera_params);
  fusion->SetLidarPointCloud(single_frame_cloud);
  fusion->SetCameraImageVector(image_v);
  fusion->Start();
  fusion->Run(false);
  fusion->Stop();

  std::vector<std::shared_ptr<cv::Mat>> fused_image_v;
  if (!fusion->GetFusedImageVector(fused_image_v) ||
      fused_image_v.size() != kTestCameraCount) {
    std::cerr << "Multi-camera projection failed." << std::endl;
    return 1;
  }

  ShowBatchResult(fused_image_v);

  return 0;
}
