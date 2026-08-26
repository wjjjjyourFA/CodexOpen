#include "modules/perception/camera_location_estimation/camera_location_estimation_legacy.h"
#include "modules/perception/camera_location_estimation/config/runtime_config.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "tools/data_loader/config/interface_config.h"
#include "tools/data_loader/config/runtime_config.h"
#include "tools/data_loader/data_loader.h"
#include "tools/data_loader/group_convert.h"

using namespace jojo::tools;
using namespace jojo::perception;
using namespace jojo::perception::fusion;
namespace cle = jojo::perception::cle;

int main(int argc, char** argv) {
  std::string config_path =
      "./../../../config/CameraLocation/CameraLocationEstimation.ini";

  std::string name = "RunYolo";
#if defined(YOLOV5)
  name = "RunYolov5";
#elif defined(YOLOV8)
  name = "RunYolov8";
#endif

  if (argc > 1) {
    config_path = argv[1];
  }

  auto runtime_config = std::make_shared<cle::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

  if (!runtime_config->valid) {
    std::cerr << "Invalid camera location configuration: "
              << runtime_config->validation_error << std::endl;
    return 1;
  }

  auto camera_params =
      std::make_shared<jojo::perception::camera::CameraParams>();
  if (!camera_params->LoadFromFile(runtime_config->calib_file_path)) return 1;
  auto matrix = camera_params->GetMatrixVector();
  if (matrix.empty() || !matrix.front()) return 1;

  auto fusion = std::make_shared<LidarCameraFusion>();
  if (!fusion->SetProjectionMatrix(matrix.front()->projection_matrix) ||
      !fusion->set_params("Lidar", runtime_config->dist_threshold)) {
    std::cerr << "Lidar-camera fusion initialization failed" << std::endl;
    return 1;
  }

  auto image_locator = std::make_shared<cle::CameraLocationEstimation>(
      static_cast<uint>(runtime_config->inference_mode));
  image_locator->Init(std::string(runtime_config->engine_file));
  image_locator->SetProjectionMatrix(matrix.front()->projection_matrix);
  image_locator->Start();

  std::string dl_rc_path = "./../../../config/DataLoader/DataLoader.ini";
  std::string dl_ic_path = "./../../../config/DataLoader/Interface.ini";

  auto dl_runtime_config = std::make_shared<jojo::tools::RuntimeConfig>();
  dl_runtime_config->set_name(name);
  dl_runtime_config->LoadConfig(dl_rc_path);

  auto dl_interface_config = std::make_shared<jojo::tools::InterfaceConfig>();
  dl_interface_config->set_name(name);
  dl_interface_config->LoadConfig(dl_ic_path);

  auto data_loader = std::make_shared<GroupConvert>();
  if (!data_loader->Init(dl_runtime_config, dl_interface_config)) {
    std::cerr << "Data loader initialization failed" << std::endl;
    return 1;
  }

  std::cout << std::fixed << std::setprecision(0);

  int frame_idx = 0;
  std::shared_ptr<const MeasureGroup> group;
  while (!data_loader->IsEnd()) {
    auto base = data_loader->ReadNext();
    group     = std::static_pointer_cast<const jojo::tools::MeasureGroup>(base);
    if (!group || group->camera.empty() || group->camera.front().data.empty() ||
        !group->lidar.data || group->lidar.data->empty()) {
      std::cerr << "Skip invalid camera-lidar group at frame " << frame_idx
                << std::endl;
      ++frame_idx;
      continue;
    }
    std::cout << "Frame " << frame_idx
              << ": Image Time = " << group->camera.at(0).time
              << ", Lidar Time = " << group->lidar.time
              << ", Imu Time = " << group->imu.time << std::endl;

    auto t0 = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr t(new pcl::PointCloud<pcl::PointXYZI>);
    const auto& src = group->lidar.data;
    // /* way 1 PointXYZIRT ==> PointXYZI
    t->points.resize(src->points.size());
    for (size_t i = 0; i < src->points.size(); ++i) {
      const auto& pt = src->points[i];

      auto& p = t->points[i];
      // 逐字段赋值
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      // intensity 字段
      p.intensity = pt.intensity;
    }
    t->width    = t->points.size();
    t->height   = 1;
    t->is_dense = true;
    // */

    if (!fusion->SetLidarPointCloud(t) ||
        !fusion->SetCameraImage(group->camera.front().data) ||
        !fusion->fuse(2, true, false)) {
      std::cerr << "Fusion failed at frame " << frame_idx << std::endl;
      ++frame_idx;
      continue;
    }

    cv::Mat cur_mask;
    if (!fusion->GetFusedImage(cur_mask)) {
      std::cerr << "Fusion mask is unavailable at frame " << frame_idx
                << std::endl;
      ++frame_idx;
      continue;
    }

    auto t1   = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "fusion cost time: " << ms << " ms" << std::endl;

    if (image_locator->isInited()) {
      cv::Mat cur_image = group->camera.front().data;

      // cv::Mat show_image = cur_image.clone();
      cv::Mat show_image = cur_image;

      // clang-format off
      switch (runtime_config->use_det_or_track) {
        case 1:
          image_locator->DetectionAndLocation(cur_image, cur_mask, show_image, true);
          break;
        case 2:
          // image_locator->TrackingAndLocation(cur_image, cur_mask, show_image, true);
          break;
        default:
          std::cout << "Error: use_det_or_track is not set" << std::endl;
          break;
      }
      // clang-format on
    }

    char key = (char)cv::waitKey(1);
    if (key == 'q' || key == 'Q') break;

    frame_idx++;
  }

  return 0;
}
