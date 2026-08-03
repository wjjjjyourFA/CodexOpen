#include "tools/data_loader/config/interface_config.h"
#include "tools/data_loader/config/runtime_config.h"
#include "tools/data_loader/data_loader.h"
#include "tools/data_loader/group_convert.h"

// #include "modules/perception/camera_location_estimation/camera_location_estimation_legacy.h"
#include "modules/perception/camera_location_estimation/camera_location_estimation.h"
#include "modules/perception/camera_location_estimation/config/runtime_config.h"

using namespace jojo::tools;
using namespace jojo::perception;
using namespace jojo::perception::fusion;
namespace cle = jojo::perception::cle;

int main(int argc, char** argv) {
  std::string config_path =
      "./../../../config/CameraLocation/CameraLocationEstimation.ini";

#if defined(YOLOV5)
  // yolo v5
  std::string engine_file = "./../../../model/yolo/yolov5s.engine";
  std::string name        = "RunYolov5";
#elif defined(YOLOV8)
  // yolo v8
  std::string engine_file = "./../../../model/yolo/yolov8s.engine";
  std::string name        = "RunYolov8";
#endif

  if (argc > 1) {
    config_path = argv[1];
  }

  auto runtime_config = std::make_shared<cle::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

  auto camera_params =
      std::make_shared<jojo::perception::camera::CameraParams>();
  camera_params->LoadFromFile(runtime_config->calib_file_path /*kk.ini*/);
  auto matrix = camera_params->GetMatrixVector();

  auto fusion = std::make_shared<LidarCameraFusion>();
  fusion->SetProjectionMatrix(matrix.at(0)->projection_matrix);

  std::shared_ptr<cle::CameraLocationEstimation> image_locator;
  // 1 ==> det || 2 ==> track
  image_locator = std::make_shared<cle::CameraLocationEstimation>(
      runtime_config->use_det_or_track);
  image_locator->Init(std::string(runtime_config->engine_file));
  image_locator->SetProjectionMatrix(matrix.at(0)->projection_matrix);
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
  data_loader->Init(dl_runtime_config, dl_interface_config);

  std::cout << std::fixed << std::setprecision(0);

  int frame_idx = 0;
  std::shared_ptr<const MeasureGroup> group;
  while (!data_loader->IsEnd()) {
    auto base = data_loader->ReadNext();
    group     = std::static_pointer_cast<const jojo::tools::MeasureGroup>(base);
    std::cout << "Frame " << frame_idx
              << ": Image Time = " << group->camera.at(0).time
              << ", Lidar Time = " << group->lidar.time
              << ", Imu Time = " << group->imu.time << std::endl;

    // group->camera.at(0).data ==> undistorted_image
    // cv::Mat undistorted_image;

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

    fusion->SetLidarPointCloud(t);
    fusion->SetCameraImage(group->camera.at(0).data);
    fusion->fuse(2, true, false);

    cv::Mat cur_mask;
    fusion->GetFusedImage(cur_mask);
    // cv::imshow("fused_image", cur_mask);
    auto t1   = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::cout << "fusion cost time: " << ms << " ms" << std::endl;

    if (image_locator->isInited()) {
      cv::Mat cur_image;
      // cur_image = group->camera.at(0).data.clone();
      cur_image = group->camera.at(0).data;

      cv::Mat show_image = cur_image;

      switch (runtime_config->use_det_or_track) {
        case 1:
          image_locator->DetectionAndLocation(cur_image, cur_mask, show_image,
                                              true);
          break;
        case 2:
          image_locator->TrackingAndLocation(cur_image, cur_mask, show_image,
                                             true);
          break;
        default:
          std::cout << "Error: use_det_or_track is not set" << std::endl;
          break;
      }
    }

    // cv::imshow("Inference Image", group.image);
    char key = (char)cv::waitKey(1);
    if (key == 'q' || key == 'Q') break;

    frame_idx++;
  }

  return 0;
}
