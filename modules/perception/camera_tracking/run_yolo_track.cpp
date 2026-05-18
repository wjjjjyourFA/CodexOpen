#include "tools/data_loader/data_loader.h"
#include "tools/data_loader/group_convert.h"
#include "tools/data_loader/config/runtime_config_offline.h"

#include "modules/perception/camera_tracking/camera_tracking.h"
#include "modules/perception/camera_tracking/config/runtime_config.h"

using namespace jojo::tools;
using namespace jojo::perception;
namespace ct = jojo::perception::ct;

int main(int argc, char** argv) {
  std::string config_path =
      "./../../../config/CameraTracking/CameraTrackingTwoStage.ini";

#if defined(YOLOV5)
  // yolo v5
  std::string det_engine_file = "./../../../model/yolo/yolov5s.engine";
#elif defined(YOLOV8)
  // yolo v8
  std::string det_engine_file = "./../../../model/yolo/yolov8s.engine";
#endif

#if defined(DEEPSORT)
  // clang-format off
  std::string sort_engine_file = "./../../../model/deepsort/deepsort.engine";
  std::string name = "RunYoloDeepSort";
  // clang-format on
#elif defined(BYTETRACK)
  std::string name = "RunYoloByteTrack";
#endif

  if (argc > 1) {
    config_path = argv[1];
  }

  auto runtime_config = std::make_shared<ct::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

  std::shared_ptr<ct::CameraTracking> image_tracking;
  image_tracking = std::make_shared<ct::CameraTracking>();
  image_tracking->Init(std::string(runtime_config->det_engine_file),
                       std::string(runtime_config->sort_engine_file));

  std::string dl_config_path = "./../../../config/DataLoader/DataLoader.ini";

  auto dl_runtime_config =
      std::make_shared<jojo::tools::RuntimeConfigOffline>();
  dl_runtime_config->set_name(name);
  dl_runtime_config->LoadConfig(dl_config_path);

  auto data_loader = std::make_shared<GroupConvert>();
  data_loader->Init(dl_runtime_config);

  std::cout << std::fixed << std::setprecision(0);

  int frame_idx = 0;
  std::shared_ptr<const MeasureGroup> group;
  while (!data_loader->IsEnd()) {
    auto base = data_loader->ReadNext();
    group     = std::static_pointer_cast<const MeasureGroup>(base);
    std::cout << "Frame " << frame_idx
              << ": Image Time = " << group->camera.at(0).time
              << ", Lidar Time = " << group->lidar.time
              << ", Imu Time = " << group->imu.time << std::endl;

    // group->camera.at(0).data ==> undistorted_image
    // cv::Mat undistorted_image;

    std::vector<jojo::perception::base::Object> detections;

    image_tracking->Start();
    if (image_tracking->isInited()) {
      cv::Mat cur_image = group->camera.at(0).data.clone();
      image_tracking->DetectionAndTracking(
          cur_image, detections, cur_image.cols, cur_image.rows, true);
    }

    // cv::imshow("Inference Image", group.image);
    char key = (char)cv::waitKey(1);
    if (key == 'q' || key == 'Q') break;

    frame_idx++;
  }

  return 0;
}
