#include "modules/dreamview/map_center_view/map_center_view.h"
#include "modules/perception/common/config/sensor_extrinsics.h"
#include "toolz/data_loader/group_convert.h"

using namespace jojo::tools;
using namespace jojo::dreamview;

int main(int argc, char** argv) {
  // clang-format off
  std::string name = "MapCenterView";
  std::string runtime_config_path = "./../../../config/MapCenterView/MapCenterView.ini";
  std::string static_config_path = "./../../../config/MapCenterView/MapCenterView.yaml";
  // clang-format on

  // auto runtime_config = std::make_shared<jojo::dreamview::RuntimeConfig>();
  // runtime_config->set_name(name);
  // runtime_config->LoadConfig(runtime_config_path);

  auto static_config = std::make_shared<jojo::dreamview::StaticConfig>();
  static_config->set_name(name);
  static_config->LoadConfig(static_config_path);

  std::shared_ptr<MapCenterView> view = std::make_shared<MapCenterView>();
  view->Init(static_config);

  std::string dl_rc_path = "./../../../config/MapCenterView/DataLoader.ini";
  std::string dl_ic_path = "./../../../config/MapCenterView/Interface.ini";

  auto dl_runtime_config = std::make_shared<jojo::tools::RuntimeConfig>();
  dl_runtime_config->set_name(name);
  dl_runtime_config->LoadConfig(dl_rc_path);

  auto dl_interface_config = std::make_shared<jojo::tools::InterfaceConfig>();
  dl_interface_config->set_name(name);
  dl_interface_config->LoadConfig(dl_ic_path);

  auto group_convert = std::make_shared<GroupConvertDataSet>();
  group_convert->Init(dl_runtime_config, dl_interface_config);

  std::cout << std::fixed << std::setprecision(9);

  bool b_first_run = true;

  int frame_idx = 0;
  std::shared_ptr<const MeasureGroupDataSet> group;
  while (!group_convert->IsEnd()) {
    auto base = group_convert->ReadNext();
    group     = std::static_pointer_cast<const MeasureGroupDataSet>(base);
    // 在这个代码中，lidar 读取完毕后，需要立即退出，因此手动判断一下，否则末尾会处理一帧旧雷达；
    if (group_convert->IsEnd()) {
      break;
    }
    std::cout << "Frame " << frame_idx << ", Lidar Time = " << group->lidar.time
              << std::endl;
    // auto frame_start = omp_get_wtime();
    frame_idx++;

    if (b_first_run) {
      const auto& p_center = group->pose_center;
      view->SetPoseCenter(p_center);

      b_first_run = false;
    }

    const auto& frame = group->lidar.data;
    const auto& pose  = group->se3_pose.data.matrix();

    auto frame_start = omp_get_wtime();
    if (static_config->b_display_roi) {
      std::cout << "Display ROI" << std::endl;
      view->ShowFrameROI(frame, pose);
    } else {
      std::cout << "Display MAP" << std::endl;
      view->ShowFrame(frame, pose);
    }
    auto frame_end = omp_get_wtime();

    auto frame_duration = frame_end - frame_start;
    std::cout << "Show frame runtime: " << frame_duration * 1000 << " ms"
              << std::endl
              << std::endl;
  }

  return 0;
}