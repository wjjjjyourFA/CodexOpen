#include "toolz/data_loader/group_convert.h"
#include "modules/mapping/map_visualization/map_visualization.h"

using namespace jojo::tools;

int main(int argc, char** argv) {
  // clang-format off
  std::string name = "MapVisualization";
  std::string runtime_config_path = "./../../../config/MapVisualization/MapVisualization.ini";
  std::string static_config_path = "./../../../config/MapVisualization/MapVisualization.yaml";
  // clang-format on

  auto runtime_config = std::make_shared<jojo::mapping::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(runtime_config_path);

  auto static_config = std::make_shared<jojo::mapping::StaticConfig>();
  static_config->set_name(name);
  static_config->LoadConfig(static_config_path);

  std::shared_ptr<MapVisualization> view = std::make_shared<MapVisualization>();
  view->Init(runtime_config, static_config);
  // view->InitViewer(); // 不需要初始化 pcl viewer

  std::string dl_config_path =
      "./../../../config/MapVisualization/DataLoader.ini";

  auto dl_runtime_config = std::make_shared<RuntimeConfigOffline>();
  dl_runtime_config->set_name(name);
  dl_runtime_config->LoadConfig(dl_config_path);

  auto group_convert = std::make_shared<GroupConvertDataSet>();
  group_convert->Init(dl_runtime_config);

  view->LoadMapLabel();
  view->FillUnknownGroundByHeight();

  bool b_first_run = true;

  int frame_idx = 0;
  std::shared_ptr<const MeasureGroupDataSet> group;
  while (!group_convert->IsEnd()) {
    auto base = group_convert->ReadNext();
    group     = std::static_pointer_cast<const MeasureGroupDataSet>(base);
    if (group_convert->IsEnd()) {
      break;
    }
    std::cout << "Frame " << frame_idx
              << ", se3_pose Time = " << group->se3_pose.time << std::endl;

    if (b_first_run) {
      const auto& p_center = group->pose_center;
      view->SetPoseCenter(p_center);

      b_first_run = false;
    }

    const auto& pose = group->se3_pose.data.matrix();

    view->GenerateSequencePassableArea(pose);

    // 等待按键
    char key = (char)cv::waitKey(1);  // 1ms，不阻塞主程序
    if (key == 'q' || key == 'Q') {
      break;
    }

    frame_idx++;
  }
  std::cout << "Generating passable area for pose sequence " << std::endl;

  view->DebugShow();
  view->FillHoleMap();
  view->SaveLabelTerrainMat();

  return 0;
}