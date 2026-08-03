#include "modules/perception/similarity_map/bird_view_map_leagecy.h"
#include "toolz/data_loader/group_convert.h"

using namespace jojo::tools;
using namespace jojo::perception;

int main(int argc, char** argv) {
  // clang-format off
  std::string name = "BridViewMap";
  std::string config_path = "./../../../config/SimilarityMap/ColorMap.ini";
  // clang-format on

  auto runtime_config = std::make_shared<jojo::perception::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

  std::shared_ptr<BirdViewMap> color_map = std::make_shared<BirdViewMap>();
  color_map->Init(runtime_config);

  std::string dl_rc_path = "./../../../config/SimilarityMap/DataLoader.ini";
  std::string dl_ic_path = "./../../../config/SimilarityMap/Interface.ini";

  auto dl_runtime_config = std::make_shared<jojo::tools::RuntimeConfig>();
  dl_runtime_config->set_name(name);
  dl_runtime_config->LoadConfig(dl_rc_path);

  auto dl_interface_config = std::make_shared<jojo::tools::InterfaceConfig>();
  dl_interface_config->set_name(name);
  dl_interface_config->LoadConfig(dl_ic_path);

  auto group_convert = std::make_shared<GroupConvertDataSet>();
  group_convert->Init(dl_runtime_config, dl_interface_config);

  std::cout << std::fixed << std::setprecision(9);

  int frame_idx = 0;
  // std::shared_ptr<const jojo::tools::MeasureGroup> group;
  std::shared_ptr<const jojo::tools::MeasureGroupDataSet> group;
  while (!group_convert->IsEnd()) {
    auto base = group_convert->ReadNext();
    group =
        std::static_pointer_cast<const jojo::tools::MeasureGroupDataSet>(base);
    // 在这个代码中，lidar 读取完毕后，需要立即退出，因此手动判断一下，否则末尾会处理一帧旧雷达；
    if (group_convert->IsEnd()) {
      break;
    }
    std::cout << "Frame " << frame_idx
              << ": Image Time = " << group->camera.at(0).time
              << ", Lidar Time = " << group->lidar.time
              << ", Pose Time = " << group->se3_pose.time << std::endl;
    frame_idx++;

    color_map->Run(group);
  }

  return 0;
}
