#include "toolz/data_loader/group_convert.h"
#include "modules/mapping/mapper/mapper.h"

using namespace jojo::tools;
using namespace jojo::mapping;

int main(int argc, char** argv) {
  // clang-format off
  std::string name = "Mapper";
  std::string runtime_config_path = "./../../../config/Mapper/Mapper.ini";
  std::string static_config_path = "./../../../config/Mapper/Mapper.yaml";
  // clang-format on

  auto runtime_config = std::make_shared<jojo::mapping::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(runtime_config_path);

  auto static_config = std::make_shared<jojo::mapping::StaticConfig>();
  static_config->set_name(name);
  static_config->LoadConfig(static_config_path);

  std::shared_ptr<Mapper> mapper = std::make_shared<Mapper>();
  mapper->Init(runtime_config, static_config);

  std::string dl_config_path = "./../../../config/Mapper/DataLoader.ini";

  auto dl_runtime_config = std::make_shared<RuntimeConfigOffline>();
  dl_runtime_config->set_name(name);
  dl_runtime_config->LoadConfig(dl_config_path);

  auto group_convert = std::make_shared<GroupConvertDataSet>();
  group_convert->Init(dl_runtime_config);

  bool b_first_run = true;

  int frame_idx = 0;
  std::shared_ptr<const MeasureGroupDataSet> group;
  while (!group_convert->IsEnd()) {
    auto base = group_convert->ReadNext();
    group     = std::static_pointer_cast<const MeasureGroupDataSet>(base);
    if (group_convert->IsEnd()) {
      break;
    }
    std::cout << "Frame " << frame_idx << ", Lidar Time = " << group->lidar.time
              << std::endl;
    frame_idx++;

    if (b_first_run) {
      const auto& p_center = group->pose_center;
      mapper->SetPoseCenter(p_center);

      b_first_run = false;
    }

    if (runtime_config->b_generate_map3d) {
      const auto& frame = group->lidar.data;
      const auto& pose  = group->se3_pose.data.matrix();
      mapper->Run(frame, pose);
    }
  }
  mapper->VisualizeMap();
  mapper->UpdateIncrementalMap();
  mapper->SaveMap(static_config->map_save_path);

  return 0;
}