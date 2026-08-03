#include "modules/mapping/mapper/mapper_color.h"
#include "toolz/data_loader/group_convert.h"

using namespace jojo::tools;
using namespace jojo::mapping;

int main(int argc, char** argv) {
  // clang-format off
  std::string name = "MapperColor";
  std::string runtime_config_path = "./../../../config/Mapper/Mapper.ini";
  std::string static_config_path = "./../../../config/Mapper/Mapper.yaml";
  // clang-format on

  auto runtime_config = std::make_shared<jojo::mapping::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(runtime_config_path);

  auto static_config = std::make_shared<jojo::mapping::StaticConfig>();
  static_config->set_name(name);
  static_config->LoadConfig(static_config_path);

  std::shared_ptr<MapperColor> mapper = std::make_shared<MapperColor>();
  mapper->Init(runtime_config, static_config);
  mapper->InitCameraParams();

  std::string dl_rc_path = "./../../../config/Mapper/DataLoader.ini";
  std::string dl_ic_path = "./../../../config/Mapper/Interface.ini";

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
    if (group_convert->IsEnd()) {
      break;
    }
    std::cout << "Frame " << frame_idx
              << ": Image Time = " << group->camera.at(0).time
              << ", Lidar Time = " << group->lidar.time
              << ", Pose Time = " << group->se3_pose.time << std::endl;
    frame_idx++;

    if (b_first_run) {
      const auto& p_center = group->pose_center;
      mapper->SetPoseCenter(p_center);

      b_first_run = false;
    }

    if (runtime_config->b_generate_color_map) {
      // 彩色化地图的构建过程中需要用的时间戳，不仅仅是单独的数据，因此传入 MeasureGroup
      mapper->Run(group);
    }
  }
  mapper->UpdateIncrementalMap();
  mapper->VisualizeMap();
  mapper->SaveMap(static_config->map_save_path);

  return 0;
}