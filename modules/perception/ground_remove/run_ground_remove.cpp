#include "modules/perception/common/config/sensor_extrinsics.h"
#include "modules/perception/ground_remove/ground_remove.h"
#include "toolz/data_loader/group_convert.h"

using namespace jojo::tools;
using namespace jojo::perception;
namespace cfg = jojo::perception::config;

int main(int argc, char** argv) {
  // clang-format off
  std::string name = "GroundRemove";
  std::string config_path = "./../../../config/GroundRemove/GroundRemove.ini";
  // clang-format on

  auto runtime_config = std::make_shared<jojo::perception::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

  auto lidar_params = std::make_shared<cfg::SensorExtrinsics>();
  lidar_params->LoadFromFile(runtime_config->gravity_lidar_calib_file_path);
  auto matrix2 = lidar_params->GetMatrixVector();

  // clang-format off
  std::shared_ptr<GroundRemove> ground_remove = std::make_shared<GroundRemove>();
  ground_remove->SetGravityLidarExtrinsicMatrix(matrix2.at(0)->extrinsic_matrix);
  ground_remove->Init(runtime_config);
  // clang-format on

  std::string dl_rc_path = "./../../../config/GroundRemove/DataLoader.ini";
  std::string dl_ic_path = "./../../../config/GroundRemove/Interface.ini";

  auto dl_runtime_config = std::make_shared<jojo::tools::RuntimeConfig>();
  dl_runtime_config->set_name(name);
  dl_runtime_config->LoadConfig(dl_rc_path);

  auto dl_interface_config = std::make_shared<jojo::tools::InterfaceConfig>();
  dl_interface_config->set_name(name);
  dl_interface_config->LoadConfig(dl_ic_path);

  auto group_convert = std::make_shared<GroupConvert>();
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
    std::cout << "Frame "
              << frame_idx
              // << ": Image Time = " << group->camera.at(0).time
              << ", Lidar Time = " << group->lidar.time
              << ", Pose Time = " << group->se3_pose.time << std::endl;
    frame_idx++;

    const auto& frame = group->lidar.data;
    const auto& pose  = group->se3_pose.data.matrix();
    ground_remove->Run(frame, pose);
  }

  return 0;
}
