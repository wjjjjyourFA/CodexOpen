#include <chrono>  // for std::chrono

#include "modules/localization/fast_lio/map_localization.h"
#include "modules/localization/fast_lio/utils.h"
#include "modules/perception/common/config/sensor_extrinsics.h"
#include "modules/perception/common/config/vehicle_config.h"
#include "tools/data_loader/group_convert.h"

using namespace std::chrono;
using namespace jojo::tools;
using namespace fastlio;
namespace cfg = jojo::perception::config;

int main(int argc, char** argv) {
  // clang-format off
  std::string name = "FastLioLocalization";
  std::string runtime_config_path = "./../../../config/FastLio/FastLioLocalization.ini";
  std::string static_config_path = "./../../../config/FastLio/FastLioLocalization.yaml";
  // clang-format on

  auto runtime_config = std::make_shared<jojo::localization::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(runtime_config_path);

  auto static_config = std::make_shared<jojo::localization::StaticConfig>();
  static_config->set_name(name);
  static_config->LoadConfig(static_config_path);

  auto lidar_params = std::make_shared<cfg::SensorExtrinsics>();
  lidar_params->LoadFromFile(runtime_config->lidar_calib_file_path /*kk.ini*/);
  auto matrix1 = lidar_params->GetMatrixVector();

  auto imu_params = std::make_shared<cfg::SensorExtrinsics>();
  imu_params->LoadFromFile(runtime_config->gravity_imu_calib_file_path);
  auto matrix2 = imu_params->GetMatrixVector();

  auto vehicle_params = std::make_shared<cfg::VehicleConfig>();
  vehicle_params->LoadFromFile(runtime_config->vehicle_config_file_path);

  // clang-format off
  std::shared_ptr<MapLocalization> localization = std::make_shared<MapLocalization>();
  localization->SetGravityImuExtrinsicMatrix(matrix2.at(0)->extrinsic_matrix);
  localization->SetExtrinsicMatrix(matrix1.at(0)->extrinsic_matrix);
  localization->Init(runtime_config, static_config);
  // clang-format on

  std::string dl_rc_path = "./../../../config/FastLio/DataLoader.ini";
  std::string dl_ic_path = "./../../../config/FastLio/Interface.ini";

  auto dl_runtime_config = std::make_shared<jojo::tools::RuntimeConfig>();
  dl_runtime_config->set_name(name);
  dl_runtime_config->LoadConfig(dl_rc_path);

  auto dl_interface_config = std::make_shared<jojo::tools::InterfaceConfig>();
  dl_interface_config->set_name(name);
  dl_interface_config->LoadConfig(dl_ic_path);

  auto group_convert = std::make_shared<GroupConvert>();
  group_convert->Init(dl_runtime_config, dl_interface_config);

  std::cout << std::fixed << std::setprecision(0);

  const auto& vp = vehicle_params;

  bool b_save_result = true;

  // 1. 在循环开始前记录起始时间
  auto start_time = std::chrono::high_resolution_clock::now();

  bool b_first_run = true;

  int frame_idx = 0;
  std::shared_ptr<const jojo::tools::MeasureGroup> group;
  while (!group_convert->IsEnd()) {
    auto base = group_convert->ReadNext();
    group     = std::static_pointer_cast<const jojo::tools::MeasureGroup>(base);
    // 在这个代码中，lidar 读取完毕后，需要立即退出，因此手动判断一下，否则末尾会处理一帧旧雷达；
    if (group_convert->IsEnd()) {
      break;
    }
    std::cout << "Frame " << frame_idx << ", Lidar Time = " << group->lidar.time
              << ", IMU Count = " << group->imu_vec.size() << std::endl;
    // auto frame_start = omp_get_wtime();
    frame_idx++;

    // 设置 frame 在 map 中的初始位姿
    if (b_first_run) {
      // clang-format off
      static_config->yaw_offset_rad = deg2rad(static_config->yaw_offset_euler);
      Eigen::Quaterniond yaw_offset(Eigen::AngleAxisd(static_config->yaw_offset_rad, Eigen::Vector3d::UnitZ()));
      // clang-format on
      Eigen::Vector3d init_pos;
      Eigen::Quaterniond init_rot;
      // 初始位置以地图中心为基准点，进行偏移设置
      if (static_config->b_chaos_start) {
        // 如果地图是 去中心 pose 坐标系，地图中心是 (0,0,0)
        init_pos = static_config->map_center;  // Eigen::Vector3d::Zero();
        init_rot = yaw_offset;

        // 如果地图是 全局坐标，那么设置中心点为 map_center
        // init_pos = static_config->map_center;
      } else {
        jojo::common_struct::SE3Pose pose;
        pose = jojo::common_struct::ConvertGnssToPose(group->gnss.data, true);
        // pose = jojo::common_struct::ConvertOdomToPose(group->odom.data);

        // pose 指的是 全局坐标系， ==> gnss pose
        // 去中心 pose 坐标系
        init_pos = pose.pos - static_config->map_center;
        // 右乘（修正传感器yaw） 保持地图坐标系不变，旋转 自身坐标系 微调对齐 地图方向
        // 有问题，实际上会导致 roll pitch 无法对齐，因为此处的 yaw_offset 是全局坐标系下的
        // init_rot = pose.rot * yaw_offset;
        // 左乘（修正地图yaw）
        init_rot = yaw_offset * pose.rot;

        // 全局坐标
        // init_pos = pose.pos
        // init_rot = pose.rot * yaw_offset;
      }
      init_rot.normalize();
      localization->SetInitPose(init_pos, init_rot);

      b_first_run = false;
    }

    fastlio::MeasureGroup t;
    // cost 16 ms
    ConvertMeasureGroup(group, t, vp);

    auto frame_start = omp_get_wtime();
    localization->run_localization(t);
    auto frame_end = omp_get_wtime();

    // 灰色为局部先验地图，红色为 IMU 预测点云，绿色为最终优化点云。
    // 每帧刷新，用于实时检查定位优化位姿是否把当前帧对齐到正确位置。
    // localization->Show();

    auto frame_duration = frame_end - frame_start;
    std::cout << "Loc frame runtime: " << frame_duration * 1000 << " ms"
              << std::endl
              << std::endl;

    if (runtime_config->b_only_times) {
      localization->SaveFrameTime(frame_duration * 1000.0);
    }

    if (b_save_result) {
      localization->save_result(runtime_config->b_save_pcd);
    }
  }
  localization->Close();

  // 2. 在循环结束后记录结束时间
  auto end_time = std::chrono::high_resolution_clock::now();

  // 3. 计算时间差并转换为秒 (double 类型)
  std::chrono::duration<double> diff = end_time - start_time;
  double total_duration_s            = diff.count();

  printf("\n================================================\n");
  printf("Algorithm total processing time: %.4f seconds\n", total_duration_s);
  printf("================================================\n");

  return 0;
}
