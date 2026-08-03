#include <chrono>  // for std::chrono

#include "modules/localization/fast_lio/lidar_odometry_legacy.h"
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
  std::string name = "FastLioOdometry";
  std::string runtime_config_path = "./../../../config/FastLio/FastLioOdometry.ini";
  std::string static_config_path = "./../../../config/FastLio/FastLioOdometry.yaml";
  // clang-format on

  auto runtime_config = std::make_shared<jojo::localization::RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(runtime_config_path);

  auto lidar_params = std::make_shared<cfg::SensorExtrinsics>();
  lidar_params->LoadFromFile(runtime_config->lidar_calib_file_path /*kk.ini*/);
  auto matrix1 = lidar_params->GetMatrixVector();

  auto imu_params = std::make_shared<cfg::SensorExtrinsics>();
  imu_params->LoadFromFile(runtime_config->gravity_imu_calib_file_path);
  auto matrix2 = imu_params->GetMatrixVector();

  auto vehicle_params = std::make_shared<cfg::VehicleConfig>();
  vehicle_params->LoadFromFile(runtime_config->vehicle_config_file_path);

  std::shared_ptr<LidarOdometry> odometry = std::make_shared<LidarOdometry>();
  odometry->Init(runtime_config);

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

  std::cout << std::fixed << std::setprecision(9);

  const auto& vp = vehicle_params;
  // const auto& vp = *vehicle_params;

  bool b_save_result = true;

  // 1. 在循环开始前记录起始时间
  auto start_time = std::chrono::high_resolution_clock::now();

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
    frame_idx++;

    fastlio::MeasureGroup t;
    ConvertMeasureGroup(group, t, vp);

    auto frame_start = omp_get_wtime();
    odometry->run_odometry(t);
    auto frame_end = omp_get_wtime();

    // 实时显示：灰色为 ikd-tree 局部地图，黄色为当前帧，青色为轨迹。
    // odometry->Show();

    std::cout << "Odom frame runtime: " << (frame_end - frame_start) * 1000
              << " ms" << std::endl
              << std::endl;

    if (b_save_result) {
      odometry->save_result(runtime_config->b_save_pcd);
    }
  }
  odometry->Close();

  // 2. 在循环结束后记录结束时间
  auto end_time = std::chrono::high_resolution_clock::now();

  // 3. 计算时间差并转换为秒 (double 类型)
  std::chrono::duration<double> diff = end_time - start_time;
  double total_duration_s            = diff.count();

  // 4. 输出结果
  printf("\n================================================\n");
  printf("Algorithm total processing time: %.4f seconds\n", total_duration_s);
  printf("================================================\n");

  // std::string cmd = "cp " + dataset_path + "/pose_odom_lio.txt " +
  //                   dataset_path + "/pose_odom_euler_kiss_icp.txt";
  // system(cmd.c_str());

  return 0;
}
