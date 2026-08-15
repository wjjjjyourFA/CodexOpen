#include "tools/data_loader/group_convert.h"
#include "tools/data_loader/data_loader.h"
// #include "toolz/data_loader/group_convert.h"
// #include "toolz/data_loader/data_loader.h"
#include "modules/tools/lidar_undistortion/config/runtime_config.h"
#include "modules/perception/common/lidar/common/motion_ompensator.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"

using namespace jojo::tools;
using namespace jojo::perception::lidar;
using namespace jojo::perception::camera;
using namespace jojo::perception::fusion;
namespace cfg = jojo::perception::config;

int main(int argc, char** argv) {
  std::string name = "LidarUndistortion";
  std::string config_path =
      "./../../../config/LidarUndistortion/LidarUndistortion.ini";

  if (argc > 1) {
    config_path = argv[1];
  }

  auto runtime_config = std::make_shared<RuntimeConfig>();
  runtime_config->set_name(name);
  runtime_config->LoadConfig(config_path);

  auto lidar_params = std::make_shared<cfg::SensorExtrinsics>();
  lidar_params->LoadFromFile(runtime_config->lidar_calib_file_path);
  auto matrix1 = lidar_params->GetMatrixVector();

  auto motion_compensator = std::make_shared<MotionCompensator>();
  motion_compensator->SetExtrinsicMatrix(matrix1.at(0)->extrinsic_matrix);
  motion_compensator->Init();

  auto camera_params = std::make_shared<CameraParams>();
  camera_params->LoadFromFile(runtime_config->camera_calib_file_path);
  auto matrix2 = camera_params->GetMatrixVector();

  auto camera_undistort = std::make_shared<UndistortionHandler>();
  Eigen::VectorXf params(17);
  params = cfg::IntrinsicParamsToVector(
      matrix2.at(0)->camera_matrix->intrinsic_matrix,
      matrix2.at(0)->camera_matrix->distortion_params);

  auto fusion = std::make_shared<LidarCameraFusion>();
  fusion->SetProjectionMatrix(matrix2.at(0)->projection_matrix);

  // clang-format off
  std::string dl_config_path = "./../../../config/LidarUndistortion/DataLoader.ini";
  // std::string dl_config_path = "./../../../config/DataLoader/DataLoader.ini";
  // clang-format on

  auto dl_runtime_config = std::make_shared<RuntimeConfigOffline>();
  dl_runtime_config->set_name(name);
  dl_runtime_config->LoadConfig(dl_config_path);

  auto group_convert = std::make_shared<GroupConvert>();
  // auto group_convert = std::make_shared<GroupConvertDataSet>();
  group_convert->Init(dl_runtime_config);

  bool init_flag = false;
  bool b_first_run = true;

  int frame_idx = 0;
  std::shared_ptr<const jojo::tools::MeasureGroup> group;
  std::cout << std::fixed << std::setprecision(0);
  while (!group_convert->IsEnd()) {
    auto base = group_convert->ReadNext();
    group     = std::static_pointer_cast<const jojo::tools::MeasureGroup>(base);
    std::cout << "Frame " << frame_idx
              << ": Image Time = " << group->camera.at(0).time
              << ", Lidar Time = " << group->lidar.time
              << ", Imu Time = " << group->imu.time << std::endl;

    // std::cout << "cloud size: " << group->lidar.data->points.size() << std::endl;
    // std::cout << "IMU size: " << group->imu_vec.size() << std::endl;
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
      // 使用 intensity 字段存储时间戳
      p.intensity = pt.timestamp;
    }
    t->width    = t->points.size();
    t->height   = 1;
    t->is_dense = true;
    // */
    // way 2
    // pcl::copyPointCloud(*src, *t);

    motion_compensator->UndistortPointCloudByImu(t, group->imu_vec,
                                                 group->lidar.start_time,
                                                 group->lidar.time, 0.5, true);

    if (!group->camera.at(0).data.empty()) {
      // cv::Mat src_img = group->camera.at(0).data.clone();
      cv::Mat src_img = group->camera.at(0).data;
      cv::Mat dst_img = cv::Mat::zeros(src_img.rows, src_img.cols, CV_8UC3);

      if (!init_flag) {
        // std::cout << src_img.rows << std::endl;
        // std::cout << params.size() << std::endl;
        camera_undistort->InitModel(CameraDistortionModel::Brown);
        camera_undistort->InitParams(src_img.cols, src_img.rows, params);
        camera_undistort->Init("camera");
        init_flag = true;
      }
      camera_undistort->Handle(src_img, &dst_img);

      fusion->SetLidarPointCloud(t);
      fusion->SetCameraImage(dst_img);
      fusion->fuse(1, false);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr fused_point_cloud_color;
      cv::Mat fused_image;
      fusion->GetFusedImage(fused_image);
      fusion->GetFusedPointCloudColor(fused_point_cloud_color);

      if (b_first_run) {
        cv::namedWindow("fused_image", cv::WINDOW_GUI_NORMAL);
        cv::resizeWindow("fused_image", 512, 256);  // 只在第一次运行时设置
        b_first_run = false;
      }
      cv::imshow("fused_image", fused_image);
    }

    char key = (char)cv::waitKey(1);
    if (key == 'q' || key == 'Q') {
      break;
    }

    frame_idx++;
  }

  return 0;
}
