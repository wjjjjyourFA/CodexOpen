#include "modules/mapping/map_visualization/config/runtime_config.h"
#include "modules/mapping/map_visualization/map_visualization.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "toolz/data_loader/group_convert.h"

using namespace jojo::tools;
using namespace jojo::perception::camera;
using namespace jojo::perception::fusion;
namespace cfg = jojo::perception::config;

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

  auto camera_params = std::make_shared<CameraParams>();
  camera_params->LoadFromFile(
      runtime_config->camera_calib_file_path /*kk.ini*/);
  auto matrix2 = camera_params->GetMatrixVector();

  auto camera_undistort = std::make_shared<UndistortionHandler>();
  Eigen::VectorXf params(17);
  params = cfg::IntrinsicParamsToVector(
      matrix2.at(0)->camera_matrix->intrinsic_matrix,
      matrix2.at(0)->camera_matrix->distortion_params);

  auto fusion = std::make_shared<LidarCameraFusion>();
  fusion->SetProjectionMatrix(matrix2.at(0)->projection_matrix);

  std::shared_ptr<MapVisualization> view = std::make_shared<MapVisualization>();
  view->Init(runtime_config, static_config);
  view->InitViewer();

  std::string dl_rc_path = "./../../../config/MapVisualization/DataLoader.ini";
  std::string dl_ic_path = "./../../../config/MapVisualization/Interface.ini";

  auto dl_runtime_config = std::make_shared<RuntimeConfigOffline>();
  dl_runtime_config->set_name(name);
  dl_runtime_config->LoadConfig(dl_rc_path);

  auto dl_interface_config = std::make_shared<jojo::tools::InterfaceConfig>();
  dl_interface_config->set_name(name);
  dl_interface_config->LoadConfig(dl_ic_path);

  auto group_convert = std::make_shared<GroupConvertDataSet>();
  group_convert->Init(dl_runtime_config, dl_interface_config);

  view->LoadMapLabel();

  bool init_flag   = false;
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

    if (b_first_run) {
      const auto& p_center = group->pose_center;
      view->SetPoseCenter(p_center);

      b_first_run = false;
    }

    const auto& frame = group->lidar.data;
    const auto& pose  = group->se3_pose.data.matrix();

    view->Run(frame, pose);

    pcl::PointCloud<pcl::PointXYZI>::Ptr t;
    view->GetPointCloud(t);

    if (!group->camera.at(0).data.empty()) {
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

      if (b_first_run) {
        cv::namedWindow("fused_image", cv::WINDOW_GUI_NORMAL);
        cv::resizeWindow("fused_image", 512, 256);  // 只在第一次运行时设置
        b_first_run = false;
      }
      cv::imshow("fused_image", fused_image);
    }

    // 等待按键
    char key = (char)cv::waitKey(1);  // 1ms，不阻塞主程序
    if (key == 'q' || key == 'Q') {
      break;
    }

    frame_idx++;
  }

  return 0;
}