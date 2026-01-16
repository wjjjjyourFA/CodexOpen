#include "modules/common/config/config_file_base.h"
#include "modules/common/math/math_utils_extra.h"
// #include "modules/perception/common/camera/common/undistortion_handler_legacy.h"
#include "modules/perception/common/camera/common/undistortion_handler_cv.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"
#include "modules/perception/tools/common/show_data_2d.h"
#include "modules/perception/common/fusion/lidar2camera/config/runtime_config.h"

using namespace jojo::common::math;
using namespace jojo::perception;
using namespace jojo::perception::camera;
using namespace jojo::perception::fusion;
namespace cfg = jojo::perception::config;

int main(int argc, char** argv) {
  printf("Lidar Image Fuse...\n");

  std::string cofing_path =
      "./../../../../config/PerceptionFuse/LidarCameraFuseTest.ini";
  auto runtime_config = std::make_shared<RuntimeConfig>();
  runtime_config->set_name("LidarImageFuseTest");
  runtime_config->LoadConfig(cofing_path);

  auto camera_params = std::make_shared<CameraParams>();
  camera_params->LoadFromFile(runtime_config->calib_file_path /*kk.ini*/);
  auto matrix = camera_params->GetMatrixVector();

  cv::Mat raw_img = cv::imread(runtime_config->image_file);
  // 这里应该开辟一个固定新的内存空间，不然会影响原图像，
  // 但是这里只是测试，直接复制的原图像的对象
  cv::Mat dst_img = raw_img.clone();

  if (runtime_config->b_undistort) {
    auto camera_undistort = std::make_shared<UndistortionHandlerCv>();
    Eigen::VectorXf params(17);
    params =
        cfg::IntrinsicParamsToVector(matrix.at(0)->camera_matrix->intrinsic_matrix,
                                     matrix.at(0)->camera_matrix->distortion_params);

    camera_undistort->InitParams(raw_img.cols, raw_img.rows, params);
    camera_undistort->Init("camera");
    camera_undistort->Handle(raw_img, &dst_img);
  } else {
    // 直接读取 undistort_img
    // dst_img = undistort_image.clone();
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  if (runtime_config->b_bin_or_pcd == 0) {
    FILE* fp = fopen(runtime_config->lidar_file.c_str(), "rb");
    int tmp_data[4];
    while (!feof(fp)) {
      fread(tmp_data, sizeof(int), 4, fp);
      pcl::PointXYZI point;
      // cm ==> m    for m matrix
      point.x = float(tmp_data[0] / 100.);
      point.y = float(tmp_data[1] / 100.);
      point.z = float(tmp_data[2] / 100.);
      // for mm matrix
      // point.x = float(tmp_data[0]) * 10;
      // point.y = float(tmp_data[1]) * 10;
      // point.z = float(tmp_data[2]) * 10;
      // for cm matrix
      // point.x = float(tmp_data[0]);
      // point.y = float(tmp_data[1]);
      // point.z = float(tmp_data[2]);
      point.intensity = tmp_data[3];
      cloud->push_back(point);
    }
    fclose(fp);
  } else {
    pcl::io::loadPCDFile<pcl::PointXYZI>(runtime_config->lidar_file, *cloud);
  }
  std::cout << "cloud size: " << cloud->size() << std::endl;
  show2d_lidar_data(cloud);  // 显示点云

  // 去除无效点
  std::vector<int> idx;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, idx);

  if (runtime_config->b_lt_none_rt == 1) {
    // nothing
  } else {
    pcl::transformPointCloud(*cloud, *cloud,
                             GetTransMatrix(runtime_config->b_lt_none_rt));
  }

  auto fusion = std::make_shared<LidarCameraFusion>();
  fusion->SetProjectionMatrix(matrix.at(0)->projection_matrix);
  fusion->SetLidarPointCloud(cloud);
  fusion->SetCameraImage(dst_img);
  fusion->fuse(1, false);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr fused_point_cloud_color;
  cv::Mat fused_image;
  fusion->GetFusedImage(fused_image);
  fusion->GetFusedPointCloudColor(fused_point_cloud_color);

  static bool first_run = true;
  if (first_run) {
    cv::namedWindow("fused_image", cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow("fused_image", 512, 256);  // 只在第一次运行时设置
    first_run = false;
  }
  cv::imshow("fused_image", fused_image);
  cv::waitKey(0);

  return 0;
}
