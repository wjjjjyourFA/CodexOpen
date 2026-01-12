#include "modules/perception/common/camera/parameter/read_camera_params.h"
#include "modules/perception/common/camera/common/undistortion_handler_legacy.h"

using namespace jojo::perception::camera;

int main(int argc, char** argv) {
  // 读取 标定参数
  auto camera_params = std::make_shared<CameraParams>();
  camera_params->ReadCameraParaBase("kk.ini");
  auto matrix = camera_params->GetMatrixVector();

  // 初始化 去畸变器
  auto camera_undistort = std::make_shared<UndistortionHandler>();
  Eigen::VectorXf params(17);
  params =
      IntrinsicParamsToVector(matrix.at(0)->camera_matrix->intrinsic_matrix,
                              matrix.at(0)->camera_matrix->distortion_params);

  cv::Mat src_img = cv::imread("test.jpg");
  cv::Mat dst_img = cv::Mat::zeros(src_img.rows, src_img.cols, CV_8UC3);

  camera_undistort->InitParams(src_img.cols, src_img.rows, params);
  camera_undistort->Init("camera");
  camera_undistort->Handle(src_img, &dst_img);

  cv::namedWindow("undistort_image", cv::WINDOW_NORMAL);
  cv::imshow("undistort_image", dst_img);
  cv::resizeWindow("undistort_image", 960, 540);  // 设置窗口大小为 800x600
  cv::waitKey(0);

  return 0;
}