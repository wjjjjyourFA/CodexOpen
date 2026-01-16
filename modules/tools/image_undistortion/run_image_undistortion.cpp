#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>

#include "eigen3/Eigen/Eigen"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "cyber/common/file.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/camera/common/undistortion_handler_legacy.h"
#include "modules/perception/common/config/utils.h"
#include "modules/tools/image_undistortion/config/runtime_config.h"

using namespace std;
using namespace apollo::cyber::common;
using namespace jojo::tools;
using namespace jojo::perception::base;
using namespace jojo::perception::camera;
using namespace jojo::perception::config;

int main(int argc, char* argv[]) {
  printf("Image Undistort...\n");

  std::string cofing_path =
      "./../../../config/ImageUndistortion/ImageUndistortion.ini";
  auto param_ = std::make_shared<RuntimeConfig>();
  param_->set_name("ImageUndistortion");
  param_->LoadConfig(cofing_path);
  // std::cout<<param_->calib_file_path<<std::endl;

  auto camera_params = std::make_shared<CameraParams>();
  camera_params->LoadFromFile(param_->calib_file_path /*kk.ini*/);
  auto matrix = camera_params->GetMatrixVector();
  // std::cout<<matrix.at(0)->camera_matrix->intrinsic_matrix<<std::endl;

  auto camera_undistort = std::make_shared<UndistortionHandler>();
  Eigen::VectorXf params(17);
  params =
      IntrinsicParamsToVector(matrix.at(0)->camera_matrix->intrinsic_matrix,
                              matrix.at(0)->camera_matrix->distortion_params);
  // camera_undistort->InitParams();
  // camera_undistort->InitHandler();

  char data_root[256];
  if (argc > 1) {
    // sprintf(data_root, argv[1]);
    snprintf(data_root, sizeof(data_root), "%s", argv[1]);
  } else {
    // strcpy(data_root, param_->data_root_path.c_str());
    snprintf(data_root, sizeof(data_root), "%s",
             param_->data_root_path.c_str());
  }
  std::cout << "data_root: " << data_root << std::endl;
  
  string raw_image_path       = "";
  string undistort_image_path = "";
  char image_list[256];
  string ImageListPath = "";

  if (param_->b_matched) {
    raw_image_path       = std::string(data_root) + "/matched/image";
    undistort_image_path = std::string(data_root) + "/matched/undistort_image";
    ImageListPath =
        std::string(data_root) + "/matched/timestamp/image_timestamp.txt";
  } else {
    raw_image_path       = std::string(data_root) + "/image";
    undistort_image_path = std::string(data_root) + "/undistort_image";
    ImageListPath = std::string(data_root) + "/timestamp/image_timestamp.txt";
  }

  if (access(undistort_image_path.c_str(), 0) == -1) {
    if (mkdir(undistort_image_path.c_str(), 0744) == -1) {
      std::cout << "Creating folder: " << undistort_image_path.c_str()
                << " falied.." << std::endl;
    };
  }

  strcpy(image_list, ImageListPath.c_str());
  // printf("%s\n", image_list);

  bool init_flag = false;

  FILE* fp = fopen(image_list, "r");
  while (!feof(fp)) {
    char image_name[256];
    char timestamp[256];
    fscanf(fp, "%s\n", &timestamp);

    sprintf(image_name, "%s/%s.jpg", raw_image_path.c_str(), timestamp);
    if (!FileExists(image_name)) {
      sprintf(image_name, "%s/%s.png", raw_image_path.c_str(), timestamp);
    }
    // printf("%s\n", image_name);

    cv::Mat src_img = cv::imread(image_name);
    cv::Mat dst_img = cv::Mat::zeros(src_img.rows, src_img.cols, CV_8UC3);

    if (!init_flag) {
      // std::cout << src_img.rows << std::endl;
      // std::cout << params.size() << std::endl;
      camera_undistort->InitParams(src_img.cols, src_img.rows, params);
      camera_undistort->Init("camera");
      init_flag = true;
    }
    camera_undistort->Handle(src_img, &dst_img);

    sprintf(image_name, "%s/%s.jpg", undistort_image_path.c_str(), timestamp);
    cv::imwrite(image_name, dst_img);
    // std::cout << "image_name: " << image_name << std::endl;

    cv::namedWindow("undistort_image", cv::WINDOW_NORMAL);
    cv::imshow("undistort_image", dst_img);
    cv::resizeWindow("undistort_image", 960, 540);  // 设置窗口大小为 800x600
    cv::waitKey(1);
  }
  std::cout << "Finished!" << std::endl;

  return 0;
}
