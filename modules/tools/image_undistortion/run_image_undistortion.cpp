#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>

#include <fstream>

#include "eigen3/Eigen/Eigen"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include "cyber/common/file.h"
#include "modules/perception/common/camera/params/camera_params.h"
#include "modules/perception/common/camera/common/undistortion_handler.h"
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

  bool preview = false;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--preview") preview = true;
  }

  std::string name = "ImageUndistortion";
  std::string cofing_path =
      "./../../../config/ImageUndistortion/ImageUndistortion.ini";
  auto param_ = std::make_shared<RuntimeConfig>();
  param_->set_name(name);
  param_->LoadConfig(cofing_path);
  if (param_->calib_file_path.empty()) {
    std::cerr << "Calibration path is empty after loading: " << cofing_path
              << std::endl;
    return EXIT_FAILURE;
  }
  // std::cout<<param_->calib_file_path<<std::endl;

  auto camera_params = std::make_shared<CameraParams>();
  camera_params->LoadFromFile(param_->calib_file_path /*kk.ini*/);
  auto matrix = camera_params->GetMatrixVector();
  if (matrix.empty() || !matrix.front() || !matrix.front()->camera_matrix) {
    std::cerr << "Calibration file contains no camera matrix: "
              << param_->calib_file_path << std::endl;
    return EXIT_FAILURE;
  }
  // std::cout<<matrix.at(0)->camera_matrix->intrinsic_matrix<<std::endl;

  auto camera_undistort = std::make_shared<UndistortionHandler>();
  Eigen::VectorXf params(17);
  params =
      IntrinsicParamsToVector(matrix.at(0)->camera_matrix->intrinsic_matrix,
                              matrix.at(0)->camera_matrix->distortion_params);
  // camera_undistort->InitModel(CameraDistortionModel::Brown);
  // camera_undistort->InitParams();
  // camera_undistort->InitHandler();

  const std::string data_root = argc > 1 ? argv[1] : param_->data_root_path;
  if (data_root.empty()) {
    std::cerr << "data_root is empty" << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "data_root: " << data_root << std::endl;

  string raw_image_path       = "";
  string undistort_image_path = "";
  string ImageListPath = "";

  if (param_->b_matched) {
    raw_image_path       = data_root + "/matched/image";
    undistort_image_path = data_root + "/matched/undistort_image";
    ImageListPath =
        data_root + "/matched/timestamp/image_timestamp.txt";
  } else {
    raw_image_path       = data_root + "/image";
    undistort_image_path = data_root + "/undistort_image";
    ImageListPath = data_root + "/timestamp/image_timestamp.txt";
  }

  if (access(undistort_image_path.c_str(), 0) == -1) {
    if (mkdir(undistort_image_path.c_str(), 0744) == -1) {
      std::cerr << "Failed to create folder: " << undistort_image_path
                << std::endl;
      return EXIT_FAILURE;
    };
  }

  bool init_flag = false;

  std::ifstream timestamp_file(ImageListPath);
  if (!timestamp_file) {
    std::cerr << "Cannot open timestamp list: " << ImageListPath << std::endl;
    return EXIT_FAILURE;
  }

  std::string timestamp;
  while (timestamp_file >> timestamp) {
    std::string image_name = raw_image_path + "/" + timestamp + ".jpg";
    if (!FileExists(image_name)) {
      image_name = raw_image_path + "/" + timestamp + ".png";
    }
    // printf("%s\n", image_name);

    cv::Mat src_img = cv::imread(image_name);
    if (src_img.empty()) {
      std::cerr << "Skip unreadable image: " << image_name << std::endl;
      continue;
    }
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

    image_name = undistort_image_path + "/" + timestamp + ".jpg";
    if (!cv::imwrite(image_name, dst_img)) {
      std::cerr << "Failed to write image: " << image_name << std::endl;
      return EXIT_FAILURE;
    }
    // std::cout << "image_name: " << image_name << std::endl;

    if (preview) {
      cv::namedWindow("undistort_image", cv::WINDOW_NORMAL);
      cv::imshow("undistort_image", dst_img);
      cv::resizeWindow("undistort_image", 960, 540);
      cv::waitKey(1);
    }
  }
  std::cout << "Finished!" << std::endl;

  return 0;
}
