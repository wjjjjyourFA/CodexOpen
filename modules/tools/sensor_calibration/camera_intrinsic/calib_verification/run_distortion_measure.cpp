#include <dirent.h>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "modules/tools/sensor_calibration/camera_intrinsic/calib_verification/calibration_harp.hpp"

using namespace jojo::tools;

const char usage[] =
    "\t./bin/run_distortion_measure <distortion_image_path>\n"
    "example:\n\t"
    "./bin/run_distortion_measure data/test.png\n";

int main(int argc, char** argv) {
  CalibrationHarp distortion_measurement;

  double d, d_max;

  if (argc < 2) {
    // std::cout << argv[0] << usage;

    std::string cofing_path =
        "./../../../config/SensorCalibration/CalibrationHarp.ini";
    auto runtime_config = std::make_shared<RuntimeConfig>();
    runtime_config->set_name("CalibrationHarp");
    runtime_config->LoadConfig(cofing_path);

    if (distortion_measurement.Init(runtime_config)) {
      auto flag = distortion_measurement.Measure(d, d_max);
    }

    // return 1;
  } else {
    std::string input_image_path = argv[1];
    // cv::Mat image = cv::imread(input_image_path.c_str(), 0);
    if (distortion_measurement.Init(input_image_path)) {
      auto flag = distortion_measurement.Measure(d, d_max);
    }
  }

  return 0;
}