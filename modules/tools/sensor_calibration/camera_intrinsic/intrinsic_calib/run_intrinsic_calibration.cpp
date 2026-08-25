#include <dirent.h>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "modules/tools/sensor_calibration/camera_intrinsic/intrinsic_calib/intrinsic_calibration.hpp"

using namespace jojo::tools;

const char usage[] =
    "\t./bin/run_intrinsic_calibration <calibration_image_dir> \n"
    "example:\n\t"
    "./bin/run_intrinsic_calibration ./data/\n";

int main(int argc, char** argv) {
  IntrinsicCalibration calibrator;

  if (argc < 2) {
    // std::cout << argv[0] << usage;

    std::string cofing_path =
        "./../../../config/SensorCalibration/IntrinsicCalib.ini";
    auto runtime_config = std::make_shared<RuntimeConfig>();
    runtime_config->set_name("IntrinsicCalib");
    runtime_config->LoadConfig(cofing_path);

    if (calibrator.Init(runtime_config)) {
      calibrator.Calibrate();
      calibrator.SaveCalibrationResult();
    }

    // return 1;
  } else {
    std::string input_image_path = argv[1];
    // cv::Mat image = cv::imread(input_image_path.c_str(), 0);
    if (calibrator.Init(input_image_path)) {
      calibrator.Calibrate();
      calibrator.SaveCalibrationResult();
    }
  }

  return 0;
}