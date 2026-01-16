#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "modules/tools/sensor_calibration/camera_intrinsic/fisheye_calib/fisheye_calibration.h"

using namespace jojo::tools;

int main() {
  FisheyeCalibration calibrator;

  std::string cofing_path =
      "./../../../config/SensorCalibration/FisheyeCalibration.ini";

  auto runtime_config = std::make_shared<RuntimeConfig>();
  runtime_config->set_name("FisheyeCalibration");
  runtime_config->LoadConfig(cofing_path);

  if (calibrator.Init(runtime_config)) {
    calibrator.Calibrate();
    calibrator.SaveCalibrationResult();
  }

  return 0;
}