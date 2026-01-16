#pragma once

#include <opencv2/opencv.hpp>

// #include "modules/tools/sensor_calibration/common/point.hpp"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/algorithm/line_segment_detector/line_segment_detector.hpp"
#include "modules/tools/sensor_calibration/camera_intrinsic/calib_verification/config/runtime_config.h"

namespace cstruct = jojo::common_struct;
namespace base    = jojo::perception::base;

inline bool ComparePointXAsc(const base::Point2DI& a, const base::Point2DI& b) {
  return (a.x < b.x);
}

inline bool ComparePointYAsc(const base::Point2DI& a, const base::Point2DI& b) {
  return (a.y < b.y);
}

class CalibrationHarp {
 public:
  CalibrationHarp();
  ~CalibrationHarp() {}

  bool Init(const std::string& img_dir_path);
  bool Init(std::shared_ptr<jojo::tools::RuntimeConfig> param);
  bool isInited() const { return initialized_; };

  bool Measure(double& _d, double& _d_max);

 protected:
  std::atomic_bool initialized_{false};

 private:
  std::shared_ptr<jojo::tools::RuntimeConfig> param_;

  std::string image_path_;

  cv::Mat origin_image_;
  std::vector<std::vector<base::Point2DI>> selected_line_points_;
  cv::Mat line_support_region_;
  double distortion_error_;
  double max_distortion_error_;

  bool interpolate_edge_points(
      const std::vector<cstruct::Vector4f>& lines,
      const std::vector<std::vector<base::Point2DI>>& edge_points,
      std::vector<std::vector<base::Point2DI>>& interpolated_edge_points);

  bool gaussion_subsample_points(
      const std::vector<std::vector<base::Point2DI>>& interpolated_edge_points,
      std::vector<std::vector<base::Point2DI>>& subsampled_edge_points,
      const int t = 30);

  bool calculate_error(
      // const std::vector<cstruct::Vector4f> &lines,
      // const std::vector< std::vector<base::Point2DI> > &edge_points,
      const std::vector<std::vector<base::Point2DI>>& subsampled_edge_point,
      double& d, double& d_max, double& d_c_median);

  bool get_line_param(const std::vector<base::Point2DI>& edge_points,
                      double& alpha, double& beta, double& gama);

  double get_theta(const std::vector<base::Point2DI>& edge_points,
                   const double& Ax, const double& Ay);
};
