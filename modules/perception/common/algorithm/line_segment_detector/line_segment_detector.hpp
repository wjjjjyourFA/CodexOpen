#pragma once

#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <atomic>

#include <opencv2/opencv.hpp>

#include "modules/common/math/math_utils_extra.h"
#include "modules/common/math/unit_converter.h"
#include "modules/common_struct/basic_msgs/VectorPoint.h"
// #include "modules/tools/sensor_calibration/common/point.hpp" ==>
#include "modules/perception/common/base/point.h"

/////////////////////////////////////////////////////////////////////////////////////////
// Default LSD parameters
// SIGMA_SCALE 0.6    - Sigma for Gaussian filter is computed as sigma =
// sigma_scale/scale.
// QUANT       2.0    - Bound to the quantization error on the gradient norm.
// ANG_TH      22.5   - Gradient angle tolerance in degrees.
// LOG_EPS     0.0    - Detection threshold: -log10(NFA) > log_eps
// DENSITY_TH  0.7    - Minimal density of region points in rectangle.
// N_BINS      1024   - Number of bins in pseudo-ordering of gradient modulus.

#ifndef M_LN10
#define M_LN10 2.30258509299404568402
#endif

#define log_gamma(x)                                                           \
  ((x) > 15.0 ? log_gamma_windschitl(x) : log_gamma_lanczos(x))

namespace cstruct = jojo::common_struct;
namespace math    = jojo::common::math;
namespace base    = jojo::perception::base;

struct edge {
  base::PointD p;
  bool taken;
};

/////////////////////////////////////////////////////////////////////////////////////////

// 二维点的“字典序排序比较器”
// 1. 先按 x 坐标升序排序
// 2. 如果 x 相等，再按 y 坐标升序排序
inline bool AsmallerB_XoverY(const edge& a, const edge& b) {
  if (a.p.x == b.p.x)
    return a.p.y < b.p.y;
  else
    return a.p.x < b.p.x;
}

/**
 *   Computes the natural logarithm of the absolute value of
 *   the gamma function of x using Windschitl method.
 *   See http://www.rskey.org/gamma.htm
 */
inline double log_gamma_windschitl(const double& x) {
  return 0.918938533204673 + (x - 0.5) * log(x) - x +
         0.5 * x * log(x * sinh(1 / x) + 1 / (810.0 * pow(x, 6.0)));
}

/**
 *   Computes the natural logarithm of the absolute value of
 *   the gamma function of x using the Lanczos approximation.
 *   See http://www.rskey.org/gamma.htm
 */
inline double log_gamma_lanczos(const double& x) {
  static double q[7] = {75122.6331530, 80916.6278952, 36308.2951477,
                        8687.24529705, 1168.92649479, 83.8676043424,
                        2.50662827511};
  double a           = (x + 0.5) * log(x + 5.5) - (x + 5.5);
  double b           = 0;
  for (int n = 0; n < 7; ++n) {
    a -= log(x + double(n));
    b += q[n] * pow(x, double(n));
  }
  return a + log(b);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum LineSegmentDetectorModes {
  LSD_REFINE_NONE = 0,
  LSD_REFINE_STD  = 1,
  LSD_REFINE_ADV  = 2
};

struct LineSegmentDetectorHyperparam {
  int refine         = LSD_REFINE_NONE;
  double scale       = 0.8;
  double sigma_scale = 0.6;
  double quant       = 2.0;
  double ang_th      = 22.5;
  double log_eps     = 0;
  double density_th  = 0.7;
  int n_bins         = 1024;

  double NOTDEF = -1024.0;  // Label for pixels with undefined gradient.
  uchar NOTUSED = 0;  // Label for pixels not used in yet.
  uchar USED    = 1;  // Label for pixels already used in detection.
};

struct RegionPoint {
  int x;
  int y;
  uchar* used;
  double angle;
  double modgrad;
};

struct NormPoint {
  base::Point2DI p;
  int norm;
};

// Compare NormPoint
static inline bool compareNorm(const NormPoint& n1, const NormPoint& n2) {
  return (n1.norm > n2.norm);
}

static inline bool sortRegionFunction(const RegionPoint& a,
                                      const RegionPoint& b) {
  return (a.x < b.x);
}

// clang-format off
struct rect {
  double x1, y1, x2, y2;  // first and second point of the line segment
  double width;           // rectangle width
  double x, y;            // center of the rectangle
  double theta;           // angle  线段方向（弧度）
  double dx, dy;          // (dx,dy) is vector oriented as the line segment
  double prec;            // tolerance angle
  double p;               // probability of a point with angle within 'prec'
};
// clang-format on

class LineSegmentDetector {
 public:
  /**
   * Create a LineSegmentDetector object. Specifying scale, number of
   * subdivisions for the image, should the lines be refined and other constants
   * as follows:
   *
   * @param _refine       How should the lines found be refined?
   *                      LSD_REFINE_NONE - No refinement applied.
   *                      LSD_REFINE_STD  - Standard refinement is applied. E.g. breaking arches into smaller line approximations.
   *                      LSD_REFINE_ADV  - Advanced refinement. Number of false alarms is calculated,
   *                      lines are refined through increase of precision, decrement in size, etc.
   * @param _scale        The scale of the image that will be used to find the
   * lines. Range (0..1].
   * @param _sigma_scale  Sigma for Gaussian filter is computed as sigma = _sigma_scale/_scale.
   * @param _quant        Bound to the quantization error on the gradient norm.
   * @param _ang_th       Gradient angle tolerance in degrees.
   * @param _log_eps      Detection threshold: -log10(NFA) > _log_eps
   * @param _density_th   Minimal density of aligned region points in rectangle.
   * @param _n_bins       Number of bins in pseudo-ordering of gradient modulus.
   */
  LineSegmentDetector();

  void init();

  bool updateHyperparam(const LineSegmentDetectorHyperparam& hps);

  /**
   * Detect lines in the input image.
   *
   * @param _image    A grayscale(CV_8UC1) input image.
   *                  If only a roi needs to be selected, use
   *                  lsd_ptr->detect(image(roi), ..., lines);
   *                  lines += Scalar(roi.x, roi.y, roi.x, roi.y);
   * @param _lines    Return: A vector of Vec4i or Vector4f elements specifying
   * the beginning and ending point of a line.
   *                          Where Vec4i/Vector4f is (x1, y1, x2, y2), point 1
   * is the start, point 2 - end.
   *                          Returned lines are strictly oriented depending on
   * the gradient.
   * @param width     Return: Vector of widths of the regions, where the lines
   * are found. E.g. Width of line.
   * @param prec      Return: Vector of precisions with which the lines are
   * found.
   */
  void detect(cv::Mat& _image);
  // void detect(cv::Mat& _image, std::vector<Vector4f>& _lines,
  //             std::vector<std::vector<base::Point2DI>>& _line_points,
  //             std::vector<std::vector<int>>& _line_point_map,
  //             std::vector<double>& _width, std::vector<double>& _prec,
  //             std::vector<double>& _nfas);

  void getResults(std::vector<cstruct::Vector4f>& lines,
                  std::vector<std::vector<int>>& line_point_map);

  /**
   * Draw lines_point on the given canvas.
   *
   * @param image         The image, where lines will be drawn.
   *                      Should have the size of the image, where the lines
   * were found
   * @param line_points   The lines that need to be drawn
   */
  void drawSegments(
      cv::Mat& _image,
      const std::vector<std::vector<base::Point2DI>>& line_points);

  /**
   * Draw lines on the given canvas.
   *
   * @param image         The image, where lines will be drawn.
   *                      Should have the size of the image, where the lines
   * were found
   * @param line          The lines that need to be drawn
   */
  void drawSegments(cv::Mat& _image,
                    const std::vector<cstruct::Vector4f>& lines);

  void drawRect(cv::Mat& _image, const rect& rec);

 protected:
  // clang-format off
  std::vector<cstruct::Vector4f> _lines;                  // 检测到的线段端点 (x1, y1, x2, y2)
  std::vector<std::vector<base::Point2DI>> _line_points;  // 每条线段对应的像素点集合
  std::vector<std::vector<int>> _line_point_map;          // 图像中每个像素对应的线段编号
  std::vector<double> _width;                             // 每条线段的宽度估计
  std::vector<double> _prec;                              // precisions 拟合精度
  std::vector<double> _nfa;                               // 显著性（Number of False Alarms，越大越可靠）
  // clang-format on

 private:
  // default hyperparameters
  LineSegmentDetectorHyperparam hps_;

  bool w_needed;
  bool p_needed;
  bool n_needed;

  cv::Mat image;
  cv::Mat scaled_image;
  cv::Mat_<double> angles;  // in rads
  cv::Mat_<double> modgrad;
  cv::Mat_<uchar> used;

  std::vector<NormPoint> ordered_points;

  int img_width;
  int img_height;
  // 基于图像大小估计的噪声常数
  double LOG_NT;

  LineSegmentDetector& operator=(const LineSegmentDetector&);  // to quiet MSVC

  /**  FLSD 算法
   * Detect lines in the whole input image.
   *
   * @param lines         Return: A vector of Vector4f elements specifying the beginning and ending point of a line.
   *                              Where Vector4f is (x1, y1, x2, y2), point 1 is the start, point 2 - end.
   *                              Returned lines are strictly oriented depending
   * on the gradient.
   * @param widths        Return: Vector of widths of the regions, where the lines are found. E.g. Width of line.
   * @param precisions    Return: Vector of precisions with which the lines are
   * found.
   * @param nfas          Return: Vector containing number of false alarms in the line region, with precision of 10%.
   *                              The bigger the value, logarithmically better the detection.
   *                                  * -1 corresponds to 10 mean false alarms
   *                                  * 0 corresponds to 1 mean false alarm
   *                                  * 1 corresponds to 0.1 mean false alarms
   */
  void flsd(std::vector<cstruct::Vector4f>& lines,
            std::vector<std::vector<base::Point2DI>>& line_points,
            std::vector<std::vector<int>>& line_point_map,
            std::vector<double>& widths, std::vector<double>& precisions,
            std::vector<double>& nfas);

  /**
   * Finds the angles and the gradients of the image. Generates a list of pseudo
   * ordered points.
   *
   * @param threshold      The minimum value of the angle that is considered
   * defined, otherwise NOTDEF
   * @param n_bins         The number of bins with which gradients are ordered
   * by, using bucket sort.
   * @param ordered_points Return: Vector of coordinate points that are pseudo
   * ordered by magnitude.
   *                       Pixels would be ordered by norm value, up to a
   * precision given by max_grad/n_bins.
   */
  void ll_angle(const cv::Mat& image, cv::Mat_<double>& angles,
                cv::Mat_<double>& modgrad,
                std::vector<NormPoint>& ordered_points, const double& threshold,
                const unsigned int& n_bins);

  /**
   * Grow a region starting from point s with a defined precision,
   * returning the containing points size and the angle of the gradients.
   *
   * @param s         Starting point for the region.
   * @param reg       Return: Vector of points, that are part of the region
   * @param reg_angle Return: The mean angle of the region.
   * @param prec      The precision by which each region angle should be aligned
   * to the mean.
   */
  void region_grow(const base::Point2DI& s, cv::Mat_<double>& angles,
                   cv::Mat_<double>& modgrad, cv::Mat_<uchar>& used,
                   std::vector<RegionPoint>& reg, double& reg_angle,
                   const double& prec);

  /**
   * Is the point at place 'address' aligned to angle theta, up to precision
   * 'prec'?
   * @return      Whether the point is aligned.
   */
  bool isAligned(const cv::Mat_<double>& angles, int x, int y,
                 const double& theta, const double& prec) const;

  /**
   * Finds the bounding rotated rectangle of a region.
   *
   * @param reg       The region of points, from which the rectangle to be
   * constructed from.
   * @param reg_angle The mean angle of the region.
   * @param prec      The precision by which points were found.
   * @param p         Probability of a point with angle within 'prec'.
   * @param rec       Return: The generated rectangle.
   */
  void region2rect(const std::vector<RegionPoint>& reg, const double reg_angle,
                   const double prec, const double p, rect& rec) const;

  /**
   * Compute region's angle as the principal inertia axis of the region.
   * @return          Regions angle.
   */
  double get_theta(const std::vector<RegionPoint>& reg, const double& x,
                   const double& y, const double& reg_angle,
                   const double& prec) const;

  /**
   * An estimation of the angle tolerance is performed by the standard deviation
   * of the angle at points
   * near the region's starting point. Then, a new region is grown starting from
   * the same point, but using the
   * estimated angle tolerance. If this fails to produce a rectangle with the
   * right density of region points,
   * 'reduce_region_radius' is called to try to satisfy this condition.
   */
  bool refine(std::vector<RegionPoint>& reg, double reg_angle,
              const double prec, double p, rect& rec, const double& density_th);

  /**
   * Reduce the region size, by elimination the points far from the starting
   * point, until that leads to
   * rectangle with the right density of region points or to discard the region
   * if too small.
   */
  bool reduce_region_radius(std::vector<RegionPoint>& reg, double reg_angle,
                            const double prec, double p, rect& rec,
                            double density, const double& density_th);

  /**
   * Try some rectangles variations to improve NFA value. Only if the rectangle
   * is not meaningful (i.e., log_nfa <= log_eps).
   * @return      The new NFA value.
   */
  double rect_improve(rect& rec) const;

  /**
   * Calculates the number of correctly aligned points within the rectangle.
   * @return      The new NFA value.
   */
  double rect_nfa(const rect& rec) const;

  /**
   * Computes the NFA values based on the total number of points, points that
   * agree.
   * n, k, p are the binomial parameters.
   * @return      The new NFA value.
   */
  double nfa(const int& n, const int& k, const double& p) const;

  /**
   * Detect Edge Points in the Region.
   *
   * @param reg_points    region points
   * @param edge_point    Return: sorted edge points.
  */
  void get_edge_point(std::vector<RegionPoint>& reg_points,
                      const double& reg_angle,
                      std::vector<base::Point2DI>& edge_point);
};