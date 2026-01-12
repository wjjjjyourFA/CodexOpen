/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/common/camera/common/undistortion_handler_legacy.h"

#include <vector>

#include "Eigen/Dense"
#include "cyber/common/log.h"
// #include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
// #include "modules/perception/common/camera/common/image_data_operations.h"

#ifdef _OPENMP
// CPU 并行计算
#include <omp.h>
// 防止忘记设置 线程数，可在环境变量中声明
// export OMP_NUM_THREADS=8
#include <atomic>
#endif

namespace jojo {
namespace perception {
namespace camera {

bool UndistortionHandler::InitParams(size_t width, size_t height, double k1,
                                     double k2, double k3, double p1, double p2,
                                     double fx, double fy, double cx,
                                     double cy) {
  if (type != CameraDistortionModel::Brown) {
    return false;
  }

  width_  = width;
  height_ = height;

  Eigen::VectorXf params(14);

  // intrinsic_params
  params(0) = fx;
  params(1) = 0;
  params(2) = cx;
  params(3) = 0;
  params(4) = fy;
  params(5) = cy;
  params(6) = 0;
  params(7) = 0;
  params(8) = 1;

  // distort_params_
  params(9)  = k1;
  params(10) = k2;
  params(11) = p1;
  params(12) = p2;
  params(13) = k3;

  distort_model->set_params(width_, height_, params);

  return true;
}

bool UndistortionHandler::InitParams(size_t width, size_t height, double k1,
                                     double k2, double k3, double k4, double fx,
                                     double fy, double cx, double cy) {
  if (type != CameraDistortionModel::Kannala) {
    return false;
  }

  width_  = width;
  height_ = height;

  Eigen::VectorXf params(14);

  // intrinsic_params
  params(0) = fx;
  params(1) = 0;
  params(2) = cx;
  params(3) = 0;
  params(4) = fy;
  params(5) = cy;
  params(6) = 0;
  params(7) = 0;
  params(8) = 1;

  // distort_params_
  params(9)  = k1;
  params(10) = k2;
  params(11) = k3;
  params(12) = k4;

  distort_model->set_params(width_, height_, params);

  return true;
}

bool UndistortionHandler::InitParams(size_t width, size_t height,
                                     const Eigen::VectorXf& params) {
  width_  = width;
  height_ = height;
  // std::cout << "width_: " << width << std::endl;
  // printf("width_: %zu\n", width);

  distort_model->set_params(width_, height_, params);

  return true;
}

bool UndistortionHandler::Init(const std::string& sensor_name) {
  if (inited_) {
    return true;
  }

  // height_ = static_cast<int>(distort_model->get_height());
  // width_ = static_cast<int>(distort_model->get_width());

  // clang-format off
  switch (this->type) {
    case CameraDistortionModel::Brown:
      // std::cout << "intrinsic:\n" << distort_model->get_intrinsic_params() << std::endl;
      // std::cout << "distort:\n" << distort_model->get_distort_params() << std::endl;

      InitUndistortRectifyMapSimple(distort_model->get_intrinsic_params(),
                                    distort_model->get_distort_params(), 
                                    width_, height_);

      UpdateIntrinsicParams(distort_model->get_intrinsic_params(),
                            distort_model->get_distort_params());

      // std::cout << "InitUndistortRectifyMapSimple()" << std::endl;
      break;
    case CameraDistortionModel::Kannala:
      InitFisheyeUndistortRectifyMapSimple(distort_model->get_intrinsic_params(),
                                           distort_model->get_distort_params(), 
                                           width_, height_);
      
      UpdateFisheyeIntrinsicParams(distort_model->get_intrinsic_params(),
                                   distort_model->get_distort_params());
      break;
    default:
      inited_ = false;
      return false;
      break;
  }
  // clang-format on

  inited_ = true;
  return true;
}

bool UndistortionHandler::Handle(const cv::Mat& src_img, cv::Mat* dst_img) {
  if (dst_img == nullptr) {
    return false;  // 防止空指针
  }

  if (!inited_) {
    return false;
  }

  return CorrectImage(src_img, *dst_img);
}

bool UndistortionHandler::Handle(const cv::Mat& src_img, cv::Mat& dst_img) {
  return this->Handle(src_img, &dst_img);  // 调用指针版本
}

bool UndistortionHandler::Release(void) {
  inited_ = false;
  return true;
}

void UndistortionHandler::UpdateIntrinsicParams(
    const Eigen::Matrix3f& intrinsic_params,
    const Eigen::Matrix<float, 8, 1>& distort_params) {
  // 图像去畸变，必须生成等效针孔模型的内参数
  // cv::Mat cameraMatrix, distCoeffs;
  cameraMatrix                 = cv::Mat::eye(3, 3, CV_32F);
  cameraMatrix.at<float>(0, 0) = intrinsic_params(0, 0);  // fx
  cameraMatrix.at<float>(1, 1) = intrinsic_params(1, 1);  // fy
  cameraMatrix.at<float>(0, 2) = intrinsic_params(0, 2);  // cx
  cameraMatrix.at<float>(1, 2) = intrinsic_params(1, 2);  // cy

  distCoeffs = cv::Mat::zeros(1, 5, CV_32F);
  for (int i = 0; i < 5; ++i) {
    distCoeffs.at<float>(0, i) = distort_params(i, 0);
  }

  cv::Size imageSize(width_, height_);

  // 计算新的内参矩阵
  newCameraMatrix =
      cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1.0);
}

// zjt add wj edit
void UndistortionHandler::InitUndistortRectifyMapSimple(
    const Eigen::Matrix3f& intrinsic_params,
    const Eigen::Matrix<float, 8, 1>& distort_params, int width, int height) {
  // std::cout << "[UndistortionHandler] InitUndistortRectifyMapSimple() called" << std::endl;

  // transform to image coordinates
  float fx = intrinsic_params(0, 0);
  float fy = intrinsic_params(1, 1);
  float cx = intrinsic_params(0, 2);
  float cy = intrinsic_params(1, 2);
  // radial distort_params coefficients
  float k1 = distort_params(0, 0);
  float k2 = distort_params(1, 0);
  // tangential distort_params coefficients
  float p1 = distort_params(2, 0);
  float p2 = distort_params(3, 0);
  float k3 = distort_params(4, 0);
  float k4 = distort_params(5, 0);  // add k4,k5,k6 for Rational model
  float k5 = distort_params(6, 0);
  float k6 = distort_params(7, 0);

  // 计算去畸变后图像的内容
  static const int numel = width_ * height_;
  // way 2 预分配，保证和循环一一对应
  uv_map_.resize(numel);

#ifdef _OPENMP
  // 在进入并行区前设置线程数
  omp_set_num_threads(4);
  // #pragma omp parallel for ：并行 for 循环，把循环迭代自动分给多个线程
  // #pragma omp critical ：这一段代码只能同时被一个线程执行。
#pragma omp parallel for
#endif
  for (int i = 0; i < numel; i++) {
    // 获取当前线程 ID（0 ~ n-1）
    // printf("线程 %d 计算 a[%d]\n", omp_get_thread_num(), i, i);

    int row = i / width_;
    int col = i % width_;

    // 标准归一化像平面坐标
    double x = (col - cx) / fx;
    double y = (row - cy) / fy;

    /*
    double xy = x * y;
    double x2 = x * x;
    double y2 = y * y;

    double r2 = x2 + y2;
    double r4 = r2 * r2;
    double r6 = r4 * r2;

    // 畸变后归一化像平面坐标
    double x_distorted = x * (1 + k1 * r2 + k2 * r4 + k3 * r6) /
                             (1 + k4 * r2 + k5 * r4 + k6 * r6) +
                         2 * p1 * xy + p2 * (r2 + 2 * x2);
    double y_distorted = y * (1 + k1 * r2 + k2 * r4 + k3 * r6) /
                             (1 + k4 * r2 + k5 * r4 + k6 * r6) +
                         p1 * (r2 + 2 * y2) + 2 * p2 * xy;
    */
    auto [x_distorted, y_distorted] =
        ApplyDistortion(x, y, k1, k2, k3, k4, k5, k6, p1, p2);

    // 像素坐标
    size_t u_d = fx * x_distorted + cx + 0.5;
    size_t v_d = fy * y_distorted + cy + 0.5;

    /* // way 1 单独加锁，保证不竞争，但效率低
#pragma omp critical
    // record valid pair
    if (u_d >= 0 && u_d < width_ && v_d >= 0 && v_d < height_) {
      uv_map_.push_back(
          std::make_pair(Eigen::Vector2i(row, col), Eigen::Vector2i(v_d, u_d)));
    }
    */
    // way 2
    if (u_d >= 0 && u_d < width_ && v_d >= 0 && v_d < height_) {
      // 直接按索引存，不需要加锁
      uv_map_[i] =
          std::make_pair(Eigen::Vector2i(row, col), Eigen::Vector2i(v_d, u_d));
    } else {
      // 标记为无效点
      uv_map_[i] =
          std::make_pair(Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, -1));
    }
  }

  // 去掉无效点（可选）
  uv_map_.erase(std::remove_if(uv_map_.begin(), uv_map_.end(),
                               [](const auto& p) { return p.first(0) < 0; }),
                uv_map_.end());
}

// Rational 畸变模型（标准针孔 + 径向/切向畸变扩展） cv::initUndistortRectifyMap()
std::pair<double, double> UndistortionHandler::ApplyDistortion(
    double x, double y, double k1, double k2, double k3, double k4, double k5,
    double k6, double p1, double p2) {
  double x2 = x * x;
  double y2 = y * y;
  double xy = x * y;
  double r2 = x2 + y2;
  double r4 = r2 * r2;
  double r6 = r4 * r2;

  double radial_distortion   = 1 + k1 * r2 + k2 * r4 + k3 * r6;
  double rational_distortion = 1 + k4 * r2 + k5 * r4 + k6 * r6;
  double scale               = radial_distortion / rational_distortion;

  double x_distorted = x * scale + 2 * p1 * xy + p2 * (r2 + 2 * x2);
  double y_distorted = y * scale + p1 * (r2 + 2 * y2) + 2 * p2 * xy;

  return {x_distorted, y_distorted};
}

void UndistortionHandler::UpdateFisheyeIntrinsicParams(
    const Eigen::Matrix3f& intrinsic_params,
    const Eigen::Matrix<float, 8, 1>& distort_params) {
  // 图像去畸变，必须生成等效针孔模型的内参数
  // cv::Mat cameraMatrix, distCoeffs;
  cameraMatrix                 = cv::Mat::eye(3, 3, CV_32F);
  cameraMatrix.at<float>(0, 0) = intrinsic_params(0, 0);  // fx
  cameraMatrix.at<float>(1, 1) = intrinsic_params(1, 1);  // fy
  cameraMatrix.at<float>(0, 2) = intrinsic_params(0, 2);  // cx
  cameraMatrix.at<float>(1, 2) = intrinsic_params(1, 2);  // cy

  distCoeffs = cv::Mat::zeros(1, 4, CV_32F);
  for (int i = 0; i < 4; ++i) {
    distCoeffs.at<float>(0, i) = distort_params(i, 0);
  }

  cv::Size imageSize(width_, height_);

  newCameraMatrix = cameraMatrix.clone();

  // 计算新的内参矩阵
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      cameraMatrix, distCoeffs, imageSize, cv::Matx33d::eye(), newCameraMatrix,
      1.0);
}

void UndistortionHandler::InitFisheyeUndistortRectifyMapSimple(
    const Eigen::Matrix3f& intrinsic_params,
    const Eigen::Matrix<float, 8, 1>& distort_params, int width, int height) {
  // transform to image coordinates
  float fx = intrinsic_params(0, 0);
  float fy = intrinsic_params(1, 1);
  float cx = intrinsic_params(0, 2);
  float cy = intrinsic_params(1, 2);
  // distort_params coefficients
  float k1 = distort_params(0, 0);
  float k2 = distort_params(1, 0);
  float k3 = distort_params(2, 0);
  float k4 = distort_params(3, 0);

  // 计算去畸变后图像的内容
  static const int numel = width_ * height_;
  // way 2 预分配，保证和循环一一对应
  uv_map_.resize(numel);

#ifdef _OPENMP
  // 在进入并行区前设置线程数
  omp_set_num_threads(4);
  // #pragma omp parallel for ：并行 for 循环，把循环迭代自动分给多个线程
  // #pragma omp critical ：这一段代码只能同时被一个线程执行。
#pragma omp parallel for
#endif
  for (int i = 0; i < numel; i++) {
    // 获取当前线程 ID（0 ~ n-1）
    // printf("线程 %d 计算 a[%d]\n", omp_get_thread_num(), i, i);

    int row = i / width_;
    int col = i % width_;

    // 标准归一化像平面坐标
    double x = (col - cx) / fx;
    double y = (row - cy) / fy;

    auto [x_distorted, y_distorted] =
        ApplyFisheyeDistortion(x, y, k1, k2, k3, k4);

    // 像素坐标
    size_t u_d = fx * x_distorted + cx + 0.5;
    size_t v_d = fy * y_distorted + cy + 0.5;

    /* // way 1 单独加锁，保证不竞争，但效率低
#pragma omp critical
    // record valid pair
    if (u_d >= 0 && u_d < width_ && v_d >= 0 && v_d < height_) {
      uv_map_.push_back(
          std::make_pair(Eigen::Vector2i(row, col), Eigen::Vector2i(v_d, u_d)));
    }
    */
    // way 2
    if (u_d >= 0 && u_d < width_ && v_d >= 0 && v_d < height_) {
      // 直接按索引存，不需要加锁
      uv_map_[i] =
          std::make_pair(Eigen::Vector2i(row, col), Eigen::Vector2i(v_d, u_d));
    } else {
      // 标记为无效点
      uv_map_[i] =
          std::make_pair(Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, -1));
    }
  }

  // 去掉无效点（可选）
  uv_map_.erase(std::remove_if(uv_map_.begin(), uv_map_.end(),
                               [](const auto& p) { return p.first(0) < 0; }),
                uv_map_.end());
}

// Kannala–Brandt（2006）多项式鱼眼畸变模型 cv::fisheye::initUndistortRectifyMap()
std::pair<double, double> UndistortionHandler::ApplyFisheyeDistortion(
    double x, double y, double k1, double k2, double k3, double k4) {
  double r = std::sqrt(x * x + y * y);
  if (r < 1e-8) return {0.0, 0.0};  // 避免除零

  double theta  = std::atan(r);
  double theta2 = theta * theta;
  double theta4 = theta2 * theta2;
  double theta6 = theta4 * theta2;
  double theta8 = theta4 * theta4;

  // fisheye 多项式畸变
  double theta_d =
      theta * (1 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);

  double scale = theta_d / r;

  double x_distorted = x * scale;
  double y_distorted = y * scale;

  return {x_distorted, y_distorted};
}

bool UndistortionHandler::CorrectImage(const cv::Mat& src_img,
                                       cv::Mat& dst_img) {
  /* // just copy value
  // 直接拷贝，不知道怎么回事，会报错；可能是内存对齐的问题
  for (const std::pair<Eigen::Vector2i, Eigen::Vector2i> &vec : uv_map_) {
    dst_img.at<cv::Vec3b>(vec.first(0), vec.first(1)) =
        src_img.at<cv::Vec3b>(vec.second(0), vec.second(1));
  }
  */

#ifdef _OPENMP
  // /* // bounds check way 2 多线程 逐像素拷贝
  omp_set_num_threads(4);
#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < uv_map_.size(); i++) {
    int r  = uv_map_[i].first(0);
    int c  = uv_map_[i].first(1);
    int sr = uv_map_[i].second(0);
    int sc = uv_map_[i].second(1);

    // 既能保证 r ≥ 0（因为负数转成 unsigned 会变成大整数，自动排除负值），
    // 又能直接和 dst_img.rows（int）比较，无需额外判断 r >= 0。
    // 好处：不改变原变量类型，循环或函数签名依然使用 int，方便与 OpenCV API、Eigen 等兼容。
    if ((unsigned)r < (unsigned)dst_img.rows &&
        (unsigned)c < (unsigned)dst_img.cols &&
        (unsigned)sr < (unsigned)src_img.rows &&
        (unsigned)sc < (unsigned)src_img.cols) {
      // dst_img.at<cv::Vec3b>(r, c) = src_img.at<cv::Vec3b>(sr, sc);
      cv::Vec3b* dst_ptr       = dst_img.ptr<cv::Vec3b>(r);
      const cv::Vec3b* src_ptr = src_img.ptr<cv::Vec3b>(sr);
      dst_ptr[c]               = src_ptr[sc];
    }
  }
  // */
#else
  // /* // bounds check way 1 单线程 逐像素拷贝
  for (const auto& vec : uv_map_) {
    int r = vec.first(0), c = vec.first(1);
    int sr = vec.second(0), sc = vec.second(1);

    if (r >= 0 && r < dst_img.rows && c >= 0 && c < dst_img.cols && sr >= 0 &&
        sr < src_img.rows && sc >= 0 && sc < src_img.cols) {
      dst_img.at<cv::Vec3b>(r, c) = src_img.at<cv::Vec3b>(sr, sc);
    }
  }
  // */
#endif

  return true;
}

void UndistortionHandler::GetRectifiedIntrinsic(
    std::shared_ptr<CameraMatrix> matrix) {
  // intrinsic_params
  matrix->intrinsic_matrix(0, 0) = newCameraMatrix.at<float>(0, 0);
  matrix->intrinsic_matrix(0, 2) = newCameraMatrix.at<float>(0, 2);
  matrix->intrinsic_matrix(1, 1) = newCameraMatrix.at<float>(1, 1);
  matrix->intrinsic_matrix(1, 2) = newCameraMatrix.at<float>(1, 2);

  // distort_params_
  matrix->distortion_params = Eigen::Matrix<float, 8, 1>::Zero();
}

}  // namespace camera
}  // namespace perception
}  // namespace jojo
