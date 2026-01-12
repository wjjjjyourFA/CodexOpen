#ifndef READ_CAMERA_PARAMS_H
#define READ_CAMERA_PARAMS_H

#pragma once

#include <iostream>
#include <fstream>  // 用于 std::ifstream
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "modules/perception/common/config/utils.h"

namespace jojo {
namespace perception {
namespace camera {
// namespace cfg = jojo::perception::config;

class CameraMatrix {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraMatrix() {
    intrinsic_matrix  = Eigen::Matrix3f::Identity();
    distortion_params = Eigen::Matrix<float, 8, 1>::Zero();
  }
  ~CameraMatrix() = default;

  Eigen::Matrix3f intrinsic_matrix;  // K
  Eigen::Matrix<float, 8, 1> distortion_params;  // K1K2P1P2K3K4K5K6

  void SetZero() {
    intrinsic_matrix.setZero();
    distortion_params.setZero();
  }
};

// 如果你用裸指针 指向一个 new 出来的对象，之后又给它赋了一个新的地址，
// 原来那个 new 出来的内存就失去管理（丢失指针），不会被释放，就造成内存泄漏。
class Lidar2CameraMatrix {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Lidar2CameraMatrix() {
    extrinsic_matrix  = Eigen::Matrix4f::Identity();
    projection_matrix = Eigen::Matrix4f::Identity();
    camera_matrix     = std::make_shared<CameraMatrix>();
  }
  ~Lidar2CameraMatrix() = default;

  std::shared_ptr<CameraMatrix> camera_matrix;  // 相机内参
  // 为了便于理解，所以用的 4X4
  Eigen::Matrix4f extrinsic_matrix;  // RT
  Eigen::Matrix4f projection_matrix;  // P

  void SetZero() {
    camera_matrix->SetZero();
    extrinsic_matrix.setZero();
    projection_matrix.setZero();
  }

  void UpdataProjectionMatrix() {
    projection_matrix = Eigen::Matrix4f::Identity();
    projection_matrix.block<3, 4>(0, 0) =
        camera_matrix->intrinsic_matrix * extrinsic_matrix.block<3, 4>(0, 0);
  }
};

/* 参数读取，并保存为 Eigen::MatrixXf 类型
 * 默认雷达点云单位为 m
 * 默认相机外参单位为 m
 * 默认标定阵方向 雷达前左上
 */
/* ID
 * 1 右前上点云 对应 右前上标定矩阵
 * 2 前左上点云 对应 前左上标定矩阵
 * 需要考虑 : 点云的单位，P阵的单位
 * default : m
 * input : Front Left Up
 * output : Front Left Up
 */
class CameraParams {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraParams() {};
  virtual ~CameraParams() = default;

  void SetLoadPath(const std::string& load_path);

  void LoadFromName(const std::string& camera_name);

  void LoadFromFile(const std::string& camera_file);

  bool InitMatrixVector(int camera_p_num);

  // clang-format off
  // 返回的是对象的常量引用，不是裸指针。
  // 不是指针，所以不用用 ->
  // 数据队列不能修改，但指针指向的值可以修改
  const std::vector<std::shared_ptr<Lidar2CameraMatrix>>& GetMatrixVector() const {
    return lidar2camera_vector;
  }

  const std::vector<std::shared_ptr<CameraMatrix>>& GetBaseMatrixVector() const {
    return camera_vector;
  }
  // clang-format on

 private:
  std::string LoadPath = "";

  bool LoadFromFileBase(const char* filename, Eigen::Matrix3f& intrinsic_matrix,
                        Eigen::Matrix<float, 8, 1>& distortion_params,
                        Eigen::Matrix4f& extrinsic_matrix,
                        Eigen::Matrix4f& projection_matrix);

  // camera_p_num 1 2 3 4
  bool ReadFileWrapper(const std::string& camera_file);

  std::vector<std::shared_ptr<Lidar2CameraMatrix>> lidar2camera_vector;
  std::vector<std::shared_ptr<CameraMatrix>> camera_vector;
};

}  // namespace camera
}  // namespace perception
}  // namespace jojo

#endif  // READ_CAMERA_PARAMS_H
