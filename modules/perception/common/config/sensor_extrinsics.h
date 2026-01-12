#ifndef READ_BASE_PARAMS_H
#define READ_BASE_PARAMS_H

#pragma once

#include <iostream>
#include <fstream>  // 用于 std::ifstream
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "modules/perception/common/config/utils.h"

namespace jojo {
namespace perception {
namespace config {

class ExtrinsicMatrix {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ExtrinsicMatrix() { matrix = Eigen::Matrix4f::Identity(); }
  ~ExtrinsicMatrix() {}

  Eigen::Matrix4f matrix;  // RT

  void SetZero() { matrix.setZero(); }
};

class SensorExtrinsics {
 public:
  SensorExtrinsics() {};
  virtual ~SensorExtrinsics() = default;

  void SetLoadPath(const std::string& load_path);

  void LoadFromName(const std::string& sensor_name);

  void LoadFromFile(const std::string& sensor_file);

  virtual bool InitMatrixVector(int sensor_p_num) {};

  // const（在返回类型前）表示返回的是一个常量引用，你不能通过这个引用去修改 vector 本身
  // 函数最后的 const 表示这个成员函数不会修改类对象的状态（即不会修改成员变量）。
  const std::vector<std::shared_ptr<ExtrinsicMatrix>>& GetMatrixVector() const {
    return extrinsics_;
  }

 private:
  std::string LoadPath = "";

  bool LoadFromFileBase(const char* filename,
                        Eigen::Matrix4f& extrinsic_matrix);

  // sensor_p_num 1 2 3 4
  bool ReadFileWrapper(const std::string& sensor_file);

  std::vector<std::shared_ptr<ExtrinsicMatrix>> extrinsics_;
};

}  // namespace config
}  // namespace perception
}  // namespace jojo

#endif  // READ_BASE_PARAMS_H
