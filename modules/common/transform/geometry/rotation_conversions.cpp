#include "modules/common/transform/geometry/rotation_conversions.h"

namespace jojo {
namespace common {
namespace transform {

void RotationToQuaternion(const Eigen::Matrix3d& R, Eigen::Quaterniond& q) {
  // ==> Eigen::Quaterniond q(R);

  // [x, y, z, w]（coeffs顺序）
  Eigen::Quaterniond q_;

  double s, s_inv;

  // double trace = R(0, 0) + R(1, 1) + R(2, 2);
  double trace = R.trace();

  if (trace > 0) {
    s = std::sqrt(trace + 1.0) * 2.0;  // s = 4 * qw
    if (s < 1e-12) {
      q = Eigen::Quaterniond::Identity();
      return;
    }
    s_inv = 1.0 / s;

    q_.w() = 0.25 * s;
    q_.x() = (R(2, 1) - R(1, 2)) * s_inv;
    q_.y() = (R(0, 2) - R(2, 0)) * s_inv;
    q_.z() = (R(1, 0) - R(0, 1)) * s_inv;
  } else {
    if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)) {
      s = std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;
      if (s < 1e-12) {
        q = Eigen::Quaterniond::Identity();
        return;
      }
      s_inv = 1.0 / s;

      q_.w() = (R(2, 1) - R(1, 2)) * s_inv;
      q_.x() = 0.25 * s;
      q_.y() = (R(0, 1) + R(1, 0)) * s_inv;
      q_.z() = (R(0, 2) + R(2, 0)) * s_inv;
    } else if (R(1, 1) > R(2, 2)) {
      s = std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;
      if (s < 1e-12) {
        q = Eigen::Quaterniond::Identity();
        return;
      }
      s_inv = 1.0 / s;

      q_.w() = (R(0, 2) - R(2, 0)) * s_inv;
      q_.x() = (R(0, 1) + R(1, 0)) * s_inv;
      q_.y() = 0.25 * s;
      q_.z() = (R(1, 2) + R(2, 1)) * s_inv;
    } else {
      s = std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;
      if (s < 1e-12) {
        q = Eigen::Quaterniond::Identity();
        return;
      }
      s_inv = 1.0 / s;

      q_.w() = (R(1, 0) - R(0, 1)) * s_inv;
      q_.x() = (R(0, 2) + R(2, 0)) * s_inv;
      q_.y() = (R(1, 2) + R(2, 1)) * s_inv;
      q_.z() = 0.25 * s;
    }
  }

  q = q_.normalized();
}

void RotationToQuaternion(const Eigen::Matrix3d& R, Eigen::Vector4d& q) {
  // from ROS tf/LinearMath/Matrix3x3.h
  Eigen::Vector4d q_;

  double s, s_inv;
  int i, j, k;

  double trace = R(0, 0) + R(1, 1) + R(2, 2);

  if (trace > 0) {
    s = std::sqrt(trace + 1) * 2.0;

    s_inv = 1.0 / s;

    q_(3) = 0.25 * s;
    q_(0) = (R(2, 1) - R(1, 2)) * s_inv;
    q_(1) = (R(0, 2) - R(2, 0)) * s_inv;
    q_(2) = (R(1, 0) - R(0, 1)) * s_inv;
  } else {
    if (R(0, 0) < R(1, 1)) {
      if (R(1, 1) < R(2, 2))
        i = 2;
      else
        i = 1;
    } else {
      if (R(0, 0) < R(2, 2))
        i = 2;
      else
        i = 0;
    }
    j = (i + 1) % 3;
    k = (i + 2) % 3;

    s = std::sqrt(R(i, i) - R(j, j) - R(k, k) + 1.0) * 2.0;

    s_inv = 1.0 / s;

    q_(i) = s * 0.5;
    q_(3) = (R(k, j) - R(j, k)) * s_inv;
    q_(j) = (R(j, i) + R(i, j)) * s_inv;
    q_(k) = (R(k, i) + R(i, k)) * s_inv;
  }

  q = q_;
}

}  // namespace transform
}  // namespace common
}  // namespace jojo