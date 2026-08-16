#include "modules/common/transform/adapters/isam_adapter.h"

namespace isam {
using namespace jojo::common::transform;

void Ro2Ang(const Eigen::Matrix3d& R, Eigen::Vector3d& ypr) {
  RotationToEulerZYX(R, ypr);
}

void Transform2Tr(float* transform, Eigen::Matrix4f& RT) {
  if (transform == nullptr) {
    RT.setIdentity();
    return;
  }
  // transformSum: Tr_w_l in isam coordinate (forward-left-up)
  // RT: Tr_w_l in isam coordinate (forward-left-up)

  double croll  = cos(transform[5]);
  double sroll  = sin(transform[5]);
  double cpitch = cos(transform[4]);
  double spitch = sin(transform[4]);
  double cyaw   = cos(transform[3]);
  double syaw   = sin(transform[3]);

  Eigen::Matrix3f Rx, Ry, Rz, R_w_l_in_isam;

  // clang-format off
  Ry << cpitch,  0, spitch, 
             0,  1,      0, 
        -spitch, 0, cpitch;
  Rx << 1,     0,      0, 
        0, croll, -sroll, 
        0, sroll,  croll;
  Rz << cyaw, -syaw, 0, 
        syaw,  cyaw, 0, 
           0,     0, 1;
  // clang-format on

  R_w_l_in_isam = Rz * Ry * Rx;

  RT.setZero();
  RT.topLeftCorner<3, 3>() = R_w_l_in_isam;

  RT(0, 3) = transform[0];
  RT(1, 3) = transform[1];
  RT(2, 3) = transform[2];
  RT(3, 0) = 0;
  RT(3, 1) = 0;
  RT(3, 2) = 0;
  RT(3, 3) = 1;
}

Eigen::Matrix4f Transform2Tr(const TransformArray& transform) {
  Eigen::Matrix4f result;
  TransformArray mutable_transform = transform;
  Transform2Tr(mutable_transform.data(), result);
  return result;
}

void Tr2Transform(const Eigen::Matrix4f& RT, float* transform) {
  if (transform == nullptr) {
    return;
  }
  // RT: Tr_AB in isam coordinate
  // transform: Tr_AB in isam coordinate (forward-left-up)

  Eigen::Matrix3f R_AB = RT.topLeftCorner<3, 3>();
  Eigen::Vector3f T_AB = RT.topRightCorner<3, 1>();

  Eigen::Vector3d ypr;
  Ro2Ang(R_AB.cast<double>(), ypr);

  transform[3] = static_cast<float>(ypr(0));
  transform[4] = static_cast<float>(ypr(1));
  transform[5] = static_cast<float>(ypr(2));
  transform[0] = T_AB(0);
  transform[1] = T_AB(1);
  transform[2] = T_AB(2);
}

TransformArray Tr2Transform(const Eigen::Matrix4f& RT) {
  TransformArray result{{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
  Tr2Transform(RT, result.data());
  return result;
}

}  // namespace isam
