#include "modules/common/transform/adapters/loam_adapter.h"

namespace loam {
using namespace jojo::common::transform;

void Ro2Ang(const Eigen::Matrix3d& R, Eigen::Vector3d& ypr) {
  // Y1X2Z3, applicable for loam coordinate (X->left, Y->upward, Z->forward)
  // Y → X → Z（YXZ）

  // ypr[0] = yaw; ypr[1] = pitch; ypr[2] = roll

  // ypr(1)  = -std::asin(R(1, 2));
  double v = std::clamp(-R(1, 2), -1.0, 1.0);
  ypr(1) = std::asin(v);

  double cp = std::cos(ypr(1));

  if (std::abs(cp) > 1e-6) {
    ypr(2) = std::atan2(R(1, 0), R(1, 1));  // roll (Z)
    ypr(0) = std::atan2(R(0, 2), R(2, 2));  // yaw (Y)
  } else {
    ypr(2) = 0.0;
    ypr(0) = std::atan2(-R(2, 0), R(0, 0));
  }
}

void Transform2NormalTr(float* transform, Eigen::Matrix4f& RT) {
  // transform: Tr_BA in loam coordinate (left-up-forward)
  // RT: Tr_AB in our coordinate (right-forward-up)

  double cry = cos(-transform[1]);
  double sry = sin(-transform[1]);
  double crx = cos(-transform[0]);
  double srx = sin(-transform[0]);
  double crz = cos(-transform[2]);
  double srz = sin(-transform[2]);

  Eigen::Matrix3f Rx, Ry, Rz, R_w_l_in_loam, R_zxy;

  // clang-format off
  Ry <<  cry, 0, sry, 
           0, 1,   0, 
        -sry, 0, cry;
  Rx << 1,   0,    0, 
        0, crx, -srx, 
        0, srx,  crx;
  Rz << crz, -srz, 0, 
        srz,  crz, 0, 
          0,    0, 1;
  // clang-format on

  R_w_l_in_loam = Ry * Rx * Rz;

  Eigen::Vector3f T_BA, T_AB_LOAM;

  T_BA << transform[3], transform[4], transform[5];
  T_AB_LOAM = -R_w_l_in_loam * T_BA;

  if (0) {
    double c_a = cos(-transform[1]);  // aizmuth-->rot z
    double s_a = sin(-transform[1]);
    double c_p = cos(transform[0]);  // pitch---> rot x
    double s_p = sin(transform[0]);
    double c_r = cos(-transform[2]);  // roll---> rot y
    double s_r = sin(-transform[2]);

    // clang-format off
    Ry << c_r, 0, s_r, 
            0, 1,   0, 
         -s_r, 0, c_r;
    Rx << 1,   0,    0, 
          0, c_p, -s_p, 
          0, s_p,  c_p;
    Rz << c_a, -s_a, 0, 
          s_a,  c_a, 0, 
            0,    0, 1;
    // clang-format on

    // firstly azimuth, next pitch, finally roll
    R_zxy = Rz * Rx * Ry;

    RT.setZero();
    RT.topLeftCorner<3, 3>() = R_zxy;

    RT(0, 3) = -T_AB_LOAM[0];
    RT(1, 3) = T_AB_LOAM[2];
    RT(2, 3) = T_AB_LOAM[1];
    RT(3, 3) = 1;
  } else {
    Eigen::Matrix3f R_loam_our;
    // clang-format off
    R_loam_our << -1, 0, 0, 
                   0, 0, 1, 
                   0, 1, 0;
    // clang-format on               
    Eigen::Matrix3f R_our_loam = R_loam_our.transpose();

    Eigen::Matrix3f R_AB_our = R_our_loam * R_w_l_in_loam * R_loam_our;
    Eigen::Vector3f T_AB_our = R_our_loam * T_AB_LOAM;

    RT.setZero();
    RT.topLeftCorner<3, 3>()  = R_AB_our;
    RT.topRightCorner<3, 1>() = T_AB_our;

    RT(3, 3) = 1;
  }
}

void NormalTr2Transform(const Eigen::Matrix4f& RT, float* transform) {
  // RT: Tr_AB in our coordinate (right-forward-up)
  // transform: Tr_BA in loam coordinate (left-up-forward)

  if (0) {
    Eigen::Vector3f ypr;
    Eigen::Matrix3f R = RT.topLeftCorner<3, 3>();
    RotationToEulerZXY(R, ypr);

    transform[0] = ypr(1);  // -(-pitch)
    transform[1] = -ypr(0);  // -yaw
    transform[2] = -ypr(2);  // -roll

    Eigen::Vector3f T_AB_LOAM, T_BA_LOAM;
    T_AB_LOAM[0] = -RT(0, 3);
    T_AB_LOAM[1] = RT(2, 3);
    T_AB_LOAM[2] = RT(1, 3);

    double cry = cos(-transform[1]);
    double sry = sin(-transform[1]);
    double crx = cos(-transform[0]);
    double srx = sin(-transform[0]);
    double crz = cos(-transform[2]);
    double srz = sin(-transform[2]);

    Eigen::Matrix3f Rx, Ry, Rz, R_AB;

    // clang-format off
    Ry <<  cry, 0, sry, 
             0, 1,   0, 
          -sry, 0, cry;
    Rx << 1,   0,    0, 
          0, crx, -srx, 
          0, srx,  crx;
    Rz << crz, -srz, 0, 
          srz,  crz, 0, 
            0,    0, 1;
    // clang-format on

    R_AB = Ry * Rx * Rz;

    T_BA_LOAM = -R_AB.transpose() * T_AB_LOAM;  // T_BA =  -R_BA * T_AB

    transform[3] = T_BA_LOAM[0];
    transform[4] = T_BA_LOAM[1];
    transform[5] = T_BA_LOAM[2];
  } else {
    // clang-format off
    Eigen::Matrix4f Tr_loam_our;
    Tr_loam_our << -1, 0, 0, 0, 
                    0, 0, 1, 0, 
                    0, 1, 0, 0, 
                    0, 0, 0, 1;
    Eigen::Matrix4f Tr_our_loam;

    // Tr_our_loam = Tr_loam_our.inverse();
    Tr_our_loam << -1, 0, 0, 0, 
                    0, 0, 1, 0, 
                    0, 1, 0, 0, 
                    0, 0, 0, 1;
    // clang-format on
    Eigen::Matrix4f Tr_AB_LOAM = Tr_loam_our * RT * Tr_our_loam;

    Eigen::Matrix3f R_AB = Tr_AB_LOAM.topLeftCorner<3, 3>();
    Eigen::Vector3f T_AB = Tr_AB_LOAM.topRightCorner<3, 1>();
    Eigen::Vector3f T_BA = -R_AB.transpose() * T_AB;

    Eigen::Vector3d ypr;
    Ro2Ang(R_AB, ypr);

    transform[0] = -ypr(1);
    transform[1] = -ypr(0);
    transform[2] = -ypr(2);
    transform[3] = T_BA(0);
    transform[4] = T_BA(1);
    transform[5] = T_BA(2);
  }
}

}  // namespace loam