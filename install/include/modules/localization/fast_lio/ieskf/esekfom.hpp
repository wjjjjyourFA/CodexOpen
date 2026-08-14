#ifndef ESEKFOM_EKF_HPP
#define ESEKFOM_EKF_HPP

#pragma once

#include <vector>
#include <cstdlib>

#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "modules/perception/common/algorithm/point_cloud_processing/ikd-Tree/ikd_Tree.h"

#include "modules/localization/fast_lio/ieskf/use-ikfom.hpp"

namespace esekfom {
using namespace Eigen;

struct dyn_share_datastruct {
  bool valid;
  bool converge;
  Eigen::Matrix<double, Eigen::Dynamic, 1> h;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x;
};

class esekf {
 public:
  typedef Matrix<double, 24, 24> cov;
  typedef Matrix<double, 24, 1> vectorized_state;

  esekf();
  ~esekf();

  state_ikfom boxplus(state_ikfom x, Eigen::Matrix<double, 24, 1> f_);

  vectorized_state boxminus(state_ikfom x1, state_ikfom x2);

  void h_share_model(dyn_share_datastruct& ekfom_data,
                     fastlio::PointCloudXYZI::Ptr& feats_down_body,
                     KD_TREE<fastlio::PointType>& ikdtree,
                     std::vector<fastlio::PointVector>& Nearest_Points,
                     bool extrinsic_est);

  // iterated error state EKF propogation
  void predict(double& dt, Eigen::Matrix<double, 12, 12>& Q,
               const input_ikfom& i_in);

  // iterated error state EKF update modified for one specific system.
  void update_iterated_dyn_share_modified(
      double R, fastlio::PointCloudXYZI::Ptr& feats_down_body,
      KD_TREE<fastlio::PointType>& ikdtree, std::vector<fastlio::PointVector>& Nearest_Points,
      int maximum_iter, bool extrinsic_est);

  void change_x(state_ikfom& input_state);

  void change_P(cov& input_cov);

  state_ikfom get_x() const;
  cov get_P() const;

 private:
  state_ikfom x_;
  cov P_;

  const double epsi = 0.001;

  fastlio::PointCloudXYZI::Ptr normvec;
  fastlio::PointCloudXYZI::Ptr laserCloudOri;
  fastlio::PointCloudXYZI::Ptr corr_normvect;

  bool point_selected_surf[100000] = {0};
};

}  // namespace esekfom

#endif  //  ESEKFOM_EKF_HPP
