#include "modules/localization/common/transform/frame2d_transform.hpp"

namespace jojo {
namespace localization {
namespace common {

void Frame2dTransform::SetSensorInBody(const double& offset_x,
                                       const double& offset_y,
                                       const double& offset_theta) {
  offset_x_     = offset_x;
  offset_y_     = offset_y;
  offset_theta_ = offset_theta;
  sinot_        = sin(offset_theta_);
  cosot_        = cos(offset_theta_);
}

void Frame2dTransform::SetBodyInOdom(const double& dr_x, const double& dr_y,
                                     const double& dr_theta) {
  dr_x_     = dr_x;
  dr_y_     = dr_y;
  dr_theta_ = dr_theta;
  sindt_    = sin(dr_theta_);
  cosdt_    = cos(dr_theta_);
}

}  // namespace common
}  // namespace localization
}  // namespace jojo