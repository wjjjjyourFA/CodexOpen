#ifndef SUPER_LIO_ROBOT_DATA_INTERFACE_H_
#define SUPER_LIO_ROBOT_DATA_INTERFACE_H_

#include <memory>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "common/ds.h"

namespace LI2Sup {

class ESKF;

// Transport-neutral boundary used by the LIO core. ROS1 conversion and
// publication live in ros1/src/mapping/super_lio_robot.
class DataInterface {
 public:
  using Ptr = std::shared_ptr<DataInterface>;

  virtual ~DataInterface() = default;

  virtual bool sync_measure(MeasureGroup& measurements) = 0;
  virtual void setESKF(std::shared_ptr<ESKF>& eskf) = 0;
  virtual void pub_odom(const NavState& state) = 0;
  virtual void pub_cloud_world(const BASIC::CloudPtr& cloud,
                               double timestamp) = 0;
  virtual void pub_registered_scan(const BASIC::CloudPtr& cloud,
                                   double timestamp) = 0;
  virtual void set_global_map(const BASIC::CloudPtr& global_map) = 0;
  virtual void set_initial_data(BASIC::SE3& initial_pose,
                                bool& has_initial_guess,
                                bool initialization_finished = false) = 0;
};

}  // namespace LI2Sup

#endif  // SUPER_LIO_ROBOT_DATA_INTERFACE_H_
