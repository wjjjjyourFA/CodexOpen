#pragma once

namespace jojo {
namespace common_struct {

// Communication-framework-neutral quaternion.  Field names intentionally
// match geometry_msgs/Quaternion so adapters can perform a lossless, explicit
// member-for-member conversion without exposing ROS in a core target.
struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

}  // namespace common_struct
}  // namespace jojo
