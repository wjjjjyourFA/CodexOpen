#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "modules/common_struct/basic_msgs/Pose.h"
#include "modules/common_struct/localization_msgs/PoseStamp.h"
#include "modules/common_struct/planning_msgs/Planning.h"

namespace jojo {
namespace planning {

struct WaypointPublisherConfig {
  std::string waypoint_file;
  std::string boundary_file;
  double waypoint_z_bound{5.0};
  double wait_time{0.0};
  double frame_rate{5.0};
  double speed{1.0};
  bool send_speed{true};
  bool send_boundary{true};
  double waypoint_xy_radius{0.5};
  double waypoint_yaw_threshold{0.1745};
  std::string waypoint_frame{"map"};
  std::string boundary_frame{"vehicle"};
};

struct WaypointPublisherOutput {
  bool goal_valid{false};
  bool publish_waypoint{false};
  bool publish_speed{false};
  bool publish_boundary{false};
  common_struct::PoseStamped waypoint;
  common_struct::PointStamped waypoint_show;
  float speed{0.0F};
  common_struct::PolygonStamped boundary;
};

class WaypointPublisher {
 public:
  explicit WaypointPublisher(const WaypointPublisherConfig& config);

  void SetOdometry(std::uint64_t timestamp_ns,
                   const common_struct::Pose& pose);
  std::size_t AddNavigationGoal(const common_struct::Pose& pose);
  WaypointPublisherOutput Step();
  std::size_t WaypointCount() const { return waypoints_.size(); }

 private:
  void ReadWaypointFile();
  void ReadBoundaryFile();
  static double Yaw(const common_struct::Quaternion& quaternion);
  static common_struct::Quaternion QuaternionFromRpy(double roll,
                                                      double pitch,
                                                      double yaw);

  WaypointPublisherConfig config_;
  std::vector<common_struct::Pose> waypoints_;
  std::vector<common_struct::Vector3f> boundary_points_;
  common_struct::Pose vehicle_pose_;
  std::uint64_t current_time_ns_{0};
  std::uint64_t waypoint_publish_time_ns_{0};
  std::uint64_t wait_start_time_ns_{0};
  std::size_t waypoint_index_{0};
  bool waiting_{false};
};

}  // namespace planning
}  // namespace jojo
