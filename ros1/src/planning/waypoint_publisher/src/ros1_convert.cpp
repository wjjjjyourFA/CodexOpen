#include "waypoint_publisher_ros1/ros1_convert.h"

#include <exception>

namespace jojo {
namespace planning {
namespace ros1 {

WaypointPublisherRos1Convert::WaypointPublisherRos1Convert(
    ::ros::NodeHandle& node, ::ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {}

bool WaypointPublisherRos1Convert::Init() {
  WaypointPublisherConfig config;
  private_node_.param("waypoint_file_dir", config.waypoint_file,
                      std::string());
  private_node_.param("boundary_file_dir", config.boundary_file,
                      std::string());
  private_node_.param("waypointZBound", config.waypoint_z_bound,
                      config.waypoint_z_bound);
  private_node_.param("waitTime", config.wait_time, config.wait_time);
  private_node_.param("frameRate", config.frame_rate, config.frame_rate);
  private_node_.param("speed", config.speed, config.speed);
  private_node_.param("sendSpeed", config.send_speed, config.send_speed);
  private_node_.param("sendBoundary", config.send_boundary,
                      config.send_boundary);
  private_node_.param("waypointXYRadius", config.waypoint_xy_radius,
                      config.waypoint_xy_radius);
  private_node_.param("waypointYawThreshold",
                      config.waypoint_yaw_threshold,
                      config.waypoint_yaw_threshold);
  private_node_.param("waypoint_frame", config.waypoint_frame,
                      config.waypoint_frame);
  private_node_.param("boundary_frame", config.boundary_frame,
                      config.boundary_frame);

  private_node_.param("odometry_topic", odometry_topic_, odometry_topic_);
  private_node_.param("waypoint_topic", waypoint_topic_, waypoint_topic_);
  private_node_.param("waypoint_show_topic", waypoint_show_topic_,
                      waypoint_show_topic_);
  private_node_.param("navigation_goal_topic", navigation_goal_topic_,
                      navigation_goal_topic_);
  private_node_.param("speed_topic", speed_topic_, speed_topic_);
  private_node_.param("boundary_topic", boundary_topic_, boundary_topic_);
  private_node_.param("goal_valid_topic", goal_valid_topic_,
                      goal_valid_topic_);
  private_node_.param("transport/processing_frequency",
                      processing_frequency_, processing_frequency_);
  private_node_.param("queues/odometry", odometry_queue_size_,
                      odometry_queue_size_);
  private_node_.param("queues/navigation_goal", navigation_goal_queue_size_,
                      navigation_goal_queue_size_);
  private_node_.param("queues/output", output_queue_size_,
                      output_queue_size_);
  if (processing_frequency_ <= 0.0 || odometry_queue_size_ <= 0 ||
      navigation_goal_queue_size_ <= 0 || output_queue_size_ <= 0) {
    ROS_ERROR("waypoint_publisher interface rates/queues must be positive");
    return false;
  }

  try {
    core_.reset(new WaypointPublisher(config));
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("waypoint_publisher configuration failed: "
                     << error.what());
    return false;
  }

  odometry_subscriber_ = node_.subscribe<nav_msgs::Odometry>(
      odometry_topic_, odometry_queue_size_,
      &WaypointPublisherRos1Convert::OdometryCallback, this);
  navigation_goal_subscriber_ = node_.subscribe<geometry_msgs::PoseStamped>(
      navigation_goal_topic_, navigation_goal_queue_size_,
      &WaypointPublisherRos1Convert::NavigationGoalCallback, this);
  waypoint_publisher_ =
      node_.advertise<geometry_msgs::PoseStamped>(waypoint_topic_,
                                                  output_queue_size_);
  waypoint_show_publisher_ =
      node_.advertise<geometry_msgs::PointStamped>(waypoint_show_topic_,
                                                   output_queue_size_);
  speed_publisher_ = node_.advertise<std_msgs::Float32>(
      speed_topic_, output_queue_size_);
  boundary_publisher_ =
      node_.advertise<geometry_msgs::PolygonStamped>(boundary_topic_,
                                                      output_queue_size_);
  goal_valid_publisher_ =
      node_.advertise<std_msgs::Bool>(goal_valid_topic_, output_queue_size_);
  if (core_->WaypointCount() == 0) {
    ROS_WARN("No waypoint available; waiting for an RViz navigation goal");
  }
  return true;
}

common_struct::Pose WaypointPublisherRos1Convert::FromRosPose(
    const geometry_msgs::Pose& pose) {
  common_struct::Pose result;
  result.position = common_struct::Vector3d(
      pose.position.x, pose.position.y, pose.position.z);
  result.orientation.x = pose.orientation.x;
  result.orientation.y = pose.orientation.y;
  result.orientation.z = pose.orientation.z;
  result.orientation.w = pose.orientation.w;
  return result;
}

geometry_msgs::Pose WaypointPublisherRos1Convert::ToRosPose(
    const common_struct::Pose& pose) {
  geometry_msgs::Pose result;
  result.position.x = pose.position.x;
  result.position.y = pose.position.y;
  result.position.z = pose.position.z;
  result.orientation.x = pose.orientation.x;
  result.orientation.y = pose.orientation.y;
  result.orientation.z = pose.orientation.z;
  result.orientation.w = pose.orientation.w;
  return result;
}

std_msgs::Header WaypointPublisherRos1Convert::ToRosHeader(
    const common_struct::Header& header) {
  std_msgs::Header result;
  result.seq = header.seq;
  result.stamp.fromNSec(header.timestamp);
  result.frame_id = header.frame_id;
  return result;
}

void WaypointPublisherRos1Convert::OdometryCallback(
    const nav_msgs::OdometryConstPtr& message) {
  core_->SetOdometry(message->header.stamp.toNSec(),
                     FromRosPose(message->pose.pose));
}

void WaypointPublisherRos1Convert::NavigationGoalCallback(
    const geometry_msgs::PoseStampedConstPtr& message) {
  const std::size_t count = core_->AddNavigationGoal(FromRosPose(message->pose));
  ROS_INFO_STREAM("Received RViz navigation goal; total waypoints: " << count);
}

void WaypointPublisherRos1Convert::Run() {
  ::ros::Rate rate(processing_frequency_);
  while (::ros::ok()) {
    ::ros::spinOnce();
    const WaypointPublisherOutput output = core_->Step();
    if (output.publish_waypoint) {
      geometry_msgs::PoseStamped waypoint;
      waypoint.header = ToRosHeader(output.waypoint.header);
      waypoint.pose = ToRosPose(output.waypoint.pose);
      waypoint_publisher_.publish(waypoint);

      geometry_msgs::PointStamped show;
      show.header = ToRosHeader(output.waypoint_show.header);
      show.point.x = output.waypoint_show.point.x;
      show.point.y = output.waypoint_show.point.y;
      show.point.z = output.waypoint_show.point.z;
      waypoint_show_publisher_.publish(show);
    }
    if (output.publish_speed) {
      std_msgs::Float32 speed;
      speed.data = output.speed;
      speed_publisher_.publish(speed);
    }
    if (output.publish_boundary) {
      geometry_msgs::PolygonStamped boundary;
      boundary.header = ToRosHeader(output.boundary.header);
      boundary.polygon.points.resize(output.boundary.points.size());
      for (std::size_t index = 0; index < output.boundary.points.size();
           ++index) {
        boundary.polygon.points[index].x = output.boundary.points[index].x;
        boundary.polygon.points[index].y = output.boundary.points[index].y;
        boundary.polygon.points[index].z = output.boundary.points[index].z;
      }
      boundary_publisher_.publish(boundary);
    }
    std_msgs::Bool valid;
    valid.data = output.goal_valid;
    goal_valid_publisher_.publish(valid);
    rate.sleep();
  }
}

}  // namespace ros1
}  // namespace planning
}  // namespace jojo
