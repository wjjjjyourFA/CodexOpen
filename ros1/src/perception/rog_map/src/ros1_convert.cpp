#include "rog_map_ros1/ros1_convert.h"

#include <array>
#include <cmath>
#include <exception>
#include <utility>
#include <vector>

#include <boost/bind/bind.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>

namespace jojo {
namespace perception {
namespace ros1 {
namespace {

bool LoadVector3(::ros::NodeHandle& node, const std::string& name,
                 const rog_map::Vec3f& default_value,
                 rog_map::Vec3f* value) {
  std::vector<double> elements;
  if (!node.getParam(name, elements)) {
    *value = default_value;
    return true;
  }
  if (elements.size() != 3U) {
    ROS_ERROR_STREAM("parameter '" << name << "' must contain 3 values");
    return false;
  }
  *value = rog_map::Vec3f(elements[0], elements[1], elements[2]);
  return true;
}

bool LoadVector2(::ros::NodeHandle& node, const std::string& name,
                 double default_minimum, double default_maximum,
                 double* minimum, double* maximum) {
  std::vector<double> elements;
  if (!node.getParam(name, elements)) {
    *minimum = default_minimum;
    *maximum = default_maximum;
    return true;
  }
  if (elements.size() != 2U) {
    ROS_ERROR_STREAM("parameter '" << name << "' must contain 2 values");
    return false;
  }
  *minimum = elements[0];
  *maximum = elements[1];
  return true;
}

void LoadProbability(::ros::NodeHandle& node, const std::string& name,
                     float* probability) {
  double value = *probability;
  node.param(name, value, value);
  *probability = static_cast<float>(value);
}

template <typename PointT>
void PublishCloud(const pcl::PointCloud<PointT>& cloud,
                  const std::string& frame_id, const ::ros::Time& stamp,
                  const ::ros::Publisher& publisher) {
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header.frame_id = frame_id;
  message.header.stamp = stamp;
  publisher.publish(message);
}

std_msgs::ColorRGBA MakeColor(float red, float green, float blue,
                             float alpha = 1.0F) {
  std_msgs::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

visualization_msgs::Marker MakeBoundsMarker(
    const rog_map::MapBounds& bounds, const std::string& frame_id,
    const ::ros::Time& stamp, const std::string& name, int id,
    const std_msgs::ColorRGBA& color) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = name;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color = color;

  const rog_map::Vec3f& low = bounds.minimum;
  const rog_map::Vec3f& high = bounds.maximum;
  const std::array<rog_map::Vec3f, 8> corners{{
      {low.x(), low.y(), low.z()},
      {high.x(), low.y(), low.z()},
      {high.x(), high.y(), low.z()},
      {low.x(), high.y(), low.z()},
      {low.x(), low.y(), high.z()},
      {high.x(), low.y(), high.z()},
      {high.x(), high.y(), high.z()},
      {low.x(), high.y(), high.z()},
  }};
  constexpr std::array<std::array<int, 2>, 12> edges{{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
      {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}},
  }};
  marker.points.reserve(edges.size() * 2U);
  for (const auto& edge : edges) {
    for (const int corner_index : edge) {
      geometry_msgs::Point point;
      point.x = corners[corner_index].x();
      point.y = corners[corner_index].y();
      point.z = corners[corner_index].z();
      marker.points.push_back(point);
    }
  }
  return marker;
}

}  // namespace

RogMapRos1Convert::RogMapRos1Convert(::ros::NodeHandle& node,
                                     ::ros::NodeHandle& private_node)
    : node_(node), private_node_(private_node) {}

bool RogMapRos1Convert::LoadCoreConfig(rog_map::Config* config) {
  private_node_.param("rog_map/resolution", config->resolution,
                      config->resolution);
  private_node_.param("rog_map/inflation_resolution",
                      config->inflation_resolution,
                      config->inflation_resolution);
  private_node_.param("rog_map/inflation_step", config->inflation_step,
                      config->inflation_step);
  private_node_.param("rog_map/unk_inflation_en",
                      config->unk_inflation_en,
                      config->unk_inflation_en);
  private_node_.param("rog_map/unk_inflation_step",
                      config->unk_inflation_step,
                      config->unk_inflation_step);
  private_node_.param("rog_map/frontier_extraction_en",
                      config->frontier_extraction_en,
                      config->frontier_extraction_en);
  private_node_.param("rog_map/virtual_ceil_height",
                      config->virtual_ceil_height,
                      config->virtual_ceil_height);
  private_node_.param("rog_map/virtual_ground_height",
                      config->virtual_ground_height,
                      config->virtual_ground_height);
  private_node_.param("rog_map/safe_margin", config->safe_margin,
                      config->safe_margin);
  private_node_.param("rog_map/intensity_thresh", config->intensity_thresh,
                      config->intensity_thresh);
  private_node_.param("rog_map/point_filt_num", config->point_filt_num,
                      config->point_filt_num);
  private_node_.param("rog_map/load_pcd_en", config->load_pcd_en,
                      config->load_pcd_en);
  private_node_.param("rog_map/pcd_name", config->pcd_name,
                      config->pcd_name);
  private_node_.param("rog_map/map_sliding/enable",
                      config->map_sliding_en, config->map_sliding_en);
  private_node_.param("rog_map/map_sliding/threshold",
                      config->map_sliding_thresh,
                      config->map_sliding_thresh);
  private_node_.param("rog_map/esdf/enable", config->esdf_en,
                      config->esdf_en);
  private_node_.param("rog_map/esdf/resolution", config->esdf_resolution,
                      config->esdf_resolution);
  private_node_.param("rog_map/raycasting/enable",
                      config->raycasting_en, config->raycasting_en);
  private_node_.param("rog_map/raycasting/batch_update_size",
                      config->batch_update_size,
                      config->batch_update_size);
  private_node_.param("rog_map/raycasting/unk_thresh",
                      config->unk_thresh, config->unk_thresh);
  LoadProbability(private_node_, "rog_map/raycasting/p_hit",
                  &config->p_hit);
  LoadProbability(private_node_, "rog_map/raycasting/p_miss",
                  &config->p_miss);
  LoadProbability(private_node_, "rog_map/raycasting/p_min",
                  &config->p_min);
  LoadProbability(private_node_, "rog_map/raycasting/p_max",
                  &config->p_max);
  LoadProbability(private_node_, "rog_map/raycasting/p_occ",
                  &config->p_occ);
  LoadProbability(private_node_, "rog_map/raycasting/p_free",
                  &config->p_free);

  if (!LoadVector3(private_node_, "rog_map/map_size", config->map_size_d,
                   &config->map_size_d) ||
      !LoadVector3(private_node_, "rog_map/fix_map_origin",
                   config->fix_map_origin, &config->fix_map_origin) ||
      !LoadVector3(private_node_, "rog_map/raycasting/local_update_box",
                   config->local_update_box_d,
                   &config->local_update_box_d) ||
      !LoadVector3(private_node_, "rog_map/esdf/local_update_box",
                   config->esdf_local_update_box,
                   &config->esdf_local_update_box)) {
    return false;
  }
  if (!LoadVector2(private_node_, "rog_map/raycasting/ray_range",
                   config->raycast_range_min, config->raycast_range_max,
                   &config->raycast_range_min,
                   &config->raycast_range_max)) {
    return false;
  }
  return true;
}

bool RogMapRos1Convert::Init() {
  rog_map::Config config;
  if (!LoadCoreConfig(&config)) {
    return false;
  }

  private_node_.param("rog_map/ros_callback/enable",
                      ros_callback_enabled_, ros_callback_enabled_);
  private_node_.param("rog_map/ros_callback/mode", mode_, mode_);
  private_node_.param("rog_map/ros_callback/cloud_topic", cloud_topic_,
                      cloud_topic_);
  private_node_.param("rog_map/ros_callback/odom_topic", odometry_topic_,
                      odometry_topic_);
  private_node_.param("rog_map/ros_callback/odom_timeout",
                      odometry_timeout_, odometry_timeout_);
  private_node_.param("rog_map/ros_callback/update_frequency",
                      update_frequency_, update_frequency_);
  private_node_.param("rog_map/ros_callback/cloud_z_scale", cloud_z_scale_,
                      cloud_z_scale_);
  private_node_.param("rog_map/ros_callback/invert_odometry_z",
                      invert_odometry_z_, invert_odometry_z_);
  private_node_.param("rog_map/ros_callback/broadcast_tf", broadcast_tf_,
                      broadcast_tf_);
  private_node_.param("rog_map/ros_callback/map_frame", map_frame_,
                      map_frame_);
  private_node_.param("rog_map/ros_callback/child_frame", child_frame_,
                      child_frame_);
  private_node_.param("rog_map/ros_callback/point_cloud_topic",
                      point_cloud_topic_, point_cloud_topic_);
  private_node_.param("rog_map/ros_callback/odometry_queue",
                      odometry_queue_size_, odometry_queue_size_);
  private_node_.param("rog_map/ros_callback/cloud_queue",
                      cloud_queue_size_, cloud_queue_size_);
  private_node_.param("rog_map/ros_callback/output_queue",
                      output_queue_size_, output_queue_size_);

  private_node_.param("rog_map/visualization/enable",
                      visualization_enabled_, visualization_enabled_);
  private_node_.param("rog_map/visualization/use_dynamic_reconfigure",
                      use_dynamic_reconfigure_, use_dynamic_reconfigure_);
  private_node_.param("rog_map/visualization/time_rate",
                      visualization_frequency_, visualization_frequency_);
  private_node_.param("rog_map/visualization/frame_rate",
                      visualization_frame_rate_, visualization_frame_rate_);
  private_node_.param("rog_map/visualization/frame_id",
                      visualization_frame_, visualization_frame_);
  private_node_.param("rog_map/visualization/pub_unknown_map_en",
                      publish_unknown_map_, publish_unknown_map_);
  private_node_.param("rog_map/visualization/occupied_topic",
                      occupied_topic_, occupied_topic_);
  private_node_.param("rog_map/visualization/unknown_topic", unknown_topic_,
                      unknown_topic_);
  private_node_.param("rog_map/visualization/inflated_occupied_topic",
                      inflated_occupied_topic_, inflated_occupied_topic_);
  private_node_.param("rog_map/visualization/inflated_unknown_topic",
                      inflated_unknown_topic_, inflated_unknown_topic_);
  private_node_.param("rog_map/visualization/frontier_topic",
                      frontier_topic_, frontier_topic_);
  private_node_.param("rog_map/visualization/esdf_topic", esdf_topic_,
                      esdf_topic_);
  private_node_.param("rog_map/visualization/negative_esdf_topic",
                      negative_esdf_topic_, negative_esdf_topic_);
  private_node_.param("rog_map/visualization/occupied_esdf_topic",
                      occupied_esdf_topic_, occupied_esdf_topic_);
  private_node_.param("rog_map/visualization/bounds_topic", bounds_topic_,
                      bounds_topic_);
  if (!LoadVector3(private_node_, "rog_map/visualization/range",
                   visualization_request_.range,
                   &visualization_request_.range)) {
    return false;
  }
  config.visualization_range = visualization_request_.range;

  if (mode_ != 2) {
    ROS_ERROR_STREAM(
        "rog_map ROS1 adapter supports standard odometry/cloud mode 2; "
        "configured mode is " << mode_);
    return false;
  }
  if (ros_callback_enabled_ && update_frequency_ <= 0.0) {
    ROS_ERROR("rog_map update_frequency must be positive");
    return false;
  }
  if (odometry_queue_size_ <= 0 || cloud_queue_size_ <= 0 ||
      output_queue_size_ <= 0) {
    ROS_ERROR("rog_map interface queues must be positive");
    return false;
  }
  if (odometry_timeout_ <= 0.0) {
    ROS_ERROR("rog_map odom_timeout must be positive");
    return false;
  }
  if ((visualization_request_.range.array() <= 0.0).any()) {
    ROS_ERROR("rog_map visualization range must contain 3 positives");
    return false;
  }

  try {
    config.Finalize();
    core_.reset(new rog_map::ROGMap(config));
  } catch (const std::exception& error) {
    ROS_ERROR_STREAM("rog_map configuration failed: " << error.what());
    return false;
  }

  point_cloud_publisher_ = private_node_.advertise<sensor_msgs::PointCloud2>(
      point_cloud_topic_, output_queue_size_);
  occupied_publisher_ = private_node_.advertise<sensor_msgs::PointCloud2>(
      occupied_topic_, output_queue_size_);
  unknown_publisher_ = private_node_.advertise<sensor_msgs::PointCloud2>(
      unknown_topic_, output_queue_size_);
  inflated_occupied_publisher_ =
      private_node_.advertise<sensor_msgs::PointCloud2>(
          inflated_occupied_topic_, output_queue_size_);
  inflated_unknown_publisher_ =
      private_node_.advertise<sensor_msgs::PointCloud2>(
          inflated_unknown_topic_, output_queue_size_);
  frontier_publisher_ = private_node_.advertise<sensor_msgs::PointCloud2>(
      frontier_topic_, output_queue_size_);
  esdf_publisher_ = private_node_.advertise<sensor_msgs::PointCloud2>(
      esdf_topic_, output_queue_size_);
  negative_esdf_publisher_ =
      private_node_.advertise<sensor_msgs::PointCloud2>(
          negative_esdf_topic_, output_queue_size_);
  occupied_esdf_publisher_ =
      private_node_.advertise<sensor_msgs::PointCloud2>(
          occupied_esdf_topic_, output_queue_size_);
  bounds_publisher_ =
      private_node_.advertise<visualization_msgs::MarkerArray>(
          bounds_topic_, output_queue_size_);

  if (ros_callback_enabled_) {
    odometry_subscriber_ = node_.subscribe<nav_msgs::Odometry>(
        odometry_topic_, odometry_queue_size_,
        &RogMapRos1Convert::OdometryCallback, this);
    cloud_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2>(
        cloud_topic_, cloud_queue_size_,
        &RogMapRos1Convert::CloudCallback, this);
    update_timer_ = node_.createTimer(
        ::ros::Duration(1.0 / update_frequency_),
        &RogMapRos1Convert::UpdateCallback, this);
  }

  if (visualization_enabled_) {
    const double frequency = visualization_frequency_ > 0.0
                                 ? visualization_frequency_
                                 : static_cast<double>(
                                       visualization_frame_rate_);
    if (frequency <= 0.0) {
      ROS_ERROR(
          "rog_map visualization requires positive time_rate or frame_rate");
      return false;
    }
    visualization_timer_ = node_.createTimer(
        ::ros::Duration(1.0 / frequency),
        &RogMapRos1Convert::VisualizationCallback, this);
  }

  if (use_dynamic_reconfigure_) {
    dynamic_server_.reset(new DynamicServer(private_node_));
    dynamic_callback_ = boost::bind(
        &RogMapRos1Convert::DynamicVisualizationCallback, this,
        boost::placeholders::_1, boost::placeholders::_2);
    dynamic_server_->setCallback(dynamic_callback_);
  }

  ROS_INFO_STREAM("rog_map ROS1 adapter initialized: cloud=" << cloud_topic_
                  << ", odometry=" << odometry_topic_
                  << ", map_frame=" << map_frame_);
  return true;
}

void RogMapRos1Convert::OdometryCallback(
    const nav_msgs::OdometryConstPtr& message) {
  rog_map::Vec3f position(message->pose.pose.position.x,
                          message->pose.pose.position.y,
                          message->pose.pose.position.z);
  if (invert_odometry_z_) {
    position.z() = -position.z();
  }

  const geometry_msgs::Quaternion& source = message->pose.pose.orientation;
  rog_map::Quatf orientation(source.w, source.x, source.y, source.z);
  if (orientation.norm() < 1.0e-9) {
    orientation = rog_map::Quatf::Identity();
  } else {
    orientation.normalize();
  }
  const double receive_time = ::ros::Time::now().toSec();
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    core_->SetRobotState(rog_map::Pose(position, orientation), receive_time);
  }

  if (broadcast_tf_) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = message->header.stamp.isZero()
                                 ? ::ros::Time::now()
                                 : message->header.stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = child_frame_;
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();
    transform_broadcaster_.sendTransform(transform);
  }
}

void RogMapRos1Convert::CloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  rog_map::PointCloud converted;
  pcl::fromROSMsg(*message, converted);
  for (rog_map::PclPoint& point : converted.points) {
    point.z = static_cast<float>(point.z * cloud_z_scale_);
  }
  std::lock_guard<std::mutex> lock(core_mutex_);
  pending_cloud_ = std::move(converted);
  pending_cloud_stamp_ = message->header.stamp.isZero()
                             ? ::ros::Time::now()
                             : message->header.stamp;
  has_pending_cloud_ = true;
}

void RogMapRos1Convert::UpdateCallback(const ::ros::TimerEvent&) {
  rog_map::PointCloud cloud;
  ::ros::Time stamp;
  bool updated = false;
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    if (!has_pending_cloud_) {
      return;
    }
    const rog_map::RobotState state = core_->getRobotState();
    if (!state.rcv) {
      ROS_WARN_THROTTLE(2.0, "rog_map is waiting for odometry");
      return;
    }
    if (::ros::Time::now().toSec() - state.rcv_time > odometry_timeout_) {
      ROS_WARN_THROTTLE(2.0, "rog_map odometry timed out");
      return;
    }
    cloud.swap(pending_cloud_);
    stamp = pending_cloud_stamp_;
    has_pending_cloud_ = false;
    updated = core_->UpdateFromLatestPose(cloud);
  }
  if (updated && point_cloud_publisher_.getNumSubscribers() > 0U) {
    PublishCloud(cloud, map_frame_, stamp, point_cloud_publisher_);
  }
}

void RogMapRos1Convert::VisualizationCallback(const ::ros::TimerEvent&) {
  rog_map::VisualizationRequest request;
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    request = visualization_request_;
  }
  request.include_occupied =
      occupied_publisher_.getNumSubscribers() > 0U;
  request.include_unknown = publish_unknown_map_ &&
      unknown_publisher_.getNumSubscribers() > 0U;
  request.include_inflated_occupied =
      inflated_occupied_publisher_.getNumSubscribers() > 0U;
  request.include_inflated_unknown = publish_unknown_map_ &&
      inflated_unknown_publisher_.getNumSubscribers() > 0U;
  request.include_frontier = frontier_publisher_.getNumSubscribers() > 0U;
  request.include_esdf = esdf_publisher_.getNumSubscribers() > 0U ||
      negative_esdf_publisher_.getNumSubscribers() > 0U ||
      occupied_esdf_publisher_.getNumSubscribers() > 0U;

  rog_map::VisualizationOutput output;
  {
    std::lock_guard<std::mutex> lock(core_mutex_);
    output = core_->BuildVisualization(request);
  }
  if (!output.ready) {
    return;
  }

  const ::ros::Time stamp = ::ros::Time::now();
  if (request.include_occupied) {
    PublishCloud(output.occupied, visualization_frame_, stamp,
                 occupied_publisher_);
  }
  if (request.include_unknown) {
    PublishCloud(output.unknown, visualization_frame_, stamp,
                 unknown_publisher_);
  }
  if (request.include_inflated_occupied) {
    PublishCloud(output.inflated_occupied, visualization_frame_, stamp,
                 inflated_occupied_publisher_);
  }
  if (request.include_inflated_unknown) {
    PublishCloud(output.inflated_unknown, visualization_frame_, stamp,
                 inflated_unknown_publisher_);
  }
  if (request.include_frontier) {
    PublishCloud(output.frontier, visualization_frame_, stamp,
                 frontier_publisher_);
  }
  if (request.include_esdf) {
    if (esdf_publisher_.getNumSubscribers() > 0U) {
      PublishCloud(output.positive_esdf, visualization_frame_, stamp,
                   esdf_publisher_);
    }
    if (negative_esdf_publisher_.getNumSubscribers() > 0U) {
      PublishCloud(output.negative_esdf, visualization_frame_, stamp,
                   negative_esdf_publisher_);
    }
    if (occupied_esdf_publisher_.getNumSubscribers() > 0U) {
      PublishCloud(output.occupied_esdf, visualization_frame_, stamp,
                   occupied_esdf_publisher_);
    }
  }
  if (bounds_publisher_.getNumSubscribers() > 0U) {
    bounds_publisher_.publish(MakeBoundsMarkers(output, stamp));
  }
}

void RogMapRos1Convert::DynamicVisualizationCallback(
    rog_map_ros1::VizConfig& config, uint32_t) {
  if (config.x_lower_bound >= config.x_upper_bound ||
      config.y_lower_bound >= config.y_upper_bound ||
      config.z_lower_bound >= config.z_upper_bound) {
    ROS_WARN("rog_map ignored invalid dynamic visualization bounds");
    return;
  }
  std::lock_guard<std::mutex> lock(core_mutex_);
  visualization_request_.use_explicit_bounds = true;
  visualization_request_.bounds_relative_to_robot = config.use_body_center;
  visualization_request_.minimum = rog_map::Vec3f(
      config.x_lower_bound, config.y_lower_bound, config.z_lower_bound);
  visualization_request_.maximum = rog_map::Vec3f(
      config.x_upper_bound, config.y_upper_bound, config.z_upper_bound);
}

visualization_msgs::MarkerArray RogMapRos1Convert::MakeBoundsMarkers(
    const rog_map::VisualizationOutput& output,
    const ::ros::Time& stamp) const {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker clear;
  clear.header.frame_id = visualization_frame_;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(clear);
  markers.markers.push_back(MakeBoundsMarker(
      output.visualization_bounds, visualization_frame_, stamp,
      "Visualization Range", 0, MakeColor(0.0F, 0.8F, 1.0F)));
  markers.markers.push_back(MakeBoundsMarker(
      output.local_map_bounds, visualization_frame_, stamp,
      "Local Map Range", 1, MakeColor(0.2F, 1.0F, 0.2F)));
  markers.markers.push_back(MakeBoundsMarker(
      output.update_bounds, visualization_frame_, stamp,
      "Updating Range", 2, MakeColor(1.0F, 0.55F, 0.0F)));

  visualization_msgs::Marker origin;
  origin.header.frame_id = visualization_frame_;
  origin.header.stamp = stamp;
  origin.ns = "Local Map Origin";
  origin.id = 3;
  origin.type = visualization_msgs::Marker::SPHERE;
  origin.action = visualization_msgs::Marker::ADD;
  origin.pose.position.x = output.local_map_origin.x();
  origin.pose.position.y = output.local_map_origin.y();
  origin.pose.position.z = output.local_map_origin.z();
  origin.pose.orientation.w = 1.0;
  origin.scale.x = 0.2;
  origin.scale.y = 0.2;
  origin.scale.z = 0.2;
  origin.color = MakeColor(1.0F, 0.2F, 0.5F);
  markers.markers.push_back(origin);
  return markers;
}

}  // namespace ros1
}  // namespace perception
}  // namespace jojo
