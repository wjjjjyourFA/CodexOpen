#include "modules/perception/ground_remove/ros1_convert.h"

using namespace jojo::perception;
namespace cfg = jojo::perception::config;

Ros1Convert::Ros1Convert(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  nh_  = nh;
  pnh_ = pnh;

  ground_remove_ = std::make_shared<GroundRemove>();
}

Ros1Convert::~Ros1Convert() {}

void Ros1Convert::LidarCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // 1. 转换点云到PCL格式
  pcl::fromROSMsg(*cloud_msg, *cloud);
  cloud_recv_ = true;

  map_msg.header.stamp = cloud_msg->header.stamp;
}

void Ros1Convert::PoseCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
  // 2. 转换位姿到Eigen::Matrix4f
  // Odometry 中有 pose.pose 字段
  const auto& p = odom_msg->pose.pose;

  Eigen::Quaternionf quat(p.orientation.w, p.orientation.x, p.orientation.y,
                          p.orientation.z);
  // set R
  pose.block<3, 3>(0, 0) = quat.toRotationMatrix();
  // set T
  pose(0, 3) = p.position.x;
  pose(1, 3) = p.position.y;
  pose(2, 3) = p.position.z;

  pose_recv_ = true;
}

bool Ros1Convert::Init(
    std::shared_ptr<jojo::perception::RuntimeConfig> rparam,
    std::shared_ptr<jojo::perception::InterfaceConfig> iparam) {
  rparam_ = rparam;
  iparam_ = iparam;

  // 订阅点云和位姿
  // clang-format on
  cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
      iparam_->lidar_topic, 1,
      std::bind(&Ros1Convert::LidarCallback, this, std::placeholders::_1));
  pose_sub_ = nh_.subscribe<nav_msgs::Odometry>(
      iparam_->pose_topic, 1,
      std::bind(&Ros1Convert::PoseCallback, this, std::placeholders::_1));
  // clang-format on

  // 发布栅格地图
  map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(iparam_->map_topic, 1);

  this->cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  this->pose = Eigen::Matrix4f::Identity();

  auto lidar_params = std::make_shared<cfg::SensorExtrinsics>();
  lidar_params->LoadFromFile(rparam_->gravity_lidar_calib_file_path);
  auto matrix2 = lidar_params->GetMatrixVector();

  // 初始化 GroundRemove
  // clang-format off
  ground_remove_->SetGravityLidarExtrinsicMatrix(matrix2.at(0)->extrinsic_matrix);
  ground_remove_->Init(rparam_);
  // clang-format on

  // 保存地图尺寸参数，用于 publish map
  map_rows_       = rparam_->map_rows;
  map_cols_       = rparam_->map_cols;
  map_resolution_ = rparam_->map_resolution;
  half_rows_      = map_rows_ / 2;
  half_cols_      = map_cols_ / 2;

  // 保存坐标系名称
  map_frame_ = iparam_->map_frame;

  return true;
}

void Ros1Convert::Run() {
  ros::Rate loop_rate(iparam_->rate);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();

    if (!cloud_recv_ || !pose_recv_) {
      continue;
    }

    // TODO：如果输入数据没有更新，后续应该如何处理

    // 3. 执行地面去除
    ground_remove_->Run(this->cloud, this->pose);

    // 5. 构造 OccupancyGrid 消息
    this->UpdateGridMap();

    // 6. 发布地图
    map_pub_.publish(map_msg);
  }
}

void Ros1Convert::UpdateGridMap() {
  static bool first_flag = false;
  if (!first_flag) {
    map_msg.header.frame_id = map_frame_;

    map_msg.info.width      = map_cols_;
    map_msg.info.height     = map_rows_;
    map_msg.info.resolution = map_resolution_;

    // 地图原点设置：使地图中心对应车辆位置（即地图原点在左下角）
    map_msg.info.origin.position.x    = -half_cols_ * map_resolution_;
    map_msg.info.origin.position.y    = -half_rows_ * map_resolution_;
    map_msg.info.origin.position.z    = 0;
    map_msg.info.origin.orientation.w = 1;

    first_flag = true;
  }

  map_msg.data.clear();
  // -1 表示未知区域 ，0 表示空闲区域，100 表示占用区域
  map_msg.data.resize(map_rows_ * map_cols_, -1);

  // 4. 获取结果栅格地图
  auto grid_map = ground_remove_->GetResult();
  if (!grid_map) {
    ROS_WARN("Obstacle grid map is null");
    return;
  }

  // 遍历所有栅格，填充障碍物信息
  for (int r = 0; r < map_rows_; ++r) {
    for (int c = 0; c < map_cols_; ++c) {
      // 获取该栅格的cell指针
      ObstacleCell* cell = grid_map->GetWorldXYFromRC(r, c);

      int ros_index = r * map_cols_ + c;

      if (cell && cell->b_valid) {
        // 如果存在障碍物，设为占用（100）
        map_msg.data[ros_index] = 100;
      } else {
        // 空闲区域（0）或未知（-1），我们设置为0表示空闲
        map_msg.data[ros_index] = 0;
      }
    }
  }
}