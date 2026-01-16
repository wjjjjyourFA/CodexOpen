#include "info_convert_node.h"

void InfoConverNode::ObjectReceive(const ars548_process::ObjectList& msg) {
  visualization_msgs::Marker my_marker;
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.clear();

  uint size = msg.objects.size();
  if (size > 0) {
    my_marker.header.frame_id = param_->frame_id;
    my_marker.ns              = "object_shapes";
    my_marker.type            = visualization_msgs::Marker::CUBE;
    my_marker.action          = visualization_msgs::Marker::ADD;

    my_marker.pose.orientation.x = 0.0;
    my_marker.pose.orientation.y = 0.0;

    my_marker.color.r = 0.0f;
    my_marker.color.g = 1.0f;
    my_marker.color.b = 0.0f;
    my_marker.color.a = 1.0;

    my_marker.lifetime = ros::Duration(0.5);

    for (uint i = 0; i < size; i++) {
      my_marker.header.stamp = msg.objects[i].header.stamp;

      my_marker.id = msg.objects[i].u_id;

      my_marker.pose.position.x = msg.objects[i].u_position_x;
      my_marker.pose.position.y = msg.objects[i].u_position_y;
      my_marker.pose.position.z = msg.objects[i].u_position_z;

      my_marker.pose.orientation.z =
          sin(msg.objects[i].u_position_orientation / 2);
      my_marker.pose.orientation.w =
          cos(msg.objects[i].u_position_orientation / 2);

      if ((msg.objects[i].u_shape_length_edge_mean > 0.2) ||
          (msg.objects[i].u_shape_width_edge_mean > 0.2)) {
        my_marker.scale.x = msg.objects[i].u_shape_length_edge_mean;
        my_marker.scale.y = msg.objects[i].u_shape_width_edge_mean;
        my_marker.scale.z = (msg.objects[i].u_shape_length_edge_mean +
                             msg.objects[i].u_shape_width_edge_mean) /
                            2;
      } else {
        my_marker.scale.x = 0.2;
        my_marker.scale.y = 0.2;
        my_marker.scale.z = 0.2;
      }

      marker_array.markers.push_back(my_marker);
    }

    objects_marker_pub.publish(marker_array);
  }
}

void InfoConverNode::CreatPointCloud2() {
  // 初始化不变的部分
  cloud_.header.frame_id = param_->frame_id;
  cloud_.height          = 1;
  cloud_.width           = 0;
  cloud_.is_bigendian    = false;
  cloud_.is_dense        = true;

  cloud_.fields.resize(8);
  // 使用成员变量进行赋值
  cloud_.fields[0].name     = "x";
  cloud_.fields[0].offset   = 0;
  cloud_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_.fields[0].count    = 1;

  cloud_.fields[1].name     = "y";
  cloud_.fields[1].offset   = 4;
  cloud_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_.fields[1].count    = 1;

  cloud_.fields[2].name     = "z";
  cloud_.fields[2].offset   = 8;
  cloud_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_.fields[2].count    = 1;

  cloud_.fields[3].name     = "range";
  cloud_.fields[3].offset   = 12;
  cloud_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_.fields[3].count    = 1;

  cloud_.fields[4].name     = "elevation";
  cloud_.fields[4].offset   = 16;
  cloud_.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_.fields[4].count    = 1;

  cloud_.fields[5].name     = "azimuth";
  cloud_.fields[5].offset   = 20;
  cloud_.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_.fields[5].count    = 1;

  cloud_.fields[6].name     = "power";
  cloud_.fields[6].offset   = 24;
  cloud_.fields[6].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_.fields[6].count    = 1;

  cloud_.fields[7].name     = "doppler";
  cloud_.fields[7].offset   = 28;
  cloud_.fields[7].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_.fields[7].count    = 1;

  cloud_.point_step = 32;  // 8 * 4 bytes
}

// 更换成 PointCloud2
void InfoConverNode::DetectionReceive(
    const ars548_process::DetectionList& msg) {
  uint size = msg.detections.size();
  if (size > 0) {
    // cloud.header.frame_id = param_->frame_id;
    cloud_.header.stamp = msg.detections[0].header.stamp;
    cloud_.width        = size;
    cloud_.row_step     = cloud_.point_step * size;
    cloud_.data.resize(cloud_.row_step);

    /*
    for (uint i = 0; i < size; i++) {
      memcpy(&cloud_.data[i * 12 + 0], &msg.detections[i].f_x, 4);
      memcpy(&cloud_.data[i * 12 + 4], &msg.detections[i].f_y, 4);
      memcpy(&cloud_.data[i * 12 + 8], &msg.detections[i].f_z, 4);
    }
    */

    PointCloud2Iterator iter_x(cloud_, "x"), iter_y(cloud_, "y"),
        iter_z(cloud_, "z"), iter_r(cloud_, "range"),
        iter_el(cloud_, "elevation"), iter_az(cloud_, "azimuth"),
        iter_power(cloud_, "power"), iter_doppler(cloud_, "doppler");

    for (uint i = 0; i < size; i++) {
      // Assign cartesian coordinates
      *iter_x = static_cast<float>(msg.detections[i].f_x);
      *iter_y = static_cast<float>(msg.detections[i].f_y);
      *iter_z = static_cast<float>(msg.detections[i].f_z);
      // Assign spherical coordinates
      *iter_r       = msg.detections[i].f_range;
      *iter_el      = msg.detections[i].f_elevation_angle;
      *iter_az      = msg.detections[i].f_azimuth_angle;
      *iter_power   = msg.detections[i].s_rcs;
      *iter_doppler = msg.detections[i].f_range_rate;

      // Advance destination iterators
      ++iter_x;
      ++iter_y;
      ++iter_z;

      ++iter_r;
      ++iter_el;
      ++iter_az;
      ++iter_power;
      ++iter_doppler;
    }

    detections_cloud_pub.publish(cloud_);
  }
}

bool InfoConverNode::init(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                          std::shared_ptr<RuntimeConfig> param) {
  nh_         = nh;
  private_nh_ = private_nh;
  param_      = param;

  std::string ns = param_->GetVehicleName();
  std::string topic;
  /*
  if(param_->index == 0){
    topic = "/" + ns + param_->channel_name;
  }else{
    topic = "/" + ns + param_->channel_name + "_" + std::to_string(param_->index);
  }
  */
  topic = "/" + ns + param_->channel_name;
  // std::cout << "topic: " << topic << std::endl;

  CreatPointCloud2();

  // clang-format off
  // ros::Subscriber obj_list_sub = nh.subscribe("/ars548_process/object_list", 10, &objectReceive);
  // ros::Subscriber det_list_sub = nh.subscribe("/ars548_process/detection_list", 10, &detectionReceive);
  // objects_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/ars548_process/object_marker", 10);
  // detections_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("/ars548_process/detection_point_cloud", 10);
  obj_list_sub = nh.subscribe(topic + "/object_list", 10, &InfoConverNode::ObjectReceive, this);
  det_list_sub = nh.subscribe(topic + "/detection_list", 10, &InfoConverNode::DetectionReceive, this);
  objects_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(topic + "/object_marker", 10);
  detections_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(topic + "/detection_point_cloud", 10);
  // clang-format on

  return true;
}

void InfoConverNode::run() {
  ros::Rate r(30);

  while (ros::ok()) {
    ros::spinOnce();

    r.sleep();
  }
  // ros::spin();
}

void SingleChannel(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                   std::shared_ptr<RuntimeConfig> param) {
  InfoConverNode info_conver_node;
  info_conver_node.init(nh, private_nh, param);
  info_conver_node.run();
}

int main(int argc, char** argv) {
  std::string base_path = ros::package::getPath("ars548_process");
  std::string config_file_path_ = base_path + "/../../../../../install/common/vehicle_sensor_config.yaml";

  auto param_manager = std::make_shared<ConfigManager>();
  param_manager->LoadConfig(config_file_path_);
  std::string name =
      param_manager->GetVehicleName() + "_ars548_info_convert_node";

  ros::init(argc, argv, name);
  // ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::vector<SensorConfig>& radar_configs =
      param_manager->vehicle_model_config.radar_configs;

  if (radar_configs.size() < 1) {
    return false;
  }

  // 初始化多个Radar Process
  int num = 1;
  std::vector<std::thread> threads_;
  for (auto config : radar_configs) {
    auto param          = std::make_shared<RuntimeConfig>();
    param->vehicle_name = param_manager->GetVehicleName();
    param->LoadConfig(config.config_file);
    // std::cout << "vehicle_name: " << param->vehicle_name << std::endl;

    // 复制当前轮的 num，确保 lambda 捕获的是正确值
    int current_num = num;
    // 启动 单线程/多线程
    threads_.emplace_back([current_num, &nh, &private_nh, param]() {
      SingleChannel(nh, private_nh, param);
    });

    num++;
  }

  // 4个线程，调整数量合适即可
  ros::MultiThreadedSpinner spinner(num);
  spinner.spin();

  // 主线程等待退出
  ros::waitForShutdown();

  return 0;
}