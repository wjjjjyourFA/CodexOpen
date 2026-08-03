#include "tools/data_loader/ros2_convert.h"

namespace jojo {
namespace tools {

Ros2Convert::Ros2Convert(std::shared_ptr<rclcpp::Node> nh) { node = nh; }

Ros2Convert::~Ros2Convert() {}

bool Ros2Convert::Init(std::shared_ptr<jojo::tools::RuntimeConfig> rparam,
                       std::shared_ptr<jojo::tools::InterfaceConfig> iparam) {
  rparam_ = rparam;
  iparam_ = iparam;

  data_loader = std::make_shared<DataLoader>();
  // data_loader = std::make_shared<DataLoaderRealtime>();
  data_loader->Init(rparam_, iparam_);
  data_loader->Start();

  dc_camera.resize(iparam_->b_camera);
  dc_infra.resize(iparam_->b_infra);
  dc_star.resize(iparam_->b_star);
  dc_radar4d.resize(iparam_->b_radar4d);

  this->InitRos2();

  return true;
}

void Ros2Convert::InitRos2() {
  if (iparam_->b_lidar) {
    dc_lidar.pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        iparam_->topic_lidar_pub, 1);

    std::string name = "lidar";
    dc_lidar.set_name(name);
    std::string ts_file = "timestamp/" + name + "_timestamp";
    if (!data_loader->LoadTimeStamp(data_loader->prefix, ts_file, dc_lidar)) {
      if (data_loader->ExtractTimestamp(data_loader->path_lidar, dc_lidar)) {
        data_loader->SaveTimeStamp(data_loader->prefix, ts_file, dc_lidar);
      } else {
        std::cerr << name << " timestamp load failed" << std::endl;
      }
    }
    dc_lidar.init_ts(rparam_->start_time);
    // update the start time
    rparam_->start_time = dc_lidar.cur_time;
  }

  if (iparam_->b_camera) {
    it = std::make_shared<image_transport::ImageTransport>(node);
    for (int i = 0; i < iparam_->b_camera; i++) {
      dc_camera.at(i).pub = it->advertise(iparam_->topic_camera_pub.at(i), 1);

      std::string name;
      // if (iparam_->b_camera == 1) {
      //   name = "camera";
      // } else {
      // 对应 MkdirDataFolderVector 从1开始
      name = "camera_" + std::to_string(i + 1);
      // }
      dc_camera.at(i).set_name(name);
      // timestamp 文件名
      std::string ts_file = "timestamp/" + name + "_timestamp";
      // clang-format off
      if (!data_loader->LoadTimeStamp(data_loader->prefix, ts_file, dc_camera.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_camera.at(i), dc_camera.at(i))) {
          data_loader->SaveTimeStamp(data_loader->prefix, ts_file, dc_camera.at(i));
        } else {
          std::cerr << name << " timestamp load failed" << std::endl;
        }
      }
      // clang-format on
      // 对于 同频率的数据 可以直接递推
      dc_camera.at(i).align_ts(rparam_->start_time);
      // 不同频率的数据 需要在运行时找到匹配的时间戳
    }
  }

  if (iparam_->b_infra) {
    for (int i = 0; i < iparam_->b_infra; i++) {
      dc_infra.at(i).pub = it->advertise(iparam_->topic_infra_pub.at(i), 1);

      std::string name;
      // if (iparam_->b_infra == 1) {
      //   name = "infra";
      // } else {
      name = "infra_" + std::to_string(i + 1);
      // }
      dc_infra.at(i).set_name(name);
      std::string ts_file = "timestamp/" + name + "_timestamp";
      // clang-format off
      if (!data_loader->LoadTimeStamp(data_loader->prefix, ts_file, dc_infra.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_infra.at(i), dc_infra.at(i))) {
          data_loader->SaveTimeStamp(data_loader->prefix, ts_file, dc_infra.at(i));
        } else {
          std::cerr << name << " timestamp load failed" << std::endl;
        }
      }
      // clang-format on
      dc_infra.at(i).align_ts(rparam_->start_time);
    }
  }

  if (iparam_->b_star) {
    // clang-format off
    for (int i = 0; i < iparam_->b_star; i++) {
      dc_star.at(i).pub = it->advertise(iparam_->topic_star_pub.at(i), 1);

      std::string name;
      // if (iparam_->b_star == 1) {
      //   name = "star";
      // } else {
      name = "star_" + std::to_string(i + 1);
      // }
      dc_star.at(i).set_name(name);
      std::string ts_file = "timestamp/" + name + "_timestamp";
      // clang-format off
      if (!data_loader->LoadTimeStamp(data_loader->prefix, ts_file, dc_star.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_star.at(i), dc_star.at(i))) {
          data_loader->SaveTimeStamp(data_loader->prefix, ts_file, dc_star.at(i));
        } else {
          std::cerr << name << " timestamp load failed" << std::endl;
        }
      }
      // clang-format on
      dc_star.at(i).align_ts(rparam_->start_time);
    }
    // clang-format on
  }

  if (iparam_->b_global_pose) {
    dc_global_pose.pub = node->create_publisher<self_state::msg::GlobalPose>(
        iparam_->topic_global_pose_pub, 1);

    dc_global_pose.set_name("global_pose");
    // LoadGlobalPose(data_loader->prefix, dc_global_pose.name);
    LoadGlobalPose(data_loader->path_global_pose);
  }

  if (iparam_->b_local_pose) {
    dc_local_pose.pub = node->create_publisher<self_state::msg::LocalPose>(
        iparam_->topic_local_pose_pub, 1);

    dc_local_pose.set_name("local_pose");
    // LoadLocalPose(data_loader->prefix, dc_local_pose.name);
    LoadLocalPose(data_loader->path_local_pose);
  }

  if (iparam_->b_radar) {
    dc_radar.pub = node->create_publisher<sensor_msgs::msg::PointCloud>(
        iparam_->topic_radar_pub, 1);

    std::string name = "radar";
    dc_radar.set_name(name);
    data_loader->LoadTimeStamp(data_loader->prefix,
                               "timestamp/" + name + "_timestamp", dc_radar);
  }

  for (int i = 0; i < iparam_->b_radar4d; i++) {
    auto type =
        SensorRegistry::Instance().GetRadar4dType(rparam_->radar4d_type);
    std::string& topic = iparam_->topic_radar4d_pub.at(i);

    if (type == Radar4dType::ARS548) {
      dc_radar4d.at(i).pub =
          node->create_publisher<ars548_interface::msg::DetectionList>(topic,
                                                                       1);
    } else if (type == Radar4dType::ARBE) {
      dc_radar4d.at(i).pub =
          node->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 1);
    }
    // std::cout << topic << std::endl;

    std::string name;
    // if (iparam_->b_radar4d == 1) {
    //   name = "radar4d";
    // } else {
    name = "radar4d" + std::to_string(i + 1);
    // }
    dc_radar4d.at(i).set_name(name);
    std::string ts_file = "timestamp/" + name + "_timestamp";
    // clang-format off
    if (!data_loader->LoadTimeStamp(data_loader->prefix, ts_file, dc_radar4d.at(i))) {
      if (data_loader->ExtractTimestamp(data_loader->path_radar4d.at(i), dc_radar4d.at(i))) {
        data_loader->SaveTimeStamp(data_loader->prefix, ts_file, dc_radar4d.at(i));
      } else {
        std::cerr << name << " timestamp load failed" << std::endl;
      }
    }
    // clang-format on
  }
}

void Ros2Convert::Run() {
  if (!RunRosTimer()) {
    std::cout << "RunRosTimer Start Error !!!" << std::endl;
  }
}

bool Ros2Convert::LoadGlobalPose(const std::string& path,
                                 const std::string& data_file) {
  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), data_file.c_str());

  return this->LoadGlobalPose(std::string(file));
}

bool Ros2Convert::LoadGlobalPose(const std::string& file) {
  // 这里将外部数据转换为这个接口
  self_state::msg::GlobalPose globalPoseMsg;

  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
    return false;
  }

  FILE* _fp = NULL;
  _fp       = fopen(file.c_str(), "r");
  if (_fp != NULL) {
    while (!feof(_fp)) {
      // clang-format off
      // UGV2025 ROS_MSG
      // /*
      fscanf(_fp, "%lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %lf %lf %lf %lf",
             &globalPoseMsg.local_time,    &globalPoseMsg.utc_time,     &globalPoseMsg.message_num, 
             &globalPoseMsg.gauss_x,       &globalPoseMsg.gauss_y,      &globalPoseMsg.height, 
             &globalPoseMsg.v_east,        &globalPoseMsg.v_north,      &globalPoseMsg.v_up, 
             &globalPoseMsg.azimuth,       &globalPoseMsg.pitch,        &globalPoseMsg.roll,

             &globalPoseMsg.dev_gauss_x,   &globalPoseMsg.dev_gauss_y,  &globalPoseMsg.dev_height, 
             &globalPoseMsg.dev_v_east,    &globalPoseMsg.dev_v_north,  &globalPoseMsg.dev_v_up,
             &globalPoseMsg.dev_azimuth,   &globalPoseMsg.dev_pitch,    &globalPoseMsg.dev_roll,

             &globalPoseMsg.longitude,     &globalPoseMsg.latitude,

             &globalPoseMsg.ins_status,    &globalPoseMsg.pos_type,
             &globalPoseMsg.reserved[0],   &globalPoseMsg.reserved[1],
             &globalPoseMsg.reserved[2],   &globalPoseMsg.reserved[3]);
      // */
      // clang-format on

      dc_global_pose.insert(uint64_t(globalPoseMsg.local_time), &globalPoseMsg);
    }
    fclose(_fp);
    return true;
  }

  return false;
}

bool Ros2Convert::LoadLocalPose(const std::string& path,
                                const std::string& data_file) {
  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), data_file.c_str());

  return this->LoadLocalPose(std::string(file));
}

bool Ros2Convert::LoadLocalPose(const std::string& file) {
  self_state::msg::LocalPose localPoseMsg;

  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
    return false;
  }

  FILE* _fp = NULL;
  _fp       = fopen(file.c_str(), "r");
  if (_fp != NULL) {
    while (!feof(_fp)) {
      // UGV2023 ROS_MSG
      // clang-format off
      // /*
      int temp1, temp2, temp3;  // 用来跳过 0, 1, -1
      fscanf(_fp,
             "%lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %lf %lf %lf %lf\n",
             &localPoseMsg.local_time,     &localPoseMsg.utc_time,     &localPoseMsg.message_num,  
             &localPoseMsg.dr_x,           &localPoseMsg.dr_y,         &localPoseMsg.dr_z,         
             &localPoseMsg.dr_heading,     &localPoseMsg.dr_roll,      &localPoseMsg.dr_pitch,
             &localPoseMsg.vehicle_speed,  
             &localPoseMsg.speed_x,        &localPoseMsg.speed_y,      &localPoseMsg.speed_z, 
             &temp1, &temp2, &temp3, 
             &localPoseMsg.driving_direction, 
             &localPoseMsg.reserved[0],    &localPoseMsg.reserved[1], 
             &localPoseMsg.reserved[2],    &localPoseMsg.reserved[3]);
      // */
      // clang-format on
      dc_local_pose.insert(uint64_t(localPoseMsg.local_time), &localPoseMsg);
    }
    fclose(_fp);
    return true;
  }

  return false;
}

// ######## 多线程 ########
bool Ros2Convert::RunRosTimer() {
  // 设置每个传感器的定时器，假设频率分别为 10Hz、20Hz 和 30Hz
  if (iparam_->b_global_pose) {  // raw is 50
    auto period = std::chrono::duration<double>(1.0 / 10.0);

    dc_global_pose.timer = node->create_wall_timer(
        period, [this]() { this->Ros2PublishBase(&dc_global_pose); });
  }

  if (iparam_->b_local_pose) {  // raw is 50
    auto period = std::chrono::duration<double>(1.0 / 10.0);

    dc_local_pose.timer = node->create_wall_timer(
        period, [this]() { this->Ros2PublishBase(&dc_local_pose); });
  }

  if (iparam_->b_lidar) {  // raw is 10
    auto period = std::chrono::duration<double>(1.0 / 10.0);

    dc_lidar.timer = node->create_wall_timer(
        period, std::bind(&Ros2Convert::PublishLidar, this));
  }

  for (int i = 0; i < iparam_->b_camera; i++) {  // raw is 10
    auto period = std::chrono::duration<double>(1.0 / 10.0);

    int index = i;

    dc_camera.at(i).timer = node->create_wall_timer(
        period, [this, index] { PublishImage(index, 1); });
  }

  for (int i = 0; i < iparam_->b_infra; i++) {  // raw is 10
    auto period = std::chrono::duration<double>(1.0 / 10.0);

    int index = i;

    dc_infra.at(i).timer = node->create_wall_timer(
        period, [this, index] { PublishImage(index, 2); });
  }

  for (int i = 0; i < iparam_->b_star; i++) {  // raw is 10
    auto period = std::chrono::duration<double>(1.0 / 10.0);

    int index = i;

    dc_star.at(i).timer = node->create_wall_timer(
        period, [this, index] { PublishImage(index, 3); });
  }

  if (iparam_->b_radar) {  // raw is 20
    auto period = std::chrono::duration<double>(1.0 / 10.0);

    dc_radar.timer = node->create_wall_timer(
        period, std::bind(&Ros2Convert::PublishRadar, this));
  }

  for (int i = 0; i < iparam_->b_radar4d; i++) {  // raw is 20
    auto period = std::chrono::duration<double>(1.0 / 10.0);

    // 必须复制到局部变量，避免引用悬挂
    int index = i;

    dc_radar4d.at(i).timer = node->create_wall_timer(
        period, [this, index] { PublishRadar4D(index); });
  }

  std::cout << "Ros2Convert::RunRosTimer() success!" << std::endl;
  return true;
}

bool Ros2Convert::PubLidarBase(
    DataContainerRos2<uint64_t>& data_c,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ptr) {
  auto tmp = &data_c;

  if (tmp->is_end()) {
    tmp->stop();
    return false;
  }

  char file[300];
  if (!rparam_->use_bin_or_pcd) {
    // clang-format off
    sprintf(file, "%s/%.13ld.bin", data_loader->path_lidar.c_str(), tmp->cur_time);
    if (!common::FileExists(file)) {
      std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
      return false;
    }
    // clang-format on

    // 读取 转为 .pcl
    auto& cloud     = cur_cloud_ptr;
    cloud->is_dense = false;

    int p_num = 0;
    int tmp_point[4];
    FILE* fp = fopen(file, "rb");
    if (!fp) {
      std::cerr << "Failed to open file: " << file << std::endl;
      return false;
    }
    while (fread(tmp_point, sizeof(int), 4, fp) == 4) {
      pcl::PointXYZI pt;
      pt.x         = tmp_point[0] / 100.0;  // cm => m
      pt.y         = tmp_point[1] / 100.0;
      pt.z         = tmp_point[2] / 100.0;
      pt.intensity = tmp_point[3];

      cloud->points.emplace_back(pt);
    }
    fclose(fp);

    cloud->width  = cloud->points.size();
    cloud->height = 1;
  } else {
    sprintf(file, "%s/%.13ld.pcd", data_loader->path_lidar.c_str(),
            tmp->cur_time);

    pcl::io::loadPCDFile(std::string(file), *cur_cloud_ptr);
    // warning
    // pcl::PointCloud<pcl::PointXYZI> cloud;
    // pcl::fromPCLPointCloud2(*cur_cloud_ptr, cloud);
    // pcl::transformPointCloud(cloud, cloud, transform_mat);
    // pcl::toPCLPointCloud2(cloud, *cur_cloud_ptr);
    // toc(t);
  }

  /* way 1
  sensor_msgs::msg::PointCloud2 lidar_msg;
  pcl::toROSMsg(*cur_cloud_ptr, lidar_msg);
  // pcl_conversions::fromPCL(*cur_cloud_ptr, lidar_msg);
  lidar_msg.header.frame_id = tmp->name;
  lidar_msg.header.stamp    = rclcpp::Time(tmp->cur_time / 1000.0);
  */

  // ROS2 推荐写法
  sensor_msgs::msg::PointCloud2::Ptr lidar_msg =
      std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cur_cloud_ptr, *lidar_msg);
  lidar_msg->header.frame_id = tmp->name;
  lidar_msg->header.stamp    = rclcpp::Time(tmp->cur_time / 1000.0);

  // 动态转换后手动 publish
  static auto typed_pub = std::dynamic_pointer_cast<
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(tmp->pub);
  if (typed_pub) typed_pub->publish(*lidar_msg);
  // std::cout << tmp->cur_time << " writing LidarData!" << std::endl;
  tmp->next();

  return true;
}

// Lidar 数据读取和发布
void Ros2Convert::PublishLidar() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  this->PubLidarBase(dc_lidar, cloud_ptr);
}

// /*  ESR 20241008
void Ros2Convert::PublishRadar() {
  auto type = SensorRegistry::Instance().GetRadarType(rparam_->radar_type);
  auto tmp  = &dc_radar;

  if (tmp->is_end()) {
    return;
  }

  if (type == RadarType::ESR) {
    sensor::msg::EsrRadarInfo radar_msg;

    char file[300];
    if (!rparam_->use_txt_or_pcd) {
      sprintf(file, "%s/%.13ld.txt", data_loader->path_radar.c_str(),
              tmp->cur_time);

      FILE* fp = fopen(file, "r");
      int tmp_id;
      float tmp_x, tmp_y;
      float tmp_range, tmp_angle;
      float tmp_range_rate, tmp_v;
      while (fscanf(fp, "%d %f %f %f %f %f %f", &tmp_id, &tmp_x, &tmp_y,
                    &tmp_range, &tmp_angle, &tmp_range_rate, &tmp_v) == 7) {
        // 读取的数据可以进行进一步的处理
        // printf("x: %f, y: %f, z: %f, v: %f, stdv: %f\n", tmp_x, tmp_y, tmp_z, tmp_v, tmp_stdv);

        // 如果需要存入你的雷达点云数据结构中
        sensor::msg::EsrRadarObject point;
        point.target_i_d     = tmp_id;
        point.front_distance = tmp_x;
        point.left_distance  = tmp_y;
        point.range          = tmp_range;
        point.angle          = tmp_angle;
        point.range_rate     = tmp_range_rate;
        point.speed          = tmp_v;

        radar_msg.object_data.push_back(point);
      }
      fclose(fp);
    }
    radar_msg.header.frame_id = tmp->name;
    radar_msg.header.stamp    = rclcpp::Time(tmp->cur_time / 1000.0);

    // 动态转换后手动 publish
    static auto typed_pub =
        std::dynamic_pointer_cast<rclcpp::Publisher<sensor::msg::EsrRadarInfo>>(
            tmp->pub);
    if (typed_pub) typed_pub->publish(radar_msg);
  }

  tmp->next();
}
// */

bool Ros2Convert::PubRadar4DBase(DataContainerRos2<uint64_t>& data_c, int id) {
  auto type = SensorRegistry::Instance().GetRadar4dType(rparam_->radar4d_type);
  auto tmp  = &data_c;

  if (tmp->is_end()) {
    tmp->stop();
    return false;
  }

  if (type == Radar4dType::ARS548) {
    ars548_interface::msg::DetectionList radar_msg;

    char file[300];
    if (!rparam_->use_txt_or_pcd) {
      sprintf(file, "%s/%.13ld.txt", data_loader->path_radar4d.at(id).c_str(),
              tmp->cur_time);

      // std::cout << "radar4d : " << file << std::endl;
      FILE* fp = fopen(file, "r");
      float tmp_x, tmp_y, tmp_z, tmp_v, tmp_stdv;
      while (fscanf(fp, "%f %f %f %f %f", &tmp_x, &tmp_y, &tmp_z, &tmp_v,
                    &tmp_stdv) == 5) {
        // 读取的数据可以进行进一步的处理
        // printf("x: %f, y: %f, z: %f, v: %f, stdv: %f\n", tmp_x, tmp_y, tmp_z, tmp_v, tmp_stdv);

        // 如果需要存入你的雷达点云数据结构中
        ars548_interface::msg::Detection point;
        // 假设需要恢复到原单位
        point.f_x              = tmp_x / 100.0;
        point.f_y              = tmp_y / 100.0;
        point.f_z              = tmp_z / 100.0;
        point.f_range_rate     = tmp_v / 100.0;
        point.f_range_rate_std = tmp_stdv / 100.0;
        point.header.stamp     = rclcpp::Time(tmp->cur_time / 1000.0);
        // std::cout << "point.header.stamp : " << point.header.stamp << std::endl;

        // 假设 Radar4D_Cloud 是你的点云容器
        radar_msg.detections.push_back(point);
      }
      fclose(fp);
    }

    static auto typed_pub = std::dynamic_pointer_cast<
        rclcpp::Publisher<ars548_interface::msg::DetectionList>>(tmp->pub);
    if (typed_pub) typed_pub->publish(radar_msg);
    // std::cout << tmp->cur_time << " writing Radar4DData!" << std::endl;
  } else if (type == Radar4dType::ARBE) {
    hugin::PointCloud PointCloud;
    // PointCloud.objectData.width = 2400000;
    // PointCloud.objectData.height = 1;
    // PointCloud.objectData.resize(PointCloud.objectData.width * PointCloud.objectData.height);

    char file[300];
    if (!rparam_->use_txt_or_pcd) {
      sprintf(file, "%s/%.13ld.txt", data_loader->path_radar4d.at(id).c_str(),
              tmp->cur_time);

      FILE* fp = fopen(file, "r");
      float tmp_x, tmp_y, tmp_z;
      float tmp_range, tmp_elevation, tmp_azimuth;
      float tmp_rcs, tmp_doppler;
      while (fscanf(fp, "%f %f %f %f %f %f %f %f", &tmp_x, &tmp_y, &tmp_z,
                    &tmp_range, &tmp_elevation, &tmp_azimuth, &tmp_rcs,
                    &tmp_doppler) == 8) {
        // 读取的数据可以进行进一步的处理
        // printf("x: %f, y: %f, z: %f, v: %f, stdv: %f\n", tmp_x, tmp_y, tmp_z, tmp_v, tmp_stdv);

        // 如果需要存入你的雷达点云数据结构中
        hugin::Point point;
        point.x         = tmp_x;
        point.y         = tmp_y;
        point.z         = tmp_z;
        point.range     = tmp_range;
        point.azimuth   = tmp_azimuth;
        point.elevation = tmp_elevation;
        point.doppler   = tmp_doppler;
        point.rcs       = tmp_rcs;
        PointCloud.data.push_back(point);
      }
      fclose(fp);
    }

    sensor_msgs::msg::PointCloud2 radar_msg;
    pcl::toROSMsg(PointCloud.data, radar_msg);
    radar_msg.header.frame_id = tmp->name;
    radar_msg.header.stamp    = rclcpp::Time(tmp->cur_time / 1000.0);

    static auto typed_pub = std::dynamic_pointer_cast<
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(tmp->pub);
    if (typed_pub) typed_pub->publish(radar_msg);
    // std::cout << tmp->cur_time << " writing Radar4DData!" << std::endl;
  } else {
    std::cout << "radar4d_type set error !!! " << std::endl;
  }

  tmp->next();

  return true;
}

void Ros2Convert::PublishRadar4D(int id) {
  this->PubRadar4DBase(dc_radar4d.at(id), id);
}

bool Ros2Convert::PubImageBase(
    DataContainerRos2<sensor_msgs::msg::Image::SharedPtr>& data_c, int id,
    int mode) {
  auto tmp = &data_c;

  if (tmp->is_end()) {
    tmp->stop();
    return false;
  }

  int index = 0;
  char file_image[300];
  switch (mode) {
    case 1:
      index = id;
      if (rparam_->use_jpg_or_png == 0) {
        sprintf(file_image, "%s/%.13ld.jpg",
                data_loader->path_camera.at(id).c_str(), tmp->cur_time);
      } else {
        sprintf(file_image, "%s/%.13ld.png",
                data_loader->path_camera.at(id).c_str(), tmp->cur_time);
      }
      break;

    case 2:
      index = iparam_->b_camera + id;
      if (rparam_->use_jpg_or_png <= 0) {
        sprintf(file_image, "%s/%.13ld.jpg",
                data_loader->path_infra.at(id).c_str(), tmp->cur_time);
      } else {
        sprintf(file_image, "%s/%.13ld.png",
                data_loader->path_infra.at(id).c_str(), tmp->cur_time);
      }
      break;

    case 3:
      index = iparam_->b_camera + iparam_->b_infra + id;
      if (rparam_->use_jpg_or_png <= 0) {
        sprintf(file_image, "%s/%.13ld.jpg",
                data_loader->path_star.at(id).c_str(), tmp->cur_time);
      } else {
        sprintf(file_image, "%s/%.13ld.png",
                data_loader->path_star.at(id).c_str(), tmp->cur_time);
      }
      break;

    default:
      std::cout << "set mode error!" << std::endl;
      break;
  }

  cv::Mat cur_image = cv::imread(std::string(file_image));

  sensor_msgs::msg::Image::SharedPtr image_msg;
  image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cur_image)
                  .toImageMsg();
  // image_msg->header.seq++;
  image_msg->header.frame_id = tmp->name;
  image_msg->header.stamp    = rclcpp::Time(tmp->cur_time / 1000.0);

  // way 1 直接发布
  tmp->pub.publish(image_msg);

  // way 2 更新队列数据后，手动发布
  // tmp->update(tmp->cur_time, &image_msg);
  // tmp->pub.publish(tmp->cur_data);
  // tmp->publish();

  // std::cout << tmp->cur_time << " writing ImageData!" << std::endl;
  tmp->next();

  return true;
}

void Ros2Convert::PublishImage(int id, int mode) {
  // std::cout << "Ros2Convert::PublishImage()" << std::endl;

  switch (mode) {
    case 1:
      this->PubImageBase(dc_camera.at(id), id, mode);
      break;
    case 2:
      this->PubImageBase(dc_infra.at(id), id, mode);
      break;
    case 3:
      this->PubImageBase(dc_star.at(id), id, mode);
      break;
    default:
      std::cout << "camera_type set error !!! " << std::endl;
      break;
  }
}

void Ros2Convert::Ros2PublishBase(DataContainerRos2Base* tmp) {
  if (tmp->is_end()) {
    tmp->stop();
    return;
  }

  // std::cout << tmp->name << " writing data!" << std::endl;

  // 封装在泛型接口中，根据类型调用不同的发布函数
  // tmp->pub.publish(tmp->cur_data);
  tmp->publish();
  tmp->next();
}
// ######## 多线程 End ########

}  // namespace tools
}  // namespace jojo
