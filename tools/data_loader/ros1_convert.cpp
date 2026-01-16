#include "tools/data_loader/ros1_convert.h"

namespace jojo {
namespace tools {
namespace common = apollo::cyber::common;

Ros1Convert::Ros1Convert() {}

Ros1Convert::~Ros1Convert() {}

void Ros1Convert::Init(ros::NodeHandle &nh, ros::NodeHandle &private_nh,
                       std::shared_ptr<RuntimeConfigRealtime> param) {
  node   = nh;
  param_ = param;

  data_loader = std::make_shared<DataLoaderRealtime>();
  data_loader->Init(param_);
  data_loader->Start();

  dc_camera.resize(param_->b_camera);
  dc_infra.resize(param_->b_infra);
  dc_star.resize(param_->b_star);
  dc_radar_4d.resize(param_->b_radar_4d);

  this->InitRos1();
}

void Ros1Convert::InitRos1() {
  if (param_->b_lidar) {
    dc_lidar.pub =
        node.advertise<sensor_msgs::PointCloud2>(param_->topic_lidar_pub, 1);

    std::string name = "lidar";
    dc_lidar.set_name(name);
    data_loader->LoadTimeStamp(data_loader->prefix,
                               "timestamp/" + name + "_timestamp", dc_lidar);
    dc_lidar.init_ts(param_->start_time);
    // update the start time
    param_->start_time = dc_lidar.cur_time;
  }

  if (param_->b_camera || param_->b_infra || param_->b_star) {
    it = std::make_shared<image_transport::ImageTransport>(node);
    for (int i = 0; i < param_->b_camera; i++) {
      dc_camera.at(i).pub = it->advertise(param_->topic_camera_pub.at(i), 1);

      std::string name = "image";
      dc_camera.at(i).set_name(name);
      data_loader->LoadTimeStamp(data_loader->prefix,
                                 "timestamp/" + name + "_timestamp",
                                 dc_camera.at(i));
      // 对于 同频率的数据 可以直接递推
      dc_camera.at(i).align_ts(param_->start_time);
      // 不同频率的数据 需要在运行时找到匹配的时间戳
    }

    for (int i = 0; i < param_->b_infra; i++) {
      dc_infra.at(i).pub = it->advertise(param_->topic_infra_pub.at(i), 1);

      std::string name = "infra";
      dc_infra.at(i).set_name(name);
      data_loader->LoadTimeStamp(data_loader->prefix,
                                 "timestamp/" + name + "_timestamp",
                                 dc_infra.at(i));
      dc_infra.at(i).align_ts(param_->start_time);
    }

    // clang-format off
    for (int i = 0; i < param_->b_star; i++) {
      dc_star.at(i).pub = it->advertise(param_->topic_star_pub.at(i), 1);

      std::string name = "star";
      dc_star.at(i).set_name(name);
      data_loader->LoadTimeStamp(data_loader->prefix, 
                                 "timestamp/" + name + "_timestamp", 
                                 dc_star.at(i));
      dc_star.at(i).align_ts(param_->start_time);
    }
    // clang-format on
  }

  if (param_->b_global_pose) {
    dc_global_pose.pub = node.advertise<self_state::GlobalPose>(
        param_->topic_global_pose_pub, 1);

    dc_global_pose.set_name("global_pose");
    LoadGlobalPose(data_loader->prefix, dc_global_pose.name);
  }

  if (param_->b_local_pose) {
    dc_local_pose.pub =
        node.advertise<self_state::LocalPose>(param_->topic_local_pose_pub, 1);

    dc_local_pose.set_name("local_pose");
    LoadLocalPose(data_loader->prefix, dc_local_pose.name);
  }

  if (param_->b_radar) {
    dc_radar.pub =
        node.advertise<sensor_msgs::PointCloud>(param_->topic_radar_pub, 1);

    std::string name = "radar";
    dc_radar.set_name(name);
    data_loader->LoadTimeStamp(data_loader->prefix,
                               "timestamp/" + name + "_timestamp", dc_radar);
  }

  for (int i = 0; i < param_->b_radar_4d; i++) {
    static int type    = Radar4DTypeParam[param_->radar_4d_type];
    std::string &topic = param_->topic_radar_4d_pub.at(i);

    if (type == 1) {
      dc_radar_4d.at(i).pub =
          node.advertise<ars548_msg::DetectionList>(topic, 1);

    } else if (type == 2) {
      dc_radar_4d.at(i).pub =
          node.advertise<sensor_msgs::PointCloud2>(topic, 1);
    }
    // std::cout << topic << std::endl;

    std::string name = "radar_4d";
    dc_radar_4d.at(i).set_name(name);
    data_loader->LoadTimeStamp(data_loader->prefix,
                               "timestamp/" + name + "_timestamp",
                               dc_radar_4d.at(i));
  }
}

void Ros1Convert::Run() {
  if (!RunRosTimer()) {
    std::cout << "RunRosTimer Start Error !!!" << std::endl;
  }
}

bool Ros1Convert::LoadGlobalPose(const std::string &path,
                                 const std::string &data_file) {
  // 这里将外部数据转换为这个接口
  self_state::GlobalPose globalPoseMsg;

  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), data_file.c_str());
  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
    return false;
  }

  FILE *_fp = NULL;
  _fp       = fopen(file, "r");
  if (_fp != NULL) {
    while (!feof(_fp)) {
      // 刘伯凯 1
      // clang-format off
      /*
      fscanf(_fp, "%lf %u %lf %lf %d %d %d %d %d %d %d %d %d %lf %lf %lf %lf %lf %lf "
                  "%f %f %f %f %f %f %f %f %f "
                  "%d %d "
                  "%d %d %d %d "
                  "%f %f %f %f\n", 
             &tmp_globalpose.pc_time,             &tmp_globalpose.gps_millisecond,
             tmp_globalpose.llhPos,               tmp_globalpose.llhPos+1,
             tmp_globalpose.gaussPos,             tmp_globalpose.gaussPos+1,             &tmp_globalpose.height, 
             &tmp_globalpose.roll,                &tmp_globalpose.pitch,                 &tmp_globalpose.azimuth,
             &tmp_globalpose.northVelocity,       &tmp_globalpose.eastVelocity,          &tmp_globalpose.upVelocity,
             &tmp_globalpose.omega_x,             &tmp_globalpose.omega_y,               &tmp_globalpose.omega_z,
             &tmp_globalpose.acc_x,               &tmp_globalpose.acc_y,                 &tmp_globalpose.acc_z, 
             &tmp_globalpose.variance_x,          &tmp_globalpose.variance_y,            &tmp_globalpose.variance_z,
             &tmp_globalpose.variance_azimuth,    &tmp_globalpose.variance_pitch,        &tmp_globalpose.variance_roll,
             &tmp_globalpose.variance_eastvel,    &tmp_globalpose.variance_northvel,     &tmp_globalpose.variance_upvel,
             &tmp_globalpose.positionStatus,      &tmp_globalpose.number_of_sattelite, 
             tmp_globalpose.reserved_int,         tmp_globalpose.reserved_int+1,         tmp_globalpose.reserved_int+2,         tmp_globalpose.reserved_int+3, 
             tmp_globalpose.reserved_float,       tmp_globalpose.reserved_float+1,       tmp_globalpose.reserved_float+2,       tmp_globalpose.reserved_float+3);
      */
      // clang-format on

      // 霍得磊 newGlobalPose
      // clang-format off
      /*
      fscanf(_fp, "%lf %lf %d %d %d  %lf %lf %lf %lf %lf %lf %lf %lf %lf  %lf %lf %lf %lf %lf %lf %lf %lf %lf  %lf %lf "
                  "%lf %lf %d %d %d  %d %d  %lf %lf %lf "
                  "%lf %lf %d  %lf %lf %lf  %lf %lf %lf "
                  "%lf %lf %lf %lf \n",
             &tmp_globalpose2.local_time,      &tmp_globalpose2.gps_time,        &tmp_globalpose2.message_num, 
             &tmp_globalpose2.ins_status,      &tmp_globalpose2.pos_type,
             &tmp_globalpose2.gaussX,          &tmp_globalpose2.gaussY,          &tmp_globalpose2.height,
             &tmp_globalpose2.vNorth,          &tmp_globalpose2.vEast,           &tmp_globalpose2.vUp, 
             &tmp_globalpose2.roll,            &tmp_globalpose2.pitch,           &tmp_globalpose2.azimuth,

             &tmp_globalpose2.dev_gaussX,      &tmp_globalpose2.dev_gaussY,      &tmp_globalpose2.dev_height, 
             &tmp_globalpose2.dev_vNorth,      &tmp_globalpose2.dev_vEast,       &tmp_globalpose2.dev_vUp,
             &tmp_globalpose2.dev_roll,        &tmp_globalpose2.dev_pitch,       &tmp_globalpose2.dev_azimuth,

             &tmp_globalpose2.latitude,        &tmp_globalpose2.longitude,

             &tmp_globalpose2.gnss_data.local_time,              &tmp_globalpose2.gnss_data.gps_time,             &tmp_globalpose2.gnss_data.message_num,
             &tmp_globalpose2.gnss_data.sol_status,              &tmp_globalpose2.gnss_data.pos_type,
             &tmp_globalpose2.gnss_data.num_satellite_tracked,   &tmp_globalpose2.gnss_data.num_satellite_used,
             &tmp_globalpose2.gnss_data.latitude_gnss,
             &tmp_globalpose2.gnss_data.longitude_gnss,          &tmp_globalpose2.gnss_data.height_gnss,

             &tmp_globalpose2.raw_imu.local_time,                &tmp_globalpose2.raw_imu.gps_time,               &tmp_globalpose2.raw_imu.message_num,
             &tmp_globalpose2.raw_imu.acc_x,                     &tmp_globalpose2.raw_imu.acc_y,                  &tmp_globalpose2.raw_imu.acc_z,
             &tmp_globalpose2.raw_imu.angular_x,                 &tmp_globalpose2.raw_imu.angular_y,              &tmp_globalpose2.raw_imu.angular_z,
             tmp_globalpose2.reserved,         tmp_globalpose2.reserved+1,
             tmp_globalpose2.reserved+2,       tmp_globalpose2.reserved+3);
      */
      // clang-format on

      // clang-format off
      // UGV2023 ROS_MSG
      // /*
      fscanf(_fp, "%lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %lf %lf %lf %lf",
             &globalPoseMsg.local_time,    &globalPoseMsg.UTC_time,    &globalPoseMsg.message_num, 
             &globalPoseMsg.gaussX,        &globalPoseMsg.gaussY,      &globalPoseMsg.height, 
             &globalPoseMsg.vEast,         &globalPoseMsg.vNorth,      &globalPoseMsg.vUp, 
             &globalPoseMsg.azimuth,       &globalPoseMsg.pitch,       &globalPoseMsg.roll,

             &globalPoseMsg.dev_gaussX,    &globalPoseMsg.dev_gaussY,  &globalPoseMsg.dev_height, 
             &globalPoseMsg.dev_vEast,     &globalPoseMsg.dev_vNorth,  &globalPoseMsg.dev_vUp,
             &globalPoseMsg.dev_azimuth,   &globalPoseMsg.dev_pitch,   &globalPoseMsg.dev_roll,

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

bool Ros1Convert::LoadLocalPose(const std::string &path,
                                const std::string &data_file) {
  self_state::LocalPose localPoseMsg;

  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), data_file.c_str());
  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
    return false;
  }

  FILE *_fp = NULL;
  _fp       = fopen(file, "r");
  if (_fp != NULL) {
    while (!feof(_fp)) {
      // UGV2023 ROS_MSG
      // clang-format off
      // /*
      int temp1, temp2, temp3;  // 用来跳过 0, 1, -1
      fscanf(_fp,
             "%lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %lf %lf %lf %lf\n",
             &localPoseMsg.local_time,     &localPoseMsg.UTC_time,     &localPoseMsg.message_num,  
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
bool Ros1Convert::RunRosTimer() {
  // 设置每个传感器的定时器，假设频率分别为 10Hz、20Hz 和 30Hz
  if (param_->b_global_pose) {  // raw is 50
    ros::Duration period(1.0 / 10.0);

    dc_global_pose.timer = node.createTimer(
        period, std::bind(&Ros1Convert::Ros1PublishBase, this,
                          std::placeholders::_1, &dc_global_pose));
  }

  if (param_->b_local_pose) {  // raw is 50
    ros::Duration period(1.0 / 10.0);

    dc_local_pose.timer = node.createTimer(
        period, std::bind(&Ros1Convert::Ros1PublishBase, this,
                          std::placeholders::_1, &dc_local_pose));
  }

  if (param_->b_lidar) {  // raw is 10
    ros::Duration period(1.0 / 10.0);

    dc_lidar.timer = node.createTimer(period, &Ros1Convert::PublishLidar, this);
  }

  for (int i = 0; i < param_->b_camera; i++) {  // raw is 10
    ros::Duration period(1.0 / 10.0);

    int index = i;

    dc_camera.at(i).timer =
        node.createTimer(period, std::bind(&Ros1Convert::PublishImage, this,
                                           std::placeholders::_1, index, 1));
  }

  for (int i = 0; i < param_->b_infra; i++) {  // raw is 10
    ros::Duration period(1.0 / 10.0);

    int index = i;

    dc_infra.at(i).timer = node.createTimer(
        period,
        [this, index](const ros::TimerEvent &e) { PublishImage(e, index, 2); });
  }

  for (int i = 0; i < param_->b_star; i++) {  // raw is 10
    ros::Duration period(1.0 / 10.0);

    int index = i;

    dc_star.at(i).timer = node.createTimer(
        period,
        [this, index](const ros::TimerEvent &e) { PublishImage(e, index, 3); });
  }

  if (param_->b_radar) {  // raw is 20
    ros::Duration period(1.0 / 10.0);

    dc_radar.timer = node.createTimer(period, &Ros1Convert::PublishRadar, this);
  }

  for (int i = 0; i < param_->b_radar_4d; i++) {  // raw is 20
    ros::Duration period(1.0 / 10.0);

    // 必须复制到局部变量，避免引用悬挂
    int index = i;

    dc_radar_4d.at(i).timer =
        node.createTimer(period, std::bind(&Ros1Convert::PublishRadar4D, this,
                                           std::placeholders::_1, index));
  }

  std::cout << "Ros1Convert::RunRosTimer() success!" << std::endl;
  return true;
}

bool Ros1Convert::PubLidarBase(
    DataContainerRos1<uint64_t> &data_c,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ptr) {
  auto tmp = &data_c;

  if (tmp->end()) {
    tmp->stop();
    return false;
  }

  char file[300];
  if (!param_->use_bin_or_pcd) {
    // clang-format off
    sprintf(file, "%s/%.13ld.bin", data_loader->path_lidar.c_str(), tmp->cur_time);
    if (!common::FileExists(file)) {
      std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
      return false;
    }
    // clang-format on

    // 读取 转为 .pcl
    auto &cloud     = cur_cloud_ptr;
    cloud->is_dense = false;

    int p_num = 0;
    int tmp_point[4];
    FILE *fp = fopen(file, "rb");
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
  sensor_msgs::PointCloud2 lidar_msg;
  pcl::toROSMsg(*cur_cloud_ptr, lidar_msg);
  // pcl_conversions::fromPCL(*cur_cloud_ptr, lidar_msg);
  lidar_msg.header.frame_id = tmp->name;
  lidar_msg.header.stamp    = ros::Time(tmp->cur_time / 1000.0);
  */

  // ROS1 推荐写法
  sensor_msgs::PointCloud2::Ptr lidar_msg =
      boost::make_shared<sensor_msgs::PointCloud2>();
  pcl::toROSMsg(*cur_cloud_ptr, *lidar_msg);
  lidar_msg->header.seq++;
  lidar_msg->header.frame_id = tmp->name;
  lidar_msg->header.stamp    = ros::Time(tmp->cur_time / 1000.0);

  tmp->pub.publish(lidar_msg);

  // std::cout << tmp->cur_time << " writing LidarData!" << std::endl;

  tmp->next();

  return true;
}

// Lidar 数据读取和发布
void Ros1Convert::PublishLidar(const ros::TimerEvent &) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  this->PubLidarBase(dc_lidar, cloud_ptr);
}

// /*  ESR 20241008
void Ros1Convert::PublishRadar(const ros::TimerEvent &) {
  static int type = RadarTypeParam[param_->radar_type];
  auto tmp        = &dc_radar;

  if (tmp->end()) {
    return;
  }

  if (type == 1) {
    sensor::ESR_Radar_Info radar_msg;

    char file[300];
    if (!param_->use_txt_or_pcd) {
      sprintf(file, "%s/%.13ld.txt", data_loader->path_radar.c_str(),
              tmp->cur_time);

      FILE *fp = fopen(file, "r");
      int tmp_id;
      float tmp_x, tmp_y;
      float tmp_range, tmp_angle;
      float tmp_range_rate, tmp_v;
      while (fscanf(fp, "%d %f %f %f %f %f %f", &tmp_id, &tmp_x, &tmp_y, &tmp_range,
                    &tmp_angle, &tmp_range_rate, &tmp_v) == 7) {
        // 读取的数据可以进行进一步的处理
        // printf("x: %f, y: %f, z: %f, v: %f, stdv: %f\n", tmp_x, tmp_y, tmp_z, tmp_v, tmp_stdv);

        // 如果需要存入你的雷达点云数据结构中
        sensor::ESR_Radar_Object point;
        point.targetID       = tmp_id;
        point.front_distance = tmp_x;
        point.left_distance  = tmp_y;
        point.range          = tmp_range;
        point.angle          = tmp_angle;
        point.rangeRate      = tmp_range_rate;
        point.Speed          = tmp_v;

        radar_msg.objectData.push_back(point);
      }
      fclose(fp);
    }
    radar_msg.header.seq++;
    radar_msg.header.frame_id = tmp->name;
    radar_msg.header.stamp    = ros::Time(tmp->cur_time / 1000.0);
    tmp->pub.publish(radar_msg);
  }

  tmp->next();
}
// */

bool Ros1Convert::PubRadar4DBase(DataContainerRos1<uint64_t> &data_c, int id) {
  static int type = RadarTypeParam[param_->radar_4d_type];
  auto tmp        = &data_c;

  if (tmp->end()) {
    tmp->stop();
    return false;
  }

  if (type == 1) {
    // ars548_msg::DetectionList radar_msg;
    ars548_msg::DetectionList::Ptr radar_msg =
        boost::make_shared<ars548_msg::DetectionList>();

    char file[300];
    if (!param_->use_txt_or_pcd) {
      sprintf(file, "%s/%.13ld.txt", data_loader->path_radar_4d.at(id).c_str(),
              tmp->cur_time);

      // std::cout << "radar_4d : " << file << std::endl;
      FILE *fp = fopen(file, "r");
      float tmp_x, tmp_y, tmp_z, tmp_v, tmp_stdv;
      while (fscanf(fp, "%f %f %f %f %f", &tmp_x, &tmp_y, &tmp_z, &tmp_v,
                    &tmp_stdv) == 5) {
        // 读取的数据可以进行进一步的处理
        // printf("x: %f, y: %f, z: %f, v: %f, stdv: %f\n", tmp_x, tmp_y, tmp_z, tmp_v, tmp_stdv);

        // 如果需要存入你的雷达点云数据结构中
        ars548_msg::detections point;
        // 假设需要恢复到原单位
        point.f_x            = tmp_x / 100.0;
        point.f_y            = tmp_y / 100.0;
        point.f_z            = tmp_z / 100.0;
        point.f_RangeRate    = tmp_v / 100.0;
        point.f_RangeRateSTD = tmp_stdv / 100.0;
        point.header.stamp   = ros::Time(tmp->cur_time / 1000.0);
        // std::cout << "point.header.stamp : " << point.header.stamp << std::endl;

        // 假设 Radar4D_Cloud 是你的点云容器
        radar_msg->detection_array.push_back(point);
      }
      fclose(fp);
    }
    // radar_msg->header.stamp = ros::Time(tmp->cur_time / 1000.0);

    tmp->pub.publish(radar_msg);
    // std::cout << tmp->cur_time << " writing Radar4DData!" << std::endl;
  } else if (type == 2) {
    hugin::PointCloud PointCloud;
    // PointCloud.objectData.width = 2400000;
    // PointCloud.objectData.height = 1;
    // PointCloud.objectData.points.resize(PointCloud.objectData.width * PointCloud.objectData.height);

    char file[300];
    if (!param_->use_txt_or_pcd) {
      sprintf(file, "%s/%.13ld.txt", data_loader->path_radar_4d.at(id).c_str(),
              tmp->cur_time);

      FILE *fp = fopen(file, "r");
      float tmp_x, tmp_y, tmp_z;
      float tmp_range, tmp_azimuth, tmp_elevation;
      float tmp_doppler, tmp_rcs;
      while (fscanf(fp, "%f %f %f %f %f %f %f %f", &tmp_x, &tmp_y, &tmp_z,
                    &tmp_range, &tmp_azimuth, &tmp_elevation, &tmp_doppler,
                    &tmp_rcs) == 8) {
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

    sensor_msgs::PointCloud2::Ptr radar_msg =
        boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(PointCloud.data, *radar_msg);
    radar_msg->header.seq++;
    radar_msg->header.frame_id = tmp->name;
    radar_msg->header.stamp    = ros::Time(tmp->cur_time / 1000.0);

    tmp->pub.publish(radar_msg);
    // std::cout << tmp->cur_time << " writing Radar4DData!" << std::endl;
  } else {
    std::cout << "radar_4d_type set error !!! " << std::endl;
  }

  tmp->next();

  return true;
}

void Ros1Convert::PublishRadar4D(const ros::TimerEvent &, int id) {
  this->PubRadar4DBase(dc_radar_4d.at(id), id);
}

bool Ros1Convert::PubImageBase(DataContainerRos1<sensor_msgs::ImagePtr> &data_c,
                               int id, int mode) {
  auto tmp = &data_c;

  if (tmp->end()) {
    tmp->stop();
    return false;
  }

  int index = 0;
  char file_image[300];
  switch (mode) {
    case 1:
      index = id;
      if (param_->use_jpg_or_png == 0) {
        sprintf(file_image, "%s/%.13ld.jpg",
                data_loader->path_camera.at(id).c_str(), tmp->cur_time);
      } else {
        sprintf(file_image, "%s/%.13ld.png",
                data_loader->path_camera.at(id).c_str(), tmp->cur_time);
      }
      break;

    case 2:
      index = param_->b_camera + id;
      if (param_->use_jpg_or_png <= 0) {
        sprintf(file_image, "%s/%.13ld.jpg",
                data_loader->path_infra.at(id).c_str(), tmp->cur_time);
      } else {
        sprintf(file_image, "%s/%.13ld.png",
                data_loader->path_infra.at(id).c_str(), tmp->cur_time);
      }
      break;

    case 3:
      index = param_->b_camera + param_->b_infra + id;
      if (param_->use_jpg_or_png <= 0) {
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

  sensor_msgs::ImagePtr image_msg;
  image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", cur_image).toImageMsg();
  image_msg->header.seq++;
  image_msg->header.frame_id = tmp->name;
  image_msg->header.stamp    = ros::Time(tmp->cur_time / 1000.0);

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

void Ros1Convert::PublishImage(const ros::TimerEvent &, int id, int mode) {
  // std::cout << "Ros1Convert::PublishImage()" << std::endl;

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

void Ros1Convert::Ros1PublishBase(const ros::TimerEvent &,
                                  DataContainerBase *tmp) {
  if (tmp->end()) {
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
