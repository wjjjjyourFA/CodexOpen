#include "tools/data_loader/group_convert.h"

namespace jojo {
namespace tools {
namespace common  = apollo::cyber::common;
namespace cstruct = jojo::common_struct;
namespace math    = jojo::common::math;

GroupConvert::GroupConvert() {}

GroupConvert::~GroupConvert() {}

void GroupConvert::Init(std::shared_ptr<RuntimeConfigOffline> param) {
  param_ = param;

  data_loader = std::make_shared<DataLoader>();
  data_loader->Init(param_);
  data_loader->Start();

  dc_camera.resize(param_->b_camera);
  dc_infra.resize(param_->b_infra);
  dc_star.resize(param_->b_star);
  dc_radar4d.resize(param_->b_radar4d);

  this->InitGroup();
  // std::cout << "GroupConvert Init End." << std::endl;
}

void GroupConvert::InitGroup() {
  // 基类指针初始化
  group = std::make_shared<MeasureGroup>();
  // 拿到子类指针
  this->group_ds = std::static_pointer_cast<MeasureGroup>(group);

  if (param_->b_lidar) {
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
    dc_lidar.init_ts(param_->start_time);
    // update the start time
    param_->start_time = dc_lidar.cur_time;

    // clang-format off
    // group_ds->lidar.data = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    group_ds->lidar.data = pcl::PointCloud<pcl::PointXYZIRT>::Ptr(new pcl::PointCloud<pcl::PointXYZIRT>);
    // 预分配，取代 GetLidarBase() 函数中的每帧分配
    group_ds->lidar.data->points.reserve(100000);
    // clang-format on
  }

  if (param_->b_camera) {
    for (int i = 0; i < param_->b_camera; i++) {
      std::string name;
      // if (param_->b_camera == 1) {
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
      dc_camera.at(i).align_ts(param_->start_time);
      // 不同频率的数据 需要在运行时找到匹配的时间戳
    }
    group->camera.resize(param_->b_camera);
  }

  if (param_->b_infra) {
    for (int i = 0; i < param_->b_infra; i++) {
      std::string name;
      // if (param_->b_infra == 1) {
      //   name = "infra";
      // } else {
        name = "infra_" + std::to_string(i + 1);
      // }
      dc_infra.at(i).set_name(name);
      std::string ts_file = "timestamp/" + name + "_timestamp";
      // clang-format off
      if (!data_loader->LoadTimeStamp(data_loader->prefix, ts_file, dc_infra.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_camera.at(i), dc_infra.at(i))) {
          data_loader->SaveTimeStamp(data_loader->prefix, ts_file, dc_infra.at(i));
        } else {
          std::cerr << name << " timestamp load failed" << std::endl;
        }
      }
      // clang-format on
      dc_infra.at(i).align_ts(param_->start_time);
    }
    group->infra.resize(param_->b_infra);
  }

  if (param_->b_star) {
    for (int i = 0; i < param_->b_star; i++) {
      std::string name;
      // if (param_->b_star == 1) {
      //   name = "star";
      // } else {
        name = "star_" + std::to_string(i + 1);
      // }
      dc_star.at(i).set_name(name);
      std::string ts_file = "timestamp/" + name + "_timestamp";
      // clang-format off
      if (!data_loader->LoadTimeStamp(data_loader->prefix, ts_file, dc_star.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_camera.at(i), dc_star.at(i))) {
          data_loader->SaveTimeStamp(data_loader->prefix, ts_file, dc_star.at(i));
        } else {
          std::cerr << name << " timestamp load failed" << std::endl;
        }
      }
      // clang-format on
      dc_star.at(i).align_ts(param_->start_time);
    }
    group->star.resize(param_->b_star);
  }

  if (param_->b_global_pose) {
    dc_global_pose.set_name("global_pose");
    // LoadGlobalPose(data_loader->prefix, dc_global_pose.name);
    LoadGlobalPose(data_loader->path_global_pose);
    // std::cout << "global_pose.size(): " << dc_global_pose.size() << std::endl;
  }

  if (param_->b_local_pose) {
    dc_local_pose.set_name("local_pose");
    // LoadLocalPose(data_loader->prefix, dc_local_pose.name);
    LoadLocalPose(data_loader->path_local_pose);
  }

  // /*
  if (param_->b_radar) {
    std::string name = "radar";
    dc_radar.set_name(name);
    data_loader->LoadTimeStamp(data_loader->prefix,
                               "timestamp/" + name + "_timestamp", dc_radar);
  }
  // */

  // /*
  for (int i = 0; i < param_->b_radar4d; i++) {
    // static int type = Radar4DTypeParam[param_->radar4d_type];
    auto type = SensorRegistry::Instance().GetRadar4dType(param_->radar4d_type);

    std::string name;
    // if (param_->b_radar4d == 1) {
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

    group->radar4d.resize(param_->b_radar4d);
  }
  // */

  if (param_->b_imu_data) {
    // std::cout << "init IMU data" << std::endl;
    dc_imu_data.set_name("imu_data");
    // LoadImuData(data_loader->prefix, dc_imu_data.name);
    LoadImuData(data_loader->path_imu_data);
  }
}

std::shared_ptr<const MeasureGroupBase> GroupConvert::ReadNext() {
  // std::cout << "GroupConvert ReadNext." << std::endl;
  static bool first_run = false;
  if (!first_run) {
    index_ts    = param_->start_time;
    is_running_ = true;
    first_run   = true;
  }

  if (param_->end_time !=0 && index_ts >= param_->end_time) {
    is_running_ = false;
    return nullptr;
  }

  uint64_t last_ts = index_ts;
  if (param_->b_lidar) {  // 加载点云
    dc_lidar.align_ts(index_ts);
    // time ==> scan end time ; start_time ==> scan start time
    // this->GetLidarBase<pcl::PointXYZI>()
    if (!this->GetLidarBase<pcl::PointXYZIRT>(dc_lidar, group_ds->lidar.data,
                                              group_ds->lidar.time,
                                              group_ds->lidar.start_time)) {
      std::cerr << "[Warning] Failed to load lidar: "
                << ", skipping frame." << std::endl;
      is_running_ = false;
    }
    // 更新 基准 时间戳，但这里是下一帧的，因为 GetLidarBase 自增了迭代器
    index_ts = dc_lidar.cur_time;
  }

  if (param_->b_camera) {  // 加载图像
    // std::cout << "GroupConvert ReadNext. Camera." << std::endl;
    for (int i = 0; i < param_->b_camera; i++) {
      dc_camera.at(i).align_ts(last_ts);
      this->GetImage(group->camera.at(i).data, group->camera.at(i).time, i, 1);
      if (group->camera.at(i).data.empty()) {
        std::cerr << "[Warning] Failed to load camera : " << i + 1
                  << ", skipping frame." << std::endl;
        is_running_ = false;
      }
    }
  }

  if (param_->b_infra) {  // 加载图像
    for (int i = 0; i < param_->b_infra; i++) {
      dc_infra.at(i).align_ts(last_ts);
      this->GetImage(group->infra.at(i).data, group->infra.at(i).time, i, 1);
      if (group->infra.at(i).data.empty()) {
        std::cerr << "[Warning] Failed to load infra : " << i + 1
                  << ", skipping frame." << std::endl;
        is_running_ = false;
      }
    }
  }

  if (param_->b_star) {  // 加载图像
    for (int i = 0; i < param_->b_star; i++) {
      dc_star.at(i).align_ts(last_ts);
      this->GetImage(group->star.at(i).data, group->star.at(i).time, i, 1);
      if (group->star.at(i).data.empty()) {
        std::cerr << "[Warning] Failed to load star : " << i + 1
                  << ", skipping frame." << std::endl;
        is_running_ = false;
      }
    }
  }

  if (param_->b_imu_data) {  // 加载IMU
    // std::cout << "GroupConvert ReadNext. IMU." << std::endl;
    dc_imu_data.align_ts(last_ts);
    if (!this->GetDataBase(&dc_imu_data, group_ds->imu.data, group_ds->imu.time)) {
      std::cerr << "[Warning] Failed to load imu_data: "
                << ", skipping frame." << std::endl;
      is_running_ = false;
    }

    if (param_->b_imu_vec) {
      dc_imu_data.GetDataSegment(group_ds->lidar.start_time, group_ds->lidar.time,
                                 group_ds->imu_vec);

      // std::cout << "imu_vec size: " << group->imu_vec.size() << std::endl;
      // this->print_imu_vec(group_ds->imu_vec);
    }
  }

  if (param_->b_local_pose) {
    // std::cout << "GroupConvert ReadNext. Odometry." << std::endl;
    dc_local_pose.align_ts(last_ts);
    if (!this->GetDataBase(&dc_local_pose, group_ds->odom.data,
                           group_ds->odom.time)) {
      std::cerr << "[Warning] Failed to load odom_data: "
                << ", skipping frame." << std::endl;
      is_running_ = false;
    }
  }

  if (param_->b_global_pose) {
    // std::cout << "GroupConvert ReadNext. GNSS." << std::endl;
    dc_global_pose.align_ts(last_ts);
    if (!this->GetDataBase(&dc_global_pose, group_ds->gnss.data,
                           group_ds->gnss.time)) {
      std::cerr << "[Warning] Failed to load gnss_data: "
                << ", skipping frame." << std::endl;
      is_running_ = false;
    }
  }

  return group;
}

bool GroupConvert::LoadGlobalPose(const std::string& path,
                                  const std::string& data_file) {
  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), data_file.c_str());

  return this->LoadGlobalPose(std::string(file));
}

bool GroupConvert::LoadGlobalPose(const std::string& file) {
  // 这里将外部数据转换为这个接口
  cstruct::GnssData globalPoseMsg;

  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
    return false;
  }

  FILE* _fp = NULL;
  _fp       = fopen(file.c_str(), "r");
  if (_fp != NULL) {
    while (!feof(_fp)) {
      double local_time;
      uint32_t UTC_time;
      double latitude, longitude;
      int gaussX, gaussY, height;
      int roll, pitch, azimuth;
      int vEast, vNorth, vUp;
      float dev_gaussX, dev_gaussY, dev_height;
      float dev_roll, dev_pitch, dev_azimuth;
      float dev_vEast, dev_vNorth, dev_vUp;
      int ins_status, pos_type;

      // clang-format off
      // UGV2023 ROS_MSG
      /*
      fscanf(_fp, "%lf %u "
             "%lf %lf "
             "%d %d %d %d %d %d "
             "%d %d %d "
             "%*d %*d %*d "            
             "%*d %*d %*d "
             "%f %f %f %f %f %f "
             "%f %f %f "
             "%d %d %*d %*d %*d %*d "
             "%*f %*f %*f %*f \n",
       &local_time, &UTC_time,
       &latitude, &longitude,
       &gaussX, &gaussY, &height, &roll, &pitch, &azimuth,
       &vEast, &vNorth, &vUp,
       &dev_gaussX, &dev_gaussY, &dev_height,
       &dev_roll, &dev_pitch, &dev_azimuth,
       &dev_vEast, &dev_vNorth, &dev_vUp,
       &ins_status, &pos_type);
      */
      // 兼容性读取
      // /*
      fscanf(_fp, "%lf %u "
             "%lf %lf "
             "%d %d %d %d %d %d "
             "%d %d %d "
             "%*lf %*lf %*lf "
             "%*lf %*lf %*lf "
             "%f %f %f %f %f %f "
             "%f %f %f "
             "%d %d %*lf %*lf %*lf %*lf "
             "%*f %*f %*f %*f \n",
       &local_time, &UTC_time,
       &latitude, &longitude,
       &gaussX, &gaussY, &height, &roll, &pitch, &azimuth,
       &vEast, &vNorth, &vUp,
       &dev_gaussX, &dev_gaussY, &dev_height,
       &dev_roll, &dev_pitch, &dev_azimuth,
       &dev_vEast, &dev_vNorth, &dev_vUp,
       &ins_status, &pos_type);
      // */
      // clang-format on

      globalPoseMsg.position.latitude  = latitude / 1e6;
      globalPoseMsg.position.longitude = longitude / 1e6;
      globalPoseMsg.position.altitude  = height / 100.0;

      globalPoseMsg.gauss_point.x = gaussX / 100.0;
      globalPoseMsg.gauss_point.y = gaussY / 100.0;

      // clang-format off
      // globalPoseMsg.orientation.azimuth = azimuth / 100.0;
      // globalPoseMsg.orientation.pitch   = pitch / 100.0;
      // globalPoseMsg.orientation.roll    = roll / 100.0;
      globalPoseMsg.orientation.azimuth = math::UnitConverter::angle_to_rad(azimuth / 100.0);
      globalPoseMsg.orientation.pitch   = math::UnitConverter::angle_to_rad(pitch / 100.0);
      globalPoseMsg.orientation.roll    = math::UnitConverter::angle_to_rad(roll / 100.0);

      // std::cout << "azimuth: " << azimuth / 100.0 << std::endl;
      // std::cout << "pitch: " << pitch / 100.0 << std::endl;
      // std::cout << "roll: " << roll / 100.0 << std::endl;
      // std::cout << "azimuth: " << math::UnitConverter::angle_to_rad(azimuth / 100.0) << std::endl;
      // std::cout << "pitch: " << math::UnitConverter::angle_to_rad(pitch / 100.0) << std::endl;
      // std::cout << "roll: " << math::UnitConverter::angle_to_rad(roll / 100.0) << std::endl;

      // std::cout << "local_time: " << local_time << std::endl;
      // clang-format on

      globalPoseMsg.velocity.east  = vEast / 100.0;
      globalPoseMsg.velocity.north = vNorth / 100.0;
      globalPoseMsg.velocity.up    = vUp / 100.0;

      globalPoseMsg.status = ins_status;
      globalPoseMsg.age    = pos_type;

      dc_global_pose.insert(uint64_t(local_time), &globalPoseMsg);
    }
    fclose(_fp);
    return true;
  }

  return false;
}

bool GroupConvert::LoadLocalPose(const std::string& path,
                                 const std::string& data_file) {
  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), data_file.c_str());

  return this->LoadLocalPose(std::string(file));
}

bool GroupConvert::LoadLocalPose(const std::string& file) {
  cstruct::OdomData localPoseMsg;

  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
    return false;
  }

  FILE* _fp = NULL;
  _fp       = fopen(file.c_str(), "r");
  if (_fp != NULL) {
    while (!feof(_fp)) {
      double local_time;
      uint32_t UTC_time;
      int dr_x, dr_y, dr_z;
      int dr_roll, dr_pitch, dr_heading;
      int speed_x, speed_y, speed_z;
      int vehicle_speed;
      int driving_direction;

      // UGV2023 ROS_MSG
      // clang-format off
      // /*
      fscanf(_fp, "%lf %u "
              "%d %d %d %d %d %d "
              "%d %d %d %d "
              "%*d %*d %*d "
              "%*d %*d %*d "
              "%*d %*d %*d %d "
              "%*d %*d \n",
              &local_time, &UTC_time,
              &dr_x, &dr_y, &dr_z,
              &dr_roll, &dr_pitch, &dr_heading,
              &speed_x, &speed_y, &speed_z,
              &vehicle_speed,
              &driving_direction);
      // */
      // clang-format on

      localPoseMsg.position.x = dr_x / 100.0;
      localPoseMsg.position.y = dr_y / 100.0;
      localPoseMsg.position.z = dr_z / 100.0;
      // clang-format off
      // localPoseMsg.orientation.azimuth = dr_heading / 100.0;
      // localPoseMsg.orientation.pitch   = dr_pitch / 100.0;
      // localPoseMsg.orientation.roll    = dr_roll / 100.0;
      localPoseMsg.orientation.azimuth = math::UnitConverter::angle_to_rad(dr_heading / 100.0);
      localPoseMsg.orientation.pitch   = math::UnitConverter::angle_to_rad(dr_pitch / 100.0);
      localPoseMsg.orientation.roll    = math::UnitConverter::angle_to_rad(dr_roll / 100.0);
      // clang-format on
      localPoseMsg.velocity.x = speed_x / 100.0;
      localPoseMsg.velocity.y = speed_y / 100.0;
      localPoseMsg.velocity.z = speed_z / 100.0;
      localPoseMsg.speed      = vehicle_speed / 100.0;

      dc_local_pose.insert(uint64_t(local_time), &localPoseMsg);
    }
    fclose(_fp);
    return true;
  }

  return false;
}

bool GroupConvert::LoadImuData(const std::string& path,
                               const std::string& data_file) {
  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), data_file.c_str());

  return this->LoadImuData(std::string(file));
}

bool GroupConvert::LoadImuData(const std::string& file) {
  cstruct::ImuData imu;

  /* way 1
  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
    return false;
  }
  */
  std::ifstream fin(file);
  if (!fin.is_open()) {
    std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
    return false;
  }

  std::string line;
  while (std::getline(fin, line)) {
    std::istringstream iss(line);

    double local_time;
    // 兼容性代码 int ==> float
    float dummy;
    int seq;
    // clang-format off
    if (iss >> local_time >> dummy >> seq
            >> imu.gyro.x >> imu.gyro.y >> imu.gyro.z
            >> imu.acc.x >> imu.acc.y >> imu.acc.z) {
      // 一般情况下是存的弧度，是不需要转换的
      /* 角度 转 弧度
      imu.gyro.x *= M_PI / 180.0;
      imu.gyro.y *= M_PI / 180.0;
      imu.gyro.z *= M_PI / 180.0;
      */

      /* debug 
      Eigen::Vector3d gyro(imu.gyro.x, imu.gyro.y, imu.gyro.z);
      Eigen::Vector3d acc(imu.acc.x, imu.acc.y, imu.acc.z);

      // clang-format off
      Eigen::Matrix3d imu_ext = Eigen::Matrix3d::Identity();
      imu_ext << 1.0, 0.0, 0.0,
                 0.0, 0.6428, -0.7660,
                 0.0, 0.7660, 0.6428;
      // clang-format on
      
      gyro = imu_ext * gyro;
      acc  = imu_ext * acc;

      imu.gyro.x = gyro.x();
      imu.gyro.y = gyro.y();
      imu.gyro.z = gyro.z();

      imu.acc.x = acc.x();
      imu.acc.y = acc.y();
      imu.acc.z = acc.z();
      */

      // imu.time = static_cast<uint64_t>(local_time);
      imu.time = static_cast<double>(local_time);

      // ms 插入依然保持整数，方便快速查找
      dc_imu_data.insert(uint64_t(imu.time), &imu);

      /* debug
      std::cout << "---- ----"
                << "time: " << imu.time << " gyro: [" << imu.gyro.x << ", " << imu.gyro.y << ", " << imu.gyro.z << "]"
                << " acc: [" << imu.acc.x << ", " << imu.acc.y << ", " << imu.acc.z << "]" << std::endl;
      */
    }
    // clang-format on
  }

  return true;
}

/*
bool GroupConvert::GetLidarBase(
    DataContainer<uint64_t>& data_c,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ptr, uint64_t& time,
    uint64_t& start_time) {
  auto tmp = &data_c;

  if (tmp->is_end()) {
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
      // cm => m
      pt.x = tmp_point[0] / 100.0;
      pt.y = tmp_point[1] / 100.0;
      pt.z = tmp_point[2] / 100.0;
      // 自动转 float
      pt.intensity = tmp_point[3];

      cloud->points.emplace_back(pt);
    }
    fclose(fp);

    cloud->width  = cloud->points.size();
    cloud->height = 1;
  } else {
    sprintf(file, "%s/%.13ld.pcd", data_loader->path_lidar.c_str(),
            tmp->cur_time);

    int ret = pcl::io::loadPCDFile(std::string(file), *cur_cloud_ptr);
    // int ret = pcl::io::loadPCDFile<pcl::PointXYZI>(std::string(file), *cur_cloud_ptr);
    if (ret == -1) {
      std::cerr << "[Warning] Failed to load PCD file: " << time
                << ", skipping frame." << std::endl;
    }

    // warning
    // pcl::PointCloud<pcl::PointXYZI> cloud;
    // pcl::fromPCLPointCloud2(*cur_cloud_ptr, cloud);
    // pcl::transformPointCloud(cloud, cloud, transform_mat);
    // pcl::toPCLPointCloud2(cloud, *cur_cloud_ptr);
    // toc(t);
  }

  // show_pointcloud_height(*cur_cloud_ptr, 1);
  // show_pointcloud_num(*cur_cloud_ptr, 1800);

  time = tmp->cur_time;
  // std::cout << tmp->cur_time << " writing LidarData!" << std::endl;
  start_time = last_pcd_time_;
  if (start_time == 0) {
    start_time = time - 100;
  }
  last_pcd_time_ = time;

  tmp->next();

  return true;
}
*/

bool GroupConvert::GetImageBase(DataContainer<uint64_t>& data_c,
                                cv::Mat& cur_image, uint64_t& time, int id,
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

  cur_image = cv::imread(std::string(file_image));
  if (cur_image.empty()) {
    std::cerr << "[Warning] Failed to load image: " << time
              << ", skipping frame." << std::endl;
  }

  time = tmp->cur_time;
  // std::cout << tmp->cur_time << " writing ImageData!" << std::endl;
  tmp->next();

  return true;
}

void GroupConvert::GetImage(cv::Mat& cur_image, uint64_t& time, int id,
                            int mode) {
  // std::cout << "GroupConvert GetImage." << std::endl;

  switch (mode) {
    case 1:
      this->GetImageBase(dc_camera.at(id), cur_image, time, id, mode);
      break;
    case 2:
      this->GetImageBase(dc_infra.at(id), cur_image, time, id, mode);
      break;
    case 3:
      this->GetImageBase(dc_star.at(id), cur_image, time, id, mode);
      break;
    default:
      std::cout << "camera_type set error !!! " << std::endl;
      break;
  }
}

void GroupConvert::print_imu_vec(const std::deque<cstruct::ImuData>& imu_vec) {
  for (size_t i = 0; i < imu_vec.size(); ++i) {
    const auto& imu = imu_vec[i];

    // clang-format off
    std::cout << std::fixed << std::setprecision(6) << "imu[" << i << "] "
              << "time: " << imu.time << " gyro: [" << imu.gyro.x << ", " << imu.gyro.y << ", " << imu.gyro.z << "]"
              << " acc: [" << imu.acc.x << ", " << imu.acc.y << ", " << imu.acc.z << "]" << std::endl;
    // clang-format on
  }
}

}  // namespace tools
}  // namespace jojo
