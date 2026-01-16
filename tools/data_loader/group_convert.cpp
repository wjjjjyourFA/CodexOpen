#include "tools/data_loader/group_convert.h"

namespace jojo {
namespace tools {
namespace common = apollo::cyber::common;

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
  dc_radar_4d.resize(param_->b_radar_4d);

  this->InitGroup();
}

void GroupConvert::InitGroup() {
  group = std::make_shared<MeasureGroup>();

  if (param_->b_lidar) {
    std::string name = "lidar";
    dc_lidar.set_name(name);
    if (!data_loader->LoadTimeStamp(data_loader->prefix,
                                    "timestamp/" + name + "_timestamp",
                                    dc_lidar)) {
      if (!data_loader->ExtractTimestamp(data_loader->path_lidar, dc_lidar)) {
        exit(1);
      }
    }
    dc_lidar.init_ts(param_->start_time);
    // update the start time
    param_->start_time = dc_lidar.cur_time;

    group->lidar.data = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
  }

  if (param_->b_camera || param_->b_infra || param_->b_star) {
    for (int i = 0; i < param_->b_camera; i++) {
      std::string name = "image";
      dc_camera.at(i).set_name(name);
      if (!data_loader->LoadTimeStamp(data_loader->prefix,
                                      "timestamp/" + name + "_timestamp",
                                      dc_camera.at(i))) {
        if (!data_loader->ExtractTimestamp(data_loader->path_camera.at(i),
                                           dc_camera.at(i))) {
          exit(1);
        }
      }
      // 对于 同频率的数据 可以直接递推
      dc_camera.at(i).align_ts(param_->start_time);
      // 不同频率的数据 需要在运行时找到匹配的时间戳

      group->camera.resize(param_->b_camera);
    }

    for (int i = 0; i < param_->b_infra; i++) {
      std::string name = "infra";
      dc_infra.at(i).set_name(name);
      data_loader->LoadTimeStamp(data_loader->prefix,
                                 "timestamp/" + name + "_timestamp",
                                 dc_infra.at(i));
      dc_infra.at(i).align_ts(param_->start_time);

      group->infra.resize(param_->b_infra);
    }

    // clang-format off
    for (int i = 0; i < param_->b_star; i++) {
      std::string name = "star";
      dc_star.at(i).set_name(name);
      data_loader->LoadTimeStamp(data_loader->prefix, 
                                 "timestamp/" + name + "_timestamp", 
                                 dc_star.at(i));
      dc_star.at(i).align_ts(param_->start_time);

      group->star.resize(param_->b_star);
    }
    // clang-format on
  }

  // if (param_->b_global_pose) {
  //   dc_global_pose.set_name("global_pose");
  //   LoadGlobalPose(data_loader->prefix, dc_global_pose.name);
  // }

  // if (param_->b_local_pose) {
  //   dc_local_pose.set_name("local_pose");
  //   LoadLocalPose(data_loader->prefix, dc_local_pose.name);
  // }

  // if (param_->b_radar) {
  //   std::string name = "radar";
  //   dc_radar.set_name(name);
  //   data_loader->LoadTimeStamp(data_loader->prefix,
  //                              "timestamp/" + name + "_timestamp", dc_radar);
  // }

  // for (int i = 0; i < param_->b_radar_4d; i++) {
  //   // static int type = Radar4DTypeParam[param_->radar_4d_type];

  //   std::string name = "radar_4d";
  //   dc_radar_4d.at(i).set_name(name);
  //   data_loader->LoadTimeStamp(data_loader->prefix,
  //                              "timestamp/" + name + "_timestamp",
  //                              dc_radar_4d.at(i));

  //   group->radar_4d.resize(param_->b_radar_4d);
  // }

  if (param_->b_imu_data) {
    std::cout << "init IMU data" << std::endl;
    dc_imu_data.set_name("imu_data");
    LoadImuData(data_loader->prefix, dc_imu_data.name);
  }
}

std::shared_ptr<const MeasureGroup> GroupConvert::ReadNext() {
  static bool first_run = false;
  if (!first_run) {
    index_ts    = param_->start_time;
    is_running_ = true;
    first_run   = true;
  }

  uint64_t last_ts = index_ts;
  if (param_->b_lidar) {  // 加载点云
    dc_lidar.align_ts(index_ts);
    if (!this->GetLidarBase(dc_lidar, group->lidar.data, group->lidar.time)) {
      std::cerr << "[Warning] Failed to load lidar: "
                << ", skipping frame." << std::endl;
      is_running_ = false;
    }
    // 更新 基准 时间戳，但这里是下一帧的，因为 GetLidarBase 自增了迭代器
    index_ts = dc_lidar.cur_time;
  }

  if (param_->b_camera || param_->b_infra || param_->b_star) {  // 加载图像
    for (int i = 0; i < param_->b_camera; i++) {
      dc_camera.at(i).align_ts(last_ts);
      this->GetImage(group->camera.at(i).data, group->camera.at(i).time, i, 1);
      if (group->camera.at(i).data.empty()) {
        std::cerr << "[Warning] Failed to load camera : " << i + 1
                  << ", skipping frame." << std::endl;
      }
    }
  }

  if (param_->b_imu_data) {  // 加载IMU
    dc_imu_data.align_ts(last_ts);
    if (!this->GetDataBase(&dc_imu_data, group->imu.data, group->imu.time)) {
      std::cerr << "[Warning] Failed to load imu_data: "
                << ", skipping frame." << std::endl;
    }
  }

  return group;
}

bool GroupConvert::LoadGlobalPose(const std::string &path,
                                  const std::string &data_file) {
  // 这里将外部数据转换为这个接口
  GnssData globalPoseMsg;

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
      // clang-format off
      // UGV2023 ROS_MSG
      // /*
      float time, tmp2;
      int tmp3;
      float tmp4, tmp5, tmp6, tmp7, tmp8, tmp9, tmp10, tmp11, tmp12;
      int tmp13, tmp14;
      float tmp15, tmp16, tmp17, tmp18;
      fscanf(_fp, "%lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %lf %lf %lf %lf",
             &time,    &tmp2,    &tmp3, 
             &globalPoseMsg.gauss_point.x,         &globalPoseMsg.gauss_point.y,       &globalPoseMsg.position.altitude, 
             &globalPoseMsg.velocity.east,         &globalPoseMsg.velocity.north,      &globalPoseMsg.velocity.up, 
             &globalPoseMsg.orientation.azimuth,   &globalPoseMsg.orientation.pitch,   &globalPoseMsg.orientation.roll,

             &tmp4,    &tmp5,   &tmp6, 
             &tmp7,    &tmp8,   &tmp9,
             &tmp10,   &tmp11,  &tmp12,
             &globalPoseMsg.position.longitude,     &globalPoseMsg.position.latitude,

             &tmp13,   &tmp14,
             &tmp15,   &tmp16,
             &tmp17,   &tmp18);
      // */
      // clang-format on

      dc_global_pose.insert(uint64_t(time), &globalPoseMsg);
    }
    fclose(_fp);
    return true;
  }

  return false;
}

bool GroupConvert::LoadLocalPose(const std::string &path,
                                 const std::string &data_file) {
  OdomData localPoseMsg;

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
      float time, tmp2;
      int tmp3;
      int temp1, temp2, temp3;  // 用来跳过 0, 1, -1
      int tmp4;
      float tmp5, tmp6, tmp7, tmp8;
      fscanf(_fp, "%lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %d %d %lf %lf %lf %lf\n",
             &time,     &tmp2,     &tmp3,  
             &localPoseMsg.position.x,           &localPoseMsg.position.y,        &localPoseMsg.position.z,         
             &localPoseMsg.orientation.azimuth,  &localPoseMsg.orientation.roll,  &localPoseMsg.orientation.pitch,
             &localPoseMsg.speed,  
             &localPoseMsg.velocity.x,           &localPoseMsg.velocity.y,        &localPoseMsg.velocity.z, 
             &temp1,    &temp2,    &temp3, 
             &tmp4, 
             &tmp5,     &tmp6, 
             &tmp7,     &tmp8);
      // */
      // clang-format on
      dc_local_pose.insert(uint64_t(time), &localPoseMsg);
    }
    fclose(_fp);
    return true;
  }

  return false;
}

bool GroupConvert::LoadImuData(const std::string &path,
                               const std::string &data_file) {
  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), data_file.c_str());

  return this->LoadImuData(std::string(file));
}

bool GroupConvert::LoadImuData(const std::string &file) {
  ImuData imu_data;

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

  double tmp1;
  int tmp2;

  std::string line;
  while (std::getline(fin, line)) {
    std::istringstream iss(line);

    // clang-format off
    if (iss >> imu_data.time >> tmp1 >> tmp2 >> 
        imu_data.gyro.y >> imu_data.gyro.x >> imu_data.gyro.z >> 
        imu_data.acc.y >> imu_data.acc.x >> imu_data.acc.z) {
      imu_data.gyro.x *= M_PI / 180.0;
      imu_data.gyro.y *= -M_PI / 180.0;
      imu_data.gyro.z *= M_PI / 180.0;
      imu_data.acc.y *= -1;

      dc_imu_data.insert(uint64_t(imu_data.time), &imu_data);
    }
    // clang-format on
  }

  return true;
}

bool GroupConvert::GetLidarBase(
    DataContainer<uint64_t> &data_c,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ptr, uint64_t &time) {
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

  time = tmp->cur_time;
  // std::cout << tmp->cur_time << " writing LidarData!" << std::endl;
  tmp->next();

  return true;
}

bool GroupConvert::GetImageBase(DataContainer<uint64_t> &data_c,
                                cv::Mat &cur_image, uint64_t &time, int id,
                                int mode) {
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

  cur_image = cv::imread(std::string(file_image));

  time = tmp->cur_time;
  // std::cout << tmp->cur_time << " writing ImageData!" << std::endl;
  tmp->next();

  return true;
}

void GroupConvert::GetImage(cv::Mat &cur_image, uint64_t &time, int id,
                            int mode) {
  // std::cout << "DataLoader::GetImage()" << std::endl;

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

}  // namespace tools
}  // namespace jojo
