#include "toolz/data_loader/group_convert.h"

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <map>

namespace jojo {
namespace tools {
namespace common    = apollo::cyber::common;
namespace cstruct   = jojo::common_struct;
namespace math      = jojo::common::math;
namespace transform = jojo::common::transform;

GroupConvertDataSet::GroupConvertDataSet() {}

GroupConvertDataSet::~GroupConvertDataSet() {}

bool GroupConvertDataSet::Init(
    std::shared_ptr<jojo::tools::RuntimeConfig> rparam,
    std::shared_ptr<jojo::tools::InterfaceConfig> iparam) {
  if (!rparam || !iparam) {
    std::cerr << "[ERROR] GroupConvert requires non-null configuration"
              << std::endl;
    return false;
  }
  if (!iparam->b_lidar) {
    std::cerr << "[ERROR] GroupConvert requires lidar as the master clock"
              << std::endl;
    return false;
  }

  rparam_ = rparam;
  iparam_ = iparam;

  data_loader = std::make_shared<DataLoaderDataSet>();
  if (!data_loader->Init(rparam_, iparam_)) {
    return false;
  }
  data_loader->Start();

  dc_camera.resize(iparam_->b_camera);
  dc_infra.resize(iparam_->b_infra);
  dc_star.resize(iparam_->b_star);
  dc_radar4d.resize(iparam_->b_radar4d);

  if (!this->InitGroup()) {
    is_running_ = false;
    return false;
  }
  // std::cout << "GroupConvertDataSet Init End." << std::endl;

  return true;
}

bool GroupConvertDataSet::InitGroup() {
  // 基类指针初始化
  group = std::make_shared<MeasureGroupDataSet>();
  // 拿到子类指针
  this->group_ds = std::static_pointer_cast<MeasureGroupDataSet>(group);

  if (iparam_->b_lidar) {
    std::string name = "lidar";
    dc_lidar.set_name(name);
    std::string ts_file = "timestamp/" + name + "_timestamp";
    if (!data_loader->LoadTimeStamp(data_loader->postfix, ts_file, dc_lidar)) {
      if (data_loader->ExtractTimestamp(data_loader->path_lidar, dc_lidar)) {
        data_loader->SaveTimeStamp(data_loader->postfix, ts_file, dc_lidar);
      } else {
        std::cerr << name << " timestamp load failed" << std::endl;
        return false;
      }
    }
    if (dc_lidar.empty() || dc_lidar.init_ts(rparam_->start_time) == 0) {
      std::cerr << name << " has no frame at or after start_time" << std::endl;
      return false;
    }
    // update the start time
    rparam_->start_time = dc_lidar.cur_time;

    // clang-format off
    group_ds->lidar.data = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    group_ds->lidar.data->points.reserve(100000);
    // clang-format on
  }

  if (iparam_->b_camera) {
    for (int i = 0; i < iparam_->b_camera; i++) {
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
      if (!data_loader->LoadTimeStamp(data_loader->postfix, ts_file, dc_camera.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_camera.at(i), dc_camera.at(i))) {
          data_loader->SaveTimeStamp(data_loader->postfix, ts_file, dc_camera.at(i));
        } else {
          std::cerr << name << " timestamp load failed" << std::endl;
          return false;
        }
      }
      // clang-format on
      // 对于 同频率的数据 可以直接递推
      dc_camera.at(i).align_ts(rparam_->start_time);
      // 不同频率的数据 需要在运行时找到匹配的时间戳
    }
    group->camera.resize(iparam_->b_camera);
    // std::cout << "camera size: " << group->camera.size() << std::endl;
  }

  if (iparam_->b_infra) {
    for (int i = 0; i < iparam_->b_infra; i++) {
      std::string name;
      // if (iparam_->b_infra == 1) {
      //   name = "infra";
      // } else {
      name = "infra_" + std::to_string(i + 1);
      // }
      dc_infra.at(i).set_name(name);
      std::string ts_file = "timestamp/" + name + "_timestamp";
      // clang-format off
      if (!data_loader->LoadTimeStamp(data_loader->postfix, ts_file, dc_infra.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_infra.at(i), dc_infra.at(i))) {
          data_loader->SaveTimeStamp(data_loader->postfix, ts_file, dc_infra.at(i));
        } else {
          std::cerr << name << " timestamp load failed" << std::endl;
          return false;
        }
      }
      // clang-format on
      dc_infra.at(i).align_ts(rparam_->start_time);
    }
    group->infra.resize(iparam_->b_infra);
  }

  if (iparam_->b_star) {
    for (int i = 0; i < iparam_->b_star; i++) {
      std::string name;
      // if (iparam_->b_star == 1) {
      //   name = "star";
      // } else {
      name = "star_" + std::to_string(i + 1);
      // }
      dc_star.at(i).set_name(name);
      std::string ts_file = "timestamp/" + name + "_timestamp";
      // clang-format off
      if (!data_loader->LoadTimeStamp(data_loader->postfix, ts_file, dc_star.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_star.at(i), dc_star.at(i))) {
          data_loader->SaveTimeStamp(data_loader->postfix, ts_file, dc_star.at(i));
        } else {
          std::cerr << name << " timestamp load failed" << std::endl;
          return false;
        }
      }
      // clang-format on
      dc_star.at(i).align_ts(rparam_->start_time);
    }
    group->star.resize(iparam_->b_star);
  }

  if (iparam_->b_global_pose) {
    dc_se3_pose.set_name("global_pose");
    // LoadPose(data_loader->postfix, dc_se3_pose.name);
    if (!LoadPose(data_loader->path_global_pose) || dc_se3_pose.empty()) {
      return false;
    }
  } else if (iparam_->b_local_pose) {
    dc_se3_pose.set_name("local_pose");
    // LoadPose(data_loader->postfix, dc_se3_pose.name);
    if (!LoadPose(data_loader->path_local_pose) || dc_se3_pose.empty()) {
      return false;
    }
  }

  return true;
}

std::shared_ptr<const MeasureGroupBase> GroupConvertDataSet::ReadNext() {
  // std::cout << "GroupConvert ReadNext." << std::endl;
  if (!is_running_) {
    return nullptr;
  }
  if (!started_) {
    index_ts    = rparam_->start_time;
    is_running_ = true;
    started_    = true;
  }

  if (rparam_->end_time != 0 &&
      index_ts >= static_cast<uint64_t>(rparam_->end_time)) {
    is_running_ = false;
    return nullptr;
  }

  uint64_t last_ts = index_ts;
  if (iparam_->b_lidar) {  // 加载点云
    dc_lidar.align_ts(index_ts);
    if (!this->GetLidarBase<pcl::PointXYZI>(dc_lidar, group_ds->lidar.data,
                                            group_ds->lidar.time,
                                            group_ds->lidar.start_time)) {
      std::cerr << "[Warning] Failed to load lidar: "
                << ", skipping frame." << std::endl;
      is_running_ = false;
      return nullptr;
    }
    // 更新 基准 时间戳，但这里是下一帧的，因为 GetLidarBase 自增了迭代器
    if (dc_lidar.is_end()) {
      is_running_ = false;
    } else {
      index_ts = dc_lidar.cur_time;
    }
  }

  if (iparam_->b_camera) {  // 加载图像
    // std::cout << "GroupConvert ReadNext. Camera." << std::endl;
    for (int i = 0; i < iparam_->b_camera; i++) {
      dc_camera.at(i).align_ts(last_ts);
      this->GetImage(group->camera.at(i).data, group->camera.at(i).time, i, 1);
      if (group->camera.at(i).data.empty()) {
        std::cerr << "[Warning] Failed to load camera : " << i + 1
                  << ", skipping frame." << std::endl;
        is_running_ = false;
      }
    }
  }

  if (iparam_->b_infra) {  // 加载图像
    for (int i = 0; i < iparam_->b_infra; i++) {
      dc_infra.at(i).align_ts(last_ts);
      this->GetImage(group->infra.at(i).data, group->infra.at(i).time, i, 2);
      if (group->infra.at(i).data.empty()) {
        std::cerr << "[Warning] Failed to load infra : " << i + 1
                  << ", skipping frame." << std::endl;
        is_running_ = false;
      }
    }
  }

  if (iparam_->b_star) {  // 加载图像
    for (int i = 0; i < iparam_->b_star; i++) {
      dc_star.at(i).align_ts(last_ts);
      this->GetImage(group->star.at(i).data, group->star.at(i).time, i, 3);
      if (group->star.at(i).data.empty()) {
        std::cerr << "[Warning] Failed to load star : " << i + 1
                  << ", skipping frame." << std::endl;
        is_running_ = false;
      }
    }
  }

  if (iparam_->b_local_pose || iparam_->b_global_pose) {
    // std::cout << "GroupConvert ReadNext. Pose SE3." << std::endl;
    dc_se3_pose.align_ts(last_ts);
    if (!this->GetDataBase(&dc_se3_pose, group_ds->se3_pose.data,
                           group_ds->se3_pose.time)) {
      std::cerr << "[Warning] Failed to load se3_pose : "
                << ", skipping frame." << std::endl;
      is_running_ = false;
    }

    if (rparam_->b_pose_vec) {
      dc_se3_pose.GetDataSegment(group_ds->lidar.start_time,
                                 group_ds->lidar.time, group_ds->se3_pose_vec);

      // std::cout << "pose_vec size: " << group_ds->se3_pose_vec.size() << std::endl;
      // this->print_pose_vec(group_ds->se3_pose_vec);
    }
  }

  return group;
}

bool GroupConvertDataSet::LoadPose(const std::string& path,
                                   const std::string& data_file) {
  if (path.empty() || data_file.empty()) {
    std::cerr << "[ERROR] Pose path and file name must not be empty"
              << std::endl;
    return false;
  }

  char file[300];
  snprintf(file, sizeof(file), "%s/%s.txt", path.c_str(), data_file.c_str());

  return this->LoadPose(std::string(file));
}

bool GroupConvertDataSet::LoadPose(const std::string& file) {
  if (file.empty()) {
    std::cerr << "[ERROR] Pose file path must not be empty" << std::endl;
    return false;
  }

  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
    return false;
  }

  FILE* fp = fopen(file.c_str(), "r");
  if (fp == nullptr) {
    std::cerr << "[ERROR] Failed to open pose file: " << file << std::endl;
    return false;
  }

  // 这里将外部数据转换为这个接口
  cstruct::SE3Pose pose;

  while (true) {
    // 优化后的位姿，时间戳使用的整型
    int64_t local_time = 0;
    double x = 0.0, y = 0.0, z = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    // clang-format off
    const int items_read = fscanf(fp, "%" SCNd64 " %lf %lf %lf %lf %lf %lf",
                                  &local_time, &x, &y, &z,
                                  &roll, &pitch, &yaw);
    // clang-format on

    if (items_read == EOF) {
      if (ferror(fp)) {
        std::cerr << "[ERROR] Failed while reading pose file: " << file
                  << std::endl;
        fclose(fp);
        return false;
      }
      break;
    }

    if (items_read != 7) {
      std::cerr << "[ERROR] Malformed pose record " << " in: " << file
                << ", expected 7 fields but read " << items_read << std::endl;
      fclose(fp);
      return false;
    }

    pose.time = static_cast<double>(local_time);
    pose.pos  = Eigen::Vector3d(x, y, z);

    // clang-format off
    const Eigen::Matrix3d rotation = transform::YPR2RotationZYX(Eigen::Matrix<double, 3, 1>(yaw, pitch, roll));
    transform::RotationToQuaternion(rotation, pose.rot);
    pose.rot.normalize();
    // clang-format on

    dc_se3_pose.insert(uint64_t(pose.time), &pose);
  }
  fclose(fp);

  // clang-format off
  // 找最后一个路径分隔符（兼容 Linux / Windows）。
  const auto pos = file.find_last_of("/\\");
  const std::string parent = (pos == std::string::npos) ? "" : file.substr(0, pos);
  const std::string pose_center_path = parent.empty() ? "pose_center.txt" : parent + "/pose_center.txt";
  std::cout << "p_c_path: " << pose_center_path << std::endl;
  // clang-format on

  // 计算 pose center 用于调整偏移
  double pose_center_x = 0.0;
  double pose_center_y = 0.0;
  double pose_center_z = 0.0;

  if (common::FileExists(pose_center_path)) {
    // std::cout << "p_c_path exists." << std::endl;

    fp = fopen(pose_center_path.c_str(), "r");
    if (fp != nullptr) {
      fscanf(fp, "%lf %lf %lf", &pose_center_x, &pose_center_y, &pose_center_z);
      fclose(fp);
    }
  } else {
    std::cout << "p_c_path don't exists.. rebuild pose_center.txt" << std::endl;

    int num_of_key_frames = 0;
    double center_x = 0, center_y = 0, center_z = 0;
    // 空间位移采样距离 1 米
    double sampling_distance = 1.0;

    // TODO：last_pose default need to be MAX
    Eigen::Vector3d last_pose(-12345678, -12345678, -12345678);
    for (const auto& kv : dc_se3_pose.data()) {
      const auto& time = kv.first;
      const auto& pose = kv.second;

      // Eigen::Vector2d cur_pose(pose.x, pose.y);
      Eigen::Vector3d cur_pose = pose.pos;
      // std::cout << "pose.pos.z(): " << pose.pos.z() << std::endl;

      double distance = (cur_pose - last_pose).norm();

      if (distance >= sampling_distance) {
        center_x += cur_pose.x();
        center_y += cur_pose.y();
        center_z += cur_pose.z();
        num_of_key_frames++;
        last_pose = cur_pose;
      }
    }
    pose_center_x = center_x / double(num_of_key_frames);
    pose_center_y = center_y / double(num_of_key_frames);
    pose_center_z = center_z / double(num_of_key_frames);

    fp = fopen(pose_center_path.c_str(), "w");
    fprintf(fp, "%lf %lf %lf", pose_center_x, pose_center_y, pose_center_z);
    fclose(fp);
  }

  group_ds->pose_center =
      Eigen::Vector3d(pose_center_x, pose_center_y, pose_center_z);
  // std::cout << std::fixed << std::setprecision(10);
  // std::cout << "pose_center: " << group_ds->pose_center.transpose() << std::endl;

  return true;
}

void GroupConvertDataSet::print_pose_vec(
    const std::deque<cstruct::SE3Pose>& pose_vec) {
  for (size_t i = 0; i < pose_vec.size(); ++i) {
    const auto& pose = pose_vec[i];

    // clang-format off
    std::cout << std::fixed << std::setprecision(6) << "pose[" << i << "] "
              << "time: " << pose.time << " xyz: [" << pose.pos.transpose() << "]" << std::endl;
    // clang-format on
  }
}

}  // namespace tools
}  // namespace jojo
