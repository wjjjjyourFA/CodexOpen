#include "toolz/data_loader/group_convert.h"

namespace jojo {
namespace tools {
namespace common    = apollo::cyber::common;
namespace cstruct   = jojo::common_struct;
namespace math      = jojo::common::math;
namespace transform = jojo::common::transform;

GroupConvertDataSet::GroupConvertDataSet() {}

GroupConvertDataSet::~GroupConvertDataSet() {}

void GroupConvertDataSet::Init(std::shared_ptr<RuntimeConfigOffline> param) {
  param_ = param;

  data_loader = std::make_shared<DataLoaderDataSet>();
  data_loader->Init(param_);
  data_loader->Start();

  dc_camera.resize(param_->b_camera);
  dc_infra.resize(param_->b_infra);
  dc_star.resize(param_->b_star);
  dc_radar4d.resize(param_->b_radar4d);

  this->InitGroup();
  // std::cout << "GroupConvertDataSet Init End." << std::endl;
}

void GroupConvertDataSet::InitGroup() {
  group          = std::make_shared<MeasureGroupDataSet>();
  this->group_ds = std::static_pointer_cast<MeasureGroupDataSet>(group);

  if (param_->b_lidar) {
    std::string name = "lidar";
    dc_lidar.set_name(name);
    std::string ts_file = "timestamp/" + name + "_timestamp";
    if (!data_loader->LoadTimeStamp(data_loader->postfix, ts_file, dc_lidar)) {
      if (data_loader->ExtractTimestamp(data_loader->path_lidar, dc_lidar)) {
        data_loader->SaveTimeStamp(data_loader->postfix, ts_file, dc_lidar);
      } else {
        std::cerr << name << " timestamp load failed" << std::endl;
      }
    }
    dc_lidar.init_ts(param_->start_time);
    // update the start time
    param_->start_time = dc_lidar.cur_time;

    // clang-format off
    group_ds->lidar.data = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
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
      if (!data_loader->LoadTimeStamp(data_loader->postfix, ts_file, dc_camera.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_camera.at(i), dc_camera.at(i))) {
          data_loader->SaveTimeStamp(data_loader->postfix, ts_file, dc_camera.at(i));
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
    // std::cout << "camera size: " << group->camera.size() << std::endl;
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
      if (!data_loader->LoadTimeStamp(data_loader->postfix, ts_file, dc_infra.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_camera.at(i), dc_infra.at(i))) {
          data_loader->SaveTimeStamp(data_loader->postfix, ts_file, dc_infra.at(i));
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
      if (!data_loader->LoadTimeStamp(data_loader->postfix, ts_file, dc_star.at(i))) {
        if (data_loader->ExtractTimestamp(data_loader->path_camera.at(i), dc_star.at(i))) {
          data_loader->SaveTimeStamp(data_loader->postfix, ts_file, dc_star.at(i));
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
    dc_se3_pose.set_name("global_pose");
    // LoadPose(data_loader->postfix, dc_se3_pose.name);
    LoadPose(data_loader->path_global_pose);
  } else if (param_->b_local_pose) {
    dc_se3_pose.set_name("local_pose");
    // LoadPose(data_loader->postfix, dc_se3_pose.name);
    LoadPose(data_loader->path_local_pose);
  }
}

std::shared_ptr<const MeasureGroupBase> GroupConvertDataSet::ReadNext() {
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
    if (!this->GetLidarBase<pcl::PointXYZI>(dc_lidar, group_ds->lidar.data,
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

  if (param_->b_local_pose || param_->b_global_pose) {
    // std::cout << "GroupConvert ReadNext. Pose SE3." << std::endl;
    dc_se3_pose.align_ts(last_ts);
    if (!this->GetDataBase(&dc_se3_pose, group_ds->se3_pose.data,
                           group_ds->se3_pose.time)) {
      std::cerr << "[Warning] Failed to load se3_pose : "
                << ", skipping frame." << std::endl;
      is_running_ = false;
    }

    if (param_->b_pose_vec) {
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
  char file[300];
  sprintf(file, "%s/%s.txt", path.c_str(), data_file.c_str());

  return this->LoadImuData(std::string(file));
}

bool GroupConvertDataSet::LoadPose(const std::string& file) {
  // std::cout << "LoadPose: " << file << std::endl;

  // 这里将外部数据转换为这个接口
  cstruct::SE3Pose pose;

  if (!common::FileExists(file)) {
    std::cerr << "[ERROR] Failed to load file: " << file << std::endl;
    return false;
  }

  FILE* _fp = NULL;
  _fp       = fopen(file.c_str(), "r");
  if (_fp != NULL) {
    while (!feof(_fp)) {
      // 优化后的位姿，时间戳使用的整型
      int64_t local_time;
      double x, y, z, roll, pitch, yaw;
      // clang-format off
      int itemsRead = fscanf(_fp, "%" SCNd64 " %lf %lf %lf %lf %lf %lf",
                             &local_time, &x, &y, &z, &roll, &pitch, &yaw);
      // clang-format on
      pose.time = static_cast<double>(local_time);
      pose.pos.x() = x;
      pose.pos.y() = y;
      pose.pos.z() = z;
      // std::cout << "pose.pos.z(): " << pose.pos.z() << std::endl;

      // clang-format off
      Eigen::Matrix3d R = transform::YPR2RotationZYX(Eigen::Matrix<double, 3, 1>(yaw, pitch, roll));
      transform::RotationToQuaternion(R, pose.rot);
      // clang-format on

      if (itemsRead != 7) break;

      dc_se3_pose.insert(uint64_t(pose.time), &pose);
    }
    fclose(_fp);
  }

  // clang-format off
  // 找最后一个路径分隔符（兼容 Linux / Windows）
  auto pos = file.find_last_of("/\\");
  std::string parent = (pos == std::string::npos) ? "" : file.substr(0, pos);
  const std::string p_c_path = parent + "/pose_center" + ".txt";
  // std::cout << "p_c_path: " << p_c_path << std::endl;
  // clang-format on

  // 计算 pose center 用于调整偏移
  double pose_center_x = 0, pose_center_y = 0, pose_center_z = 0;

  if (common::FileExists(p_c_path)) {
    std::cout << "p_c_path exists." << std::endl;

    _fp = fopen(p_c_path.c_str(), "r");
    fscanf(_fp, "%lf %lf %lf", &pose_center_x, &pose_center_y, &pose_center_z);
    fclose(_fp);
  } else {
    std::cout << "p_c_path don't exists." << std::endl;

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

    _fp = fopen(p_c_path.c_str(), "w");
    fprintf(_fp, "%lf %lf %lf", pose_center_x, pose_center_y, pose_center_z);
    fclose(_fp);
  }

  group_ds->pose_center =
      Eigen::Vector3d(pose_center_x, pose_center_y, pose_center_z);
  // std::cout << std::fixed << std::setprecision(10);
  // std::cout << "pose_center: " << group_ds->pose_center.transpose() << std::endl;

  return false;
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
