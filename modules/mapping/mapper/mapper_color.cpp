#include "modules/mapping/mapper/mapper_color.h"

namespace jojo {
namespace mapping {
using namespace jojo::perception::camera;
namespace cstruct = jojo::common_struct;

MapperColor::MapperColor() {
  world_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  world_point_cloud->points.resize(point_memory_size);

  ds_world_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  ds_history_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

MapperColor::~MapperColor() {}

void MapperColor::InitCameraParams() {
  auto camera_params = std::make_shared<CameraParams>();
  camera_params->LoadFromFile(rparam_->camera_calib_file_path);
  matrix = camera_params->GetMatrixVector().at(0);
  // std::cout << "camera_params: \n" << matrix->extrinsic_matrix << std::endl;
}

void MapperColor::Reset() {
  ds_history_point_cloud->clear();
  ds_world_point_cloud->clear();

  world_point_cloud->clear();
  world_point_cloud->points.resize(point_memory_size);
  world_point_cloud_idx = 0;
}

/* pipeline：使用图像数据，生成彩色点云地图
  1. 利用 image 和 lidar 的时间戳，在对应的 pose 中插值：
    a. 找到最近的 pose->image
    b. 对于 pose->lidar，输入已经是位姿优化后的 pose，就是 lidar_pose
  2. 计算 pose ==> image->lidar 的变换矩阵
  3. 在静态标定的外参数下，计算新的投影矩阵
  4. 获取 color frame
  5. create color map
*/
// TODO：添加多视角相机，以及多线程支持
void MapperColor::Run(
    std::shared_ptr<const jojo::tools::MeasureGroupDataSet> Measures) {
  // !! 对于 硬触发相机，图像数据和点云数据已经是同步了的，不需要再插值
  // TODO：使用一个统一的数据结构，这样就不用存储多份 map；
  // 对比原生 pcl::PointXYZI，增添 RGB 字段，牺牲一些内存，但节省了计算量

  const auto& frame   = Measures->lidar.data;
  const auto& in_pose = Measures->se3_pose.data.matrix();
  // color 数据
  const auto& image      = Measures->camera.at(0).data;
  const auto& image_time = Measures->camera.at(0).time;
  // pose vector 数据
  const auto& v_pose = Measures->se3_pose_vec;
  // std::cout << "frame size: " << frame->size() << std::endl;
  // std::cout << "v_pose size: " << v_pose.size() << std::endl;

  Eigen::Matrix4f cur_pose = in_pose;
  if (sparam_->b_use_pose_center) {
    cur_pose.block<3, 1>(0, 3) -= pose_center.cast<float>();
  }

  double distance =
      (cur_pose.block<3, 1>(0, 3) - last_pose.block<3, 1>(0, 3)).norm();

  Eigen::Matrix3f R_last = last_pose.block<3, 3>(0, 0);
  Eigen::Matrix3f R_curr = cur_pose.block<3, 3>(0, 0);
  Eigen::Matrix3f R_diff = R_last.transpose() * R_curr;
  // rad
  float angle = Eigen::AngleAxisf(R_diff).angle();

  if (distance < hps_.sampling_distance && angle < hps_.sampling_angle_rad) {
    ignore_count++;
    return;
  }

  last_pose = cur_pose;
  // std::cout << "in_pose: \n" << in_pose << std::endl;
  // std::cout << "cur_pose: \n" << cur_pose << std::endl;

  if (rparam_->b_realtime_show) {
    static bool vis_frame_init = false;
    if (!vis_frame_init) {
      vis_frame_init = true;
      vis_frame.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
      vis_frame->points.reserve(frame->points.size());
    }
    vis_frame->clear();
  }

  Eigen::Matrix4f projection_matrix = Eigen::Matrix4f::Identity();
  if (rparam_->b_use_time_interpolate) {
    // std::cout << "pipeline: trans lidar pose to image pose ..." << std::endl;

    // image 数据，插值后的 pose
    Eigen::Matrix4d pose_out = Eigen::Matrix4d::Identity();  
    for (auto it = v_pose.begin(); it < (v_pose.end() - 1); it++) {
      auto&& head = *(it);  // 拿到当前帧的pose数据
      auto&& tail = *(it + 1);  // 拿到下一帧的pose数据
      // std::cout << "head.time: " << head.time << " image_time: " << image_time
      //           << " tail.time: " << tail.time << std::endl;

      // 直到 head.time < image_time < tail.time
      if (head.time <= image_time && image_time <= tail.time) {
        if (!this->InterpolatePose(image_time, head, tail, pose_out)) {
          return;
        }

        // 已经找到了对应的 pose time
        break;
      }
    }
    // std::cout << "world pose_out: \n" << pose_out << std::endl;
    if (sparam_->b_use_pose_center) {
      pose_out.block<3, 1>(0, 3) -= pose_center.cast<double>();
    }
    // std::cout << "pose_out: \n" << pose_out << std::endl;

    // 计算新的 变换矩阵、投影矩阵
    /* 参数解释：
    cur_pose: lidar_pose to world
    pose_out: image_pose to world
    lidar2image 外参: matrix.extrinsic_matrix RT
    image 内参: matrix.camera_matrix->intrinsic_matrix K
    */
    // lidar pose ==> image pose
    Eigen::Matrix4f T_delta = pose_out.inverse().cast<float>() * cur_pose;
    // std::cout << "T_delta: \n" << T_delta << std::endl;

    // lidar to cur image
    Eigen::Matrix4f RT_cam_lidar = matrix->extrinsic_matrix * T_delta;

    // 投影矩阵
    projection_matrix.block<3, 4>(0, 0) =
        matrix->camera_matrix->intrinsic_matrix *
        RT_cam_lidar.block<3, 4>(0, 0);
    // std::cout << "projection_matrix: \n" << projection_matrix << std::endl;
  } else {
    projection_matrix = matrix->projection_matrix;
  }

  // 使用 map 内存进行优化，避免多次 frame 拷贝；因此手动实现投影过程，不调用 fusion 模块
  const Eigen::Matrix3f R = cur_pose.block<3, 3>(0, 0);
  const Eigen::Vector3f T = cur_pose.block<3, 1>(0, 3);

  // TODO：使用 未去畸变的图像，实现这一过程，增大 FOV
  // 转换为图像范围点并过滤
  for (int i = 0; i < frame->points.size(); ++i) {
    const auto& point = frame->points[i];

    // if (isnan(point.x)) continue;
    if (!pcl::isFinite(point)) continue;

    const float x = point.x;
    const float y = point.y;
    const float z = point.z;

    if (x * x + y * y > 10000.0) continue;

    Eigen::Vector4f point_homo(x, y, z, 1.0f);

    // clang-format off
    Eigen::Vector3f projected_point = projection_matrix.block<3, 4>(0, 0) * point_homo;
    // clang-format on

    const float z_proj = projected_point(2);
    if (z_proj <= 0) {
      continue;  // Skip points behind the camera
    }

    int u = static_cast<int>(projected_point(0) / z_proj);
    int v = static_cast<int>(projected_point(1) / z_proj);

    if (u >= 0 && v >= 0 && u < image.cols && v < image.rows) {
      const cv::Vec3b& pix = image.at<cv::Vec3b>(v, u);

      auto& pt = world_point_cloud->points[world_point_cloud_idx];
      world_point_cloud_idx++;

      pt.x = R(0, 0) * x + R(0, 1) * y + R(0, 2) * z + T(0);
      pt.y = R(1, 0) * x + R(1, 1) * y + R(1, 2) * z + T(1);
      pt.z = R(2, 0) * x + R(2, 1) * y + R(2, 2) * z + T(2);

      // pt.intensity = point.intensity;

      pt.b = pix[0];
      pt.g = pix[1];
      pt.r = pix[2];

      if (rparam_->b_realtime_show) {
        vis_frame->points.emplace_back(pt);
      }
    }
  }

  if (world_point_cloud_idx > point_memory_size - frame_memory_size) {
    this->FlushBufferToMap();
  }

  if (rparam_->b_realtime_show) {
    this->RealTimeShow(vis_frame);
  }
}

bool MapperColor::InterpolatePose(double timestamp,
                                  const cstruct::SE3Pose& pose1,
                                  const cstruct::SE3Pose& pose2,
                                  Eigen::Matrix4d& pose_out) {
  if (pose2.time <= pose1.time) {
    std::cerr << "[InterpolatePose] invalid pose time range." << std::endl;

    pose_out.setIdentity();
    pose_out.block<3, 1>(0, 3) = pose1.pos;
    pose_out.block<3, 3>(0, 0) = pose1.rot.toRotationMatrix();
    return false;
  }

  double ratio = 0.0;  // 插值因子 ∈ (0, 1)
  double dt    = pose2.time - pose1.time;
  if (dt > 1e-6) {
    double t = (timestamp - pose1.time) / dt;
    ratio    = apollo::common::math::Clamp(t, 0.0, 1.0);
  }

  // 平移线性插值
  Eigen::Vector3d interp_pos = pose1.pos + ratio * (pose2.pos - pose1.pos);
  // 旋转球面插值
  Eigen::Quaterniond q1 = pose1.rot;
  Eigen::Quaterniond q2 = pose2.rot;
  q1.normalize();
  q2.normalize();
  if (q1.dot(q2) < 0.0) {
    q2.coeffs() *= -1.0;
  }
  Eigen::Quaterniond interp_rot = q1.slerp(ratio, q2);
  interp_rot.normalize();

  pose_out.setIdentity();
  pose_out.block<3, 1>(0, 3) = interp_pos;
  pose_out.block<3, 3>(0, 0) = interp_rot.toRotationMatrix();

  return true;
}

void MapperColor::UpdateIncrementalMap() {
  std::cout << "ignore_count: " << ignore_count << std::endl;
  std::cout << "UpdateIncrementalMap ... " << std::endl;
  std::cout << world_point_cloud->size() << " points!" << std::endl;
  std::cout << world_point_cloud_idx << " points set idx!" << std::endl;

  // 补交最后一批数据
  this->FlushBufferToMap();

  // 对“整个历史地图”再做一次降采样
  VoxelizeLargeScalePreserveLabel(ds_history_point_cloud, ds_world_point_cloud,
                                  hps_.map_resolution);

  // way 2 swap（零拷贝 + 不共享内存）
  ds_history_point_cloud.swap(ds_world_point_cloud);

  std::cout << "UpdateIncrementalMap ... DONE " << std::endl;
}

void MapperColor::FlushBufferToMap() {
  if (world_point_cloud_idx == 0) return;

  // 截断有效数据，只保留真正写入的点
  world_point_cloud->points.resize(world_point_cloud_idx);
  std::cout << "batch map: " << world_point_cloud->size() << " points resized!"
            << std::endl;

  // 体素下采样（压缩地图）
  VoxelizeLargeScalePreserveLabel(world_point_cloud, ds_world_point_cloud,
                                  hps_.map_resolution);

  // 把这次 voxel 结果 加入 “全局地图”
  *ds_history_point_cloud += *ds_world_point_cloud;
  std::cout << "history_map: " << ds_history_point_cloud->size() << " points!"
            << std::endl;

  // 缓存重新开始写
  world_point_cloud_idx = 0;
  // 恢复预分配大小，旧数据未删除；后续写入时直接覆盖旧数据；
  world_point_cloud->points.resize(point_memory_size);
}

void MapperColor::SaveMap(const std::string& path) {
  // std::cout << ds_history_point_cloud->size() << " points!" << std::endl;

  std::string path_c_map = path + "/map3d/colorized_map_point_cloud.pcd";
  pcl::io::savePCDFileBinary(path_c_map, *ds_history_point_cloud);
}

void MapperColor::VisualizeMap(bool b_pause) {
  this->InitViewer();

  // clang-format off
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> map_color(world_point_cloud);
  // clang-format on

  if (!vis_->contains("map")) {
    vis_->addPointCloud(ds_history_point_cloud, map_color, "map");
    vis_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map");
  } else {
    vis_->updatePointCloud(ds_history_point_cloud, map_color, "map");
  }

  if (b_pause) {
    vis_->spin();
  } else {
    vis_->spinOnce(1);
  }
}

void MapperColor::RealTimeShow(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in) {
  this->InitViewer();

  // 接管数据
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  cloud.swap(cloud_in);

  std::string name = "map_" + std::to_string(vis_chunk_id_++);

  // clang-format off
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloud);

  vis_->addPointCloud<pcl::PointXYZRGB>(cloud, color, name);
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "map");
  // clang-format on

  vis_chunks_.push_back(cloud);

  // 控制chunk数量
  const int MAX_CHUNKS = 200;
  if (vis_chunks_.size() > MAX_CHUNKS) {
    // clang-format off
    int remove_id = vis_chunk_id_ - MAX_CHUNKS - 1;
    std::string remove_name = "map_" + std::to_string(remove_id);
    vis_->removePointCloud(remove_name);
    vis_chunks_.erase(vis_chunks_.begin());
    // clang-format on
  }

  vis_->spinOnce(1);
}

}  // namespace mapping
}  // namespace jojo