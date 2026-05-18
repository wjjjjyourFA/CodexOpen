#include "modules/mapping/mapper/mapper.h"

namespace jojo {
namespace mapping {

Mapper::Mapper() {
  world_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  world_point_cloud->points.resize(point_memory_size);

  ds_world_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  ds_history_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

Mapper::~Mapper() {}

void Mapper::Init(std::shared_ptr<jojo::mapping::RuntimeConfig> rparam,
                  std::shared_ptr<jojo::mapping::StaticConfig> sparam) {
  rparam_ = rparam;
  sparam_ = sparam;

  hps_.map_resolution    = rparam_->map_resolution;
  hps_.sampling_distance = rparam_->sampling_distance;
  std::cout << "map resolution: " << hps_.map_resolution << std::endl;
  std::cout << "sampling distance: " << hps_.sampling_distance << std::endl;
}

void Mapper::InitViewer() {
  if (vis_inited_) return;

  vis_.reset(new pcl::visualization::PCLVisualizer("Map Viewer"));
  vis_->setBackgroundColor(0, 0, 0);
  vis_->initCameraParameters();
  vis_->setCameraPosition(-50, -50, 200,  // 相机位置（原点上方）
                          0, 0, 0,  // 看向原点
                          0, 1, 0  // up方向
  );
  vis_inited_ = true;

  std::cout << "init map viewer" << std::endl;
}

void Mapper::SetPoseCenter(const Eigen::Vector3d& p_center) {
  pose_center = p_center;
}

void Mapper::Reset() {
  ds_history_point_cloud->clear();
  ds_world_point_cloud->clear();

  world_point_cloud->clear();
  world_point_cloud->points.resize(point_memory_size);
  world_point_cloud_idx = 0;
}

void Mapper::Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame,
                 const Eigen::Matrix4f& in_pose) {
  // !! 这里能直接位姿变换+累积建图，就是因为 pose 对应的是 frame 的优化位姿
  std::cout << "frame size: " << frame->points.size() << std::endl;

  // ==> pose_center
  Eigen::Matrix4f curr_pose = in_pose;
  if (sparam_->b_use_pose_center) {
    curr_pose.block<3, 1>(0, 3) -= pose_center.cast<float>();
  }

  // 依据 位移量 判断是否需要更新地图
  double distance =
      (curr_pose.block<3, 1>(0, 3) - last_pose.block<3, 1>(0, 3)).norm();

  // 依据 旋转量 判断是否需要更新地图
  Eigen::Matrix3f R_last = last_pose.block<3, 3>(0, 0);
  Eigen::Matrix3f R_curr = curr_pose.block<3, 3>(0, 0);
  Eigen::Matrix3f R_diff = R_last.transpose() * R_curr;
  // rad
  float angle = Eigen::AngleAxisf(R_diff).angle();

  /* 仅按距离过滤
  if (distance < hps_.sampling_distance) {
    ignore_count++;
    return;
  }
  */
  // 阈值判断（两个条件都小才跳过）
  if (distance < hps_.sampling_distance && angle < hps_.sampling_angle_rad) {
    ignore_count++;
    return;
  }

  last_pose = curr_pose;

  if (rparam_->b_realtime_show) {
    static bool vis_frame_init = false;
    if (!vis_frame_init) {
      vis_frame_init = true;
      vis_frame.reset(new pcl::PointCloud<pcl::PointXYZI>);
      vis_frame->points.reserve(frame->points.size());
    }
    vis_frame->clear();
  }

  // 先把 4x4 pose 拆成 R / T（避免每点做4x4乘法）
  const Eigen::Matrix3f R = curr_pose.block<3, 3>(0, 0);
  const Eigen::Vector3f T = curr_pose.block<3, 1>(0, 3);

  // frame ==> map coords
  for (size_t i = 0; i < frame->points.size(); i++) {
    const auto& point = frame->points[i];

    // if (isnan(point.x)) continue;
    if (!pcl::isFinite(point)) continue;

    // 放进寄存器,减少结构体访问开销
    const float x = point.x;
    const float y = point.y;
    const float z = point.z;

    // do not process points beyond 100m range
    if (x * x + y * y > 10000.0) continue;

    // 使用索引直接写入
    auto& pt = world_point_cloud->points[world_point_cloud_idx];
    world_point_cloud_idx++;

    pt.x = R(0, 0) * x + R(0, 1) * y + R(0, 2) * z + T(0);
    pt.y = R(1, 0) * x + R(1, 1) * y + R(1, 2) * z + T(1);
    pt.z = R(2, 0) * x + R(2, 1) * y + R(2, 2) * z + T(2);

    pt.intensity = point.intensity;

    // 安全写入
    // world_point_cloud->points.emplace_back(pt);

    if (rparam_->b_realtime_show) {
      vis_frame->points.emplace_back(pt);
    }
  }

  // 当缓存的点云快满时，把当前批次点云做一次体素下采样压缩，然后合并到“大地图”，并清空缓存继续累积。
  // 200000 is enough to store one scan
  if (world_point_cloud_idx > point_memory_size - frame_memory_size) {
    this->FlushBufferToMap();
  }
  // 最后一定会剩下一部分点（不满 buffer）

  if (rparam_->b_realtime_show) {
    this->RealTimeShow(vis_frame);
  }
}

void Mapper::UpdateIncrementalMap() {
  std::cout << "ignore_count: " << ignore_count << std::endl;
  std::cout << "UpdateIncrementalMap ... " << std::endl;
  std::cout << world_point_cloud->size() << " points!" << std::endl;
  std::cout << world_point_cloud_idx << " points set idx!" << std::endl;

  // 补交最后一批数据
  this->FlushBufferToMap();

  // 对“整个历史地图”再做一次降采样
  VoxelizeLargeScalePreserveLabel(ds_history_point_cloud, ds_world_point_cloud,
                                  hps_.map_resolution);

  // way 1 深拷贝
  // *ds_history_point_cloud = *ds_world_point_cloud;
  // way 2 swap（零拷贝 + 不共享内存）
  ds_history_point_cloud.swap(ds_world_point_cloud);

  std::cout << "UpdateIncrementalMap ... DONE " << std::endl;
}

void Mapper::FlushBufferToMap() {
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

void Mapper::SaveMap(const std::string& path) {
  // std::cout << ds_history_point_cloud->size() << " points!" << std::endl;

  std::string path_map = path + "/map3d/map_point_cloud.pcd";
  pcl::io::savePCDFileBinary(path_map, *ds_history_point_cloud);
}

void Mapper::VisualizeMap(bool b_pause) {
  std::cout << "VisualizeMap ... " << std::endl;
  this->InitViewer();

  // clang-format off
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> map_color(world_point_cloud, 128, 128, 128);
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> map_color(map_, 0, 255, 0);
  // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> map_color(world_point_cloud, "z");
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

void Mapper::RealTimeShow(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in) {
  this->InitViewer();

  // 接管数据
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  cloud.swap(cloud_in);

  std::string name = "map_" + std::to_string(vis_chunk_id_++);

  // clang-format off
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color(cloud, "z");
  // clang-format on

  vis_->addPointCloud<pcl::PointXYZI>(cloud, color, name);
  vis_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "map");

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