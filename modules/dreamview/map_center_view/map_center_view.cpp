#include "modules/dreamview/map_center_view/map_center_view.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace jojo {
namespace dreamview {

MapCenterView::MapCenterView() {
  inv_dist = 1.0 / this->roi_radius;

  if (!map_roi) {
    map_roi.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
  if (!trajectory_) {
    trajectory_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  }
};

void MapCenterView::Init(
    std::shared_ptr<jojo::dreamview::StaticConfig> sparam) {
  sparam_ = sparam;

  if (sparam_) {
    roi_radius      = std::max(1.0f, sparam_->roi_radius);
    roi_update_dist = std::max(1.0f, sparam_->roi_update_dist);
    inv_dist        = 1.0f / roi_radius;
  }

  this->LoadInitMap(sparam_->map_file_path);
}

void MapCenterView::BuildDisplayMap() {
  if (!map_ || map_->empty()) {
    map_display_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    return;
  }

  const float leaf_size = sparam_ ? sparam_->map_display_voxel_size : 0.30f;
  if (leaf_size <= 0.0f) {
    map_display_ = map_;
    std::cout << "[Map] Display voxel filter disabled, rendering "
              << map_display_->size() << " points." << std::endl;
    return;
  }

  // 大多数 PCD 是 dense 点云，直接复用原始地图，避免额外复制。
  // 仅在包含无效点时创建临时清理结果，不修改 map_。
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr voxel_input = map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr valid_map;
  if (!map_->is_dense) {
    valid_map.reset(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*map_, *valid_map, valid_indices);
    voxel_input = valid_map;
  }

  map_display_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  // 标准 VoxelGrid 使用 int32 线性体素索引。超大范围地图即使实际
  // 点数不密集，也可能因为理论网格数量过大而溢出并直接返回原点云。
  // ApproximateVoxelGrid 使用哈希历史表，更适合这里只用于渲染的地图。
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(voxel_input);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

  const double start = omp_get_wtime();
  voxel_filter.filter(*map_display_);
  const double end = omp_get_wtime();

  const double keep_ratio = 100.0 * static_cast<double>(map_display_->size()) /
                            static_cast<double>(map_->size());
  std::cout << "[Map] Approximate display voxel size=" << leaf_size
            << "m, points=" << map_->size() << " -> " << map_display_->size()
            << " (" << keep_ratio << "%), cost=" << (end - start) * 1000.0
            << "ms" << std::endl;
}

void MapCenterView::InitKDTree() {
  if (!kdtree_built && map_display_ && !map_display_->empty()) {
    kdtree.setInputCloud(map_display_);
    kdtree_built = true;
  }
}

void MapCenterView::InitViewer() {
  if (vis_inited_) return;

  vis_.reset(new pcl::visualization::PCLVisualizer("Map Viewer"));
  vis_->setBackgroundColor(0, 0, 0);
  vis_->initCameraParameters();
  vis_->setCameraPosition(-50, -50, 100,  // 相机位置（原点上方）
                          0, 0, 0,  // 看向原点
                          0, 1, 0  // up方向
  );
  vis_inited_ = true;
}

void MapCenterView::LoadInitMap(const std::string& map_path) {
  // “变量命名：数据类型在前，作用/用途在后

  // 一般构建的高精地图是这个格式
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  double load_start = omp_get_wtime();
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *raw_cloud) == -1) {
    std::cerr << "Couldn't read map file from " << map_path;
    abort();
  }
  double load_end = omp_get_wtime();

  std::cout << "Loaded map " << raw_cloud->points.size()
            << " points from " + map_path << std::endl;
  std::cout << "Load map cost: " << (load_end - load_start) * 1000 << "ms"
            << std::endl;

  this->SetInitMap(raw_cloud);
}

void MapCenterView::SetInitMap(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& map) {
  // 指针持有初始地图，避免地图生命周期内释放
  // 深拷贝（deep copy）
  // this->map_.reset(new PointCloudXYZI);
  // this->*map_ = *map;
  // 浅拷贝（shared_ptr 共享）
  this->map_ = map;
  std::cout << "map size: " << map->points.size() << std::endl;

  // 原始地图保留完整精度；可视化使用一次性降采样地图，减少每帧绘制量。
  this->BuildDisplayMap();

  // 全局 map
  // this->InitKDTree();
  crop_box.setInputCloud(map_display_);

  this->InitViewer();

  // ROI 显示模式：完整地图仍保留在 CPU 内存中用于动态裁剪，
  // 但不提交给可视化器；界面只渲染当前位置附近的局部地图。
  if (sparam_ && sparam_->b_display_roi) {
    if (vis_->contains("map")) {
      vis_->removePointCloud("map");
    }
    return;
  }

  // clang-format off
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> map_color(map_display_, 128, 128, 128);
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> map_color(map_, 0, 255, 0);
  // clang-format on

  if (!vis_->contains("map")) {
    vis_->addPointCloud(map_display_, map_color, "map");
    vis_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map");
  } else {
    vis_->updatePointCloud(map_display_, map_color, "map");
  }
}

void MapCenterView::SetMapCenter(const Eigen::Vector3d& center) {
  map_center = center;
}

void MapCenterView::SetPoseCenter(const Eigen::Vector3d& p_center) {
  pose_center = p_center;
  // TODO：check pose_center is inside map
  // std::cout << std::fixed << std::setprecision(10);
  // std::cout << "pose_center: " << pose_center.transpose() << std::endl;

  /* 触发 map 变换
  if (sparam_->b_use_pose_center) {
    Eigen::Matrix4f p_pose = Eigen::Matrix4f::Identity();
    // p_pose(0, 3) -= static_cast<double>(pose_center.x());
    // p_pose(1, 3) -= static_cast<double>(pose_center.y());
    // p_pose(2, 3) -= static_cast<double>(pose_center.z());
    p_pose.block<3, 1>(0, 3) -= pose_center.cast<float>();

    // UAV 地图 loc 的结果 ==> 出来的 pose 是基于该 map 的
    // 使用 lidar 建图 ==> 出来的 map 是基于该 pose - pose_center 的
    pcl::transformPointCloud(*map_, *map_, p_pose);
  }
  */
}

void MapCenterView::ShowFrame(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame,
                              const Eigen::Matrix4f& in_pose) {
  // global ==> local
  Eigen::Matrix4f pose = in_pose;
  if (sparam_->b_use_pose_center) {
    // pose(0, 3) -= static_cast<float>(pose_center.x());
    // pose(1, 3) -= static_cast<float>(pose_center.y());
    // pose(2, 3) -= static_cast<float>(pose_center.z());
    pose.block<3, 1>(0, 3) -= pose_center.cast<float>();
  }

  // 1. transform
  if (!frame_world) {
    frame_world.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
  frame_world->clear();
  frame_world->points.reserve(frame->size());
  pcl::transformPointCloud(*frame, *frame_world, pose);

  /* 2. color
  vis_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  vis_cloud->points.reserve(frame_world->points.size());

  float min_i = std::numeric_limits<float>::max();
  float max_i = std::numeric_limits<float>::lowest();

  for (const auto& pt : frame_world->points) {
    min_i = std::min(min_i, pt.intensity);
    max_i = std::max(max_i, pt.intensity);
  }

  float inv_range = 1.0f / std::max(1e-6f, max_i - min_i);

  for (const auto& pt : frame_world->points) {
    pcl::PointXYZRGB p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;

    // way 1
    // float z_norm = std::min(1.0f, std::max(0.0f, (pt.z + 3.0f) / 6.0f));
    // p.r = 255 * z_norm;
    // p.g = 0;
    // p.b = 255 * (1.0f - z_norm);

    // way 2
    int idx = std::min(
        static_cast<int>((p.intensity - min_i) * inv_range * 640.0f), 639);
    p.r = jet_color_map[idx][0];
    p.g = jet_color_map[idx][1];
    p.b = jet_color_map[idx][2];

    // vis_cloud->push_back(p);
    vis_cloud->emplace_back(p);
  }
  */

  if (!intensity_handler) {
    intensity_handler.reset(
        new pcl::visualization::PointCloudColorHandlerGenericField<
            pcl::PointXYZI>(frame_world, "intensity"));
  }

  // 3. update frame
  if (!vis_->contains("frame")) {
    // vis_->addPointCloud(vis_cloud, "frame");
    vis_->addPointCloud<pcl::PointXYZI>(frame_world, *intensity_handler,
                                        "frame");
    vis_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "frame");
  } else {
    // vis_->updatePointCloud(vis_cloud, "frame");
    vis_->updatePointCloud<pcl::PointXYZI>(frame_world, *intensity_handler,
                                           "frame");
  }

  // 4. traj
  UpdateTrajectory(pose);

  vis_->spinOnce(1);
  // vis_->spin();
}

void MapCenterView::UpdateMapROI(const Eigen::Matrix4f& pose) {
  if (!map_display_ || map_display_->empty()) {
    std::cerr << "[ROI] Cannot update ROI: the display map is empty."
              << std::endl;
    return;
  }

  Eigen::Vector3f center = pose.block<3, 1>(0, 3);

  if (!NeedUpdateROI(center)) {
    return;
  }

  // ROI 只限制 XY 范围，Z 不裁剪；
  // 避免地图海拔与 pose.z 不一致时，将有效底图全部过滤掉。
  Eigen::Vector4f min_pt(center.x() - roi_radius, center.y() - roi_radius,
                         std::numeric_limits<float>::lowest(), 1.0f);
  Eigen::Vector4f max_pt(center.x() + roi_radius, center.y() + roi_radius,
                         std::numeric_limits<float>::max(), 1.0f);

  crop_box.setMin(min_pt);
  crop_box.setMax(max_pt);

  const double crop_start = omp_get_wtime();
  // 1. CropBox 先快速取出圆形 ROI 的外接正方形
  crop_box.filter(*map_roi);
  const std::size_t box_points = map_roi->size();

  // 2. 再按 XY 距离原地压缩，
  // 最终显示范围是半径 roi_radius 的圆（Z 方向不限制，即竖直圆柱）。
  /*
  const float radius_sq   = roi_radius * roi_radius;
  std::size_t write_index = 0;
  for (std::size_t i = 0; i < map_roi->points.size(); ++i) {
    const auto& point = map_roi->points[i];
    const float dx    = point.x - center.x();
    const float dy    = point.y - center.y();
    if (dx * dx + dy * dy <= radius_sq) {
      if (write_index != i) {
        map_roi->points[write_index] = point;
      }
      ++write_index;
    }
  }
  map_roi->points.resize(write_index);
  map_roi->width  = static_cast<std::uint32_t>(write_index);
  map_roi->height = 1;
  */
  const double crop_end = omp_get_wtime();

  std::cout << "[ROI] circle center=" << center.transpose()
            << ", radius=" << roi_radius << "m, source=" << map_display_->size()
            << ", crop_cost=" << (crop_end - crop_start) * 1000.0 << "ms"
            << std::endl;

  if (map_roi->empty()) {
    // 防止之前成功显示的 ROI 在本次裁剪为空后仍残留在界面中。
    if (vis_->contains("map_roi")) {
      vis_->removePointCloud("map_roi");
    }
    std::cerr << "[ROI] No map points found around the current pose. Check "
                 "whether map and pose use the same coordinate system, and "
                 "whether b_use_pose_center is configured correctly."
              << std::endl;
    return;
  }

  // clang-format off
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> map_color(map_roi, 150, 150, 150);
  // clang-format on

  bool render_ok = false;
  if (!vis_->contains("map_roi")) {
    render_ok = vis_->addPointCloud(map_roi, map_color, "map_roi");
    vis_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_roi");
  } else {
    render_ok = vis_->updatePointCloud(map_roi, map_color, "map_roi");
  }

  if (!render_ok) {
    std::cerr << "[ROI] Failed to add/update map_roi in PCLVisualizer."
              << std::endl;
  }
}

void MapCenterView::UpdateMapROIKdTree(const Eigen::Matrix4f& pose) {
  Eigen::Vector3f center = pose.block<3, 1>(0, 3);

  // 直接复用 map_roi
  if (!NeedUpdateROI(center)) {
    return;
  }

  pcl::PointXYZI search_pt;
  search_pt.x = center.x();
  search_pt.y = center.y();
  search_pt.z = center.z();

  std::vector<int> indices;
  std::vector<float> dists;

  kdtree.radiusSearch(search_pt, roi_radius, indices, dists);

  map_roi->clear();
  // map_roi->points.reserve(indices.size());
  map_roi->points.resize(indices.size());

  /* way 1
  for (int idx : indices) {
    map_roi->push_back(map_->points[idx]);
  }
  */
  for (size_t i = 0; i < indices.size(); ++i) {
    map_roi->points[i] = map_display_->points[indices[i]];
  }
  /* way 2
  pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);
  indices_ptr->indices = indices;
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(map_);
  extract.setIndices(indices_ptr);
  extract.setNegative(false);
  extract.filter(*map_roi);
  */

  map_roi->width  = map_roi->points.size();
  map_roi->height = 1;

  // clang-format off
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> map_color(map_roi, 150, 150, 150);
  // clang-format on

  if (!vis_->contains("map_roi")) {
    vis_->addPointCloud(map_roi, map_color, "map_roi");
    vis_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_roi");
  } else {
    vis_->updatePointCloud(map_roi, map_color, "map_roi");
  }
}

void MapCenterView::ShowFrameROI(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame,
    const Eigen::Matrix4f& in_pose) {
  // global ==> local
  Eigen::Matrix4f pose = in_pose;
  if (sparam_->b_use_pose_center) {
    pose.block<3, 1>(0, 3) -= pose_center.cast<float>();
    // std::cout << pose.block<3, 1>(0, 3).transpose() << std::endl;
  }

  // ---------- 1. 更新 ROI ----------
  // UpdateMapROIKdTree(pose);
  UpdateMapROI(pose);

  // ---------- 3. transform frame ----------
  if (!frame_world) {
    frame_world.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
  frame_world->clear();
  // frame_world->points.reserve(frame->size());
  // pcl::transformPointCloud(*frame, *frame_world, pose);

  frame_world->points.resize(frame->size());
#pragma omp parallel for
  for (int i = 0; i < frame->size(); ++i) {
    const auto& src = frame->points[i];
    auto& dst       = frame_world->points[i];

    // Eigen::Vector4f p(src.x, src.y, src.z, 1.0f);
    Eigen::Vector4f p;
    p << src.x, src.y, src.z, 1.0f;
    p = pose * p;

    dst.x         = p.x();
    dst.y         = p.y();
    dst.z         = p.z();
    dst.intensity = src.intensity;
  }

  // ---------- 4. intensity 着色 ----------
  if (!intensity_handler) {
    intensity_handler.reset(
        new pcl::visualization::PointCloudColorHandlerGenericField<
            pcl::PointXYZI>(frame_world, "intensity"));
  }

  if (!vis_->contains("frame")) {
    vis_->addPointCloud(frame_world, *intensity_handler, "frame");
  } else {
    vis_->updatePointCloud(frame_world, *intensity_handler, "frame");
  }

  // 4. traj
  UpdateTrajectory(pose);

  // ---------- 5. 相机跟随（关键） ----------
  BridView(pose);

  vis_->spinOnce(1);
  // vis_->spin();
}

bool MapCenterView::NeedUpdateROI(const Eigen::Vector3f& center) {
  if (!has_last_center) {
    last_center     = center;
    has_last_center = true;
    return true;
  }

  // ROI 是 XY 平面上的圆，更新判定也只计算 XY 位移，避免高度变化触发重裁剪。
  const float dx   = center.x() - last_center.x();
  const float dy   = center.y() - last_center.y();
  const float dist = std::sqrt(dx * dx + dy * dy);

  if (dist > roi_update_dist) {
    last_center = center;
    return true;
  }

  return false;
}

void MapCenterView::UpdateTrajectory(const Eigen::Matrix4f& pose) {
  constexpr std::size_t kMaxTrajectoryPoints = 10000;

  pcl::PointXYZRGB pt;
  pt.x = pose(0, 3);
  pt.y = pose(1, 3);
  pt.z = pose(2, 3);
  pt.r = 255;
  pt.g = 0;
  pt.b = 0;
  trajectory_->points.push_back(pt);

  if (trajectory_->points.size() > kMaxTrajectoryPoints) {
    const auto erase_end = trajectory_->points.begin() +
                           (trajectory_->points.size() - kMaxTrajectoryPoints);
    trajectory_->points.erase(trajectory_->points.begin(), erase_end);
  }
  trajectory_->width  = trajectory_->points.size();
  trajectory_->height = 1;

  if (!vis_->contains("traj")) {
    vis_->addPointCloud(trajectory_, "traj");
    vis_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "traj");
  } else {
    vis_->updatePointCloud(trajectory_, "traj");
  }
}

void MapCenterView::BridView(const Eigen::Matrix4f& pose) {
  // false：平移跟随车体，保留用户鼠标调整的视角（相对观察角度与距离）
  // true：自动设置相机视角为车体后上方视角（仅跟随 Yaw 旋转）

  // 位置跟随
  Eigen::Vector3f center = pose.block<3, 1>(0, 3);
  // camera view ≈ 100
  // if ((center - last_cam_center).norm() > 5.0f) {
  //   // vis_->spinOnce(1);
  //   return;
  // }
  // last_cam_center = center;

  if (!sparam_->b_display_body) {
    // 获取当前相机参数
    std::vector<pcl::visualization::Camera> cameras;
    vis_->getCameras(cameras);
    if (cameras.empty()) {
      return;
    }

    const auto& cam = cameras[0];

    // 当前相机相对于焦点的偏移向量（保留用户鼠标旋转和缩放的相对姿态）
    double dx = cam.pos[0] - cam.focal[0];
    double dy = cam.pos[1] - cam.focal[1];
    double dz = cam.pos[2] - cam.focal[2];

    // 将焦点平移到车体中心，同时平移相机自身位置
    vis_->setCameraPosition(
        center.x() + dx, center.y() + dy, center.z() + dz,  // 相机新位置
        center.x(), center.y(), center.z(),  // 焦点为车体位置
        cam.view[0], cam.view[1], cam.view[2]  // 保留用户当前的 Up 向量
    );
  } else {
    // clang-format off
    /* 平移跟随车体，不带姿态旋转
    vis_->setCameraPosition(center.x() - 50, center.y() - 50, center.z() + 50,  // camera pos
                            center.x(), center.y(), center.z(),  // look at point
                            0, 0, 1);  // up
    */
    // clang-format on

    // clang-format off
    /* 跟随车体，带姿态旋转，车体的抖动带动相机抖动
    // 相机位置：后 + 左 + 上
    // 姿态跟随
    Eigen::Matrix3f R = pose.block<3, 3>(0, 0);
    // 约定（自动驾驶常见）
    Eigen::Vector3f forward = R.col(0);  // 车头方向 (x)
    Eigen::Vector3f left    = R.col(1);  // 左 (y)
    Eigen::Vector3f up      = R.col(2);  // 上 (z)

    Eigen::Vector3f cam_pos = center - forward * 50.0f  // 往后
                              // + left * 50.0f  // 往左
                              + up * 50.0f;  // 往上

    vis_->setCameraPosition(cam_pos.x(), cam_pos.y(), cam_pos.z(),
                            center.x(), center.y(), center.z(),
                            up.x(), up.y(), up.z());
    */

    // 带姿态旋转，只变换 yaw 平滑相机视角
    Eigen::Matrix3f R = pose.block<3, 3>(0, 0);
    // 原始 forward
    Eigen::Vector3f forward = R.col(0);
    // 只保留水平分量（去掉 pitch）
    forward.z() = 0.0f;
    if (forward.squaredNorm() < 1e-6f) {
      return;
    }
    forward.normalize();
    // 固定世界up（避免roll/pitch影响）
    static Eigen::Vector3f up(0, 0, 1);

    // 相机位置（后 + 上）
    Eigen::Vector3f cam_pos = center - forward * 50.0f 
                              + up * 50.0f;

    vis_->setCameraPosition(cam_pos.x(), cam_pos.y(), cam_pos.z(), 
                            center.x(), center.y(), center.z(), 
                            up.x(), up.y(), up.z());
    // clang-format on
  }
}

}  // namespace dreamview
}  // namespace jojo
