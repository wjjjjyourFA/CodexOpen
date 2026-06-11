#include "modules/perception/ground_remove/ground_remove.h"

namespace jojo {
namespace perception {

GroundRemove::GroundRemove() {
  ground_segmentation = std::make_shared<GroundSegmentation>();
}

GroundRemove::~GroundRemove() {}

void GroundRemove::Init(
    std::shared_ptr<jojo::perception::RuntimeConfig> rparam) {
  rparam_ = rparam;
  ground_segmentation->Init(rparam);

  // 初始化 栅格 地图
  hps_.map_resolution = rparam_->map_resolution;
  hps_.map_rows       = rparam_->map_rows;
  hps_.map_cols       = rparam_->map_cols;
  hps_.half_rows      = hps_.map_rows / 2;
  hps_.half_cols      = hps_.map_cols / 2;

  obstacle_grid_map = std::make_shared<CLocalWindowMap<ObstacleCell>>(
      hps_.map_rows, hps_.map_cols, hps_.map_resolution);

  show_mat.create(hps_.map_rows, hps_.map_cols, CV_8UC3);
  show_mat.setTo(cv::Scalar(0, 0, 0));

  if (rparam_->b_show_color_point) {
    obstacle_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    obstacle_cloud_->points.reserve(230400);
    this->InitViewer();
  }
}

/* 极坐标栅格地面分割（Polar Grid Ground Segmentation）
  点云
  → 极坐标分桶
  → 统计每个bin高度特征
  → 判定ground / obstacle
  → 投影生成occupancy map
*/
void GroundRemove::Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame,
                       const Eigen::Matrix4f& in_pose) {
  auto& pose_x = in_pose(0, 3);
  auto& pose_y = in_pose(1, 3);

  // TODO：点云变换 ==> 车辆中心
  // 现在传入的是 激光雷达坐标系 的点云 ==> 地面是 -2.5m 左右
  ground_segmentation->Run(frame);

  // this->remove_noise_points();

  if (rparam_->b_use_legacy) {
    obstacle_grid_map->ResetMap();
  }
  obstacle_grid_map->ReCenterByPose(pose_x, pose_y);

  this->build_occupancy(frame);

  this->ShowObstacleGridMap();
  // TODO：使用全局系的时候才能用该函数
  // this->ShowColorGridMapRotation(in_pose);
}

void GroundRemove::build_occupancy(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& frame) {
  // 3. build occupancy
  // 障碍物 --> 直角坐标化 发布障碍物地图
  // VariableFusionMap.positive_map.clear();
  // TODO：因为此点云要用于输出，所以需要每帧重新分配？
  // obstacle_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  if (rparam_->b_show_color_point) obstacle_cloud_->clear();

  auto& polar_grid_map = ground_segmentation->GetPolarGridMap();

  for (const auto& grid : polar_grid_map) {
    if (!grid.active || grid.is_ground) continue;

    for (auto idx : grid.indices) {
      const auto& pt = frame->points[idx];

      if (rparam_->b_show_color_point) obstacle_cloud_->points.push_back(pt);

      // 以栅格图的形式输出障碍图
      // 与默认输入保持一致 X 向右， Y向上
      // ObstacleCell* cell = obstacle_grid_map->GetValueFromXY(pt.x, pt.y);
      // 将栅格图变换到 X 向上，Y 向左
      ObstacleCell* cell = obstacle_grid_map->GetValueFromXY(-pt.y, pt.x);
      if (cell != nullptr) {
        cell->UpdateCellByType(ObstacleType::MAX, pt.z);
      }
    }
  }

  if (rparam_->b_show_color_point) {
    // std::cout << "obstacle cloud size: " << obstacle_cloud_->points.size() << std::endl;
    this->VisColorCloud(obstacle_cloud_);
  }
}

void GroundRemove::ShowObstacleGridMap() {
  // std::cout << "show obstacle grid map" << std::endl;

  // 纯 local map
  show_mat.setTo(cv::Scalar(0, 0, 0));

  for (int r = 0; r < hps_.map_rows; r++) {
    for (int c = 0; c < hps_.map_cols; c++) {
      ObstacleCell* cell = obstacle_grid_map->GetWorldXYFromRC(r, c);
      if (cell->b_valid) {
        auto& color = show_mat.at<cv::Vec3b>(r, c);
        // BGR
        // color = cv::Vec3b(0, cell->max_z, 255);
        int mapped_color_index = std::min(
            static_cast<int>((cell->max_z / hps_.height_z) * 640), 639);
        color[0] = jet_color_map[mapped_color_index][2];
        color[1] = jet_color_map[mapped_color_index][1];
        color[2] = jet_color_map[mapped_color_index][0];
      }
    }
  }

  cv::line(show_mat, cv::Point(0, hps_.half_rows),
           cv::Point(hps_.map_rows, hps_.half_rows), cv::Scalar(125, 125, 125));
  cv::line(show_mat, cv::Point(hps_.half_cols, 0),
           cv::Point(hps_.half_cols, hps_.map_cols), cv::Scalar(125, 125, 125));

  static bool init = true;
  if (init) {
    cv::namedWindow("obstacle map", cv::WINDOW_NORMAL);
    cv::resizeWindow("obstacle map", hps_.map_cols, hps_.map_rows);
    init = false;
  }
  cv::imshow("obstacle map", show_mat);
  cv::waitKey(1);
}

void GroundRemove::ShowColorGridMapRotation(const Eigen::Matrix4f& pose) {
  // 局部障碍栅格图
  //     ↓
  // 按照车辆朝向 theta，进行二维旋转
  //     ↓
  // 生成“车头对齐”的 BEV 图

  show_mat.setTo(cv::Scalar(0, 0, 0));

  double theta     = std::atan2(pose(1, 0), pose(0, 0));
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);

  for (int r = 0; r < hps_.map_rows; r++) {
    for (int c = 0; c < hps_.map_cols; c++) {
      double local_y = (hps_.half_cols - c) * hps_.map_resolution;
      double local_x = (hps_.half_rows - r) * hps_.map_resolution;

      double world_x = cos_theta * local_x - sin_theta * local_y + pose(0, 3);
      double world_y = sin_theta * local_x + cos_theta * local_y + pose(1, 3);

      // clang-format off
      ObstacleCell* cell = obstacle_grid_map->GetValueFromWorldXY(world_x, world_y);
      if (!cell || !cell->b_valid) {
        continue;
      }
      // clang-format on

      auto& dst_c = c;
      auto& dst_r = r;

      cv::Vec3b& color = show_mat.at<cv::Vec3b>(dst_r, dst_c);
      // RGB ==> BGR
      int mapped_color_index =
          std::min(static_cast<int>((cell->max_z / hps_.height_z) * 640), 639);
      color[0] = jet_color_map[mapped_color_index][2];
      color[1] = jet_color_map[mapped_color_index][1];
      color[2] = jet_color_map[mapped_color_index][0];
    }
  }

  cv::line(show_mat, cv::Point(0, hps_.half_rows),
           cv::Point(hps_.map_rows, hps_.half_rows), cv::Scalar(125, 125, 125));
  cv::line(show_mat, cv::Point(hps_.half_cols, 0),
           cv::Point(hps_.half_cols, hps_.map_cols), cv::Scalar(125, 125, 125));

  static bool init = true;
  if (init) {
    cv::namedWindow("obstacle map rot", cv::WINDOW_NORMAL);
    cv::resizeWindow("obstacle map rot", hps_.map_cols, hps_.map_rows);
    init = false;
  }
  cv::imshow("obstacle map rot", show_mat);
  cv::waitKey(1);
}

void GroundRemove::InitViewer() {
  if (vis_inited_) return;

  vis_.reset(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
  vis_->setBackgroundColor(0, 0, 0);
  vis_->initCameraParameters();
  vis_->setCameraPosition(-50, -50, 200,  // 相机位置（原点上方）
                          0, 0, 0,  // 看向原点
                          0, 1, 0  // up方向
  );
  vis_inited_ = true;
}

void GroundRemove::VisColorCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  if (!intensity_handler) {
    intensity_handler.reset(
        new pcl::visualization::PointCloudColorHandlerGenericField<
            pcl::PointXYZI>(cloud, "intensity"));
  }

  vis_->removeAllPointClouds();

  // clang-format off
  vis_->addPointCloud<pcl::PointXYZI>(cloud, *intensity_handler, "color cloud");
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "color cloud");
  // clang-format on

  vis_->spinOnce(10);
}

}  // namespace perception
}  // namespace jojo
