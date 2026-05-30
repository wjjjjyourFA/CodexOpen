#include "modules/perception/similarity_map/ColorMap.h"

namespace jojo {
namespace perception {
using namespace jojo::perception::camera;

ColorMap::ColorMap() {}

ColorMap::~ColorMap() {}

void ColorMap::Init(std::shared_ptr<jojo::perception::RuntimeConfig> rparam) {
  rparam_ = rparam;

  // 初始化 栅格 地图
  hps_.map_resolution = rparam_->map_resolution;
  hps_.map_rows       = rparam_->map_rows;
  hps_.map_cols       = rparam_->map_cols;
  hps_.half_rows      = hps_.map_rows / 2;
  hps_.half_cols      = hps_.map_cols / 2;

  color_grid_map = std::make_shared<CLocalWindowMap<ColorCell>>(
      hps_.map_rows, hps_.map_cols, hps_.map_resolution);

  // 读取车辆参数
  const auto& min_b = rparam_->min_bound;
  const auto& max_b = rparam_->max_bound;
  // 设置点云过滤器
  boxFilter.setMin(Eigen::Vector4f(min_b.x(), min_b.y(), min_b.z(), 1.0f));
  boxFilter.setMax(Eigen::Vector4f(max_b.x(), max_b.y(), max_b.z(), 1.0f));
  boxFilter.setNegative(true);

  show_mat.create(hps_.map_rows, hps_.map_cols, CV_8UC3);
  show_mat.setTo(cv::Scalar(0, 0, 0));

  auto camera_params = std::make_shared<CameraParams>();
  camera_params->LoadFromFile(rparam_->camera_calib_file_path);
  auto matrix = camera_params->GetMatrixVector();
  // std::cout << "camera_params: \n" << matrix->extrinsic_matrix << std::endl;

  fusion = std::make_shared<jojo::perception::fusion::LidarCameraFusion>();
  fusion->SetProjectionMatrix(matrix.at(0)->projection_matrix);

  if (rparam_->b_show_color_point) {
    this->InitViewer();
  }
}

// TODO：修改成使用数据输入，而不是用 Measures
// TODO：添加多视角相机，以及多线程支持
void ColorMap::Run(
    std::shared_ptr<const jojo::tools::MeasureGroupDataSet> Measures) {
  // 1. 获取各传感器数据对应的 pose
  const auto& frame   = Measures->lidar.data;
  const auto& in_pose = Measures->se3_pose.data.matrix();
  const auto& image   = Measures->camera.at(0).data;

  Eigen::Matrix4f cur_pose = in_pose;

  // 2. 投影到图像上，并获取对应的彩色化点云。（只有图像窗口的点云被保留）
  fusion->SetLidarPointCloud(frame);
  fusion->SetCameraImage(image);
  fusion->fuse(1, false, true);
  fusion->GetFusedPointCloudColor(cur_cloud);
  // std::cout << "cur_cloud size: " << cur_cloud->size() << std::endl;

  // 3. 将 cur frame 对应的多组彩色点云数据，融合到一起

  // 4. 将点云数据进行预处理过滤不需要的部分
  // this->PreProcessEgoBev(cur_cloud);
  // 将当前点云变换到全局坐标系
  this->PreProcessGlobalBev(cur_cloud, cur_pose);

  // 5. 更新栅格地图，map_center
  // this->UpdateColorGridMapEgoBev(cur_cloud, cur_pose);
  this->UpdateColorGridMapGlobalBev(cur_cloud, cur_pose);

  // this->ShowColorGridMap();
  this->ShowColorGridMapRotation(cur_pose);

  if (rparam_->b_show_color_point) {
    this->VisColorCloud(cur_cloud);
  }
}

void ColorMap::PreProcessEgoBev(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  boxFilter.setInputCloud(cloud);
  boxFilter.filter(*cloud);
}

void ColorMap::PreProcessGlobalBev(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                   const Eigen::Matrix4f& pose) {
  this->PreProcessEgoBev(cloud);

  // const Eigen::Matrix3f R = cur_pose.block<3, 3>(0, 0);
  // const Eigen::Vector3f T = cur_pose.block<3, 1>(0, 3);

  pcl::transformPointCloud(*cloud, *cloud, pose);
}

void ColorMap::UpdateColorGridMapEgoBev(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const Eigen::Matrix4f& pose) {
  // !! Ego Rolling Map 无法进行历史点的累积

  // 从 pose 提取平移（用于 window shift）==> 遵循的是 pose 的坐标系
  auto& pose_x = pose(0, 3);
  auto& pose_y = pose(1, 3);

  color_grid_map->ResetMap();
  // 更新中心点，自动更新 Residual 和 坐标语义 ==> 这意味着 color_grid_map 保留了世界坐标数据
  color_grid_map->ReCenterByPose(pose_x, pose_y);

  // 写入的点云坐标，是基于 cur_pose 的，也就是局部坐标系
  // 更新每个 cell 的颜色、置信度
  for (auto& p : *cloud) {
    // 与默认输入保持一致 X 向右， Y向上
    // CellType* 本质是返回已有对象的地址
    ColorCell* cell = color_grid_map->GetValueFromXY(p.x, p.y);
    // 将栅格图变换到 X 向上，Y 向左
    // ColorCell* cell = color_grid_map->GetValueFromXY(-p.y, p.x);
    if (cell != nullptr) {
      cell->UpdateCellByAvg(p);
    }
  }
}

void ColorMap::UpdateColorGridMapGlobalBev(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const Eigen::Matrix4f& pose) {
  // !! “CLocalWindowMap 固定在世界坐标系方向，然后只进行轴向移动”

  auto& pose_x = pose(0, 3);
  auto& pose_y = pose(1, 3);

  color_grid_map->ReCenterByPose(pose_x, pose_y);

  // 写入的点云坐标，是基于 global_pose 的，也就是全局坐标系
  // !! 现在无法显示，要么 grid 的 GetValueFromXY 没有正确处理世界坐标
  // 要么 ReCenterByPose 没有正确滑窗到指定位置
  for (auto& p : *cloud) {
    // 已经在预处理中，变换到全局坐标系
    auto& world_x = p.x;
    auto& world_y = p.y;

    ColorCell* cell = color_grid_map->GetValueFromWorldXY(world_x, world_y);
    // ColorCell* cell = color_grid_map->GetValueFromWorldXY(-world_y, world_x);
    if (cell != nullptr) {
      cell->UpdateCellByAvg(p);
    }
  }
}

void ColorMap::ShowColorGridMap() {
  // std::cout << "show color grid map" << std::endl;

  // 纯 local map
  show_mat.setTo(cv::Scalar(0, 0, 0));

  for (int r = 0; r < hps_.map_rows; r++) {
    for (int c = 0; c < hps_.map_cols; c++) {
      // ColorCell* cell = color_grid_map->GetValueFromRC(r, c);
      ColorCell* cell = color_grid_map->GetWorldXYFromRC(r, c);
      if (cell->b_valid) {
        auto& color = show_mat.at<cv::Vec3b>(r, c);
        // BGR
        color = cv::Vec3b(cell->rgb[2], cell->rgb[1], cell->rgb[0]);
      }
    }
  }

  cv::line(show_mat, cv::Point(0, hps_.half_rows),
           cv::Point(hps_.map_rows, hps_.half_rows), cv::Scalar(125, 125, 125));
  cv::line(show_mat, cv::Point(hps_.half_cols, 0),
           cv::Point(hps_.half_cols, hps_.map_cols), cv::Scalar(125, 125, 125));

  static bool init = true;
  if (init) {
    cv::namedWindow("color map", cv::WINDOW_NORMAL);
    cv::resizeWindow("color map", hps_.map_cols, hps_.map_rows);
    init = false;
  }
  cv::imshow("color map", show_mat);
  cv::waitKey(1);
}

void ColorMap::ShowColorGridMapRotation(const Eigen::Matrix4f& pose) {
  // 局部颜色栅格图
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
      /* way 1 Forward Mapping（正向投影）有空洞
      // 得到 cell 中心点世界坐标 (x, y)
      // clang-format off
      double world_x, world_y;
      ColorCell* cell = color_grid_map->GetWorldXYFromRC(r, c, world_x, world_y);
      if (!cell || !cell->b_valid) {
        continue;
      }
      // clang-format on

      // world -> ego-centered world
      double local_x = world_x - pose(0,3);
      double local_y = world_y - pose(1,3);
      // local -> rot local ==> 旋转到车头朝向
      double rot_x = cos_theta * local_x + sin_theta * local_y;
      double rot_y = -sin_theta * local_x + cos_theta * local_y;

      // !! 映射到 show_mat 坐标系，不是 color_grid_map 坐标系
      // clang-format off
      // show_mat 中，车头朝右
      // int dst_c = static_cast<int>(std::floor(hps_.half_cols + rot_x / hps_.map_resolution));
      // int dst_r = static_cast<int>(std::floor(hps_.half_rows - rot_y / hps_.map_resolution));

      // show_mat 中，车头朝上
      int dst_r = static_cast<int>(std::floor(hps_.half_cols - rot_x / hps_.map_resolution));
      int dst_c = static_cast<int>(std::floor(hps_.half_rows - rot_y / hps_.map_resolution));
      // clang-format on

      // 这里意味着 show_mat 和 color_grid_map 的坐标系 一样大
      if (dst_r < 0 || dst_r >= hps_.map_rows || dst_c < 0 ||
          dst_c >= hps_.map_cols) {
        continue;
      }
      */

      // /* way 2 Inverse Mapping（逆向采样）
      // 1. target pixel -> local coord (BEV image coordinate system)
      // show_mat 中，车头朝右
      // double local_x = (c - hps_.half_cols) * hps_.map_resolution;
      // double local_y = (hps_.half_rows - r) * hps_.map_resolution;
      // show_mat 中，车头朝上
      double local_y = (hps_.half_cols - c) * hps_.map_resolution;
      double local_x = (hps_.half_rows - r) * hps_.map_resolution;

      // 2. inverse rotation
      // ego local -> rot ego local -> world
      double world_x = cos_theta * local_x - sin_theta * local_y + pose(0, 3);
      double world_y = sin_theta * local_x + cos_theta * local_y + pose(1, 3);

      // 3. sample source map
      ColorCell* cell = color_grid_map->GetValueFromWorldXY(world_x, world_y);
      if (!cell || !cell->b_valid) {
        continue;
      }

      // !! 永远“写像素，不算像素位置”
      auto& dst_c = c;
      auto& dst_r = r;
      // */

      // TODO：nearest neighbor ==> 4邻域插值

      // 4. write dst pixel
      cv::Vec3b& color = show_mat.at<cv::Vec3b>(dst_r, dst_c);
      // RGB ==> BGR
      // color = cv::Vec3b(cell->rgb[2], cell->rgb[1], cell->rgb[0]);
      color[0] = cell->rgb[2];
      color[1] = cell->rgb[1];
      color[2] = cell->rgb[0];
    }
  }

  cv::line(show_mat, cv::Point(0, hps_.half_rows),
           cv::Point(hps_.map_rows, hps_.half_rows), cv::Scalar(125, 125, 125));
  cv::line(show_mat, cv::Point(hps_.half_cols, 0),
           cv::Point(hps_.half_cols, hps_.map_cols), cv::Scalar(125, 125, 125));

  static bool init = true;
  if (init) {
    cv::namedWindow("color map rot", cv::WINDOW_NORMAL);
    cv::resizeWindow("color map rot", hps_.map_cols, hps_.map_rows);
    init = false;
  }
  cv::imshow("color map rot", show_mat);
  cv::waitKey(1);
}

void ColorMap::InitViewer() {
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

void ColorMap::VisColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  vis_->removeAllPointClouds();

  // clang-format off
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color(cloud);

  vis_->addPointCloud<pcl::PointXYZRGB>(cloud, color, "color cloud");
  vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "color cloud");
  // clang-format on

  vis_->spinOnce(10);
}

}  // namespace perception
}  // namespace jojo
