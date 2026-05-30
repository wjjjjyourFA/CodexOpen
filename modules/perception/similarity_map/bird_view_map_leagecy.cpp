#include "modules/perception/similarity_map/bird_view_map_leagecy.h"

namespace jojo {
namespace perception {
using namespace jojo::perception::camera;

BirdViewMap::BirdViewMap() {}

BirdViewMap::~BirdViewMap() {}

void BirdViewMap::Init(
    std::shared_ptr<jojo::perception::RuntimeConfig> rparam) {
  LocalMappingBase<pcl::PointXYZRGB>::Init();
  rparam_ = rparam;

  // 初始化 栅格 地图
  hps_.map_resolution = rparam_->map_resolution;
  hps_.map_rows       = rparam_->map_rows;
  hps_.map_cols       = rparam_->map_cols;
  hps_.half_rows      = hps_.map_rows / 2;
  hps_.half_cols      = hps_.map_cols / 2;

  frame_color.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  global_bev =
      cv::Mat(hps_.map_rows, hps_.map_cols, CV_8UC3, cv::Scalar(0, 0, 0));

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

void BirdViewMap::Run(
    std::shared_ptr<const jojo::tools::MeasureGroupDataSet> Measures) {
  const auto& frame   = Measures->lidar.data;
  const auto& in_pose = Measures->se3_pose.data.matrix();
  const auto& image   = Measures->camera.at(0).data;

  if (!this->NeedNewKeyFrame(in_pose)) {
    return;
  }

  fusion->SetLidarPointCloud(frame);
  fusion->SetCameraImage(image);
  fusion->fuse(1, false, true);
  fusion->GetFusedPointCloudColor(frame_color);

  // clang-format off
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cur_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>);
  this->BuildCurrentFrameCloud(frame_color, cur_cloud_);
  // clang-format on

  Eigen::Matrix4f new_pose = in_pose;

  this->AddKeyFrame(cur_cloud_, new_pose);
  // std::cout << "add key frame: " << this->keyframes_.size() << std::endl;

  this->UpdateLocalMap();

  this->GenerateBirdView();

  if (rparam_->b_show_color_point) {
    // this->VisColorCloud(frame_color);
    // this->VisColorCloud(cur_cloud_);
    this->VisColorCloud(local_map_);
  }
}

void BirdViewMap::GenerateBirdView() {
  global_bev.setTo(0);

  for (const auto& pt : local_map_->points) {
    // x -> 前 , y -> 左
    int u = static_cast<int>(hps_.half_cols - pt.y / hps_.map_resolution);
    int v = static_cast<int>(hps_.map_rows - 1 - pt.x / hps_.map_resolution);

    if (u < 0 || u >= hps_.map_cols || v < 0 || v >= hps_.map_rows) {
      continue;
    }

    global_bev.at<cv::Vec3b>(v, u) = cv::Vec3b(pt.b, pt.g, pt.r);
  }

  static bool init = true;
  if (init) {
    cv::namedWindow("color map", cv::WINDOW_NORMAL);
    cv::resizeWindow("color map", hps_.map_cols, hps_.map_rows);
    init = false;
  }
  cv::imshow("color map", global_bev);
  cv::waitKey(1);
}

void BirdViewMap::InitViewer() {
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

void BirdViewMap::VisColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  if (cloud == nullptr) return;

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
