#include "modules/perception/common/fusion/lidar2camera/lidar_multi_camera_fusion.h"

#include <cmath>
#include <future>
#include <limits>
#include <utility>

namespace jojo {
namespace perception {
namespace fusion {
namespace base   = jojo::cyber::base;
namespace camera = jojo::perception::camera;

bool LidarMultiCameraFusion::Init(
    const std::shared_ptr<base::ThreadPool>& thread_pool) {
  thread_pool_ = thread_pool ? thread_pool : base::ThreadPool::Instance(8);
  return static_cast<bool>(thread_pool_);
}

void LidarMultiCameraFusion::Start() { is_running_ = true; }

void LidarMultiCameraFusion::Stop() { is_running_ = false; }

void LidarMultiCameraFusion::Run(bool is_mask) {
  std::lock_guard<std::mutex> run_lock(run_mutex_);
  if (!is_running_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::shared_ptr<camera::CameraParams> camera_params;
  std::vector<std::shared_ptr<cv::Mat>> images;
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    cloud         = cloud_;
    camera_params = camera_params_;
    images        = image_v_;
  }

  if (!ValidateBatchInput(cloud, camera_params, images)) {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    mask_v_.clear();
    cloud_color_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    return;
  }

  // 一份点云只转换一次，所有相机任务共享只读的齐次坐标矩阵。
  PointsMatrix points;
  pcl_to_eigen<pcl::PointXYZ>(cloud, points);

  const auto& camera_matrices = camera_params->GetMatrixVector();
  std::vector<ProjectionMatrix> projection_matrices;
  projection_matrices.reserve(camera_matrices.size());
  for (const auto& matrix : camera_matrices) {
    projection_matrices.emplace_back(
        matrix->projection_matrix.block<3, 4>(0, 0));
  }

  std::vector<std::shared_ptr<cv::Mat>> outputs;
  outputs.reserve(images.size());
  for (const auto& image : images) {
    if (is_mask) {
      // 最终输出始终是可直接 imshow/发布的 CV_8UC3 图像。
      // outputs.emplace_back(std::make_shared<cv::Mat>(cv::Mat::zeros(image->rows, image->cols, CV_8UC3)));
      outputs.emplace_back(std::make_shared<cv::Mat>(
          cv::Mat::zeros(image->rows, image->cols, CV_32FC3)));
    } else {
      // clone 保证 batch 投影不会修改调用方持有的原图。
      outputs.emplace_back(std::make_shared<cv::Mat>(image->clone()));
    }
  }

  ProjectBatch(points, projection_matrices, images, outputs);

  // 每批结果使用新的对象，避免覆盖正在被外部异步消费的上一批结果。
  std::lock_guard<std::mutex> data_lock(data_mutex_);
  mask_v_ = std::move(outputs);
  cloud_color_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
}

bool LidarMultiCameraFusion::ValidateBatchInput(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::shared_ptr<camera::CameraParams>& camera_params,
    const std::vector<std::shared_ptr<cv::Mat>>& image_v) const {
  if (!cloud || cloud->empty() || !camera_params || image_v.empty()) {
    return false;
  }

  const auto& matrix_v = camera_params->GetMatrixVector();
  if (matrix_v.size() != image_v.size()) {
    return false;
  }

  for (size_t i = 0; i < image_v.size(); ++i) {
    if (!image_v[i] || image_v[i]->empty() || image_v[i]->type() != CV_8UC3 ||
        !matrix_v[i] || !matrix_v[i]->projection_matrix.allFinite()) {
      return false;
    }
  }
  return true;
}

void LidarMultiCameraFusion::ProjectBatch(
    const PointsMatrix& points,
    const std::vector<ProjectionMatrix>& projection_matrices,
    const std::vector<std::shared_ptr<cv::Mat>>& image_v,
    std::vector<std::shared_ptr<cv::Mat>>& output_v) {
  const size_t camera_count = image_v.size();
  const float invalid_value = std::numeric_limits<float>::quiet_NaN();

  // 父类公共函数负责批量矩阵乘法、深度过滤和图像边界过滤。独立的
  // CV_32FC3 buffer 保存投影到像素后的原始 XYZ，避免把米制 XYZ 直接写入
  // CV_8UC3 原图后变成几乎不可见的暗色单像素。
  std::vector<cv::Mat> projections;
  projections.reserve(camera_count);
  for (const auto& image : image_v) {
    projections.emplace_back(
        image->rows, image->cols, CV_32FC3,
        cv::Scalar(invalid_value, invalid_value, invalid_value));
  }

  // 每个相机拥有独立 workspace；一份 points 只读共享。
  // clang-format off
  std::vector<Eigen::Matrix<float, 3, Eigen::Dynamic>> workspaces(camera_count);
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> unused_clouds(camera_count);
  // clang-format on

  auto project_camera = [this, &points, &projection_matrices, &image_v,
                         &output_v, &projections, &workspaces,
                         &unused_clouds](size_t camera_index) {
    project_lidar_to_camera_fast_impl(
        points, projection_matrices[camera_index], *image_v[camera_index],
        projections[camera_index], unused_clouds[camera_index],
        workspaces[camera_index], false);
    DrawProjectionResult(projections[camera_index], *output_v[camera_index]);
  };

  if (!thread_pool_ || camera_count == 1) {
    for (size_t i = 0; i < camera_count; ++i) {
      project_camera(i);
    }
    return;
  }

  std::vector<std::future<void>> futures;
  futures.reserve(camera_count);
  for (size_t i = 0; i < camera_count; ++i) {
    futures.emplace_back(thread_pool_->Enqueue(project_camera, i));
  }
  for (auto& future : futures) {
    future.get();
  }
}

size_t LidarMultiCameraFusion::DrawProjectionResult(const cv::Mat& projection,
                                                    cv::Mat& output) const {
  if (projection.empty() || projection.type() != CV_32FC3 || output.empty() ||
      projection.size() != output.size()) {
    return 0;
  }

  size_t visible_pixel_count = 0;
  for (int v = 0; v < projection.rows; ++v) {
    const cv::Vec3f* projection_row = projection.ptr<cv::Vec3f>(v);
    for (int u = 0; u < projection.cols; ++u) {
      // 公共快速投影按 BGR 通道保存 Z、Y、X。
      const cv::Vec3f& pixel = projection_row[u];

      const float z = pixel[0];
      const float y = pixel[1];
      const float x = pixel[2];

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      const float distance = std::sqrt(x * x + y * y + z * z);
      const cv::Scalar color =
          GetColorByDistance(distance, static_cast<float>(dist_));
      cv::circle(output, cv::Point(u, v), 2, color, -1, cv::LINE_AA);
      ++visible_pixel_count;
    }
  }
  return visible_pixel_count;
}

void LidarMultiCameraFusion::SetCameraParams(
    std::shared_ptr<camera::CameraParams> camera_params) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  camera_params_ = std::move(camera_params);
}

void LidarMultiCameraFusion::SetCameraImageVector(
    const std::vector<std::shared_ptr<cv::Mat>>& image_v) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  image_v_ = image_v;
}

bool LidarMultiCameraFusion::GetFusedImageVector(
    std::vector<std::shared_ptr<cv::Mat>>& image_v) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (mask_v_.empty()) {
    return false;
  }
  image_v = mask_v_;
  return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace jojo
