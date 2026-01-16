#include "modules/perception/common/fusion/lidar2camera/lidar_multi_camera_fusion.h"

namespace jojo {
namespace perception {
namespace fusion {
namespace base = jojo::cyber::base;
namespace camera = jojo::perception::camera;

LidarMultiCameraFusion::LidarMultiCameraFusion() : LidarCameraFusion() {
  //  : LidarCameraFusion() ==> 调用父类构造函数
}

void LidarMultiCameraFusion::Init(
    const std::shared_ptr<base::ThreadPool> &thread_pool) {
  if (!thread_pool) {
    thread_pool_ = base::ThreadPool::Instance(8);
  } else {
    thread_pool_ = thread_pool;
  }
}

void LidarMultiCameraFusion::Start() { isRunning_ = true; }

void LidarMultiCameraFusion::Stop() { isRunning_ = false; }

void LidarMultiCameraFusion::Run(bool is_mask) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // clang-format off
  cloud_color_ = 
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  // clang-format on

  //
  const size_t num = image_v_.size();

  mask_v_.clear();
  mask_v_.reserve(num);

  for (int i = 0; i < num; ++i) {
    std::shared_ptr<cv::Mat> mask;

    // 绘制在 黑色底图 还是 原图 上
    if (is_mask) {
      mask = std::make_shared<cv::Mat>(
          cv::Mat::zeros(image_v_[i]->rows, image_v_[i]->cols, CV_8UC3));
    } else {
      mask = std::make_shared<cv::Mat>(*(image_v_[i]));
    }
    mask_v_.emplace_back(mask);
  }

  // auto start_time = std::chrono::high_resolution_clock::now();

  project_lidar_to_camera_fast(cloud_, camera_params_, image_v_, mask_v_);

  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
  //     end_time - start_time);
  // double run_time_ms = duration.count();  // 运行时间，单位为毫秒
  // std::cerr << "project_lidar_to_camera_fast run run_time_ms: " << run_time_ms
  //           << " ms" << std::endl;

  // std::cout << "cloud_color_ size: " << cloud_color_->size() << std::endl;
}

void LidarMultiCameraFusion::project_lidar_to_camera_fast(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const std::shared_ptr<CameraParams> &camera_params,
    const std::vector<std::shared_ptr<cv::Mat>> &image_v,
    std::vector<std::shared_ptr<cv::Mat>> &mask_v) {
  // 全局转换 将点云转为 Eigen 矩阵 减少计算消耗
  Eigen::Matrix<float, 4, Eigen::Dynamic> points;
  pcl_to_eigen<pcl::PointXYZ>(cloud, points);

  auto &matrix_vector = camera_params->GetMatrixVector();

  /* // way 1 单线程 将一簇点云串行投影到多个图像上  cost 6 ms */
  for (uint i = 0; i < image_v.size(); ++i) {
    Eigen::Ref<Eigen::Matrix<float, 3, 4>> projection_matrix =
        matrix_vector.at(i)->projection_matrix.block<3, 4>(0, 0);
    auto cloud_color = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGB>());

    project_lidar_to_camera_fast_impl(points, projection_matrix,
                                      *(image_v.at(i)), *(mask_v.at(i)),
                                      cloud_color);

    *cloud_color_ += *cloud_color;
  }
  /* */

  /* // way 2 多线程 cost 3 ms
  const size_t num = image_v.size();
  // 每个线程生成自己的点云指针
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> thread_clouds(num);

  // 存储 future
  std::vector<std::future<void>> futures;
  futures.reserve(num);

  for (size_t i = 0; i < num; ++i) {
    // clang-format off
    thread_clouds[i] = 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    // clang-format on

    // 拷贝 projection_matrix, image 只读
    // mask 和 cloud 会修改，每个线程独立对象
    auto projection_matrix =
        matrix_vector.at(i)->projection_matrix.block<3, 4>(0, 0);
    auto &image     = *(image_v.at(i));
    auto &mask      = *(mask_v.at(i));
    auto &cloud_ptr = thread_clouds[i];

    // 异步任务
    futures.emplace_back(thread_pool_->Enqueue(
        [this, points, projection_matrix, &image, &mask, &cloud_ptr]() mutable {
          // 每个线程独立计算
          this->project_lidar_to_camera_fast_impl(points, projection_matrix,
                                                  image, mask, cloud_ptr);
        }));
  }

  // 等待所有任务完成
  // 不等待任务完成就读取 cloud_ptr 的数据一定是错误的
  // 因为每个线程可能还没有计算完成 cloud_ptr
  for (auto &f : futures) {
    f.get();
  }

  // 合并到全局 cloud_color_（加锁保护）
  {
    std::unique_lock<std::mutex> lock(cloud_mutex_);
    for (auto &c : thread_clouds) {
      // PCL 支持累积
      *cloud_color_ += *c;  
    }
  }
  */
}

void LidarMultiCameraFusion::SetCameraParams(
    std::shared_ptr<CameraParams> camera_params) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  camera_params_ = camera_params;
}

void LidarMultiCameraFusion::SetCameraImageVector(
    const std::vector<std::shared_ptr<cv::Mat>> &image_v) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  image_v_ = image_v;
}

bool LidarMultiCameraFusion::GetFusedImageVector(
    std::vector<std::shared_ptr<cv::Mat>> &image_v) {
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