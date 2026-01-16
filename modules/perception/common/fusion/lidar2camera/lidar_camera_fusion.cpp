#include "modules/perception/common/fusion/lidar2camera/lidar_camera_fusion.h"

namespace jojo {
namespace perception {
namespace fusion {

LidarCameraFusion::LidarCameraFusion(/* args */) {
  // clang-format off
  cloud_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  // cloud_color_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  // clang-format on

  inv_dist = 1.0 / dist_;
}

LidarCameraFusion::~LidarCameraFusion() {}

void LidarCameraFusion::set_params(const std::string& name,
                                   int dist_threshold) {
  dist_ = dist_threshold;
  // 只算一次
  inv_dist = 1.0 / dist_;

  name_ = name;
}

void LidarCameraFusion::fuse(int mode, bool is_mask, bool color) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // way 1 复用 会影响外部
  // cloud_color_->clear();
  // way 2 每次都会新建对象，不会影响外部异步消费（显示、发布）
  // clang-format off
  if (color) {
    cloud_color_ = 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  }
  // clang-format on

  // 绘制在 黑色底图 还是 原图 上
  if (is_mask) {
    mask_ = cv::Mat::zeros(image_.rows, image_.cols, CV_32FC3);
  } else {
    // image 是 const 类型，同时为了保护外界的图像数据，所以这里需要重新拷贝一份
    mask_ = image_.clone();  // CV_8UC3
  }

  // input: 已经畸变矫正的图像
  // output: mask_
  // clang-format off
  switch (mode) {
    case 1:
      project_lidar_to_camera(cloud_, projection_matrix_, image_, mask_, color);
      break;
    case 2:
      project_lidar_to_camera_fast(cloud_, projection_matrix_, image_, mask_, color);
      break;
    default:
      std::cout << name_ + "CameraFusion mode set error" << std::endl;
      break;
  }
  // clang-format on
}

void LidarCameraFusion::project_lidar_to_camera(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Matrix<float, 3, 4>& projection_matrix, const cv::Mat& image,
    cv::Mat& mask, bool color) {
  // default lidar coords --> Front Left Up
  for (const auto& point : cloud->points) {
    // Create a homogeneous coordinate vector for the point
    Eigen::Vector4f point_homo(point.x, point.y, point.z, 1.0f);

    // 这里的 point 没有经过坐标变换，如果直接过滤，是以雷达坐标系为基准的
    // if(point.x > 25){  // m
    //   continue;
    // }

    // Apply the projection matrix
    // Eigen::Vector4f projected_point = projection_matrix * point_homo;
    Eigen::Vector3f projected_point = projection_matrix * point_homo;

    // Check if the point is in front of the camera (z' > 0)
    if (projected_point(2) <= 0) {
      continue;  // Skip points behind the camera
    }

    // Convert to pixel coordinates
    int u = static_cast<int>(projected_point(0) / projected_point(2));
    int v = static_cast<int>(projected_point(1) / projected_point(2));

    // Ensure the point is within the image bounds
    if (u >= 0 && v >= 0 && u < image.cols && v < image.rows) {
      if (color) {
        const cv::Vec3b& pix = image.at<cv::Vec3b>(v, u);
        pcl::PointXYZRGB tmp_point;
        tmp_point.x = point.x;
        tmp_point.y = point.y;
        tmp_point.z = point.z;
        tmp_point.b = pix[0];
        tmp_point.g = pix[1];
        tmp_point.r = pix[2];
        cloud_color_->emplace_back(tmp_point);
        // cloud_color_->push_back(tmp_point);
      }

      // way 1 绘制一个像素点，而不是一个圆
      // cv::circle(mask, cv::Point(u, v), 1,
      //            cv::Scalar(point.z, point.y, point.x), -1, 8);

      // way 2
      // Calculate distance for color mapping
      double dist =
          std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

      // int mapped_color_index =
      //     std::min(static_cast<int>((dist * inv_dist) * 640), 639);

      /* way 1 opencv bgr
      int type = mask_.type();
      if (type == CV_32FC3) {
        cv::Vec3f &mPix = mask_.at<cv::Vec3f>(v, u);
        mPix[0] = jet_color_map[mapped_color_index][2];
        mPix[1] = jet_color_map[mapped_color_index][1];
        mPix[2] = jet_color_map[mapped_color_index][0];
      } else if (type == CV_8UC3) {
        cv::Vec3b &mPix = mask.at<cv::Vec3b>(v, u);
        mPix[0] = jet_color_map[mapped_color_index][2];
        mPix[1] = jet_color_map[mapped_color_index][1];
        mPix[2] = jet_color_map[mapped_color_index][0];
      }
      */
      /* way 2 draw the point on the image using the Jet Color Map
      // cv::Scalar 顺序是 (B, G, R)
      cv::circle(mask, cv::Point(u, v), 2,
                 cv::Scalar(jet_color_map[mapped_color_index][2],
                            jet_color_map[mapped_color_index][1],
                            jet_color_map[mapped_color_index][0]),
                 -1, cv::LINE_AA);
      */
      // /* way 3
      cv::Scalar color = GetColorByDistance(dist);
      cv::circle(mask, cv::Point(u, v), 2, color, -1, cv::LINE_AA);
      // */
    }
  }
  // cv::imshow("projected_image", mask);
}

void LidarCameraFusion::project_lidar_to_camera_fast(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Matrix<float, 3, 4>& projection_matrix, const cv::Mat& image,
    cv::Mat& mask, bool color) {
  // 将点云转为 Eigen 矩阵
  // Eigen::MatrixXf points;
  Eigen::Matrix<float, 4, Eigen::Dynamic> points;
  pcl_to_eigen<pcl::PointXYZ>(cloud, points);

  project_lidar_to_camera_fast_impl(points, projection_matrix, image, mask,
                                    cloud_color_, color);
}

void LidarCameraFusion::project_lidar_to_camera_fast_impl(
    const Eigen::Matrix<float, 4, Eigen::Dynamic>& points,
    const Eigen::Matrix<float, 3, 4>& projection_matrix, const cv::Mat& image,
    cv::Mat& mask, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_color,
    bool color) {
  if (color && !cloud_color) {
    return;
  }

  // 执行批量投影
  Eigen::Matrix<float, 3, Eigen::Dynamic> projected_points =
      projection_matrix * points;

  // 执行批量投影并归一化
  Eigen::VectorXi u =
      (projected_points.row(0).array() / projected_points.row(2).array())
          .cast<int>();
  Eigen::VectorXi v =
      (projected_points.row(1).array() / projected_points.row(2).array())
          .cast<int>();

  // 转换为图像范围点并过滤
  for (int i = 0; i < projected_points.cols(); ++i) {
    if (projected_points(2, i) <= 0) {
      continue;  // Skip points behind the camera
    }

    if (u[i] >= 0 && v[i] >= 0 && u[i] < image.cols && v[i] < image.rows) {
      // 获取当前点的空间坐标
      // Eigen::Vector3f point(points(0, i), points(1, i), points(2, i));
      // float dist_squared = point.squaredNorm();

      // way 1 opencv bgr
      int type = mask_.type();
      if (type == CV_32FC3) {
        cv::Vec3f& mPix = mask_.at<cv::Vec3f>(v[i], u[i]);
        mPix[0]         = points(2, i);  // B <- Z
        mPix[1]         = points(1, i);  // G <- Y
        mPix[2]         = points(0, i);  // R <- X
      } else if (type == CV_8UC3) {
        cv::Vec3b& mPix = mask.at<cv::Vec3b>(v[i], u[i]);
        mPix[0]         = points(2, i);
        mPix[1]         = points(1, i);
        mPix[2]         = points(0, i);
      }

      // way 2
      // float ==> uchar (0-255)
      // CV_8UC3: 小数会截断; 超过 255 会 clamp 为 255; 小于 0 会 clamp 为 0
      // CV_32FC3: 小数会保留;不会被限制到 0–255; 写进去多少，mask 里就是多少
      // cv::circle(mask, cv::Point(u[i], v[i]), 1,
      //            cv::Scalar(points(2, i), points(1, i), points(0, i)), -1, 8);

      /* debug
      std::cout << "Before write:" << std::endl;
      std::cout << "B = " << points(2, i) << std::endl;
      std::cout << "G = " << points(1, i) << std::endl;
      std::cout << "R = " << points(0, i) << std::endl;
      const cv::Vec3f& pix = mask.at<cv::Vec3f>(v[i], u[i]);
      std::cout << "After write (uchar stored value):" << std::endl;
      std::cout << "B = " << (float)pix[0] << std::endl;
      std::cout << "G = " << (float)pix[1] << std::endl;
      std::cout << "R = " << (float)pix[2] << std::endl;
      std::cout << "\n" << std::endl;
      */

      if (color) {
        const cv::Vec3b& pix = image.at<cv::Vec3b>(v[i], u[i]);
        pcl::PointXYZRGB tmp_point;
        tmp_point.x = points(0, i);
        tmp_point.y = points(1, i);
        tmp_point.z = points(2, i);
        tmp_point.b = pix[0];
        tmp_point.g = pix[1];
        tmp_point.r = pix[2];
        cloud_color->emplace_back(tmp_point);
        // cloud_color_->push_back(tmp_point);
      }
    }
  }
  // cv::imshow("projected_image", mask);
}

void LidarCameraFusion::show_lidar_color_cloud() {
  // clang-format off
  cloud_color_ = 
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  // clang-format on
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cloud_color_->clear();

    pcl::PointXYZRGB tmp_point;
    for (const auto& point : cloud_->points) {
      tmp_point.x = point.x;
      tmp_point.y = point.y;
      tmp_point.z = point.z;

      Eigen::Vector4f point_homo(point.x, point.y, point.z, 1.0f);

      Eigen::Vector3f projected_point =
          projection_matrix_.block<3, 4>(0, 0).cast<float>() * point_homo;

      if (projected_point(2) <= 0) {
        tmp_point.b = 80;
        tmp_point.g = 80;
        tmp_point.r = 80;
        cloud_color_->emplace_back(tmp_point);
        // cloud_color_->push_back(tmp_point);
        continue;
      }

      int u = static_cast<int>(projected_point(0) / projected_point(2));
      int v = static_cast<int>(projected_point(1) / projected_point(2));

      if (u >= 0 && v >= 0 && u < image_.cols && v < image_.rows) {
        const cv::Vec3b& pix = image_.at<cv::Vec3b>(v, u);
        tmp_point.b          = pix[0];
        tmp_point.g          = pix[1];
        tmp_point.r          = pix[2];
      } else {
        tmp_point.b = 125;
        tmp_point.g = 125;
        tmp_point.r = 125;
      }
      cloud_color_->emplace_back(tmp_point);
      // cloud_color_->push_back(tmp_point);
    }
  }
  // show3d_lidar_data(cloud_color_, name_ + "Color");
}

void LidarCameraFusion::show_image_proj() {
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    mask_ = image_.clone();
    project_lidar_to_camera(cloud_, projection_matrix_, image_, mask_);
  }
  cv::imshow("projected_image", mask_);
  cv::waitKey(1);
}

void LidarCameraFusion::SetProjectionMatrix(
    const Eigen::Matrix4f& projection_matrix) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // 取4x4的前3行4列赋值
  projection_matrix_ = projection_matrix.block<3, 4>(0, 0);
}

void LidarCameraFusion::SetProjectionMatrix(
    const Eigen::Matrix<float, 3, 4>& projection_matrix) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  projection_matrix_ = projection_matrix;
}

void LidarCameraFusion::SetLidarPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  cloud_ = cloud;
}

void LidarCameraFusion::SetLidarPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  pcl::copyPointCloud(*cloud, *cloud_);
}

void LidarCameraFusion::SetCameraImage(const cv::Mat& image) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  // undistort_image
  image_ = image;
}

bool LidarCameraFusion::GetFusedPointCloudColor(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_color) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  cloud_color = cloud_color_;
  return true;
}

bool LidarCameraFusion::GetFusedImage(cv::Mat& image) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  image = mask_;
  return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace jojo
