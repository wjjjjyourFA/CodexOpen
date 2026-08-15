#include "modules/perception/tools/common/show_data_2d.h"

namespace base = jojo::perception::base;

void show2d_lidar_data(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                       const uint& idx, const uint& mode,
                       const std::string& name, cv::Mat* ext_img) {
  if (!cloud || cloud->empty()) {
    return;
  }

  // 将点云的 y 坐标映射到图像的 x 坐标，x 坐标映射到图像的 y 坐标并取反
  // 其实只是显示的变换，实际上直观效果仍是 原有坐标系。
  // 在 OpenCV 中，图像的坐标系是左上角为原点 (0, 0)，x 轴向右，y 轴向下。
  // x 轴：从左到右，值增加。
  // y 轴：从上到下，值增加。
  int width  = 1024;
  int height = 768;
  // static int width  = 1920;
  // static int height = 1080;

  static constexpr float resolution = 100 / 20.0f;

  cv::Mat LidarImage;
  if (ext_img != nullptr) {
    if (!jojo::perception::tools::IsValidBgrImage(*ext_img)) {
      std::cerr << "ext_img must be a non-empty CV_8UC3 image" << std::endl;
      return;
    }
    // 用户传入了 Mat，直接使用，不深拷贝
    LidarImage = *ext_img;
    width      = LidarImage.cols;
    height     = LidarImage.rows;
  } else {
    // 用户未传入 Mat，自行创建
    LidarImage = cv::Mat::zeros(height, width, CV_8UC3);
  }
  const int width_half  = width / 2;
  const int height_half = height / 2;
  const jojo::perception::tools::BevProjector projector(
      jojo::perception::tools::BevRenderConfig{width, height, resolution});

  for (size_t i = 0; i < cloud->size(); i++) {
    jojo::perception::tools::BevPixel pixel;
    if (projector.Project(cloud->points[i].x, cloud->points[i].y, &pixel)) {
      cv::Vec3b color(0, 97, static_cast<uchar>(cloud->points[i].intensity));

      if (mode == 0) {
        // LidarImage.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 97, 255);
        LidarImage.at<cv::Vec3b>(pixel.y, pixel.x) = color;
      } else if (mode == 1) {
        // 画半径为 2 的圆点（大原点）
        cv::circle(LidarImage, cv::Point(pixel.x, pixel.y), 1, color, -1);
      }
    }
  }

  // 画水平线，穿过图像的中点 (512, 384)
  cv::line(LidarImage, cv::Point(0, height_half), cv::Point(width, height_half),
           cv::Scalar(125, 125, 125));
  // 画垂直线，穿过图像的中点 (512, 384)
  cv::line(LidarImage, cv::Point(width_half, 0), cv::Point(width_half, height),
           cv::Scalar(125, 125, 125));

  std::string win_name = name + "_" + std::to_string(idx);
  cv::namedWindow(win_name, cv::WINDOW_NORMAL);
  cv::resizeWindow(win_name, width, height);
  cv::imshow(win_name, LidarImage);
  cv::waitKey(1);
}

void show2d_lidar_data(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const uint& idx, const uint& mode,
                       const std::string& name, cv::Mat* ext_img) {
  if (!cloud) {
    std::cerr << "Input cloud is null!" << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(
      new pcl::PointCloud<pcl::PointXYZI>);
  ConvertXYZtoXYZI(cloud, cloud_xyzi);

  show2d_lidar_data(cloud_xyzi, idx, mode, name, ext_img);
}

void show2d_lidar_bev(const base::Point3DF p[8], const Eigen::Vector3f& center,
                      cv::Scalar color, cv::Mat* ext_img) {
  if (p == nullptr) {
    return;
  }

  int width  = 1024;
  int height = 768;
  // static int width  = 1920;
  // static int height = 1080;

  static constexpr float resolution = 100 / 20.0f;

  cv::Mat bev;
  if (ext_img != nullptr) {
    if (!jojo::perception::tools::IsValidBgrImage(*ext_img)) {
      std::cerr << "ext_img must be a non-empty CV_8UC3 image" << std::endl;
      return;
    }
    // 用户传入了 Mat，直接使用，不深拷贝
    // bev = cv::Mat(*ext_img);   // 共享同一数据，更明确
    bev = *ext_img;
    width  = bev.cols;
    height = bev.rows;
  } else {
    // 用户未传入 Mat，自行创建
    bev = cv::Mat::zeros(height, width, CV_8UC3);
  }
  const jojo::perception::tools::BevProjector projector(
      jojo::perception::tools::BevRenderConfig{width, height, resolution});

  // 投影 bbox 的 8 个点到 BEV
  std::vector<cv::Point> bev_pts(8);

  for (int i = 0; i < 8; i++) {
    jojo::perception::tools::BevPixel pixel;
    projector.Project(p[i].x, p[i].y, &pixel);
    bev_pts[i] = cv::Point(pixel.x, pixel.y);
  }

  // 前面 (0-1-2-3)
  for (int i = 0; i < 4; i++)
    cv::line(bev, bev_pts[i], bev_pts[(i + 1) % 4], color, 2);

  // 后面 (4-5-6-7)
  for (int i = 4; i < 8; i++)
    cv::line(bev, bev_pts[i], bev_pts[4 + (i + 1) % 4], color, 2);

  // 垂直边
  for (int i = 0; i < 4; i++)
    cv::line(bev, bev_pts[i], bev_pts[i + 4], color, 2);

  // 画中心点
  jojo::perception::tools::BevPixel center_pixel;
  if (projector.Project(center.x(), center.y(), &center_pixel)) {
    cv::circle(bev, {center_pixel.x, center_pixel.y}, 3, {0, 0, 255}, -1);
  }
}

void show2d_camera_data(const cv::Mat& image_float, const int max_depth = 100) {
  if (image_float.empty() || image_float.channels() != 1 ||
      (image_float.depth() != CV_32F && image_float.depth() != CV_64F) ||
      max_depth <= 0) {
    std::cerr << "image_float must be a non-empty floating-point depth image "
                 "and max_depth must be positive"
              << std::endl;
    return;
  }

  // 掩码: 正深度值
  cv::Mat valid_mask = image_float > 0;
  /* way 1
  // 掩码: 深度值大于 maxdepth
  cv::Mat mask_above_maxdepth = image_float > max_depth;
  // 掩码: 深度值小于等于 maxdepth
  cv::Mat mask_below_maxdepth = image_float <= max_depth;

  // 确保掩码为 8 位单通道 useless
  // mask_zero.convertTo(mask_zero, CV_8U);
  // mask_above_maxdepth.convertTo(mask_above_maxdepth, CV_8U);
  // mask_below_maxdepth.convertTo(mask_below_maxdepth, CV_8U);

  // 应用掩码
  cv::Mat mask_processed =
      image_float.mul(mask_below_maxdepth) + mask_above_maxdepth * max_depth;
  */
  cv::Mat mask_processed;
  cv::min(image_float, static_cast<double>(max_depth), mask_processed);
  mask_processed.setTo(0, valid_mask == 0);

  // 归一化深度值到 [0, 255]
  double minVal, maxVal;
  cv::minMaxIdx(mask_processed, &minVal, &maxVal);
  if (maxVal > minVal) {
    mask_processed = 255 * (mask_processed - minVal) / (maxVal - minVal);
  } else {
    mask_processed.setTo(0);
  }

  cv::Mat image_uint = cv::Mat::zeros(image_float.size(), CV_8UC1);
  mask_processed.convertTo(image_uint, CV_8U);

  // 应用伪彩色映射
  cv::applyColorMap(image_uint, image_uint, cv::COLORMAP_JET);

  // 无效深度保持黑色。
  image_uint.setTo(cv::Scalar(0, 0, 0), valid_mask == 0);

  cv::namedWindow("image_float-->image_uint", cv::WINDOW_NORMAL);
  cv::imshow("image_float-->image_uint", image_uint);
  cv::waitKey(1);
}

void show2d_camera_data(const cv::Mat& image, const uint& idx,
                        const std::string& name) {
  if (image.empty()) {
    return;
  }

  cv::namedWindow(name + "_" + std::to_string(idx), cv::WINDOW_NORMAL);
  cv::imshow(name + "_" + std::to_string(idx), image);
  cv::waitKey(1);
}

void show_cv_splits_cloud(std::vector<cv::Mat>& splits) {
  if (splits.size() < 3 || splits[0].type() != CV_32F ||
      splits[1].type() != CV_32F || splits[2].type() != CV_32F) {
    std::cerr << "Invalid input splits for show_cv_splits_cloud" << std::endl;
    return;
  }

  if (!splits[0].isContinuous() || !splits[1].isContinuous() ||
      !splits[2].isContinuous()) {
    std::cerr << "Splits must be continuous!" << std::endl;
    return;
  }

  if (splits[0].size() != splits[1].size() ||
      splits[0].size() != splits[2].size()) {
    std::cerr << "Splits must have identical dimensions!" << std::endl;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  float* x_ptr = reinterpret_cast<float*>(splits[2].data);
  float* y_ptr = reinterpret_cast<float*>(splits[1].data);
  float* z_ptr = reinterpret_cast<float*>(splits[0].data);

  size_t total = splits[0].total();  // 总元素数
  cloud->points.reserve(total);
  for (size_t index = 0; index < total; index++) {
    // 操作的是点云投影的mask图像
    if (x_ptr[index] == 0 && y_ptr[index] == 0 && z_ptr[index] == 0) continue;

    pcl::PointXYZ pt;
    pt.x = x_ptr[index];
    pt.y = y_ptr[index];
    pt.z = z_ptr[index];

    cloud->points.emplace_back(pt);
  }

  cloud->width    = cloud->points.size();
  cloud->height   = 1;
  cloud->is_dense = false;
  // std::cout << "show_cv_splits_cloud size: " << cloud->points.size() << std::endl;

  show2d_lidar_data(cloud, 0, 0, "show_cv_splits_cloud");
}

int display_image_pause(const std::string& window_title, const cv::Mat& image) {
  if (image.empty()) {
    return -1;
  }

  cv::imshow(window_title, image);

  while (true) {
    // 检查窗口是否被关闭
    if (cv::getWindowProperty(window_title, cv::WND_PROP_AUTOSIZE) < 0) {
      return -1;
    }

    int key = cv::waitKey(1) & 0xFF;
    if (key == 'q' || key == 'Q') {
      return -1;
    }

    // 检测 Enter 键
    if (key == 13) {
      return 1;
    }
  }
}
