#include "modules/perception/tools/common/show_data_2d.h"

void show2d_lidar_data(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                       const uint& idx, const uint& mode,
                       const std::string& name, cv::Mat* ext_img) {
  // 将点云的 y 坐标映射到图像的 x 坐标，x 坐标映射到图像的 y 坐标并取反
  // 其实只是显示的变换，实际上直观效果仍是 原有坐标系。
  // 在 OpenCV 中，图像的坐标系是左上角为原点 (0, 0)，x 轴向右，y 轴向下。
  // x 轴：从左到右，值增加。
  // y 轴：从上到下，值增加。
  static int width  = 1024;
  static int height = 768;
  // static int width  = 1920;
  // static int height = 1080;

  static float resolution = 100 / 20.0f;
  static int width_half   = width / 2;
  static int height_half  = height / 2;

  cv::Mat LidarImage;
  if (ext_img != nullptr) {
    // 用户传入了 Mat，直接使用，不深拷贝
    LidarImage = *ext_img;
  } else {
    // 用户未传入 Mat，自行创建
    LidarImage = cv::Mat::zeros(height, width, CV_8UC3);
  }

  for (size_t i = 0; i < cloud->size(); i++) {
    int x = (int)(-cloud->points[i].y * resolution + width_half);
    int y = (int)(height_half - cloud->points[i].x * resolution);
    if (x > 0 && y > 0 && x < width && y < height) {
      cv::Vec3b color(0, 97, static_cast<uchar>(cloud->points[i].intensity));

      if (mode == 0) {
        // LidarImage.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 97, 255);
        LidarImage.at<cv::Vec3b>(y, x) = color;
      } else if (mode == 1) {
        // 画半径为 2 的圆点（大原点）
        cv::circle(LidarImage, cv::Point(x, y), 1, color, -1);
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

void show2d_lidar_bev(const base::Point3DF p[8],
                      const Eigen::Vector3f& center, cv::Scalar color,
                      cv::Mat* ext_img) {
  static int width  = 1024;
  static int height = 768;
  // static int width  = 1920;
  // static int height = 1080;

  static float resolution = 100 / 20.0f;
  static int width_half   = width / 2;
  static int height_half  = height / 2;

  cv::Mat bev;
  if (ext_img != nullptr) {
    // 用户传入了 Mat，直接使用，不深拷贝
    // bev = cv::Mat(*ext_img);   // 共享同一数据，更明确
    bev = *ext_img;
  } else {
    // 用户未传入 Mat，自行创建
    bev = cv::Mat::zeros(height, width, CV_8UC3);
  }

  // 投影 bbox 的 8 个点到 BEV
  std::vector<cv::Point> bev_pts(8);

  for (int i = 0; i < 8; i++) {
    float px = p[i].x;  // 前后
    float py = p[i].y;  // 左右

    int x = (int)(-py * resolution + width_half);
    int y = (int)(height_half - px * resolution);

    bev_pts[i] = cv::Point(x, y);
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
  float cx = center.x();
  float cy = center.y();

  int cx_img = (int)(-cy * resolution + width_half);
  int cy_img = (int)(height_half - cx * resolution);

  cv::circle(bev, {cx_img, cy_img}, 3, {0, 0, 255}, -1);
}

void show2d_camera_data(const cv::Mat& image_float, const int max_depth = 100) {
  // 掩码: 非零值
  cv::Mat mask_zero = image_float != 0;
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
  // way 2
  cv::Mat mask_processed = (image_float > max_depth) * max_depth +
                           (image_float <= max_depth).mul(image_float);

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

  // 非零值处理 分离和处理通道
  std::vector<cv::Mat> splits;
  cv::split(image_uint, splits);
  for (int i = 0; i < 3; ++i) {
    splits[i] = splits[i].mul(mask_zero);
  }
  cv::merge(splits, image_uint);

  cv::namedWindow("image_float-->image_uint", cv::WINDOW_NORMAL);
  cv::imshow("image_float-->image_uint", image_uint);
  cv::waitKey(1);
}

void show2d_camera_data(const cv::Mat& image, const uint& idx,
                        const std::string& name) {
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
