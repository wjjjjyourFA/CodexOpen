#include "modules/tools/sensor_calibration/camera_intrinsic/calib_verification/calibration_harp.hpp"

namespace tools   = jojo::tools;

CalibrationHarp::CalibrationHarp() { return; }

bool CalibrationHarp::Init(const std::string& img_dir_path) {
  std::cout << "img_dir_path: " << img_dir_path << std::endl;

  image_path_ = img_dir_path;

  initialized_ = true;

  return true;
}

bool CalibrationHarp::Init(std::shared_ptr<tools::RuntimeConfig> param) {
  param_ = param;

  if (!this->Init(param_->image_file)) {
    return false;
  }

  initialized_ = true;

  return true;
}

// void maskImage(cv::Mat &img, std::vector)

bool CalibrationHarp::Measure(double& _d, double& _d_max) {
  if (!initialized_) {
    return false;
  }

  // get gray image
  cv::Mat image = cv::imread(this->image_path_, 0);
  image.copyTo(origin_image_);

  /* way 1
  cv::Ptr<cv::LineSegmentDetector> LSD = cv::createLineSegmentDetector();
  std::vector<cv::Vec4f> lines;
  LSD->detect(image, lines);
  */
  // way 2
  // get line edge points
  // get line support region
  LineSegmentDetector LSD;
  // LSD.updateHyperparam();
  LSD.init();
  LSD.detect(image);

  std::vector<cstruct::Vector4f> lines;
  std::vector<std::vector<int>> line_point_map;
  LSD.getResults(lines, line_point_map);

  // canny edge detect
  cv::Mat blur_image;
  cv::Mat edge_image;
  cv::GaussianBlur(image, blur_image, cv::Size(3, 3), 0);
  cv::Canny(blur_image, edge_image, 20, 40);
  // cv::imwrite("canny.jpg", edge_image);
  // cv::imshow("canny", edge_image);
  // cv::waitKey(0);

  // get edge line point in line support region
  // clang-format off
  int line_num = lines.size();
  std::vector<std::vector<base::Point2DI>> edge_line_points(line_num);
  cv::Mat edge_point_image(edge_image.size(), edge_image.type(), cv::Scalar(0));
  for (int i = 0; i < line_point_map.size(); i++) {
    for (int j = 0; j < line_point_map[0].size(); j++) {
      auto& line_idx = line_point_map[i][j];  //  1-based index
      if (edge_image.at<uchar>(i, j) == 255 && line_idx != 0) {
        edge_point_image.at<uchar>(i, j) = 255;
        edge_line_points[line_idx - 1].push_back(base::Point2DI(j, i));
      }
    }
  }
  // cv::imwrite("edge_point_image.jpg", edge_point_image);
  // cv::imshow("edge_point_image", edge_point_image);
  // cv::waitKey();
  // clang-format on

  cv::Mat drawn_img;
  origin_image_.copyTo(drawn_img);
  // LSD.drawSegments(drawn_img, lines);
  // LSD.drawSegments(drawn_img, edge_line_points);
  // cv::imwrite("edge_line_points.jpg", drawn_img);
  // cv::imshow("edge_line_points", drawn_img);
  // cv::waitKey();

  // interpolate line points
  std::vector<std::vector<base::Point2DI>> interpolated_edge_points;
  this->interpolate_edge_points(lines, edge_line_points,
                                interpolated_edge_points);

  // gaussion blur and subsample points
  std::vector<std::vector<base::Point2DI>> subsampled_edge_points;
  this->gaussion_subsample_points(interpolated_edge_points,
                                  subsampled_edge_points, 20);

  origin_image_.copyTo(drawn_img);
  LSD.drawSegments(drawn_img, subsampled_edge_points);
  // cv::imwrite("subsampled_edge_points.jpg", drawn_img);
  cv::imshow("subsampled_edge_points", drawn_img);
  cv::waitKey();
  cv::destroyAllWindows();

  // calculate error
  double d, d_max, d_c_median;
  this->calculate_error(subsampled_edge_points, d, d_max, d_c_median);

  _d     = d;
  _d_max = d_max;

  std::cout << "d: " << d << " pixels" << std::endl;
  std::cout << "d_max: " << d_max << " pixels" << std::endl;
  std::cout << "d_c_median: " << d_c_median << " pixels" << std::endl;

  return true;
}

bool CalibrationHarp::interpolate_edge_points(
    const std::vector<cstruct::Vector4f>& lines,
    const std::vector<std::vector<base::Point2DI>>& edge_points,
    std::vector<std::vector<base::Point2DI>>& interpolated_edge_points) {
  // 对每一条检测到的直线，把其对应的边缘像素点按线方向排序，并沿线段长度做“等弧长插值重采样”，生成分布均匀的新点集
  // lines            // 每条线段的两个端点 (x1, y1, x2, y2)
  // edge_points[i]   // 第 i 条线段上原始提取的边缘像素点（不规则、稀疏）

  // sort line points
  int line_num = lines.size();
  std::vector<std::vector<base::Point2DI>> sorted_edge_points(edge_points);
  for (int i = 0; i < line_num; i++) {
    double rough_angle = abs(static_cast<double>(lines[i].x - lines[i].z) /
                             static_cast<double>(lines[i].y - lines[i].w));
    if (rough_angle < 1) {
      // 更接近竖直线（dy 大） → 按 y 排
      std::sort(sorted_edge_points[i].begin(), sorted_edge_points[i].end(),
                ComparePointYAsc);
    } else {
      // 更接近水平线（dx 大） → 按 x 排
      std::sort(sorted_edge_points[i].begin(), sorted_edge_points[i].end(),
                ComparePointXAsc);
    }
  }

  // interpolate line points
  // 计算各线的“沿线的累积弧长”
  interpolated_edge_points.resize(line_num);
  std::vector<std::vector<double>> line_point_distance(line_num);
  for (int i = 0; i < line_num; i++) {
    if (edge_points[i].size() == 0) continue;
    // double line_length = (lines[i].x - lines[i].z) * (lines[i].x - lines[i].z) +
    //                      (lines[i].y - lines[i].w) * (lines[i].y - lines[i].w);
    // line_length = sqrt(line_length);

    // D[0] = 0
    // D[k] = D[k-1] + || P[k] - P[k-1] ||
    line_point_distance[i].push_back(0);
    for (int idx = 0; idx < sorted_edge_points[i].size() - 1; idx++) {
      double dx =
          sorted_edge_points[i][idx].x - sorted_edge_points[i][idx + 1].x;
      double dy =
          sorted_edge_points[i][idx].y - sorted_edge_points[i][idx + 1].y;
      double dist = sqrt(dx * dx + dy * dy);
      line_point_distance[i].push_back(line_point_distance[i][idx] + dist);
    }
  }

  // 等弧长重采样
  for (int i = 0; i < line_num; i++) {
    if (sorted_edge_points[i].size() == 0) continue;

    // 总长度
    double L = line_point_distance[i][line_point_distance[i].size() - 1];
    // 原始点数
    double N = sorted_edge_points[i].size();
    // 目标采样间隔
    // double d = L / N;  // 最后一个端点通常不会被采样到。
    double d = L / (N - 1);
    // std::cout << "\nd: " << d << " pixels\n";

    double L_pos = 0;
    double p_pos = 1;

    interpolated_edge_points[i].push_back(sorted_edge_points[i][0]);

    while (p_pos < N) {
      L_pos += d;
      while (p_pos < N && L_pos > line_point_distance[i][p_pos]) {
        p_pos++;
      }
      if (p_pos >= N) break;

      // 在相邻两个边缘点之间，按比例插一个“虚拟点”
      // 线性插值：P = (b / (a+b)) * P[k-1] + (a / (a+b)) * P[k]
      double a     = L_pos - line_point_distance[i][p_pos - 1];
      double b     = line_point_distance[i][p_pos] - L_pos;
      double x1    = sorted_edge_points[i][p_pos - 1].x;
      double y1    = sorted_edge_points[i][p_pos - 1].y;
      double x2    = sorted_edge_points[i][p_pos].x;
      double y2    = sorted_edge_points[i][p_pos].y;
      double new_x = b / (a + b) * x1 + a / (a + b) * x2;
      double new_y = b / (a + b) * y1 + a / (a + b) * y2;

      // clang-format off
      interpolated_edge_points[i].push_back(base::Point2DI(int(new_x + 0.5), int(new_y + 0.5)));
      // interpolated_edge_points[i].push_back(base::Point2DI(int(new_x), int(new_y)));
      // clang-format on
    }
  }

  return true;
}

bool CalibrationHarp::gaussion_subsample_points(
    const std::vector<std::vector<base::Point2DI>>& interpolated_edge_points,
    std::vector<std::vector<base::Point2DI>>& subsampled_edge_points,
    const int t) {
  // 将各线段的均匀点进行高斯平滑 噪声小、稳定性更好

  int line_num = interpolated_edge_points.size();
  subsampled_edge_points.resize(line_num);

  // 经验公式
  const double sigma = 0.8 * sqrt(t * t - 1);
  const double deno  = 1.0 / (sigma * sqrt(2.0 * M_PI));
  const double nume  = -1.0 / (2.0 * sigma * sigma);

  // generate gauss matrix
  std::vector<double> gauss_matrix;
  double gauss_matrix_sum = 0;
  for (int i = 0, x = -t; x <= t; i++, x++) {
    double g = deno * exp(nume * x * x);
    gauss_matrix.push_back(g);
    gauss_matrix_sum += g;
  }
  // normalize
  for (int i = 0; i < gauss_matrix.size(); i++) {
    gauss_matrix[i] /= gauss_matrix_sum;
  }

  // gaussian blur
  for (int line_idx = 0; line_idx < line_num; line_idx++) {
    int point_size = interpolated_edge_points[line_idx].size();
    if (point_size == 0) continue;

    // 下采样：每隔 t 个点，取一个中心 p
    for (int p = t; p < point_size - t; p += t) {
      double gauss_x_sum = 0;
      double gauss_y_sum = 0;
      double gauss_sum   = 0;
      for (int i = -t; i <= t; i++) {
        int k = p + i;
        if (k >= 0 && k < point_size) {
          double x = interpolated_edge_points[line_idx][k].x;
          double y = interpolated_edge_points[line_idx][k].y;
          gauss_x_sum += x * gauss_matrix[i + t];
          gauss_y_sum += y * gauss_matrix[i + t];
          gauss_sum += gauss_matrix[i + t];
        }
      }
      if (gauss_sum == 0) continue;
      int selected_x = static_cast<int>(gauss_x_sum / gauss_sum + 0.5);
      int selected_y = static_cast<int>(gauss_y_sum / gauss_sum + 0.5);
      // std::cout << selected_x << "\t" << selected_y << std::endl;

      // clang-format off
      subsampled_edge_points[line_idx].push_back(base::Point2DI(selected_x, selected_y));
      // clang-format on
    }
  }

  return true;
}

bool CalibrationHarp::calculate_error(
    // const std::vector<cstruct::Vector4f> &lines,
    // const std::vector< std::vector<base::Point2DI> > &edge_points,
    const std::vector<std::vector<base::Point2DI>>& subsampled_edge_point,
    double& d, double& d_max, double& d_c_median) {
  // double& d;           // 所有点的均方根误差 RMS
  // double& d_max;       // 所有线的最大-最小差平方均值的平均开方
  // double& d_c_median;  // 最大单条线的最大-最小差值

  int real_line_num   = 0;
  int total_point_num = 0;
  double S            = 0;
  double S_max_total  = 0;
  double S_d_max      = 0;

  for (int i = 0; i < subsampled_edge_point.size(); i++) {
    if (subsampled_edge_point[i].size() == 0) continue;
    // std::cout << std::endl;
    int point_num = subsampled_edge_point[i].size();
    real_line_num++;
    total_point_num += point_num;

    // 拟合直线参数
    double alpha, beta, gama;
    this->get_line_param(subsampled_edge_point[i], alpha, beta, gama);
    // std::cout << alpha << "\t" << beta << "\t" << gama << std::endl;

    // 计算点到直线的误差
    double Si_max = -10000;
    double Si_min = 10000;
    for (int j = 0; j < point_num; j++) {
      double x  = subsampled_edge_point[i][j].x;
      double y  = subsampled_edge_point[i][j].y;
      double Si = (alpha * x + beta * y - gama);
      // Si 表示点到直线的代数距离（未归一化）
      double Si_square = Si * Si;
      // S 累加所有点的平方误差，用于 RMS 计算
      S += Si_square;
      // 最大、最小的代数距离 ==> 用于计算 单条线的误差范围
      if (Si > Si_max) Si_max = Si;
      if (Si < Si_min) Si_min = Si;

      // std::cout << "x=" << x << "   " << "y=" << y
      //           << "  Si=" << Si << std::endl;
    }
    // std::cout << "Si_max: " << Si_max << std::endl;
    // std::cout << "Si_min: " << Si_min << std::endl;
    // 累加全局最大差
    S_max_total += (Si_max - Si_min) * (Si_max - Si_min);
    // 最大单条线误差范围
    double max_current = Si_max - Si_min;
    if (max_current > S_d_max) S_d_max = max_current;
  }
  d          = sqrt(S / static_cast<double>(total_point_num));
  d_max      = sqrt(S_max_total / static_cast<double>(real_line_num));
  d_c_median = S_d_max;

  std::cout << "Detected " << real_line_num << " lines. " << total_point_num
            << " edge points.\n";
  return true;
}

bool CalibrationHarp::get_line_param(
    const std::vector<base::Point2DI>& edge_points, double& alpha, double& beta,
    double& gama) {
  double Ax = 0, Ay = 0;
  int point_num = edge_points.size();
  // if (point_num == 0) {
  //     std::cerr << "[ERROR] edge_points empty!" << std::endl;
  //     return false;
  // }

  for (int i = 0; i < point_num; i++) {
    Ax += edge_points[i].x;
    Ay += edge_points[i].y;
  }
  Ax /= static_cast<double>(point_num);
  Ay /= static_cast<double>(point_num);

  // 通过重心 + 主方向确定的一条直线
  // (x−Ax)sinθ − (y−Ay)cosθ = 0
  double theta = this->get_theta(edge_points, Ax, Ay);
  alpha        = sin(theta);
  beta         = -cos(theta);
  gama         = Ax * sin(theta) - Ay * cos(theta);

  return true;
}

double CalibrationHarp::get_theta(
    const std::vector<base::Point2DI>& edge_points, const double& Ax,
    const double& Ay) {
  // (Ax, Ay) 是点集重心
  // [Vxx Vxy; Vxy Vyy] 是点集的协方差矩阵：

  double Vxx = 0, Vxy = 0, Vyy = 0;
  int point_num = edge_points.size();
  for (int i = 0; i < point_num; i++) {
    double x = edge_points[i].x;
    double y = edge_points[i].y;
    Vxx += (x - Ax) * (x - Ax);
    Vxy += (x - Ax) * (y - Ay);
    Vyy += (y - Ay) * (y - Ay);
  }

  Vxx /= static_cast<double>(point_num);
  Vxy /= static_cast<double>(point_num);
  Vyy /= static_cast<double>(point_num);

  // PCA 主方向求法
  double theta = atan2(2 * Vxy, Vxx - Vyy) / 2.0;
  // std::cout << "theta = " << theta * 180 / M_PI << std::endl;
  return theta;
}