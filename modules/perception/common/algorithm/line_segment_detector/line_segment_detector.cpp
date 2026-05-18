#include "modules/perception/common/algorithm/line_segment_detector/line_segment_detector.hpp"

LineSegmentDetector::LineSegmentDetector()
    : img_width(0),
      img_height(0),
      LOG_NT(0),
      w_needed(false),
      p_needed(false),
      n_needed(false) {}

bool LineSegmentDetector::updateHyperparam(
    const LineSegmentDetectorHyperparam& hps) {
  hps_ = hps;

  if (!(hps_.scale > 0 && hps_.sigma_scale > 0 && hps_.quant >= 0 &&
        hps_.ang_th > 0 && hps_.ang_th < 180 && hps_.density_th >= 0 &&
        hps_.density_th < 1 && hps_.n_bins > 0)) {
    std::cerr << "[ERROR] WRONG INPUT PARAM" << std::endl;
    return false;
  }

  return true;
}

void LineSegmentDetector::init() {
  // w_needed = this->_width.needed();
  // p_needed = this->_prec.needed();
  w_needed = true;
  p_needed = true;
  // 0 ==> 不进行线条细化
  if (hps_.refine < LSD_REFINE_ADV) {
    n_needed = false;
  } else {
    // n_needed = this->_nfa.needed();
    n_needed = false;
  }
}

void LineSegmentDetector::detect(cv::Mat& _image) {
  // image = _image.getMat();
  image = _image;
  if (image.empty() && image.type() != CV_8UC1) {
    std::cerr << "[ERROR] WRONG INPUT IMAGE PARAM" << std::endl;
    return;
  }

  // w: 线宽
  // p: 置信度/精度
  // n: NFA（false alarm 概率）
  std::vector<double> w, p, n;

  // 有点像霍夫变换的步骤
  _line_point_map.resize(image.rows, std::vector<int>(image.cols));
  this->flsd(_lines, _line_points, _line_point_map, w, p, n);

  if (w_needed) _width = w;
  if (p_needed) _prec = p;
  if (n_needed) _nfa = n;

  // Clear used structures
  ordered_points.clear();
}

void LineSegmentDetector::flsd(
    std::vector<cstruct::Vector4f>& lines,
    std::vector<std::vector<base::Point2DI>>& line_points,
    std::vector<std::vector<int>>& line_point_map, std::vector<double>& widths,
    std::vector<double>& precisions, std::vector<double>& nfas) {
  // Angle tolerance
  // 梯度方向允许多少角度偏差，越小检测越精确，但容易漏线。
  // 角度容差（弧度制）
  const double prec = M_PI * hps_.ang_th / 180;
  // 角度比例
  const double p = hps_.ang_th / 180;
  // 梯度强度阈值（小角度时 sin(prec) 很小，意味着更严格）
  // gradient magnitude threshold
  const double rho = hps_.quant / sin(prec);

  if (hps_.scale != 1) {
    // 通过 GaussianBlur 去除高频噪声
    cv::Mat gaussian_img;
    const double sigma =
        (hps_.scale < 1) ? (hps_.sigma_scale / hps_.scale) : (hps_.sigma_scale);
    const double sprec = 3;
    const unsigned int h =
        (unsigned int)(ceil(sigma * sqrt(2 * sprec * log(10.0))));
    cv::Size ksize(1 + 2 * h, 1 + 2 * h);  // kernel size
    cv::GaussianBlur(image, gaussian_img, ksize, sigma);

    // Scale image to needed size
    // clang-format off
    cv::resize(gaussian_img, scaled_image, cv::Size(), hps_.scale, hps_.scale, cv::INTER_LINEAR);
    // cv::resize(gaussian_img, scaled_image, cv::Size(), hps_.scale, hps_.scale, cv::INTER_CUBIC);
    // clang-format on
  } else {
    scaled_image = image;
  }
  // 计算梯度方向直方图：计算每个像素的梯度角度和强度。
  this->ll_angle(scaled_image, angles, modgrad, ordered_points, rho,
                 hps_.n_bins);

  this->img_width  = scaled_image.cols;
  this->img_height = scaled_image.rows;

  // 计算统计显著性阈值
  // clang-format off
  LOG_NT = 5 * (log10(double(img_width)) + log10(double(img_height))) / 2 + log10(11.0);
  // clang-format on
  // 最小区域像素数阈值（小于这个区域不可信）
  // minimal number of points in region that can give a meaningful event
  const size_t min_reg_size = size_t(-LOG_NT / log10(p)) * 3;
  // const size_t min_reg_size = 1500;

  // Initialize region only when needed
  // cv::Mat region = cv::Mat::zeros(scaled_image.size(), CV_8UC1);
  used = cv::Mat_<uchar>::zeros(scaled_image.size());  // zeros = hps_.NOTUSED
  std::vector<RegionPoint> reg;

  // Search for line segments
  // clang-format off
  for (size_t i = 0, points_size = ordered_points.size(); i < points_size; ++i) {
    const base::Point2DI& point = ordered_points[i].p;
    // std::cout << point.x << "  " << point.y;
    // if((used.at<uchar>(point) == hps_.NOTUSED) && (angles.at<double>(point) != hps_.NOTDEF))
    if ((used.at<uchar>(point.y, point.x) == hps_.NOTUSED) &&
        (angles.at<double>(point.y, point.x) != hps_.NOTDEF)) {
      // 从“最强梯度点”开始生长
      double reg_angle;
      this->region_grow(ordered_points[i].p, angles, modgrad, used, reg, reg_angle, prec);
      // clang-format on

      // Ignore small regions
      if (reg.size() < min_reg_size) {
        continue;
      }

      // Construct rectangular approximation for the region
      rect rec;
      this->region2rect(reg, reg_angle, prec, p, rec);
      // this->drawRect(this->scaled_image, rec);

      double log_nfa = -1;
      if (hps_.refine > LSD_REFINE_NONE) {
        // At least REFINE_STANDARD lvl.
        if (!this->refine(reg, reg_angle, prec, p, rec, hps_.density_th)) {
          continue;
        }

        if (hps_.refine >= LSD_REFINE_ADV) {
          // Compute NFA
          log_nfa = this->rect_improve(rec);
          if (log_nfa <= hps_.log_eps) {
            continue;
          }
        }
      }
      // Found new line

      // Add the offset
      rec.x1 += 0.5;
      rec.y1 += 0.5;
      rec.x2 += 0.5;
      rec.y2 += 0.5;

      // scale the result values if a sub-sampling was performed
      if (hps_.scale != 1) {
        rec.x1 /= hps_.scale;
        rec.y1 /= hps_.scale;
        rec.x2 /= hps_.scale;
        rec.y2 /= hps_.scale;
        rec.width /= hps_.scale;
        rec.x /= hps_.scale;
        rec.y /= hps_.scale;
        // this->drawRect(this->image, rec);
      }

      // Store the relevant data
      // std::cout << "\nFind Line " << line_points.size() << "("<< reg.size() << ")!\n";
      // 用 两个端点 表示一条线段
      lines.push_back(cstruct::Vector4f(float(rec.x1), float(rec.y1),
                                        float(rec.x2), float(rec.y2)));
      // 第 i 条线段 对应的 所有像素点
      // 1-based index
      // 第 1 条线 → index = 1
      // 0 被留出来表示 “无所属线段”
      line_points.push_back(std::vector<base::Point2DI>());
      int line_index = line_points.size();
      // this->get_edge_point(reg, reg_angle, line_points[line_points.size() - 1]);

      // 把 reg 中的像素“归档”到这条线
      for (size_t p_idx = 0; p_idx < reg.size(); p_idx++) {
        int x = int((reg[p_idx].x + 0.5) / hps_.scale);
        int y = int((reg[p_idx].y + 0.5) / hps_.scale);
        line_points[line_points.size() - 1].push_back(base::Point2DI(x, y));
        // 建立“像素 → 线段”的反向索引 ==> O(1) 查询某个像素“被哪条线段解释了”
        // 每个像素存：
        //   0：不属于任何线段
        //   k：属于第 k 条线段
        if (y < line_point_map.size() && x < line_point_map[0].size()) {
          line_point_map[y][x] = line_index;
        }
      }

      if (w_needed) widths.push_back(rec.width);
      if (p_needed) precisions.push_back(rec.p);
      if (n_needed && hps_.refine >= LSD_REFINE_ADV) nfas.push_back(log_nfa);
    }
  }

  bool show = false;
  if (show) {
    // cv::imshow("scaled_image", scaled_image);
    cv::imshow("image", image);
    cv::waitKey(0);
  }
}

void LineSegmentDetector::ll_angle(const cv::Mat& image,
                                   cv::Mat_<double>& angles,
                                   cv::Mat_<double>& modgrad,
                                   std::vector<NormPoint>& ordered_points,
                                   const double& threshold,
                                   const unsigned int& n_bins) {
  CV_Assert(image.type() == CV_8UC1);

  // Initialize data
  angles  = cv::Mat_<double>(image.size());
  modgrad = cv::Mat_<double>(image.size());
  ordered_points.clear();

  const int img_width  = image.cols;
  const int img_height = image.rows;

  // Undefined the down and right boundaries
  angles.row(img_height - 1).setTo(hps_.NOTDEF);
  angles.col(img_width - 1).setTo(hps_.NOTDEF);

  // Computing gradient for remaining pixels
  double max_grad = -1;
  for (int y = 0; y < img_height - 1; ++y) {
    const uchar* image_row      = image.ptr<uchar>(y);
    const uchar* next_image_row = image.ptr<uchar>(y + 1);
    double* angles_row          = angles.ptr<double>(y);
    double* modgrad_row         = modgrad.ptr<double>(y);
    for (int x = 0; x < img_width - 1; ++x) {
      // LSD 2x2 梯度计算
      int DA = next_image_row[x + 1] - image_row[x];
      int BC = image_row[x + 1] - next_image_row[x];
      int gx = DA + BC;  // gradient x component
      int gy = DA - BC;  // gradient y component
      // gradient norm
      double norm = std::sqrt((gx * gx + gy * gy) / 4.0);

      modgrad_row[x] = norm;  // store gradient

      if (norm <= threshold)  // norm too small, gradient no defined
      {
        angles_row[x] = hps_.NOTDEF;
      } else {
        // gradient angle computation
        angles_row[x] = cv::fastAtan2(float(gx), float(-gy)) *
                        math::UnitConverter::DEG_TO_RAD;
        if (norm > max_grad) {
          max_grad = norm;
        }
      }
    }
  }

  // Compute histogram of gradient values
  // If all image is smooth, max_grad <= 0
  double bin_coef = (max_grad > 0) ? double(n_bins - 1) / max_grad : 0;
  for (int y = 0; y < img_height - 1; ++y) {
    const double* modgrad_row = modgrad.ptr<double>(y);
    for (int x = 0; x < img_width - 1; ++x) {
      NormPoint _point;
      int i       = int(modgrad_row[x] * bin_coef);
      _point.p    = base::Point2DI(x, y);
      _point.norm = i;
      ordered_points.push_back(_point);
    }
  }

  // Sort
  std::sort(ordered_points.begin(), ordered_points.end(), compareNorm);

  bool show = false;
  if (show) {
    // modgrad
    cv::Mat modgrad_vis;
    double minv, maxv;
    cv::minMaxLoc(modgrad, &minv, &maxv);
    // 归一化到 0~255
    modgrad.convertTo(modgrad_vis, CV_8UC1, 255.0 / maxv);
    // jet color
    // cv::applyColorMap(modgrad_vis, modgrad_vis, cv::COLORMAP_JET);
    // 显示
    cv::imshow("modgrad", modgrad_vis);
    cv::waitKey(0);

    // angles
    cv::Mat hsv(image.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int y = 0; y < image.rows - 1; ++y) {
      for (int x = 0; x < image.cols - 1; ++x) {
        double a = angles(y, x);
        if (a == hps_.NOTDEF) continue;

        uchar H =
            static_cast<uchar>(a * 180.0 / CV_PI / 2.0);  // [0,2π) → [0,180)
        hsv.at<cv::Vec3b>(y, x) = cv::Vec3b(H, 255, 255);
      }
    }
    cv::Mat angle_bgr;
    cv::cvtColor(hsv, angle_bgr, cv::COLOR_HSV2BGR);
    cv::imshow("angles", angle_bgr);
    cv::waitKey(0);

    cv::Mat vis;
    cv::cvtColor(image, vis, cv::COLOR_GRAY2BGR);

    int N;
    // 画梯度最大的前 1% 像素
    N = ordered_points.size() * 0.01;
    // N = ordered_points.size();
    for (int i = 0; i < N; ++i) {
      const auto& p = ordered_points[i].p;
      cv::circle(vis, cv::Point(p.x, p.y), 1, cv::Scalar(0, 0, 255), -1);
    }

    cv::imshow("ordered_points_top", vis);
    cv::waitKey(0);
  }
}

void LineSegmentDetector::region_grow(const base::Point2DI& s,
                                      cv::Mat_<double>& angles,
                                      cv::Mat_<double>& modgrad,
                                      cv::Mat_<uchar>& used,
                                      std::vector<RegionPoint>& reg,
                                      double& reg_angle, const double& prec) {
  /* LSD 稳定的核心原因之一：
    sumdx += cos(angle);
    sumdy += sin(angle);
    reg_angle = atan2(sumdy, sumdx);
    这一步非常重要：
      region 的方向不是固定的
      会随着新点加入逐渐“收敛”
    这解决了：
      噪声点
      梯度角轻微抖动
      线段略有弯曲的情况
  */

  reg.clear();

  // Point to this region
  RegionPoint seed;
  seed.x       = s.x;
  seed.y       = s.y;
  seed.used    = &used.at<uchar>(s.y, s.x);
  reg_angle    = angles.at<double>(s.y, s.x);
  seed.angle   = reg_angle;
  seed.modgrad = modgrad.at<double>(s.y, s.x);
  reg.push_back(seed);

  // 把“角度”转成单位向量，做 向量累加 → 平均方向
  // 避免角度在 0 / 2π 处不连续的问题。
  float sumdx = float(std::cos(reg_angle));
  float sumdy = float(std::sin(reg_angle));
  *seed.used  = hps_.USED;

  // Try neighboring regions
  // BFS 区域扩张 reg 的大小是动态增长的
  for (size_t i = 0; i < reg.size(); i++) {
    const RegionPoint& rpoint = reg[i];
    // 检查 8 邻域
    int xx_min = std::max(rpoint.x - 1, 0),
        xx_max = std::min(rpoint.x + 1, img_width - 1);
    int yy_min = std::max(rpoint.y - 1, 0),
        yy_max = std::min(rpoint.y + 1, img_height - 1);
    for (int yy = yy_min; yy <= yy_max; ++yy) {
      uchar* used_row           = used.ptr<uchar>(yy);
      const double* angles_row  = angles.ptr<double>(yy);
      const double* modgrad_row = modgrad.ptr<double>(yy);
      for (int xx = xx_min; xx <= xx_max; ++xx) {
        uchar& is_used = used_row[xx];
        if (is_used != hps_.USED &&
            (this->isAligned(angles, xx, yy, reg_angle, prec))) {
          const double& angle = angles_row[xx];
          // Add point
          is_used = hps_.USED;
          RegionPoint region_point;
          region_point.x       = xx;
          region_point.y       = yy;
          region_point.used    = &is_used;
          region_point.modgrad = modgrad_row[xx];
          region_point.angle   = angle;
          reg.push_back(region_point);

          // Update region's angle
          sumdx += cos(float(angle));
          sumdy += sin(float(angle));
          // reg_angle is used in the isAligned, so it needs to be updates?
          reg_angle =
              cv::fastAtan2(sumdy, sumdx) * math::UnitConverter::DEG_TO_RAD;
        }
      }
    }
  }

  // reg = 一条线段的 所有支持像素集合
}

inline bool LineSegmentDetector::isAligned(const cv::Mat_<double>& angles,
                                           int x, int y, const double& theta,
                                           const double& prec) const {
  if (x < 0 || y < 0 || x >= angles.cols || y >= angles.rows) {
    return false;
  }

  const double& a = angles.at<double>(y, x);
  // 这个像素还没被用过 保证：每个像素只属于一条线段
  if (a == hps_.NOTDEF) {
    return false;
  }

  // It is assumed that 'theta' and 'a' are in the range [-pi,pi]
  // 新点的梯度方向 是否和当前 region 的主方向 差异小于 prec
  double n_theta = theta - a;
  if (n_theta < 0) {
    n_theta = -n_theta;
  }
  if (n_theta > M_3_2_PI) {
    n_theta -= M_2_1_PI;
    if (n_theta < 0) n_theta = -n_theta;
  }

  return n_theta <= prec;
}

void LineSegmentDetector::region2rect(const std::vector<RegionPoint>& reg,
                                      const double reg_angle, const double prec,
                                      const double p, rect& rec) const {
  // 把一条已经生长完成的线段支持区域（region），
  // 拟合成一个“方向正确、长度最小、宽度最小”的旋转矩形（rect）

  double x = 0, y = 0, sum = 0;
  for (size_t i = 0; i < reg.size(); ++i) {
    const RegionPoint& pnt = reg[i];
    const double& weight   = pnt.modgrad;
    x += double(pnt.x) * weight;
    y += double(pnt.y) * weight;
    sum += weight;
  }

  // Weighted sum must differ from 0
  if (sum <= 0) {
    std::cerr << "[ERROR] WEIGHTED SUM ERROR IN region2rect" << std::endl;
    return;
  }

  x /= sum;
  y /= sum;

  // 估计线段方向θ （不是梯度方向）
  double theta = this->get_theta(reg, x, y, reg_angle, prec);

  // Find length and width
  double dx    = cos(theta);
  double dy    = sin(theta);
  double l_min = 0, l_max = 0, w_min = 0, w_max = 0;

  // 把点云从图像坐标 → 线段局部坐标
  for (size_t i = 0; i < reg.size(); ++i) {
    double regdx = double(reg[i].x) - x;
    double regdy = double(reg[i].y) - y;

    double l = regdx * dx + regdy * dy;  // 沿线方向
    double w = -regdx * dy + regdy * dx;  // 垂直线方向

    // 沿线段方向的最小旋转矩形，不是 axis-aligned 的 bbox
    if (l > l_max)
      l_max = l;
    else if (l < l_min)
      l_min = l;
    if (w > w_max)
      w_max = w;
    else if (w < w_min)
      w_min = w;
  }

  // Store values
  rec.x1    = x + l_min * dx;
  rec.y1    = y + l_min * dy;
  rec.x2    = x + l_max * dx;
  rec.y2    = y + l_max * dy;
  rec.width = w_max - w_min;
  rec.x     = x;
  rec.y     = y;
  rec.theta = theta;
  rec.dx    = dx;
  rec.dy    = dy;
  rec.prec  = prec;
  rec.p     = p;

  // Min width of 1 pixel
  if (rec.width < 1.0) rec.width = 1.0;
}

double LineSegmentDetector::get_theta(const std::vector<RegionPoint>& reg,
                                      const double& x, const double& y,
                                      const double& reg_angle,
                                      const double& prec) const {
  double Ixx = 0.0;
  double Iyy = 0.0;
  double Ixy = 0.0;

  // Compute inertia matrix
  for (size_t i = 0; i < reg.size(); ++i) {
    const double& regx   = reg[i].x;
    const double& regy   = reg[i].y;
    const double& weight = reg[i].modgrad;
    double dx            = regx - x;
    double dy            = regy - y;
    Ixx += dy * dy * weight;
    Iyy += dx * dx * weight;
    Ixy -= dx * dy * weight;
  }

  // Check if inertia matrix is null
  if (math::double_equal(Ixx, 0) && math::double_equal(Iyy, 0) &&
      math::double_equal(Ixy, 0)) {
    std::cerr << "[ERROR] PARAM ERROR IN get_theta" << std::endl;
    return 0;
  };

  // Compute smallest eigenvalue
  double lambda =
      0.5 * (Ixx + Iyy - sqrt((Ixx - Iyy) * (Ixx - Iyy) + 4.0 * Ixy * Ixy));

  // Compute angle
  double theta =
      (fabs(Ixx) > fabs(Iyy))
          ? double(cv::fastAtan2(float(lambda - Ixx), float(Ixy)))
          : double(cv::fastAtan2(float(Ixy), float(lambda - Iyy)));  // in degs
  theta *= math::UnitConverter::DEG_TO_RAD;

  // Correct angle by 180 deg if necessary
  if (apollo::common::math::AngleDiff(theta, reg_angle) > prec) {
    theta += M_PI;
  }

  return theta;
}

bool LineSegmentDetector::refine(std::vector<RegionPoint>& reg,
                                 double reg_angle, const double prec, double p,
                                 rect& rec, const double& density_th) {
  // 生长 → 评估 → 自我修正 → 再生长

  double density = double(reg.size()) /
                   (math::dist(rec.x1, rec.y1, rec.x2, rec.y2) * rec.width);

  // 用“密度”判断这是不是一条线
  if (density >= density_th) {
    return true;
  }

  // Try to reduce angle tolerance
  // 让 region_grow 的角度容差 从“固定经验值” 变成“数据自适应值”
  double xc           = double(reg[0].x);
  double yc           = double(reg[0].y);
  const double& ang_c = reg[0].angle;
  double sum = 0, s_sum = 0;
  int n = 0;

  for (size_t i = 0; i < reg.size(); ++i) {
    *(reg[i].used) = hps_.NOTUSED;
    if (math::dist(xc, yc, reg[i].x, reg[i].y) < rec.width) {
      const double& angle = reg[i].angle;
      double ang_d        = apollo::common::math::AngleDiff(angle, ang_c);
      sum += ang_d;
      s_sum += ang_d * ang_d;
      ++n;
    }
  }
  if (n <= 0) {
    std::cerr << "[ERROR] PARAM ERROR IN refine" << std::endl;
    return false;
  }
  double mean_angle = sum / double(n);
  // 2 * standard deviation
  double tau = 2.0 * sqrt((s_sum - 2.0 * mean_angle * sum) / double(n) +
                          mean_angle * mean_angle);

  // Try new region
  // clang-format off
  this->region_grow(base::Point2DI(reg[0].x, reg[0].y), angles, modgrad, used, reg, reg_angle, tau);
  // clang-format on

  if (reg.size() < 2) {
    return false;
  }

  this->region2rect(reg, reg_angle, prec, p, rec);
  density = double(reg.size()) /
            (math::dist(rec.x1, rec.y1, rec.x2, rec.y2) * rec.width);

  // 如果还是不合格？最后兜底策略
  if (density < density_th) {
    return this->reduce_region_radius(reg, reg_angle, prec, p, rec, density,
                                      density_th);
  } else {
    return true;
  }
}

bool LineSegmentDetector::reduce_region_radius(std::vector<RegionPoint>& reg,
                                               double reg_angle,
                                               const double prec, double p,
                                               rect& rec, double density,
                                               const double& density_th) {
  // Compute region's radius
  double xc     = double(reg[0].x);
  double yc     = double(reg[0].y);
  double radSq1 = math::distSquare(xc, yc, rec.x1, rec.y1);
  double radSq2 = math::distSquare(xc, yc, rec.x2, rec.y2);
  double radSq  = radSq1 > radSq2 ? radSq1 : radSq2;

  while (density < density_th) {
    // Reduce region's radius to 75% of its value
    radSq *= 0.75 * 0.75;
    // Remove points from the region and update 'used' map
    for (size_t i = 0; i < reg.size(); ++i) {
      // clang-format off
      if (math::distSquare(xc, yc, double(reg[i].x), double(reg[i].y)) > radSq) {
        // clang-format on

        // Remove point from the region
        *(reg[i].used) = hps_.NOTUSED;
        std::swap(reg[i], reg[reg.size() - 1]);
        reg.pop_back();
        --i;  // To avoid skipping one point
      }
    }

    if (reg.size() < 2) {
      return false;
    }

    // Re-compute rectangle
    this->region2rect(reg, reg_angle, prec, p, rec);

    // Re-compute region points density
    density = double(reg.size()) /
              (math::dist(rec.x1, rec.y1, rec.x2, rec.y2) * rec.width);
  }

  return true;
}

double LineSegmentDetector::rect_improve(rect& rec) const {
  // 在不改变线段方向和位置大结构的前提下，
  // 通过微调“角度精度 p”和“矩形宽度 / 边界位置”，
  // 最大化该线段的统计显著性（log NFA）。

  double delta   = 0.5;
  double delta_2 = delta / 2.0;

  double log_nfa = this->rect_nfa(rec);

  if (log_nfa > hps_.log_eps) return log_nfa;  // Good rectangle

  /* 启发式离散搜索
  尝试顺序  改什么	       目的
  1       减小角度容差p  提高方向一致性
  2       减小整体宽度   去掉边缘噪声
  3       只收缩一侧     纠正矩形偏移
  4       收缩另一侧     同上
  5       再次细化角度   最后榨干显著性
  */

  // Try to improve
  // Finer precision
  rect r = rect(rec);  // Copy
  for (int n = 0; n < 5; ++n) {
    r.p /= 2;
    r.prec             = r.p * M_PI;
    double log_nfa_new = this->rect_nfa(r);
    if (log_nfa_new > log_nfa) {
      log_nfa = log_nfa_new;
      rec     = rect(r);
    }
  }
  if (log_nfa > hps_.log_eps) return log_nfa;

  // Try to reduce width
  r = rect(rec);
  for (unsigned int n = 0; n < 5; ++n) {
    if ((r.width - delta) >= 0.5) {
      r.width -= delta;
      double log_nfa_new = this->rect_nfa(r);
      if (log_nfa_new > log_nfa) {
        rec     = rect(r);
        log_nfa = log_nfa_new;
      }
    }
  }
  if (log_nfa > hps_.log_eps) return log_nfa;

  // Try to reduce one side of rectangle
  r = rect(rec);
  for (unsigned int n = 0; n < 5; ++n) {
    if ((r.width - delta) >= 0.5) {
      r.x1 += -r.dy * delta_2;
      r.y1 += r.dx * delta_2;
      r.x2 += -r.dy * delta_2;
      r.y2 += r.dx * delta_2;
      r.width -= delta;
      double log_nfa_new = this->rect_nfa(r);
      if (log_nfa_new > log_nfa) {
        rec     = rect(r);
        log_nfa = log_nfa_new;
      }
    }
  }
  if (log_nfa > hps_.log_eps) return log_nfa;

  // Try to reduce other side of rectangle
  r = rect(rec);
  for (unsigned int n = 0; n < 5; ++n) {
    if ((r.width - delta) >= 0.5) {
      r.x1 -= -r.dy * delta_2;
      r.y1 -= r.dx * delta_2;
      r.x2 -= -r.dy * delta_2;
      r.y2 -= r.dx * delta_2;
      r.width -= delta;
      double log_nfa_new = this->rect_nfa(r);
      if (log_nfa_new > log_nfa) {
        rec     = rect(r);
        log_nfa = log_nfa_new;
      }
    }
  }
  if (log_nfa > hps_.log_eps) return log_nfa;

  // Try finer precision
  r = rect(rec);
  for (unsigned int n = 0; n < 5; ++n) {
    if ((r.width - delta) >= 0.5) {
      r.p /= 2;
      r.prec             = r.p * M_PI;
      double log_nfa_new = this->rect_nfa(r);
      if (log_nfa_new > log_nfa) {
        rec     = rect(r);
        log_nfa = log_nfa_new;
      }
    }
  }

  return log_nfa;
}

double LineSegmentDetector::rect_nfa(const rect& rec) const {
  int total_pts = 0, alg_pts = 0;
  double half_width = rec.width / 2.0;
  double dyhw       = rec.dy * half_width;
  double dxhw       = rec.dx * half_width;

  edge ordered_x[4];
  edge* min_y = &ordered_x[0];
  edge* max_y = &ordered_x[0];  // Will be used for loop range

  ordered_x[0].p.x   = int(rec.x1 - dyhw);
  ordered_x[0].p.y   = int(rec.y1 + dxhw);
  ordered_x[0].taken = false;
  ordered_x[1].p.x   = int(rec.x2 - dyhw);
  ordered_x[1].p.y   = int(rec.y2 + dxhw);
  ordered_x[1].taken = false;
  ordered_x[2].p.x   = int(rec.x2 + dyhw);
  ordered_x[2].p.y   = int(rec.y2 - dxhw);
  ordered_x[2].taken = false;
  ordered_x[3].p.x   = int(rec.x1 + dyhw);
  ordered_x[3].p.y   = int(rec.y1 - dxhw);
  ordered_x[3].taken = false;

  std::sort(ordered_x, ordered_x + 4, AsmallerB_XoverY);

  // Find min y. And mark as taken. find max y.
  for (unsigned int i = 1; i < 4; ++i) {
    if (min_y->p.y > ordered_x[i].p.y) {
      min_y = &ordered_x[i];
    }
    if (max_y->p.y < ordered_x[i].p.y) {
      max_y = &ordered_x[i];
    }
  }
  min_y->taken = true;

  // Find leftmost untaken point;
  edge* leftmost = 0;
  for (unsigned int i = 0; i < 4; ++i) {
    if (!ordered_x[i].taken) {
      if (!leftmost)  // if uninitialized
      {
        leftmost = &ordered_x[i];
      } else if (leftmost->p.x > ordered_x[i].p.x) {
        leftmost = &ordered_x[i];
      }
    }
  }
  if (leftmost == NULL) {
    std::cerr << "[ERROR] INVALID PARAM IN this->rect_nfa" << std::endl;
    return 0;
  }
  leftmost->taken = true;

  // Find rightmost untaken point;
  edge* rightmost = 0;
  for (unsigned int i = 0; i < 4; ++i) {
    if (!ordered_x[i].taken) {
      if (!rightmost)  // if uninitialized
      {
        rightmost = &ordered_x[i];
      } else if (rightmost->p.x < ordered_x[i].p.x) {
        rightmost = &ordered_x[i];
      }
    }
  }
  if (rightmost == NULL) {
    std::cerr << "[ERROR] INVALID PARAM IN this->rect_nfa" << std::endl;
    return 0;
  }
  rightmost->taken = true;

  // Find last untaken point;
  edge* tailp = 0;
  for (unsigned int i = 0; i < 4; ++i) {
    if (!ordered_x[i].taken) {
      if (!tailp)  // if uninitialized
      {
        tailp = &ordered_x[i];
      } else if (tailp->p.x > ordered_x[i].p.x) {
        tailp = &ordered_x[i];
      }
    }
  }
  if (tailp == NULL) {
    std::cerr << "[ERROR] INVALID PARAM IN this->rect_nfa" << std::endl;
    return 0;
  }
  tailp->taken = true;

  // first left step
  double flstep =
      (min_y->p.y != leftmost->p.y)
          ? (min_y->p.x - leftmost->p.x) / (min_y->p.y - leftmost->p.y)
          : 0;
  // second left step
  double slstep =
      (leftmost->p.y != tailp->p.x)
          ? (leftmost->p.x - tailp->p.x) / (leftmost->p.y - tailp->p.x)
          : 0;
  // first right step
  double frstep =
      (min_y->p.y != rightmost->p.y)
          ? (min_y->p.x - rightmost->p.x) / (min_y->p.y - rightmost->p.y)
          : 0;
  // second right step
  double srstep =
      (rightmost->p.y != tailp->p.x)
          ? (rightmost->p.x - tailp->p.x) / (rightmost->p.y - tailp->p.x)
          : 0;

  double lstep = flstep, rstep = frstep;

  double left_x = min_y->p.x, right_x = min_y->p.x;

  // Loop around all points in the region and count those that are aligned.
  int min_iter = min_y->p.y;
  int max_iter = max_y->p.y;
  for (int y = min_iter; y <= max_iter; ++y) {
    if (y < 0 || y >= img_height) continue;

    for (int x = int(left_x); x <= int(right_x); ++x) {
      if (x < 0 || x >= img_width) continue;

      ++total_pts;
      if (this->isAligned(this->angles, x, y, rec.theta, rec.prec)) {
        ++alg_pts;
      }
    }

    if (y >= leftmost->p.y) {
      lstep = slstep;
    }
    if (y >= rightmost->p.y) {
      rstep = srstep;
    }

    left_x += lstep;
    right_x += rstep;
  }

  return this->nfa(total_pts, alg_pts, rec.p);
}

double LineSegmentDetector::nfa(const int& n, const int& k,
                                const double& p) const {
  // Trivial cases
  if (n == 0 || k == 0) {
    return -LOG_NT;
  }
  if (n == k) {
    return -LOG_NT - double(n) * log10(p);
  }

  double p_term = p / (1 - p);

  double log1term = (double(n) + 1) - log_gamma(double(k) + 1) -
                    log_gamma(double(n - k) + 1) + double(k) * log(p) +
                    double(n - k) * log(1.0 - p);
  double term = exp(log1term);

  if (math::double_equal(term, 0)) {
    if (k > n * p)
      return -log1term / M_LN10 - LOG_NT;
    else
      return -LOG_NT;
  }

  // Compute more terms if needed
  double bin_tail  = term;
  double tolerance = 0.1;  // an error of 10% in the result is accepted
  for (int i = k + 1; i <= n; ++i) {
    double bin_term  = double(n - i + 1) / double(i);
    double mult_term = bin_term * p_term;
    term *= mult_term;
    bin_tail += term;
    if (bin_term < 1) {
      double err =
          term *
          ((1 - pow(mult_term, double(n - i + 1))) / (1 - mult_term) - 1);
      if (err < tolerance * std::fabs(-log10(bin_tail) - LOG_NT) * bin_tail)
        break;
    }
  }
  return -log10(bin_tail) - LOG_NT;
}

void LineSegmentDetector::get_edge_point(
    std::vector<RegionPoint>& reg_points, const double& reg_angle,
    std::vector<base::Point2DI>& edge_point) {
  edge_point.clear();
  std::sort(reg_points.begin(), reg_points.end(), sortRegionFunction);

  // Angle tolerance
  const double prec = M_PI * hps_.ang_th / 180;
  const double p    = hps_.ang_th / 180;
  const double rho  = hps_.quant / sin(prec);  // gradient magnitude threshold

  int gap = 20;
  for (int i = 0; i < reg_points.size() - gap; i += gap) {
    rect rec;
    this->region2rect(std::vector<RegionPoint>(reg_points.begin() + i,
                                               reg_points.begin() + i + gap),
                      reg_angle, prec, p, rec);
    rec.x1 += 0.5;
    rec.y1 += 0.5;
    rec.x2 += 0.5;
    rec.y2 += 0.5;

    // clang-format off
    edge_point.push_back(base::Point2DI(int(rec.x1 / hps_.scale), int(rec.y1 / hps_.scale)));
    // edge_point.push_back(base::Point2DI(int(rec.x / hps_.scale), int(rec.y / hps_.scale)));
    edge_point.push_back(base::Point2DI(int(rec.x2 / hps_.scale), int(rec.y2 / hps_.scale)));
    // std::cout << int(rec.x1 / hps_.scale) << "\t" << int(rec.y1 / hps_.scale) <<
    // std::endl;
    // std::cout << int(rec.x / hps_.scale) << "\t" << int(rec.y / hps_.scale) <<
    // std::endl;
    // std::cout << int(rec.x2 / hps_.scale) << "\t" << int(rec.y2 / hps_.scale) <<
    // std::endl;
    // clang-format on
  }
}

void LineSegmentDetector::drawSegments(
    cv::Mat& _image,
    const std::vector<std::vector<base::Point2DI>>& line_points) {
  if (_image.empty()) {
    std::cerr << "[ERROR] WRONG INPUT IMAGE" << std::endl;
  }

  if (_image.channels() == 1) {
    cv::cvtColor(_image, _image, cv::COLOR_GRAY2BGR);
  }

  const int line_num = line_points.size();
  cv::Scalar color(0, 0, 255);

  // for(int line_idx = 0; line_idx < line_num; line_idx ++){
  for (int line_idx = 0; line_idx < line_num; line_idx++) {
    if (line_idx % 2 == 0) {
      color = cv::Scalar(0, 55, 255);
    } else {
      color = cv::Scalar(155, 255, 50);
    }
    for (size_t p_idx = 0; p_idx < line_points[line_idx].size(); p_idx++) {
      const cv::Point p(line_points[line_idx][p_idx].x,
                        line_points[line_idx][p_idx].y);
      // cv::circle(_image, p, 1, color, 0);
      cv::circle(_image, p, 2, color, 2);
    }
  }
}

void LineSegmentDetector::drawSegments(
    cv::Mat& _image, const std::vector<cstruct::Vector4f>& lines) {
  if (_image.empty()) {
    std::cerr << "[ERROR] WRONG INPUT IMAGE" << std::endl;
  }

  if (_image.channels() == 1) {
    cv::cvtColor(_image, _image, cv::COLOR_GRAY2BGR);
  }

  const int N = lines.size();
  cv::Scalar color(0, 0, 255);

  // Draw segments
  for (int i = 0; i < N; ++i) {
    if (i % 2 == 0) {
      color = cv::Scalar(0, 0, 255);
    } else {
      color = cv::Scalar(255, 0, 0);
    }
    const cstruct::Vector4f& v = lines[i];
    const cv::Point2f b(v.x, v.y);
    const cv::Point2f e(v.z, v.w);
    cv::line(_image, b, e, color, 1);
  }
}

void LineSegmentDetector::drawRect(cv::Mat& _image, const rect& rec) {
  // 线段方向
  cv::Point2d dir(rec.dx, rec.dy);
  // 垂直方向（左法线）
  cv::Point2d n(-rec.dy, rec.dx);
  double hw = rec.width * 0.5;

  cv::Point2d p1(rec.x1, rec.y1);
  cv::Point2d p2(rec.x2, rec.y2);

  cv::Point2d c1 = p1 + n * hw;
  cv::Point2d c2 = p1 - n * hw;
  cv::Point2d c3 = p2 - n * hw;
  cv::Point2d c4 = p2 + n * hw;

  std::vector<cv::Point> box;
  box.emplace_back(cv::Point(cvRound(c1.x), cvRound(c1.y)));
  box.emplace_back(cv::Point(cvRound(c2.x), cvRound(c2.y)));
  box.emplace_back(cv::Point(cvRound(c3.x), cvRound(c3.y)));
  box.emplace_back(cv::Point(cvRound(c4.x), cvRound(c4.y)));

  const cv::Point* pts = box.data();
  int npts             = 4;

  cv::polylines(_image, &pts, &npts, 1, true, cv::Scalar(0, 255, 0), 1);

  // 画中心
  cv::circle(_image, cv::Point(cvRound(rec.x), cvRound(rec.y)), 2,
             cv::Scalar(255, 0, 0), -1);
}

void LineSegmentDetector::getResults(
    std::vector<cstruct::Vector4f>& lines,
    std::vector<std::vector<int>>& line_point_map) {
  lines          = _lines;
  line_point_map = _line_point_map;
}
