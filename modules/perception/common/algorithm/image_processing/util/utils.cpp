#include "modules/perception/common/algorithm/image_processing/util/utils.h"

// 提取方式是工业上常用的 ZYX 欧拉角（Tait-Bryan角）标准
// 输入：旋转向量 rvec (Rodrigues形式)
// 输出：欧拉角 {roll, pitch, yaw}
//      单位：弧度，
//      顺序：绕X轴 roll，绕Y轴 pitch，绕Z轴 yaw
void ConvertRodriguesToEulerZYX(const cv::Mat& rvec, cv::Mat& rotR,
                                std::vector<double>& euler) {
  // 计算旋转矩阵
  cv::Rodrigues(rvec, rotR);

  // sine or cosine of yaw/pitch
  double sy = std::sqrt(rotR.at<double>(0, 0) * rotR.at<double>(0, 0) +
                        rotR.at<double>(1, 0) * rotR.at<double>(1, 0));
  // 判断是否接近奇异点（万向锁）
  bool singular = sy < 1e-6;

  double roll, pitch, yaw;
  if (!singular) {
    // 绕X轴旋转
    roll = std::atan2(rotR.at<double>(2, 1), rotR.at<double>(2, 2));
    // 绕Y轴旋转
    pitch = std::atan2(-rotR.at<double>(2, 0), sy);
    // 绕Z轴旋转
    yaw = std::atan2(rotR.at<double>(1, 0), rotR.at<double>(0, 0));
  } else {
    // 万向锁情况下的处理
    roll  = std::atan2(-rotR.at<double>(1, 2), rotR.at<double>(1, 1));
    pitch = std::atan2(-rotR.at<double>(2, 0), sy);
    yaw   = 0;
  }

  euler.clear();
  euler.push_back(yaw);
  euler.push_back(pitch);
  euler.push_back(roll);
}

// 输入：rotR: 3x3旋转矩阵 (cv::Mat, double)
//      tvec: 3x1平移向量 (cv::Mat, double)
// 输出: RT: 4x4齐次变换矩阵 (cv::Mat, double)
void ComposeTransformationMatrix(const cv::Mat& rotR, const cv::Mat& tvec,
                                 cv::Mat& RT) {
  CV_Assert(rotR.rows == 3 && rotR.cols == 3);
  CV_Assert(tvec.rows == 3 && (tvec.cols == 1 || tvec.cols == 3));

  RT = cv::Mat::eye(4, 4, rotR.type());
  // 复制旋转部分
  rotR.copyTo(RT(cv::Rect(0, 0, 3, 3)));
  // 复制平移部分，确保tvec是3x1
  cv::Mat rotT = tvec.reshape(1, 3);
  rotT.copyTo(RT(cv::Rect(3, 0, 1, 3)));
}

cv::Rect GetBoundingRect(const std::vector<cv::Point2f>& corners) {
  cv::Rect rect = cv::boundingRect(corners);
  return rect;
}

// 输入：ZYX顺序的欧拉角 (roll, pitch, yaw)
// 输出：旋转矩阵 rotR 和 Rodrigues 向量 rvec
void ConvertEulerZYXToRodrigues(const std::vector<double>& euler, cv::Mat& rvec,
                                cv::Mat& rotR) {
  // 确保输入有3个角度
  CV_Assert(euler.size() == 3);

  double yaw   = euler[0];  // Z
  double pitch = euler[1];  // Y
  double roll  = euler[2];  // X

  // 构造各轴旋转矩阵 (OpenCV默认 double)
  // clang-format off
  cv::Mat Rz = (cv::Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0, 
                                          sin(yaw), cos(yaw), 0, 
                                          0, 0, 1);

  cv::Mat Ry = (cv::Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch), 
                                          0, 1, 0,
                                          -sin(pitch), 0, cos(pitch));

  cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 
                                          0, cos(roll), -sin(roll), 
                                          0, sin(roll), cos(roll));
  // clang-format on

  // ZYX 顺序旋转
  rotR = Rz * Ry * Rx;

  // 转成 Rodrigues 向量
  cv::Rodrigues(rotR, rvec);
}

cv::Rect AdjustRectWithScale(const cv::Rect& rect, const float& scale,
                             const cv::Size& image_size) {
  // 原矩形中心
  float cx = rect.x + rect.width * 0.5f;
  float cy = rect.y + rect.height * 0.5f;

  // 新的宽高（>1 放大，<1 缩小）
  int new_width  = static_cast<int>(rect.width * scale);
  int new_height = static_cast<int>(rect.height * scale);

  // 左上角坐标（保持中心位置不变）
  int new_x = static_cast<int>(cx - new_width * 0.5f);
  int new_y = static_cast<int>(cy - new_height * 0.5f);

  // 确保新的矩形不超出图像边界
  new_x = std::max(0, std::min(new_x, image_size.width - 1));
  new_y = std::max(0, std::min(new_y, image_size.height - 1));

  // 确保新的宽高不超出图像边界
  if (new_x + new_width > image_size.width) {
    new_width = image_size.width - new_x;
  }
  if (new_y + new_height > image_size.height) {
    new_height = image_size.height - new_y;
  }

  return cv::Rect(new_x, new_y, new_width, new_height);
}

size_t RoiMask2PointCloud(const std::vector<cv::Mat>& splits,
                          std::vector<cv::Mat>& splits_roi, const cv::Rect& roi,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          size_t RoiLimit) {
  if (!cloud) return 0U;
  cloud->clear();

  if (splits.size() < 3 || splits_roi.size() < 3) return;
  // 直接清除
  // splits_roi.clear();

  // 预估总点数，reserve 提高效率
  size_t total = roi.width * roi.height;
  cloud->points.reserve(total);

  if (total > RoiLimit) {
    // 取 ROI 拷贝，Mat 内存连续，后续可以线性指针访问；支持上传 GPU
    /* way 1
    splits_roi.push_back(splits[2](roi).clone());  // x
    splits_roi.push_back(splits[1](roi).clone());  // y
    splits_roi.push_back(splits[0](roi).clone());  // z
    */
    splits_roi[0] = (splits[2](roi).clone());  // x
    splits_roi[1] = (splits[1](roi).clone());  // y
    splits_roi[2] = (splits[0](roi).clone());  // z

    const float* x_ptr = reinterpret_cast<const float*>(splits_roi[0].data);
    const float* y_ptr = reinterpret_cast<const float*>(splits_roi[1].data);
    const float* z_ptr = reinterpret_cast<const float*>(splits_roi[2].data);

    for (size_t c = 0; c < total; c++) {
      float x = x_ptr[c];
      float y = y_ptr[c];
      float z = z_ptr[c];

      // mask == 0 过滤
      if (x == 0.f && y == 0.f && z == 0.f) continue;

      cloud->points.emplace_back(x, y, z);
    }
  } else {
    // 获取 ROI 引用，Mat 内存不连续，不 clone，节省内存
    splits_roi[0] = (splits[2](roi));  // x
    splits_roi[1] = (splits[1](roi));  // y
    splits_roi[2] = (splits[0](roi));  // z

    for (int r = 0; r < splits_roi[0].rows; ++r) {
      const float* x_row = splits_roi[0].ptr<float>(r);
      const float* y_row = splits_roi[1].ptr<float>(r);
      const float* z_row = splits_roi[2].ptr<float>(r);

      for (int c = 0; c < splits_roi[0].cols; ++c) {
        float x = x_row[c];
        float y = y_row[c];
        float z = z_row[c];

        if (x == 0.f && y == 0.f && z == 0.f) continue;

        cloud->points.emplace_back(x, y, z);
      }
    }
  }

  cloud->width    = static_cast<uint32_t>(cloud->points.size());
  cloud->height   = 1U;
  cloud->is_dense = false;

  return cloud->points.size();
}

size_t RoiMask2PointCloud(const cv::Mat& mask, const cv::Rect& requested_roi,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  if (!cloud) return 0U;
  cloud->clear();

  if (mask.empty() || mask.type() != CV_32FC3) return 0U;

  const cv::Rect roi = requested_roi & cv::Rect(0, 0, mask.cols, mask.rows);
  if (roi.empty()) return 0U;

  cloud->points.reserve(static_cast<size_t>(roi.width) * roi.height);

  for (int row = roi.y; row < roi.y + roi.height; ++row) {
    const cv::Vec3f* values = mask.ptr<cv::Vec3f>(row);
    for (int col = roi.x; col < roi.x + roi.width; ++col) {
      const cv::Vec3f& value = values[col];
      // (z, y, x)
      const float x = value[2];
      const float y = value[1];
      const float z = value[0];

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      if (x == 0.0f && y == 0.0f && z == 0.0f) continue;

      cloud->points.emplace_back(x, y, z);
    }
  }

  cloud->width    = static_cast<uint32_t>(cloud->points.size());
  cloud->height   = 1U;
  cloud->is_dense = false;

  return cloud->points.size();
}
