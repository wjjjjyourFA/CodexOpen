#ifndef CV_UTILS_H
#define CV_UTILS_H

#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>

// 1. rvec（Rodrigues 向量）
// 形状：3×1 或 1×3 的向量（cv::Mat）。
// 表示方法：轴角表示法 (axis-angle representation)。
// 向量方向 = 旋转轴
// 向量长度 = 旋转角度（弧度）

// 2. rotR（旋转矩阵）
// 形状：3×3 矩阵。
// 表示方法：正交矩阵，满足 R * R^T = I，det(R) = 1。
// 可以直接用来旋转 3D 点：

// 提取方式是工业上常用的 ZYX 欧拉角（Tait-Bryan角）标准
// 输入：旋转向量 rvec (Rodrigues形式)
// 输出：欧拉角 (yaw, pitch, roll)
//      单位：弧度，
//      顺序：绕X轴 roll，绕Y轴 pitch，绕Z轴 yaw
void ConvertRodriguesToEulerZYX(const cv::Mat& rvec, cv::Mat& rotR,
                                std::vector<double>& euler);

// 输入：rotR: 3x3旋转矩阵 (cv::Mat, double)
//      tvec: 3x1平移向量 (cv::Mat, double)
// 输出: RT: 4x4齐次变换矩阵 (cv::Mat, double)
void ComposeTransformationMatrix(const cv::Mat& rotR, const cv::Mat& tvec,
                                 cv::Mat& RT);

// 二维点云的外接矩形
cv::Rect GetBoundingRect(const std::vector<cv::Point2f>& corners);

// 输入：ZYX顺序的欧拉角 (yaw, pitch, roll)
// 输出：旋转矩阵 rotR 和 Rodrigues 向量 rvec
void ConvertEulerZYXToRodrigues(const std::vector<double>& euler, cv::Mat& rvec,
                                cv::Mat& rotR);

cv::Rect AdjustRectWithScale(const cv::Rect& rect, const float& scale,
                             const cv::Size& image_size);

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

void RoiMask2PointCloud(const std::vector<cv::Mat>& splits,
                        std::vector<cv::Mat>& splits_roi, const cv::Rect& roi,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        size_t RoiLimit = 128 * 128);

#endif
