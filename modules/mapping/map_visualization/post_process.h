#pragma once

#include <opencv2/opencv.hpp>

#include "modules/mapping/map_representation/common.h"

// 纯解析，读取 xml 文件中的 点 坐标
std::vector<cv::Point> ParseCVATPoints(const std::string& points_str);

// 将XML坐标转换为OpenCV坐标
cv::Point XMLToMatIndex(const cv::Point& p_xml, int xml_rows);

bool IsNearBoundary(const cv::Mat& map, int r, int c);

void FillHoleSemanticMap(cv::Mat& label_mat);

cv::Vec3b GetLabelColor(int label_type);