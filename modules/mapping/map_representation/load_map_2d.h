#pragma once

#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "modules/mapping/map_representation/common.h"

typedef struct {
  int rows;  // 行数
  int cols;  // 列数
  int type;  // 类型
} MatHeader;

void WriteMat(std::string fileName, cv::Mat& src);

void ReadMat(std::string fileName, cv::Mat& dst);

void ValidateSemanticLabels(cv::Mat& semantic_mat);

void LoadSemanticMap(const std::string& gray_mat_path, cv::Mat& semantic_mat);

void LoadCubeMap(const std::string& gray_mat_path, cv::Mat& cube_structure_mat);