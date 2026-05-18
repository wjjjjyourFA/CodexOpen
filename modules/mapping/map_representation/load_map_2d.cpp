#include "modules/mapping/map_representation/load_map_2d.h"

void WriteMat(std::string fileName, cv::Mat& src) {
  MatHeader matHeader{src.rows, src.cols, src.type()};

  std::ofstream out(fileName, std::ios::binary);

  out.write((char*)&matHeader, sizeof(MatHeader));
  out.write((char*)src.data, src.rows * src.cols * src.elemSize());

  out.flush();
  out.close();
}

void ReadMat(std::string fileName, cv::Mat& dst) {
  std::ifstream in(fileName, std::ios::binary);

  MatHeader matHeader{0, 0, 0};
  in.read((char*)&matHeader, sizeof(MatHeader));

  cv::Mat mat(matHeader.rows, matHeader.cols, matHeader.type);
  in.read((char*)mat.data, mat.rows * mat.cols * mat.elemSize());

  in.close();
  dst = mat;
}

void ValidateSemanticLabels(cv::Mat& semantic_mat) {
  int invalid_cnt = 0;

  for (int r = 0; r < semantic_mat.rows; ++r) {
    for (int c = 0; c < semantic_mat.cols; ++c) {
      int& v = semantic_mat.at<int>(r, c);

      switch (v) {
        case SemanticLabel::UNKNOWN:
        case SemanticLabel::ROAD_BRIDGE:
        case SemanticLabel::ROAD_SURFACE:
        case SemanticLabel::GROUND:
          break;

        default:
          v = SemanticLabel::UNKNOWN;
          ++invalid_cnt;
          break;
      }
    }
  }

  if (invalid_cnt > 0) {
    std::cout << "[SemanticMap] Warning: " << invalid_cnt
              << " invalid labels reset to UNKNOWN" << std::endl;
  }
}

void LoadSemanticMap(const std::string& gray_mat_path, cv::Mat& semantic_mat) {
  // 1. 读取灰度图
  cv::Mat gray_u8 = cv::imread(gray_mat_path, cv::IMREAD_GRAYSCALE);

  if (gray_u8.empty()) {
    std::cerr << "[SemanticMap] Failed to load gray image: " << gray_mat_path
              << std::endl;
    return;
  }

  // 2. 转回 CV_32SC1
  gray_u8.convertTo(semantic_mat, CV_32SC1);

  // 3. 合法性检查（防止脏数据）
  ValidateSemanticLabels(semantic_mat);

  std::cout << "[SemanticMap] Loaded label map: " << gray_mat_path << " ("
            << semantic_mat.cols << " x " << semantic_mat.rows << ")"
            << std::endl;
};

void LoadCubeMap(const std::string& gray_mat_path,
                 cv::Mat& cube_structure_mat) {
  cv::Mat gray_u8 = cv::imread(gray_mat_path, cv::IMREAD_GRAYSCALE);  // 0-255

  if (gray_u8.empty()) {
    std::cerr << "[CubeMap] Failed to load gray image: " << gray_mat_path
              << std::endl;
    return;
  }

  cube_structure_mat = gray_u8.clone();

  std::cout << "[CubeMap] Loaded label map: " << gray_mat_path << " ("
            << cube_structure_mat.cols << " x " << cube_structure_mat.rows
            << ")" << std::endl;
};
