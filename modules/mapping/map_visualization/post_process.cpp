#include "modules/mapping/map_visualization/post_process.h"

// 纯解析，读取 xml 文件中的 点 坐标
std::vector<cv::Point> ParseCVATPoints(const std::string& points_str) {
  std::vector<cv::Point> pts;

  std::stringstream ss(points_str);
  std::string item;

  while (std::getline(ss, item, ';')) {
    if (item.empty()) continue;

    std::stringstream ps(item);
    std::string xs, ys;

    if (std::getline(ps, xs, ',') && std::getline(ps, ys, ',')) {
      int x = static_cast<int>(std::round(std::stod(xs)));
      int y = static_cast<int>(std::round(std::stod(ys)));
      pts.emplace_back(x, y);
    }
  }
  return pts;
};

// 将XML坐标转换为OpenCV坐标
cv::Point XMLToMatIndex(const cv::Point& p_xml, int xml_rows) {
  // Lambda 表达式：将 XML 中的坐标点转换为当前 label_terrain_mat 的坐标点
  const int c_mat = p_xml.x;
  const int r_mat = p_xml.y;
  // 翻转 y 轴坐标
  // const int r_mat = (xml_rows - 1 - p_xml.y);

  return cv::Point(c_mat, r_mat);
}

bool IsNearBoundary(const cv::Mat& map, int r, int c) {
  // 定义一个 Lambda 表达式用于邻域边界检测
  // 检查以 (r, c) 为中心 3x3 范围内是否有边界标签
  const int height = map.rows;
  const int width  = map.cols;

  /* way 1
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      int nr = r + dr;
      int nc = c + dc;
      if (nr < 0 || nr >= height || nc < 0 || nc >= width) {
        continue;
      }

      if (map.at<int>(nr, nc) == SemanticLabel::ROAD_BRIDGE) {
        return true;
      }
    }
  }
    */

  // /* way 2
  int r0 = std::max(0, r - 1);
  int r1 = std::min(height - 1, r + 1);
  int c0 = std::max(0, c - 1);
  int c1 = std::min(width - 1, c + 1);

  for (int nr = r0; nr <= r1; ++nr) {
    const int* row_ptr = map.ptr<int>(nr);
    for (int nc = c0; nc <= c1; ++nc) {
      if (row_ptr[nc] == SemanticLabel::ROAD_BRIDGE) {
        return true;
      }
    }
  }
  // */

  return false;
};

void FillHoleSemanticMap(cv::Mat& label_mat) {
  const int H = label_mat.rows;
  const int W = label_mat.cols;

  cv::Mat visited = cv::Mat::zeros(H, W, CV_8U);

  const int dx[4] = {1, -1, 0, 0};
  const int dy[4] = {0, 0, 1, -1};

  for (int r = 0; r < H; ++r) {
    for (int c = 0; c < W; ++c) {
      int& v = label_mat.at<int>(r, c);

      // 1、2 代表：道路边界、道路表面
      // 只处理“非 1、非 2”的区域（空洞候选）
      if (v == SemanticLabel::ROAD_BRIDGE || v == SemanticLabel::ROAD_SURFACE ||
          visited.at<uchar>(r, c)) {
        continue;
      }

      std::vector<cv::Point> component;
      std::queue<cv::Point> q;

      bool touch_border   = false;
      bool touch_obstacle = false;

      visited.at<uchar>(r, c) = 1;
      q.emplace(c, r);

      while (!q.empty()) {
        auto p = q.front();
        q.pop();
        component.push_back(p);

        // 是否接触地图边界
        if (p.x <= 0 || p.x >= W - 1 || p.y <= 0 || p.y >= H - 1) {
          touch_border = true;
        }

        for (int k = 0; k < 4; ++k) {
          int nx = p.x + dx[k];
          int ny = p.y + dy[k];
          if (nx < 0 || nx > W - 1 || ny < 0 || ny > H - 1) continue;

          int nv = label_mat.at<int>(ny, nx);
          if (nv == SemanticLabel::ROAD_BRIDGE) {
            touch_obstacle = true;
            continue;
          }

          if (visited.at<uchar>(ny, nx)) continue;

          // 只在非 2 区域内扩展++
          if (nv != SemanticLabel::ROAD_SURFACE) {
            visited.at<uchar>(ny, nx) = 1;
            q.emplace(nx, ny);
          }
        }
      }

      // 判定是否为“可填补空洞”，填充道路表面
      if (!touch_border && !touch_obstacle) {
        for (const auto& p : component) {
          label_mat.at<int>(p.y, p.x) = SemanticLabel::ROAD_SURFACE;
        }
      }
    }
  }
};

cv::Vec3b GetLabelColor(int label_type) {
  cv::Vec3b pix;

  switch (label_type) {
    case SemanticLabel::UNKNOWN:
      // 黑色 (BGR)
      pix = cv::Vec3b(0, 0, 0);
      break;

    case SemanticLabel::ROAD_BRIDGE:
      // 紫色 (BGR)
      pix = cv::Vec3b(230, 75, 200);
      break;

    case SemanticLabel::ROAD_SURFACE:
      // 绿色 (BGR)
      pix = cv::Vec3b(0, 255, 0);
      break;

    case SemanticLabel::GROUND:
      // 深绿色 (BGR)
      // pix = cv::Vec3b(0, 200, 0);
      pix = cv::Vec3b(40, 120, 40);
      break;

    case SemanticLabel::OBSTACLE:
      // 红 (BGR)
      pix = cv::Vec3b(0, 0, 255);
      break;

    case SemanticLabel::HANGING:
      // 黄 (BGR)
      pix = cv::Vec3b(0, 255, 255);
      break;

    case SemanticLabel::FUSIONOB:
      // 深绿色 (BGR)
      pix = cv::Vec3b(0, 80, 255);
      break;

    default:
      // 其他值（可选：标成蓝色或白色，方便 debug）
      pix = cv::Vec3b(255, 255, 255);
      break;
  }

  return pix;
}