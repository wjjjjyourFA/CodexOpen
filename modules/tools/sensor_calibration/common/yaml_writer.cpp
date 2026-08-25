#include "modules/tools/sensor_calibration/common/yaml_writer.h"

void YamlMatWriter::WriteMat(const std::string& name, const cv::Mat& mat) {
  std::ofstream fout;
  if (first_write_) {
    fout.open(filename_, std::ios::out | std::ios::trunc);  // ✅ 第一次：覆盖
    fout << "%YAML:1.0\n---\n";
    first_write_ = false;
  } else {
    fout.open(filename_, std::ios::app);  // ✅ 后续：追加
  }

  if (!fout.is_open()) {
    std::cerr << "Cannot open " << filename_ << " for writing!" << std::endl;
    return;
  }

  fout << name << ": !!opencv-matrix\n";
  fout << "   rows: " << mat.rows << "\n";
  fout << "   cols: " << mat.cols << "\n";
  const char* depth_code = nullptr;
  switch (mat.depth()) {
    case CV_8U:  depth_code = "u"; break;
    case CV_8S:  depth_code = "c"; break;
    case CV_16U: depth_code = "w"; break;
    case CV_16S: depth_code = "s"; break;
    case CV_32S: depth_code = "i"; break;
    case CV_32F: depth_code = "f"; break;
    case CV_64F: depth_code = "d"; break;
    default:
      std::cerr << "Unsupported OpenCV matrix depth: " << mat.depth()
                << std::endl;
      return;
  }
  fout << "   dt: ";
  if (mat.channels() > 1) fout << mat.channels();
  fout << depth_code << "\n";
  fout << "   data: [ ";

  fout << std::fixed;
  // fout << std::fixed << std::setprecision(10);

  const cv::Mat continuous = mat.isContinuous() ? mat : mat.clone();
  const cv::Mat flat       = continuous.reshape(1, 1);
  const int total          = flat.cols;
  for (int i = 0; i < total; ++i) {
    double val;
    switch (mat.depth()) {
      case CV_64F:
        val = flat.at<double>(0, i);
        break;
      case CV_32F:
        val = flat.at<float>(0, i);
        break;
      case CV_32S:
        val = flat.at<int>(0, i);
        break;
      case CV_16S:
        val = flat.at<short>(0, i);
        break;
      case CV_16U:
        val = flat.at<unsigned short>(0, i);
        break;
      case CV_8S:
        val = flat.at<signed char>(0, i);
        break;
      case CV_8U:
        val = flat.at<unsigned char>(0, i);
        break;
      default: return;
    }

    // 输出格式处理
    if (std::fabs(val - std::round(val)) < 1e-9) {
      // 整数输出成 960. 这样的形式
      fout << static_cast<long long>(std::llround(val)) << ".";
    } else {
      // 浮点数保留 8 位小数
      fout << std::setprecision(8) << val;
    }

    if (i != total - 1) fout << ", ";
  }
  fout << " ]\n";
}
